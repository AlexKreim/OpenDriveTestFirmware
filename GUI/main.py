"""ESC Control: Windows GUI for the custom STM32G474 ESC.

Stage 1: open-loop speed control with automatic parameters, motor presets, live telemetry/plots/CSV, parameters in flash.
Stage 2: field-oriented control with torque, speed and position modes, tunable PID values, motor calibration,
step-response helper and overtemperature shutdown settings.
"""
import collections
import csv
import queue
import sys
import time

try:
    import numpy as np
    import pyqtgraph as pg
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError as _e:      # pragma: no cover
    print("Missing dependency: %s\nInstall with:  pip install -r requirements.txt" % _e)
    raise

try:
    import serial.tools.list_ports as list_ports
except ImportError:            # pragma: no cover
    list_ports = None

import presets as PR
import preset_dialog
import protocol as P
import stepmetrics
from link import EscLink

APP_TITLE = "ESC Control"
MAX_SAMPLES = 100 * 120        # two minutes at 100 Hz
PLOT_KEYS = ["rpm_cmd", "rpm_spi", "rpm_abi", "ia", "ib", "ic", "vbus", "vll", "angle",
             "imag", "id", "iq", "iqref", "possp", "step_target"]

# step-response helper: name, firmware mode, unit, default low, default high, default half period (s), allowed range, decimals,
#                        key of the reference the firmware reports, key of the measurement
STEP_KINDS = [
    ("Torque (A, FOC)",   P.MODE_TORQUE,   "A",   -0.3, 0.3,  0.3, (-100.0, 100.0), 2, "iqref",   "iq"),
    ("Speed (rpm, FOC)",  P.MODE_SPEED,    "rpm", 100.0, 300.0, 1.5, (0.0, 20000.0), 0, "rpm_cmd", "rpm_abi"),
    ("Position (deg, FOC)", P.MODE_POSITION, "deg", 90.0, 120.0, 1.0, (0.0, 360.0), 1, "possp",   "angle"),
]

STOP_STYLE = "QPushButton { background-color: #c62828; color: white; font-weight: bold; font-size: 15px; }" \
             "QPushButton:disabled { background-color: #888; }"
START_STYLE = "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; font-size: 15px; }" \
              "QPushButton:disabled { background-color: #888; }"

ERR_TEXT = {
    "motor_running": "The motor is running. Stop it first.",
    "no_saved_data": "Nothing valid is stored in flash yet.",
    "flash_erase": "Flash erase failed.",
    "flash_program": "Flash programming failed.",
    "flash_verify": "Flash verify failed.",
    "bus_voltage": "Bus voltage is outside the allowed range (check the supply).",
    "no_cal": "Current-sensor calibration is missing. Restart the board with the motor idle.",
    "fault": "The board is in a fault state. Press 'Clear fault' first.",
    "not_idle": "The motor is not idle.",
    "invalid": "Value rejected by the firmware (out of range).",
    "noisy": "Current-sensor reading is too noisy to calibrate (is current flowing, or is the supply connected?).",
    "saturated": "A current-sense output is stuck at 0 V or 3.3 V (see the Raw ADC check). Check the op-amp settings and the gain; nothing was stored.",
    "cal_failed": "Current-sensor calibration failed (no ADC samples arriving).",
    "no_enc_cal": "FOC needs the encoder calibration first: Tuning tab, 'Calibrate motor'.",
    "range": "Value outside the allowed range.",
    "refused": "The firmware refused the command.",
}

TUNE_ERR_TEXT = {
    "rpm_range": "The test speed would be below 100 rpm (speed limit, Kv or bus voltage too low).",
    "current_range": "The test current (half of the FOC current limit) is too small.",
    "wrong_direction": "The motor turned the wrong way. Run the motor calibration again.",
    "no_accel": "The motor did not reach the test speed within 4 s (load too high, or current limit too low).",
    "no_decel": "The motor did not slow down within 4 s.",
    "fit_accel": "Too few speed samples while accelerating (the motor accelerates too fast: lower the FOC current limit).",
    "fit_decel": "Too few speed samples while braking (lower the FOC current limit).",
    "bad_fit": "The measured acceleration / deceleration is not plausible.",
    "inertia_range": "The measured inertia is not plausible.",
    "range": "The computed gains are outside the allowed parameter range. Nothing was changed.",
    "fault": "A fault stopped the test.",
    "aborted": "The test was stopped.",
}

CAL_ERR_TEXT = {
    "encoder": "The encoder (AS5047P) did not answer reliably.",
    "no_current": "The calibration current did not flow. Check the motor wires, the supply and the current sensors.",
    "bad_resistance": "The measured resistance is not plausible. Check the motor wires.",
    "bad_inductance": "The measured inductance is not plausible (very low or very high).",
    "no_data": "Not enough measurement data was collected.",
    "no_motion": "The shaft did not turn. Is it free to rotate? Raise the calibration current if the motor sticks.",
    "pole_pairs": "The measured number of pole pairs is not an integer. The shaft may have been blocked or the encoder is noisy. Nothing was changed except R, L and the direction.",
    "range": "A measured value is outside the allowed range of the parameters.",
    "fault": "A fault stopped the calibration.",
    "aborted": "The calibration was stopped.",
}


def fmt_num(v):
    return ("%.6g" % v)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.link = EscLink()
        self.params = {}
        self.sample = None
        self.state = P.ST_IDLE
        self.fault = 0
        self.dirty = False
        self.cur_rate = None
        self.pending_sets = {}
        self.paused = False
        self.window_s = 10
        self.t_base = None
        self.last_t = None
        self.rec = None
        self.rec_writer = None
        self.rec_rows = 0
        self.presets = PR.load_presets()
        self._updating = False

        self.times = collections.deque(maxlen=MAX_SAMPLES)
        self.data = {k: collections.deque(maxlen=MAX_SAMPLES) for k in PLOT_KEYS}
        self.hist = collections.deque(maxlen=MAX_SAMPLES)

        self.pw = {}                       # parameter name -> widget (Speed tab and Advanced tab)
        self.start_btns, self.stop_btns = [], []
        self.mode_pref = P.MODE_OPEN
        self.step_active = False
        self.step_value = None
        self.step_next = None
        self.step_seen_run = False
        self.step_deadline = 0.0
        self.step_phase = 0
        self.step_mark = None              # (t_s of the last step, from, to) in plot time
        self._build_ui()
        self.btn_start, self.btn_stop = self.start_btns[0], self.stop_btns[0]

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.setInterval(20)
        self.poll_timer.timeout.connect(self.poll_link)
        self.poll_timer.start()

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.setInterval(40)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start()

        self.heartbeat = QtCore.QTimer(self)
        self.heartbeat.setInterval(200)
        self.heartbeat.timeout.connect(self.send_ping)

        self.flush_timer = QtCore.QTimer(self)
        self.flush_timer.setSingleShot(True)
        self.flush_timer.setInterval(150)
        self.flush_timer.timeout.connect(self.flush_sets)

        self.tq_timer = QtCore.QTimer(self)
        self.tq_timer.setSingleShot(True)
        self.tq_timer.setInterval(60)
        self.tq_timer.timeout.connect(self.send_tq)
        self.pos_timer = QtCore.QTimer(self)
        self.pos_timer.setSingleShot(True)
        self.pos_timer.setInterval(60)
        self.pos_timer.timeout.connect(self.send_pos)
        self.step_timer = QtCore.QTimer(self)
        self.step_timer.setInterval(40)
        self.step_timer.timeout.connect(self.step_tick)

        QtGui.QShortcut(QtGui.QKeySequence("Esc"), self, activated=self.on_stop)
        self.refresh_ports()
        self.update_enabled()
        self.log_line("Ready. Choose the COM port of the ESC and press Connect.  Esc = STOP.")

    # ================================================================== UI construction
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.addLayout(self._build_topbar())

        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        split.addWidget(self._build_left())
        split.addWidget(self._build_right())
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([430, 960])
        # log + console below the main area; drag the splitter handle to make it bigger
        vsplit = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        vsplit.addWidget(split)
        cons = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(cons)
        cl.setContentsMargins(0, 0, 0, 0)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        f = QtGui.QFont("Consolas")
        f.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        self.log.setFont(f)
        cl.addWidget(self.log, 1)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Command:"))
        self.cmd_box = QtWidgets.QComboBox()          # editable: keeps a history of the commands sent (arrow keys / drop-down)
        self.cmd_box.setEditable(True)
        self.cmd_box.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
        self.cmd_box.setMaxCount(50)
        self.cmd_box.setToolTip("Any firmware command (see 'help'), e.g. status, faultlog, diag, raw, test, enc 200, focdump. "
                                "Replies appear above. 'stop' always stops the motor.")
        self.cmd_box.lineEdit().returnPressed.connect(lambda: self.on_console_send())
        row.addWidget(self.cmd_box, 1)
        b = QtWidgets.QPushButton("Send")
        b.clicked.connect(lambda _=False: self.on_console_send())
        row.addWidget(b)
        b = QtWidgets.QPushButton("Clear log")
        b.clicked.connect(lambda _=False: self.log.clear())
        row.addWidget(b)
        cl.addLayout(row)
        vsplit.addWidget(cons)
        vsplit.setStretchFactor(0, 1)
        vsplit.setStretchFactor(1, 0)
        vsplit.setSizes([700, 170])
        root.addWidget(vsplit, 1)

        self.status_label = QtWidgets.QLabel("")
        self.statusBar().addWidget(self.status_label, 1)

    def _build_topbar(self):
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(260)
        bar.addWidget(self.port_combo)
        b = QtWidgets.QPushButton("Refresh")
        b.clicked.connect(lambda _=False: self.refresh_ports())
        bar.addWidget(b)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(lambda _=False: self.on_connect_clicked())
        bar.addWidget(self.btn_connect)
        bar.addSpacing(20)
        self.state_label = QtWidgets.QLabel("Disconnected")
        f = self.state_label.font()
        f.setPointSize(14)
        f.setBold(True)
        self.state_label.setFont(f)
        bar.addWidget(self.state_label)
        bar.addStretch(1)
        self.btn_clear = QtWidgets.QPushButton("Clear fault")
        self.btn_clear.clicked.connect(lambda _=False: self.link.send("clear"))
        bar.addWidget(self.btn_clear)
        self.btn_stop_top = QtWidgets.QPushButton("STOP  (Esc)")
        self.btn_stop_top.setMinimumHeight(40)
        self.btn_stop_top.setMinimumWidth(160)
        self.btn_stop_top.setStyleSheet(STOP_STYLE)
        self.btn_stop_top.clicked.connect(lambda _=False: self.on_stop())
        bar.addWidget(self.btn_stop_top)
        return bar

    # ------------------------------------------------------------------ parameter widgets
    def _make_param_widget(self, spec):
        name, label, kind, lo, hi, step, dec, unit, tip = spec
        if kind == "bool":
            w = QtWidgets.QCheckBox(label)
            w.toggled.connect(lambda b, n=name: self.on_param_widget(n, 1.0 if b else 0.0))
        else:
            if kind == "int":
                w = QtWidgets.QSpinBox()
                w.setRange(int(lo), int(hi))
                w.setSingleStep(int(step))
            else:
                w = QtWidgets.QDoubleSpinBox()
                w.setDecimals(dec)
                w.setRange(lo, hi)
                w.setSingleStep(step)
            if unit:
                w.setSuffix(" " + unit)
            w.setKeyboardTracking(False)
            w.valueChanged.connect(lambda v, n=name: self.on_param_widget(n, float(v)))
        w.setToolTip(tip)
        self.pw[name] = w
        return w

    def _form_row(self, form, name):
        spec = P.PARAM_SPEC_BY_NAME[name]
        w = self._make_param_widget(spec)
        if spec[2] == "bool":
            form.addRow(w)
        else:
            form.addRow(spec[1] + ":", w)
        return w

    # ------------------------------------------------------------------ left panel
    def _build_left(self):
        self.tabs = QtWidgets.QTabWidget()
        self.tab_speed = self._tab_speed()
        self.tab_torque = self._tab_torque()
        self.tab_pos = self._tab_position()
        self.tabs.addTab(self.tab_speed, "Speed")
        self.tabs.addTab(self.tab_torque, "Torque")
        self.tabs.addTab(self.tab_pos, "Position")
        self.tabs.addTab(self._scrolled(self._tab_tuning()), "Tuning")
        self.tabs.addTab(self._scrolled(self._tab_advanced()), "Motor && Advanced")
        self.tabs.setMinimumWidth(400)
        return self.tabs

    @staticmethod
    def _scrolled(widget):
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        sa.setWidget(widget)
        sa.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        return sa

    def _start_stop_row(self):
        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("START")
        b.setMinimumHeight(44)
        b.setStyleSheet(START_STYLE)
        b.clicked.connect(lambda _=False: self.on_start())
        row.addWidget(b)
        self.start_btns.append(b)
        b = QtWidgets.QPushButton("STOP")
        b.setMinimumHeight(44)
        b.setStyleSheet(STOP_STYLE)
        b.clicked.connect(lambda _=False: self.on_stop())
        row.addWidget(b)
        self.stop_btns.append(b)
        return row

    def _tab_speed(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)

        g = QtWidgets.QGroupBox("Motor preset")
        gl = QtWidgets.QVBoxLayout(g)
        self.preset_combo = QtWidgets.QComboBox()
        self.preset_combo.currentIndexChanged.connect(lambda _=0: self.on_preset_changed())
        gl.addWidget(self.preset_combo)
        self.preset_info = QtWidgets.QLabel("")
        self.preset_info.setWordWrap(True)
        gl.addWidget(self.preset_info)
        self.preset_active = QtWidgets.QLabel("Current preset on the board: ---")
        f = self.preset_active.font()
        f.setBold(True)
        self.preset_active.setFont(f)
        gl.insertWidget(0, self.preset_active)
        row = QtWidgets.QHBoxLayout()
        self.btn_preset_apply = QtWidgets.QPushButton("Use this motor")
        self.btn_preset_apply.setToolTip("Sends the preset's values to the board, makes it the current preset and writes it to flash (motor must be stopped).")
        self.btn_preset_apply.clicked.connect(lambda _=False: self.on_preset_apply())
        row.addWidget(self.btn_preset_apply)
        self.btn_preset_new = QtWidgets.QPushButton("New...")
        self.btn_preset_new.clicked.connect(lambda _=False: self.on_preset_new())
        row.addWidget(self.btn_preset_new)
        self.btn_preset_edit = QtWidgets.QPushButton("Edit...")
        self.btn_preset_edit.clicked.connect(lambda _=False: self.on_preset_edit())
        row.addWidget(self.btn_preset_edit)
        self.btn_preset_del = QtWidgets.QPushButton("Delete")
        self.btn_preset_del.clicked.connect(lambda _=False: self.on_preset_delete())
        row.addWidget(self.btn_preset_del)
        gl.addLayout(row)
        row = QtWidgets.QHBoxLayout()
        self.btn_preset_save = QtWidgets.QPushButton("Save board settings as new preset...")
        self.btn_preset_save.clicked.connect(lambda _=False: self.on_preset_save())
        row.addWidget(self.btn_preset_save)
        gl.addLayout(row)
        lay.addWidget(g)
        self.fill_presets()

        g = QtWidgets.QGroupBox("Speed")
        form = QtWidgets.QFormLayout(g)
        crow = QtWidgets.QHBoxLayout()
        self.rb_ol = QtWidgets.QRadioButton("Open loop")
        self.rb_ol.setToolTip("Stage 1 drive: sinusoidal or 6-step, speed set by voltage/Kv, no feedback.")
        self.rb_foc = QtWidgets.QRadioButton("Closed loop (FOC)")
        self.rb_foc.setToolTip("Field-oriented control with a speed PI loop. Needs the motor calibration (Tuning tab).")
        self.rb_ol.setChecked(True)
        self.ctl_group = QtWidgets.QButtonGroup(self)     # own group: radio buttons with the same parent are otherwise one group
        self.ctl_group.addButton(self.rb_ol)
        self.ctl_group.addButton(self.rb_foc)
        crow.addWidget(self.rb_ol)
        crow.addWidget(self.rb_foc)
        form.addRow("Control:", crow)
        rpm_row = QtWidgets.QHBoxLayout()
        self.rpm_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.rpm_slider.setRange(0, 1500)
        self.rpm_slider.setToolTip("The slider ends at the 'Speed limit' (rpmmax, Motor & Advanced tab), which is stored per preset. "
                                   "Raise it there; FOC trips overspeed at 1.5 x rpmmax + 50 rpm.")
        rpm_row.addWidget(self.rpm_slider, 1)
        rpm_spin = self._make_param_widget(P.PARAM_SPEC_BY_NAME["rpm"])
        rpm_spin.setMinimumWidth(110)
        rpm_row.addWidget(rpm_spin)
        form.addRow("Target speed:", rpm_row)
        self.rpm_slider.valueChanged.connect(lambda v: self._slider_to_spin(v))
        rpm_spin.valueChanged.connect(lambda v: self._spin_to_slider(v))

        drow = QtWidgets.QHBoxLayout()
        self.dir_fwd = QtWidgets.QRadioButton("Forward")
        self.dir_rev = QtWidgets.QRadioButton("Reverse")
        self.dir_fwd.setChecked(True)
        self.dir_group = QtWidgets.QButtonGroup(self)
        self.dir_group.addButton(self.dir_fwd)
        self.dir_group.addButton(self.dir_rev)
        self.dir_fwd.toggled.connect(lambda b: self.on_dir_toggled(b))
        drow.addWidget(self.dir_fwd)
        drow.addWidget(self.dir_rev)
        form.addRow("Direction:", drow)
        self._form_row(form, "accel")
        self._form_row(form, "idrive")
        self._form_row(form, "auto")
        self.derived_label = QtWidgets.QLabel("")
        self.derived_label.setWordWrap(True)
        form.addRow(self.derived_label)
        lay.addWidget(g)

        lay.addLayout(self._start_stop_row())

        g = QtWidgets.QGroupBox("Board memory")
        gl = QtWidgets.QVBoxLayout(g)
        self.btn_write = QtWidgets.QPushButton("Write settings to flash")
        self.btn_write.setToolTip("Stores all parameters in the board's flash so they survive a reboot. Only possible while the motor is stopped.")
        self.btn_write.clicked.connect(lambda _=False: self.on_write())
        gl.addWidget(self.btn_write)
        row = QtWidgets.QHBoxLayout()
        self.btn_reload = QtWidgets.QPushButton("Reload from board")
        self.btn_reload.clicked.connect(lambda _=False: self.link.send("get"))
        row.addWidget(self.btn_reload)
        self.btn_loadflash = QtWidgets.QPushButton("Load saved (flash)")
        self.btn_loadflash.clicked.connect(lambda _=False: self.link.send("load"))
        row.addWidget(self.btn_loadflash)
        self.btn_factory = QtWidgets.QPushButton("Factory defaults")
        self.btn_factory.clicked.connect(lambda _=False: self.link.send("factory"))
        row.addWidget(self.btn_factory)
        gl.addLayout(row)
        lay.addWidget(g)
        lay.addStretch(1)
        return w

    def _tab_torque(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        note = QtWidgets.QLabel("Torque control (FOC): the q-axis current is regulated, positive torque turns the shaft towards rising "
                                "encoder angle. The motor needs the calibration (Tuning tab) first. An unloaded motor accelerates "
                                "until it reaches the speed limit (Motor && Advanced), where the torque is faded out.")
        note.setWordWrap(True)
        lay.addWidget(note)
        g = QtWidgets.QGroupBox("Torque setpoint")
        form = QtWidgets.QFormLayout(g)
        row = QtWidgets.QHBoxLayout()
        self.tq_nm = QtWidgets.QRadioButton("Nm")
        self.tq_a = QtWidgets.QRadioButton("A (q-axis current)")
        self.tq_nm.setChecked(True)
        row.addWidget(self.tq_nm)
        row.addWidget(self.tq_a)
        form.addRow("Unit:", row)
        self.tq_spin = QtWidgets.QDoubleSpinBox()
        self.tq_spin.setDecimals(3)
        self.tq_spin.setRange(-1000, 1000)
        self.tq_spin.setSingleStep(0.01)
        self.tq_spin.setKeyboardTracking(False)
        form.addRow("Setpoint:", self.tq_spin)
        self.tq_conv = QtWidgets.QLabel("")
        form.addRow(self.tq_conv)
        lay.addWidget(g)
        self.btn_tq0 = QtWidgets.QPushButton("Torque = 0")
        self.btn_tq0.clicked.connect(lambda _=False: self.tq_spin.setValue(0.0))
        lay.addWidget(self.btn_tq0)
        lay.addLayout(self._start_stop_row())
        self.tq_live = QtWidgets.QLabel("Iq measured: ---")
        lay.addWidget(self.tq_live)
        lay.addStretch(1)
        self._tq_unit_is_nm = True
        self.tq_nm.toggled.connect(lambda b: self.on_torque_unit(b))
        self.tq_spin.valueChanged.connect(lambda v: self.on_torque_changed())
        self.update_torque_label()
        return w

    def _tab_position(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        note = QtWidgets.QLabel("Position control (FOC): the shaft moves to the target by the shortest way with the speed and "
                                "acceleration limits of the Tuning tab and holds it. It starts by holding the position it has.")
        note.setWordWrap(True)
        lay.addWidget(note)
        self.pos_live = QtWidgets.QLabel("Shaft angle: ---")
        f = self.pos_live.font()
        f.setPointSize(18)
        f.setBold(True)
        self.pos_live.setFont(f)
        lay.addWidget(self.pos_live)
        self.pos_sp_live = QtWidgets.QLabel("Profile set point: ---")
        lay.addWidget(self.pos_sp_live)
        g = QtWidgets.QGroupBox("Target position (0-360 deg)")
        form = QtWidgets.QFormLayout(g)
        self.pos_spin = QtWidgets.QDoubleSpinBox()
        self.pos_spin.setRange(0, 360)
        self.pos_spin.setDecimals(1)
        self.pos_spin.setSuffix(" deg")
        self.pos_spin.setKeyboardTracking(False)
        form.addRow("Target:", self.pos_spin)
        self.pos_dial = QtWidgets.QDial()
        self.pos_dial.setRange(0, 359)
        self.pos_dial.setWrapping(True)
        self.pos_dial.setNotchesVisible(True)
        form.addRow(self.pos_dial)
        lay.addWidget(g)
        self.pos_dial.valueChanged.connect(lambda v: self._dial_to_spin(v))
        self.pos_spin.valueChanged.connect(lambda v: self._spin_to_dial(v))
        self.pos_spin.valueChanged.connect(lambda v: self.on_pos_changed())
        self.btn_pos_here = QtWidgets.QPushButton("Hold current position")
        self.btn_pos_here.clicked.connect(lambda _=False: self.on_pos_here())
        lay.addWidget(self.btn_pos_here)
        lay.addLayout(self._start_stop_row())
        lay.addStretch(1)
        return w

    def _tab_tuning(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)

        g = QtWidgets.QGroupBox("Motor calibration")
        gl = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Measures phase resistance, inductance, the encoder direction and the number of pole pairs. "
                             "Do it once per motor and wiring. The shaft turns a little: run it unloaded and free to rotate.")
        t.setWordWrap(True)
        gl.addWidget(t)
        self.btn_calib = QtWidgets.QPushButton("Calibrate motor (R, L, encoder)")
        self.btn_calib.clicked.connect(lambda _=False: self.on_calibrate())
        gl.addWidget(self.btn_calib)
        self.enc_label = QtWidgets.QLabel("Encoder direction: ---")
        gl.addWidget(self.enc_label)
        self.cal_label = QtWidgets.QLabel("")
        self.cal_label.setWordWrap(True)
        gl.addWidget(self.cal_label)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Current loop (d and q)")
        form = QtWidgets.QFormLayout(g)
        for n in ("curauto", "curbw", "curkp", "curki"):
            self._form_row(form, n)
        h = QtWidgets.QLabel("Automatic: Kp = 2 pi * bandwidth * L, Ki = 2 pi * bandwidth * R, from the calibrated values.")
        h.setWordWrap(True)
        form.addRow(h)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Speed / position gain auto-tune")
        gl = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Accelerates the motor with half the FOC current limit (up to half the speed limit, at most 1500 rpm), "
                             "brakes it, and computes inertia and friction. Sets speed Kp / Ki and position Kp for the chosen "
                             "speed-loop bandwidth. Run it with the load that will be used, free to rotate. Needs the calibration.")
        t.setWordWrap(True)
        gl.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        self.tune_bw = QtWidgets.QDoubleSpinBox()
        self.tune_bw.setRange(1.0, 20.0)
        self.tune_bw.setDecimals(1)
        self.tune_bw.setValue(5.0)
        self.tune_bw.setSuffix(" Hz")
        self.tune_bw.setToolTip("Speed-loop bandwidth. Higher = stiffer but noisier. Position bandwidth is a quarter of it.")
        row.addWidget(QtWidgets.QLabel("Bandwidth:"))
        row.addWidget(self.tune_bw)
        self.btn_tune = QtWidgets.QPushButton("Auto-tune speed / position gains")
        self.btn_tune.clicked.connect(lambda _=False: self.on_tune())
        row.addWidget(self.btn_tune, 1)
        gl.addLayout(row)
        self.tune_label = QtWidgets.QLabel("")
        self.tune_label.setWordWrap(True)
        gl.addWidget(self.tune_label)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Speed loop (PI)")
        form = QtWidgets.QFormLayout(g)
        for n in ("spdkp", "spdki"):
            self._form_row(form, n)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Position loop (PID + motion profile)")
        form = QtWidgets.QFormLayout(g)
        for n in ("poskp", "poski", "poskd", "posvmax", "posacc"):
            self._form_row(form, n)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Currents")
        form = QtWidgets.QFormLayout(g)
        for n in ("imax", "ialign", "ical"):
            self._form_row(form, n)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Scope (FOC loops, 2 kHz)")
        gl = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Records 1000 samples of speed, speed set point, Iq, Iq reference, Id, Vq and position inside the "
                             "firmware at 2 kHz / divider, starting with the next FOC run (immediately if FOC is running). "
                             "Works with every FOC mode, the step helper and the auto-tune.")
        t.setWordWrap(True)
        gl.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Divider:"))
        self.scope_div = QtWidgets.QSpinBox()
        self.scope_div.setRange(1, 50)
        self.scope_div.setValue(1)
        self.scope_div.valueChanged.connect(lambda v: self.scope_len.setText("= %.2f s" % (v * 0.5)))
        row.addWidget(self.scope_div)
        self.scope_len = QtWidgets.QLabel("= 0.50 s")
        row.addWidget(self.scope_len)
        self.btn_scope = QtWidgets.QPushButton("Arm capture")
        self.btn_scope.clicked.connect(lambda _=False: self.on_scope_arm())
        row.addWidget(self.btn_scope)
        b = QtWidgets.QPushButton("Read now")
        b.setToolTip("Ends the capture and reads what has been recorded so far.")
        b.clicked.connect(lambda _=False: self.link.send("scope dump") if self.connected() else None)
        row.addWidget(b)
        b = QtWidgets.QPushButton("Save CSV...")
        b.clicked.connect(lambda _=False: self.on_scope_save())
        row.addWidget(b)
        gl.addLayout(row)
        self.scope_label = QtWidgets.QLabel("No capture yet.")
        gl.addWidget(self.scope_label)
        self.scope_plots = []
        self.scope_curves = {}
        for title, unit, curves in (("Speed", "rpm", (("rpm", 0, "c"), ("set point", 1, "w"))),
                                    ("Current", "A", (("Iq", 2, "g"), ("Iq ref", 3, "y"), ("Id", 4, "r")))):
            pl = pg.PlotWidget()
            pl.setMinimumHeight(160)
            pl.showGrid(x=True, y=True, alpha=0.3)
            pl.setLabel("left", title, units=unit)
            pl.getAxis("left").enableAutoSIPrefix(False)
            pl.addLegend(offset=(5, 5))
            for name, col, color in curves:
                self.scope_curves[col] = pl.plot(pen=pg.mkPen(color, width=1.5), name=name)
            if self.scope_plots:
                pl.setXLink(self.scope_plots[0])
            self.scope_plots.append(pl)
            gl.addWidget(pl)
        self.scope_data = None
        self.scope_timer = QtCore.QTimer(self)
        self.scope_timer.setInterval(300)
        self.scope_timer.timeout.connect(lambda: self.link.send("scope state") if self.connected() else self.scope_timer.stop())
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Step-response helper")
        gl = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Alternates the set point between two values and plots reference and measurement, with rise time, "
                             "overshoot and settling time of the last step. Change the gains above while it runs.")
        t.setWordWrap(True)
        gl.addWidget(t)
        form = QtWidgets.QFormLayout()
        self.step_kind = QtWidgets.QComboBox()
        for k in STEP_KINDS:
            self.step_kind.addItem(k[0])
        form.addRow("Loop:", self.step_kind)
        self.step_lo = QtWidgets.QDoubleSpinBox()
        self.step_hi = QtWidgets.QDoubleSpinBox()
        self.step_half = QtWidgets.QDoubleSpinBox()
        self.step_half.setRange(0.1, 30.0)
        self.step_half.setDecimals(2)
        self.step_half.setSuffix(" s")
        form.addRow("Low value:", self.step_lo)
        form.addRow("High value:", self.step_hi)
        form.addRow("Time at each value:", self.step_half)
        gl.addLayout(form)
        row = QtWidgets.QHBoxLayout()
        self.btn_step = QtWidgets.QPushButton("Start step test")
        self.btn_step.clicked.connect(lambda _=False: self.on_step_start())
        row.addWidget(self.btn_step)
        self.btn_step_stop = QtWidgets.QPushButton("Stop test")
        self.btn_step_stop.clicked.connect(lambda _=False: self.on_step_stop(True))
        row.addWidget(self.btn_step_stop)
        gl.addLayout(row)
        self.step_result = QtWidgets.QLabel("No step measured yet.")
        self.step_result.setWordWrap(True)
        gl.addWidget(self.step_result)
        self.step_plot = pg.PlotWidget()
        self.step_plot.setMinimumHeight(220)
        self.step_plot.showGrid(x=True, y=True, alpha=0.3)
        self.step_plot.addLegend(offset=(5, 5))
        self.step_curves = {
            "target": self.step_plot.plot(pen=pg.mkPen("w", width=1.5), name="target"),
            "ref": self.step_plot.plot(pen=pg.mkPen("y", width=1.5), name="reference (after limits)"),
            "meas": self.step_plot.plot(pen=pg.mkPen("c", width=1.5), name="measured"),
        }
        gl.addWidget(self.step_plot)
        lay.addWidget(g)
        lay.addStretch(1)
        self.step_kind.currentIndexChanged.connect(lambda _=0: self.on_step_kind())
        self.on_step_kind()
        return w

    def _tab_advanced(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        g = QtWidgets.QGroupBox("Motor constants and limits")
        form = QtWidgets.QFormLayout(g)
        for n in ("pp", "kv", "rl", "ll", "kt", "rpmmax", "ilim"):
            self._form_row(form, n)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox("Drive mode")
        form = QtWidgets.QFormLayout(g)
        self._form_row(form, "six")
        for n in ("alignms", "boost", "maxmod", "alignv"):
            self._form_row(form, n)
        hint = QtWidgets.QLabel("Boost, modulation limit and align voltage are only used when 'Automatic parameters' is off.")
        hint.setWordWrap(True)
        form.addRow(hint)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox("Overtemperature shutdown")
        form = QtWidgets.QFormLayout(g)
        for en, val, title in (("otdrven", "otdrv", "Driver NTC"), ("otmoten", "otmot", "Motor NTC")):
            row = QtWidgets.QHBoxLayout()
            cb = self._make_param_widget(P.PARAM_SPEC_BY_NAME[en])
            cb.setText("Trip above")
            sp = self._make_param_widget(P.PARAM_SPEC_BY_NAME[val])
            cb.toggled.connect(lambda b, spin=sp: spin.setEnabled(b))
            row.addWidget(cb)
            row.addWidget(sp)
            form.addRow(title + ":", row)
        h = QtWidgets.QLabel("Tick 'Trip above' to enable a limit. With a limit enabled, a missing sensor (shown as -999) also stops "
                             "the motor, so leave the motor NTC off when none is fitted.")
        h.setWordWrap(True)
        form.addRow(h)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox("Current sensors")
        gl = QtWidgets.QVBoxLayout(g)
        row = QtWidgets.QHBoxLayout()
        self.btn_zero = QtWidgets.QPushButton("Zero (motor stopped, no current)")
        self.btn_zero.setToolTip("Measures the sensor offsets with no current flowing and stores them in flash.")
        self.btn_zero.clicked.connect(lambda _=False: self.on_zero())
        row.addWidget(self.btn_zero)
        self.btn_raw = QtWidgets.QPushButton("Raw ADC check")
        self.btn_raw.setToolTip("Reports mean / min / max of the three current-sense ADC channels over 0.2 s (result in the log below).")
        self.btn_raw.clicked.connect(lambda _=False: self.on_raw())
        row.addWidget(self.btn_raw)
        gl.addLayout(row)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox("Fault log (kept in flash)")
        gl = QtWidgets.QVBoxLayout(g)
        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Refresh")
        b.clicked.connect(lambda _=False: self.link.send("faultlog") if self.connected() else None)
        row.addWidget(b)
        self.btn_flog_clear = QtWidgets.QPushButton("Clear log")
        self.btn_flog_clear.clicked.connect(lambda _=False: self.on_faultlog_clear())
        row.addWidget(self.btn_flog_clear)
        row.addStretch(1)
        gl.addLayout(row)
        self.flog_view = QtWidgets.QPlainTextEdit()
        self.flog_view.setReadOnly(True)
        self.flog_view.setMinimumHeight(160)
        self.flog_view.setPlainText("Press Refresh.")
        gl.addWidget(self.flog_view)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox("Diagnostics")
        row = QtWidgets.QHBoxLayout(g)
        for title, cmd, tip in (("Status", "status", "State, last stop reason, last MCU reset, current trigger, interrupt time."),
                                ("Fault log", "faultlog", "Last 8 faults / watchdog stops / abnormal resets, kept in flash."),
                                ("Self test", "test", "Clock, ADCs, op-amps, encoder, NTCs.")):
            b = QtWidgets.QPushButton(title)
            b.setToolTip(tip + " Result in the console below.")
            b.clicked.connect(lambda _=False, c=cmd: self.link.send(c) if self.connected() else None)
            row.addWidget(b)
        lay.addWidget(g)
        lay.addStretch(1)
        return w

    # ------------------------------------------------------------------ right panel
    def _build_right(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)

        grid = QtWidgets.QGridLayout()
        self.ro = {}
        items = [("state", "State"), ("fault", "Fault"), ("vbus", "Bus voltage"), ("vll", "Applied V (L-L)"),
                 ("imag", "Phase current |I|"), ("id", "Id (flux)"), ("iq", "Iq (torque)"),
                 ("ia", "Phase A"), ("ib", "Phase B"), ("ic", "Phase C"),
                 ("rpm_cmd", "Speed cmd"), ("rpm_spi", "Speed (SPI)"), ("angle", "Angle"),
                 ("tdrv", "Driver temp"), ("tmot", "Motor temp")]
        for i, (key, title) in enumerate(items):
            r, c = divmod(i, 4)
            box = QtWidgets.QVBoxLayout()
            t = QtWidgets.QLabel(title)
            t.setStyleSheet("color: gray;")
            v = QtWidgets.QLabel("---")
            f = v.font()
            f.setPointSize(13)
            f.setBold(True)
            v.setFont(f)
            box.addWidget(t)
            box.addWidget(v)
            grid.addLayout(box, r, c)
            self.ro[key] = v
        lay.addLayout(grid)

        self.plots = {}
        self.curves = {}
        specs = [
            ("speed", "Speed", "rpm", [("rpm_cmd", "command", "w"), ("rpm_spi", "measured (SPI)", "c"), ("rpm_abi", "measured (FOC / ABI)", "y")]),
            ("current", "Phase currents", "A", [("ia", "A", "r"), ("ib", "B", "g"), ("ic", "C", "b")]),
            ("dq", "Current magnitude / dq", "A", [("imag", "|I| phase", "w"), ("id", "Id", "r"), ("iq", "Iq", "g"), ("iqref", "Iq reference", "y")]),
            ("vbus", "Bus voltage", "V", [("vbus", "Vbus", "y")]),
            ("vll", "Applied line-line voltage", "V", [("vll", "V L-L", "m")]),
            ("angle", "Shaft angle", "deg", [("angle", "angle", "g"), ("possp", "position set point", "y")]),
        ]
        first = None
        for pid, title, unit, curves in specs:
            pl = pg.PlotWidget()
            pl.setMinimumHeight(110)
            pl.showGrid(x=True, y=True, alpha=0.3)
            pl.setLabel("left", title, units=unit)
            pl.getAxis("left").enableAutoSIPrefix(False)          # always A / V / rpm, never mA
            if pid == "vbus":                                     # fixed scale: 0 V to just above the 62 V limit
                pl.setYRange(0, 65, padding=0)
                pl.enableAutoRange(axis="y", enable=False)
            if len(curves) > 1:
                pl.addLegend(offset=(5, 5))
            for key, name, color in curves:
                self.curves[key] = pl.plot(pen=pg.mkPen(color, width=1.5), name=name)
            if first is None:
                first = pl
            else:
                pl.setXLink(first)
            self.plots[pid] = pl
            lay.addWidget(pl, 1)

        ctl = QtWidgets.QHBoxLayout()
        ctl.addWidget(QtWidgets.QLabel("Window:"))
        self.win_combo = QtWidgets.QComboBox()
        for s in (5, 10, 30, 60, 120):
            self.win_combo.addItem("%d s" % s, s)
        self.win_combo.setCurrentIndex(1)
        self.win_combo.currentIndexChanged.connect(lambda _=0: self.on_window_changed())
        ctl.addWidget(self.win_combo)
        self.chk_pause = QtWidgets.QCheckBox("Pause plots")
        self.chk_pause.toggled.connect(lambda b: self.on_pause(b))
        ctl.addWidget(self.chk_pause)
        ctl.addWidget(QtWidgets.QLabel("Rate running:"))
        self.rate_run = QtWidgets.QSpinBox()
        self.rate_run.setRange(1, 200)
        self.rate_run.setValue(100)
        self.rate_run.setSuffix(" Hz")
        self.rate_run.valueChanged.connect(lambda _=0: self.apply_rate())
        ctl.addWidget(self.rate_run)
        ctl.addWidget(QtWidgets.QLabel("idle:"))
        self.rate_idle = QtWidgets.QSpinBox()
        self.rate_idle.setRange(1, 200)
        self.rate_idle.setValue(5)
        self.rate_idle.setSuffix(" Hz")
        self.rate_idle.valueChanged.connect(lambda _=0: self.apply_rate())
        ctl.addWidget(self.rate_idle)
        ctl.addStretch(1)
        self.btn_rec = QtWidgets.QPushButton("Record CSV...")
        self.btn_rec.clicked.connect(lambda _=False: self.on_record())
        ctl.addWidget(self.btn_rec)
        b = QtWidgets.QPushButton("Export buffer...")
        b.clicked.connect(lambda _=False: self.on_export())
        ctl.addWidget(b)
        b = QtWidgets.QPushButton("Clear")
        b.clicked.connect(lambda _=False: self.clear_buffers())
        ctl.addWidget(b)
        lay.addLayout(ctl)
        return w

    # ================================================================== helpers
    def log_line(self, text):
        self.log.appendPlainText("%s  %s" % (time.strftime("%H:%M:%S"), text))

    def on_console_send(self):
        cmd = self.cmd_box.currentText().strip()
        if not cmd:
            return
        if not self.connected():
            self.log_line("not connected")
            return
        if len(cmd) > 62:                              # the firmware line buffer holds 63 characters
            self.log_line("command too long (max 62 characters)")
            return
        self.log_line("> " + cmd)
        self.link.send(cmd)
        i = self.cmd_box.findText(cmd)
        if i >= 0:
            self.cmd_box.removeItem(i)
        self.cmd_box.insertItem(0, cmd)
        self.cmd_box.setCurrentIndex(-1)
        self.cmd_box.clearEditText()

    def set_status(self, text):
        self.status_label.setText(text)

    def connected(self):
        return self.link.is_open()

    def running(self):
        return self.state in P.RUNNING_STATES

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(list_ports.comports()) if list_ports else []
        ports.sort(key=lambda p: (0 if p.vid == 0x0483 else 1, p.device))
        for p in ports:
            tag = " (STM32)" if p.vid == 0x0483 else ""
            self.port_combo.addItem("%s  %s%s" % (p.device, p.description or "", tag), p.device)
        if not ports:
            self.port_combo.addItem("no serial ports found", None)

    # ------------------------------------------------------------------ connection
    def on_connect_clicked(self):
        if self.connected():
            self.disconnect_port("Disconnected.")
            return
        port = self.port_combo.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self, APP_TITLE, "No serial port selected.")
            return
        try:
            self.link.open(port)
            self.last_telem_mono = None
            self.telem_lost = False
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, APP_TITLE, "Could not open %s:\n%s" % (port, e))
            return
        self.clear_buffers()
        self.params = {}
        self.state, self.fault, self.cur_rate = P.ST_IDLE, 0, None
        self.dirty = False
        self.btn_connect.setText("Disconnect")
        for c in ("stream 0", "ver", "wd 3000", "get"):
            self.link.send(c)
        self.apply_rate()
        self.heartbeat.start()
        self.log_line("Connected to %s" % port)
        self.update_enabled()

    def disconnect_port(self, message):
        if self.connected():
            self.link.send("stop")
            self.link.send("stream 0")
            time.sleep(0.05)
        self.heartbeat.stop()
        self.link.close()
        self.btn_connect.setText("Connect")
        self.state_label.setText("Disconnected")
        self.log_line(message)
        self.update_enabled()

    TELEMETRY_TIMEOUT_S = 1.0

    def send_ping(self):
        """Every 200 ms. While the motor runs and no telemetry came for 1 s, the link is not healthy: send 'stop' and
        withhold the pings, so the board's host watchdog stops the motor even if the 'stop' is lost."""
        if not self.connected():
            return
        last = getattr(self, "last_telem_mono", None)
        stale = last is not None and (time.monotonic() - last) > self.TELEMETRY_TIMEOUT_S
        if stale and self.running():
            if not getattr(self, "telem_lost", False):
                self.telem_lost = True
                self.link.send("stop")
                self.log_line("No telemetry from the board for %.1f s while running: sent STOP and stopped the keep-alive pings."
                              % self.TELEMETRY_TIMEOUT_S)
                self.set_status("Connection problem: no data from the board, motor stop requested.")
            return
        self.link.send("ping")

    def apply_rate(self):
        if not self.connected():
            return
        want = self.rate_run.value() if self.running() else self.rate_idle.value()
        if want != self.cur_rate:
            self.link.send("stream %d" % want)
            self.cur_rate = want

    # ------------------------------------------------------------------ parameter handling
    def on_param_widget(self, name, value):
        if self._updating:
            return
        self.params[name] = value
        if name == "rpmmax":
            self.set_rpm_range(value)               # at once, not only after the next 'get'
        if name in ("imax", "kt"):
            self.update_torque_range()
            self.update_torque_label()
        self.pending_sets[name] = value
        self.flush_timer.start()
        self.refresh_derived()
        self.update_preset_label()

    def flush_sets(self):
        for name, value in list(self.pending_sets.items()):
            self.link.send("set %s %s" % (name, fmt_num(value)))
        self.pending_sets.clear()

    def set_widget(self, w, value):
        w.blockSignals(True)
        if isinstance(w, QtWidgets.QCheckBox):
            w.setChecked(bool(value))
        else:
            w.setValue(value)
        w.blockSignals(False)

    def _slider_to_spin(self, v):
        spin = self.pw["rpm"]
        if abs(spin.value() - v) >= 1.0:
            spin.setValue(float(v))                       # fires valueChanged -> on_param_widget

    def _spin_to_slider(self, v):
        self.rpm_slider.blockSignals(True)
        self.rpm_slider.setValue(int(round(v)))
        self.rpm_slider.blockSignals(False)

    def on_dir_toggled(self, forward):
        if self._updating:
            return
        self.on_param_widget("dir", 1.0 if forward else -1.0)

    def set_rpm_range(self, rpmmax):
        """Target-speed slider and spin box end at the speed limit (rpmmax); the spin box cannot exceed the 'rpm' range."""
        top = min(float(rpmmax), float(P.PARAM_SPEC_BY_NAME["rpm"][4]))
        self.rpm_slider.blockSignals(True)
        self.rpm_slider.setMaximum(int(top))
        self.rpm_slider.blockSignals(False)
        self.pw["rpm"].blockSignals(True)
        self.pw["rpm"].setMaximum(top)
        self.pw["rpm"].blockSignals(False)

    def apply_params_to_widgets(self):
        self._updating = True
        try:
            self.set_rpm_range(self.params.get("rpmmax", 1500.0))
            for name, w in self.pw.items():
                if name in self.params:
                    self.set_widget(w, self.params[name])
            if "rpm" in self.params:
                self._spin_to_slider(self.params["rpm"])
            if "dir" in self.params:
                self.dir_fwd.setChecked(self.params["dir"] >= 0)
                self.dir_rev.setChecked(self.params["dir"] < 0)
        finally:
            self._updating = False
        self.refresh_derived()
        self.update_torque_range()
        self.update_torque_label()
        for en, val in (("otdrven", "otdrv"), ("otmoten", "otmot")):
            self.pw[val].setEnabled(bool(self.params.get(en, 0)))
        ed = self.params.get("encdir")
        if ed is None:
            self.enc_label.setText("Encoder direction: ---")
        elif ed == 0:
            self.enc_label.setText("Encoder direction: NOT calibrated, FOC modes are locked until you run the calibration.")
            self.enc_label.setStyleSheet("color: #e65100;")
        else:
            self.enc_label.setText("Encoder direction: %s (calibrated)" % ("+1" if ed > 0 else "-1"))
            self.enc_label.setStyleSheet("")
        nm = self.params.get("_name")
        if nm and nm in self.presets and self.preset_combo.currentText() != nm:
            self.select_preset(nm)
        self.update_preset_label()

    def refresh_derived(self):
        p = self.params
        if not all(k in p for k in ("auto", "idrive", "rl", "kv", "rpm", "rpmmax", "boost", "alignv", "maxmod")):
            self.derived_label.setText("")
            return
        vbus = self.sample["vbus"] if self.sample else 16.0
        d = P.derive(p, vbus)
        txt = ("%s: boost %.2f V, align %.2f V, modulation cap %.2f. Speed possible at %.1f V bus: %.0f rpm."
               % ("Automatic" if p["auto"] else "Manual", d["boost"], d["align"], d["maxmod"], vbus, d["rpm_limit"]))
        if p["rpm"] > d["rpm_limit"] + 0.5:
            txt += "  Target is above this and will be limited."
            self.derived_label.setStyleSheet("color: #e65100;")
        else:
            self.derived_label.setStyleSheet("")
        self.derived_label.setText(txt)

    # ------------------------------------------------------------------ presets
    def fill_presets(self):
        keep = self.preset_combo.currentText()
        self.preset_combo.blockSignals(True)
        self.preset_combo.clear()
        for name in sorted(self.presets):
            self.preset_combo.addItem(name)
        i = self.preset_combo.findText(keep)
        self.preset_combo.setCurrentIndex(i if i >= 0 else 0)
        self.preset_combo.blockSignals(False)
        self.on_preset_changed()

    def current_preset(self):
        return self.presets.get(self.preset_combo.currentText())

    def on_preset_changed(self):
        pr = self.current_preset()
        if not pr:
            self.preset_info.setText("")
            self.update_preset_label()
            return
        self.preset_info.setText("%d pole pairs, %g rpm/V, %g mOhm L-L, %g Nm/A. %s" % (
            pr.get("pole_pairs", 0), pr.get("kv", 0), pr.get("r_ll_mohm", 0), pr.get("kt_nm_a", 0), pr.get("notes", "")))
        self.btn_preset_del.setEnabled(bool(pr.get("_user")))
        self.update_preset_label()

    def update_preset_label(self):
        name = self.params.get("_name") if self.params else None
        if not self.params:
            self.preset_active.setText("Current preset on the board: ---")
            return
        txt = "Current preset on the board: %s" % (name or "(none)")
        pr = self.presets.get(name) if name else None
        if pr is not None and not PR.values_match(pr, self.params):
            txt += "  (modified)"
        elif pr is None and name:
            txt += "  (not in your preset list)"
        self.preset_active.setText(txt)

    def select_preset(self, name):
        i = self.preset_combo.findText(name)
        if i >= 0:
            self.preset_combo.setCurrentIndex(i)

    def ask_preset_values(self, initial=None):
        return preset_dialog.run_preset_dialog(self, initial)

    def on_preset_apply(self):
        """'Use this motor': values -> board, name -> board, and written to flash so the board keeps it."""
        pr = self.current_preset()
        if not pr or not self.connected():
            return
        if self.running():
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Stop the motor first.")
            return
        cmds, skipped = PR.preset_commands(pr)
        if skipped:
            QtWidgets.QMessageBox.warning(self, APP_TITLE, "Not applied, values outside the firmware limits: " + ", ".join(skipped))
            return
        for c in cmds:
            self.link.send(c)
        if "enc_dir" not in pr and self.params and not PR.values_match(pr, self.params):
            self.link.send("set encdir 0")              # another motor that was never calibrated: measure it first
            self.log_line("Motor changed: run 'Calibrate motor' (Tuning tab) before using FOC.")
        self.link.send("setname " + pr["name"])
        self.link.send("save")
        self.link.send("get")
        self.log_line("Preset '%s' sent to the board and written to flash (%d values)" % (pr["name"], len(cmds)))
        self.set_status("'%s' is now the current preset of the board." % pr["name"])

    def _store_preset(self, d, replace=None):
        PR.save_user_preset(d)
        if replace and replace.get("_user") and replace["name"] != d["name"]:
            PR.delete_user_preset(replace)
        self.presets = PR.load_presets()
        self.fill_presets()
        self.select_preset(d["name"])

    def on_preset_new(self):
        d = self.ask_preset_values(None)
        if d:
            self._store_preset(d)
            self.log_line("Created preset '%s'" % d["name"])

    def on_preset_edit(self):
        pr = self.current_preset()
        if not pr:
            return
        d = self.ask_preset_values(pr)
        if d:
            self._store_preset(d, replace=pr)
            self.log_line("Saved preset '%s'" % d["name"])

    def on_preset_save(self):
        if not self.params:
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Connect to the board first, the preset is taken from its current settings.")
            return
        name, ok = QtWidgets.QInputDialog.getText(self, APP_TITLE, "Name of the new motor preset:")
        name = name.strip()
        if not ok or not name:
            return
        err = PR.validate_name(name)
        if err:
            QtWidgets.QMessageBox.warning(self, APP_TITLE, err)
            return
        self._store_preset(PR.preset_from_params(name, self.params, "User preset"))
        if self.connected() and not self.running():
            self.link.send("setname " + name)
            self.link.send("get")
            self.dirty = True
        self.log_line("Saved preset '%s'" % name)

    def on_preset_delete(self):
        pr = self.current_preset()
        if pr and pr.get("_user"):
            if QtWidgets.QMessageBox.question(self, APP_TITLE, "Delete preset '%s'?" % pr["name"]) == QtWidgets.QMessageBox.StandardButton.Yes:
                PR.delete_user_preset(pr)
                self.presets = PR.load_presets()
                self.fill_presets()

    def on_zero(self):
        if not self.connected():
            return
        if self.running():
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Stop the motor first.")
            return
        self.link.send("zero")
        self.set_status("Measuring current-sensor offsets (no current may flow)...")

    def on_raw(self):
        if self.connected():
            self.link.send("raw")

    # ------------------------------------------------------------------ actions
    def desired_mode(self):
        """Control mode that belongs to the tab in front (Tuning / Advanced keep the last one)."""
        cur = self.tabs.currentWidget()
        if cur is self.tab_speed:
            self.mode_pref = P.MODE_SPEED if self.rb_foc.isChecked() else P.MODE_OPEN
        elif cur is self.tab_torque:
            self.mode_pref = P.MODE_TORQUE
        elif cur is self.tab_pos:
            self.mode_pref = P.MODE_POSITION
        return self.mode_pref

    def on_start(self, mode=None):
        if not self.connected():
            return
        if mode is None:
            mode = self.desired_mode()
        if mode != P.MODE_OPEN and not self.params.get("encdir"):
            QtWidgets.QMessageBox.information(self, APP_TITLE, ERR_TEXT["no_enc_cal"])
            self.step_active = False
            return
        self.flush_timer.stop()
        self.flush_sets()
        self.start_mode = mode
        self.link.send("mode %d" % mode)
        if mode == P.MODE_TORQUE:
            self.tq_spin.blockSignals(True)
            self.tq_spin.setValue(0.0)                  # always start without torque
            self.tq_spin.blockSignals(False)
            self.update_torque_label()
        self.link.send("start")

    def on_stop(self):
        self.on_step_stop(False)
        if self.connected():
            self.link.send("stop")
            self.tq_spin.blockSignals(True)
            self.tq_spin.setValue(0.0)                  # the firmware clears the torque at stop
            self.tq_spin.blockSignals(False)
            self.update_torque_label()

    def on_write(self):
        if not self.connected():
            return
        if self.running():
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Stop the motor before writing to flash.")
            return
        self.flush_timer.stop()
        self.flush_sets()
        self.link.send("save")

    # ------------------------------------------------------------------ torque / position / calibration
    def torque_limit_a(self):
        return float(self.params.get("imax", 2.0))

    def update_torque_range(self):
        kt = self.params.get("kt", 0.083)
        lim = self.torque_limit_a()
        top = lim * kt if self._tq_unit_is_nm else lim
        self.tq_spin.blockSignals(True)
        self.tq_spin.setRange(-top, top)
        self.tq_spin.blockSignals(False)

    def torque_a(self):
        """Set point in ampere (q axis)."""
        kt = max(self.params.get("kt", 0.083), 1e-6)
        v = self.tq_spin.value()
        a = v / kt if self._tq_unit_is_nm else v
        lim = self.torque_limit_a()
        return max(-lim, min(lim, a))

    def on_torque_unit(self, nm_checked):
        if nm_checked == self._tq_unit_is_nm:
            return
        kt = self.params.get("kt", 0.083)
        v = self.tq_spin.value()
        self._tq_unit_is_nm = nm_checked
        self.update_torque_range()
        self.tq_spin.blockSignals(True)
        self.tq_spin.setValue(v * kt if nm_checked else v / kt)       # A -> Nm or Nm -> A
        self.tq_spin.blockSignals(False)
        self.update_torque_label()

    def update_torque_label(self):
        kt = self.params.get("kt", 0.083)
        v = self.tq_spin.value()
        if self._tq_unit_is_nm:
            self.tq_conv.setText("= %.2f A q-axis current (Kt = %.4f Nm/A, limit %.1f A)" % (v / max(kt, 1e-6), kt, self.torque_limit_a()))
        else:
            self.tq_conv.setText("= %.4f Nm (Kt = %.4f Nm/A, limit %.1f A)" % (v * kt, kt, self.torque_limit_a()))

    def on_torque_changed(self):
        self.update_torque_label()
        if not self._updating:
            self.tq_timer.start()

    def send_tq(self):
        if self.connected() and self.running() and self.sample and self.sample.get("mode") == P.MODE_TORQUE:
            self.link.send("tq %.4f" % self.torque_a())

    def _dial_to_spin(self, v):
        self.pos_spin.blockSignals(True)
        self.pos_spin.setValue(float(v))
        self.pos_spin.blockSignals(False)
        self.on_pos_changed()

    def _spin_to_dial(self, v):
        self.pos_dial.blockSignals(True)
        self.pos_dial.setValue(int(round(v)) % 360)
        self.pos_dial.blockSignals(False)

    def on_pos_changed(self):
        if not self._updating:
            self.pos_timer.start()

    def send_pos(self):
        if self.connected() and self.running() and self.sample and self.sample.get("mode") == P.MODE_POSITION:
            self.link.send("pos %.2f" % self.pos_spin.value())

    def on_pos_here(self):
        if self.sample and self.sample.get("angle") is not None:
            self.pos_spin.setValue(float(self.sample["angle"]))

    def on_calibrate(self):
        if not self.connected():
            return
        if self.running():
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Stop the motor first.")
            return
        pp = max(1.0, float(self.params.get("pp", 7)))
        ical = float(self.params.get("ical", 2.0))
        q = QtWidgets.QMessageBox.question(
            self, APP_TITLE,
            "The motor will be driven with %.1f A. It first locks to a position, then turns about %.0f degrees "
            "(two electrical turns) in each direction of the test. Keep it unloaded and free to rotate.\n\nStart the calibration?" % (ical, 720.0 / pp))
        if q != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.flush_timer.stop()
        self.flush_sets()
        self.link.send("calibrate")
        self.cal_label.setText("Calibrating... (takes about 8 seconds)")
        self.set_status("Calibrating the motor...")

    def on_calibration(self, r):
        if r["ok"]:
            txt = ("Resistance %.1f mOhm line-line, inductance %.1f uH line-line, encoder direction %+d, %.2f pole pairs measured. "
                   % (r["r_ll_mohm"], r["l_ll_uh"], r["enc_dir"], r["pp"]))
            self.link.send("get")
            saved = self.store_calibration_in_preset(r)
            self.link.send("save")                       # the motor is stopped again, keep the values over a reboot
            txt += " Written to flash" + (" and saved in the preset '%s'." % saved if saved else ".")
            self.cal_label.setText(txt)
            self.log_line("Calibration finished: " + txt)
            self.set_status("Calibration finished and stored.")
        else:
            why = CAL_ERR_TEXT.get(r["reason"], r["reason"])
            self.cal_label.setText("Calibration failed: " + why)
            self.log_line("Calibration failed (%s)" % r["reason"])
            self.set_status("Calibration failed.")
            QtWidgets.QMessageBox.warning(self, APP_TITLE, "Calibration failed:\n" + why)
            self.link.send("get")
        self.update_enabled()

    def store_calibration_in_preset(self, r):
        """Put the measured values into the board's current preset (as your own preset, bundled ones stay untouched)."""
        name = self.params.get("_name") or self.preset_combo.currentText()
        base = self.presets.get(name)
        if not base:
            self.log_line("No preset selected: the calibration is on the board only. Use 'Save board settings as new preset...' to keep it.")
            return None
        d = {k: v for k, v in base.items() if not k.startswith("_")}
        d.update({"pole_pairs": int(round(r["pp"])), "r_ll_mohm": round(r["r_ll_mohm"], 3),
                  "l_ll_uh": round(r["l_ll_uh"], 3), "enc_dir": int(r["enc_dir"])})
        self._store_preset(d, replace=base if base.get("_user") else None)
        return name

    # ------------------------------------------------------------------ gain auto-tune
    def on_tune(self):
        if not self.connected():
            return
        if self.running():
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Stop the motor first.")
            return
        if int(self.params.get("encdir", 0)) == 0:
            QtWidgets.QMessageBox.information(self, APP_TITLE, "Run 'Calibrate motor' first.")
            return
        imax = float(self.params.get("imax", 0.0))
        rpm = min(1500.0, 0.5 * float(self.params.get("rpmmax", 1500.0)))
        q = QtWidgets.QMessageBox.question(
            self, APP_TITLE,
            "The motor will be accelerated with %.1f A to about %.0f rpm and braked again. Keep it free to rotate "
            "and keep clear of the shaft.\n\nStart the auto-tune?" % (0.5 * imax, rpm))
        if q != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.flush_timer.stop()
        self.flush_sets()
        self.link.send("tune %.1f" % self.tune_bw.value())
        self.tune_label.setText("Auto-tune running...")
        self.set_status("Auto-tune running...")

    def on_tune_result(self, r):
        if r["ok"]:
            txt = ("Inertia %.1f g cm^2, friction %.2f mNm. At %.1f Hz: speed Kp %.5f A/rpm, Ki %.5f A/(rpm s), position Kp %.4f rpm/deg."
                   % (r["j_gcm2"], r["tf_mnm"], r["bw"], r["spdkp"], r["spdki"], r["poskp"]))
            self.link.send("get")
            saved = self.store_tune_in_preset(r)
            self.link.send("save")                       # the motor is stopped again
            txt += " Written to flash" + (" and saved in the preset '%s'." % saved if saved else ".")
            self.tune_label.setText(txt)
            self.log_line("Auto-tune finished: " + txt)
            self.set_status("Auto-tune finished and stored.")
        else:
            why = TUNE_ERR_TEXT.get(r["reason"], r["reason"])
            self.tune_label.setText("Auto-tune failed: " + why)
            self.log_line("Auto-tune failed (%s)" % r["reason"])
            self.set_status("Auto-tune failed.")
            QtWidgets.QMessageBox.warning(self, APP_TITLE, "Auto-tune failed:\n" + why)
            self.link.send("get")
        self.update_enabled()

    def store_tune_in_preset(self, r):
        """Put the tuned gains into the board's current preset (as your own preset, bundled ones stay untouched)."""
        name = self.params.get("_name") or self.preset_combo.currentText()
        base = self.presets.get(name)
        if not base:
            self.log_line("No preset selected: the gains are on the board only.")
            return None
        d = {k: v for k, v in base.items() if not k.startswith("_")}
        d.update({"spd_kp": r["spdkp"], "spd_ki": r["spdki"], "pos_kp": r["poskp"]})
        self._store_preset(d, replace=base if base.get("_user") else None)
        return name

    # ------------------------------------------------------------------ fault log
    def on_faultlog(self, d):
        flash = {"ok": "written to flash", "none": "nothing written since power-on", "pending": "waiting to be written (motor running)",
                 "fail": "LAST FLASH WRITE FAILED"}.get(d["flash"], d["flash"])
        lines = ["%d entr%s, this is boot #%d, %s." % (d["count"], "y" if d["count"] == 1 else "ies", d["boot"], flash),
                 "Newest last. Same boot number = same power-on; time = since that power-on.", ""]
        for e in d["entries"]:
            try:
                t = float(e["uptime_ms"]) / 1000.0
                mode = P.MODE_NAMES[int(e["mode"])] if hasattr(P, "MODE_NAMES") and int(e["mode"]) < len(P.MODE_NAMES) else e["mode"]
                lines.append("boot %s, t = %.1f s: %s (%s) in %s, mode %s" % (e["boot"], t, e["what"], e["kind"], e["state"], mode))
                if e["kind"] != "reset":
                    lines.append("    Vbus %s V, Ia/Ib/Ic %s / %s / %s A, |I| %s A, speed %s rpm, Iq ref %s A, last OC sample %s A"
                                 % (e["vbus"], e["ia"], e["ib"], e["ic"], e["imag"], e["rpm"], e["iq_ref"], e["oc_a"]))
            except (KeyError, ValueError):
                lines.append(str(e))
        self.flog_view.setPlainText("\n".join(lines))

    def on_faultlog_clear(self):
        if not self.connected() or self.running():
            return
        q = QtWidgets.QMessageBox.question(self, APP_TITLE, "Erase the fault log on the board?")
        if q == QtWidgets.QMessageBox.StandardButton.Yes:
            self.link.send("faultlog clear")
            self.link.send("faultlog")

    # ------------------------------------------------------------------ scope
    def on_scope_arm(self):
        if not self.connected():
            return
        self.link.send("scope %d" % self.scope_div.value())
        self.scope_label.setText("Armed: waiting for FOC to run...")
        self.scope_timer.start()

    def on_scope_state(self, state, n):
        if state == "capturing":
            self.scope_label.setText("Capturing... %d / 1000" % n)
        elif state == "armed":
            self.scope_label.setText("Armed: waiting for FOC to run...")
        elif state == "done":
            self.scope_timer.stop()
            self.scope_label.setText("Reading %d samples..." % n)
            self.link.send("scope dump")
        else:
            self.scope_timer.stop()

    def on_scope_data(self, d):
        self.scope_timer.stop()
        self.scope_data = d
        rows = d["rows"]
        t = np.arange(len(rows)) * d["dt"]
        a = np.array(rows) if rows else np.zeros((0, 7))
        for col, c in self.scope_curves.items():
            c.setData(t, a[:, col] if len(rows) else [])
        self.scope_label.setText("%d samples, %.2f ms each (%.2f s)." % (len(rows), d["dt"] * 1e3, len(rows) * d["dt"]))

    def on_scope_save(self):
        if not self.scope_data or not self.scope_data["rows"]:
            QtWidgets.QMessageBox.information(self, APP_TITLE, "No capture to save yet.")
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save scope capture", "scope.csv", "CSV (*.csv)")
        if not fn:
            return
        with open(fn, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t_s", "rpm", "rpm_sp", "iq_a", "iq_ref_a", "id_a", "vq_v", "pos_deg"])
            for i, r in enumerate(self.scope_data["rows"]):
                wr.writerow(["%.5f" % (i * self.scope_data["dt"])] + r)
        self.log_line("Scope capture saved to %s" % fn)

    # ------------------------------------------------------------------ step-response helper
    def on_step_kind(self):
        k = STEP_KINDS[self.step_kind.currentIndex()]
        for w, v in ((self.step_lo, k[3]), (self.step_hi, k[4])):
            w.blockSignals(True)
            w.setRange(*k[6])
            w.setDecimals(k[7])
            w.setSuffix(" " + k[2])
            w.setValue(v)
            w.blockSignals(False)
        self.step_half.setValue(k[5])
        self.step_plot.setLabel("left", k[0], units=k[2])
        self.step_plot.getAxis("left").enableAutoSIPrefix(False)

    def _step_send(self, value):
        mode = STEP_KINDS[self.step_kind.currentIndex()][1]
        self.step_value = value
        if mode == P.MODE_TORQUE:
            self.link.send("tq %.4f" % value)
        elif mode == P.MODE_SPEED:
            self.link.send("set rpm %.3f" % value)
        else:
            self.link.send("pos %.2f" % value)

    def on_step_start(self):
        if not self.connected() or self.step_active:
            return
        k = STEP_KINDS[self.step_kind.currentIndex()]
        if not self.params.get("encdir"):
            QtWidgets.QMessageBox.information(self, APP_TITLE, ERR_TEXT["no_enc_cal"])
            return
        if self.running() and (not self.sample or self.sample.get("mode") != k[1]):
            QtWidgets.QMessageBox.information(self, APP_TITLE, "The motor runs in another mode. Stop it first.")
            return
        if k[1] == P.MODE_TORQUE:
            if QtWidgets.QMessageBox.question(self, APP_TITLE, "An unloaded motor accelerates during a torque step test. "
                                              "The speed limit stops it, but make sure nothing can be hit. Continue?") != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        self.step_active = True
        self.step_next = None
        self.step_phase = 0
        self.step_seen_run = False
        self.step_mark = None
        self.step_deadline = time.monotonic() + 4.0
        self.step_result.setText("Starting...")
        if not self.running():
            self.on_start(mode=k[1])
            if not self.step_active:
                return
        self.step_timer.start()
        self.update_enabled()

    def on_step_stop(self, stop_motor):
        was = self.step_active
        self.step_active = False
        self.step_timer.stop()
        if was and stop_motor and self.connected():
            self.link.send("stop")
        self.update_enabled()

    def step_tick(self):
        if not self.step_active:
            self.step_timer.stop()
            return
        now = time.monotonic()
        if self.state == P.ST_FAULT:
            self.on_step_stop(False)
            self.step_result.setText("Stopped by a fault.")
            return
        if self.state != P.ST_RUN:
            if self.step_seen_run or now > self.step_deadline:
                self.on_step_stop(False)
                self.step_result.setText("The motor is not running, test ended.")
            return
        k = STEP_KINDS[self.step_kind.currentIndex()]
        lo, hi, half = self.step_lo.value(), self.step_hi.value(), self.step_half.value()
        tnow = self.times[-1] if self.times else 0.0
        if self.step_next is None:
            self.step_seen_run = True
            self.step_phase = 0
            self._step_send(lo)
            self.step_mark = None
            self.step_next = now + half
            return
        if now >= self.step_next:
            self.evaluate_step(k)
            self.step_phase ^= 1
            frm, to = (lo, hi) if self.step_phase else (hi, lo)
            self._step_send(hi if self.step_phase else lo)
            self.step_mark = (tnow, frm, to)
            self.step_next = now + half

    def evaluate_step(self, k):
        """Rise time, overshoot and settling of the step that ended now."""
        if self.step_mark is None or len(self.times) < 20:
            return
        t0 = self.step_mark[0]
        t = list(self.times)
        sp = [x for x in self.data["step_target"]]
        meas = [x for x in self.data[k[9]]]
        i0 = 0
        while i0 < len(t) and t[i0] < t0 - 0.3:
            i0 += 1
        r = stepmetrics.analyze(t[i0:], sp[i0:], meas[i0:], t0)
        if r is None:
            return
        def ms(x):
            return "n/a" if x is None else "%.0f ms" % (x * 1000.0)
        self.step_result.setText("Last step %.3g -> %.3g %s:  rise (10-90 %%) %s,  overshoot %.1f %%,  settling (5 %%) %s,  final error %.3g %s" % (
            self.step_mark[1], self.step_mark[2], k[2], ms(r["rise_s"]), r["overshoot_pct"], ms(r["settle_s"]), r["final_err"], k[2]))

    # ================================================================== link events
    def poll_link(self):
        for _ in range(400):
            try:
                ev = self.link.events.get_nowait()
            except queue.Empty:
                break
            kind = ev[0]
            if kind == "telemetry":
                self.on_telemetry(ev[1])
            elif kind == "params":
                self.params = ev[1]
                self.apply_params_to_widgets()
            elif kind == "ack":
                self.on_ack(ev[1], ev[2], ev[3])
            elif kind == "version":
                self.log_line("Firmware %s (parameter format %d, %d parameters)" % (
                    ev[1]["fw"], ev[1]["params_version"], ev[1]["count"]))
                if ev[1]["params_version"] != P.EXPECTED_PARAMS_VERSION:
                    QtWidgets.QMessageBox.warning(self, APP_TITLE, "The board firmware uses a different parameter format "
                                                  "than this program. Update one of them.")
            elif kind == "calibration":
                self.on_calibration(ev[1])
            elif kind == "tune":
                self.on_tune_result(ev[1])
            elif kind == "faultlog":
                self.on_faultlog(ev[1])
            elif kind == "scope_state":
                self.on_scope_state(ev[1], ev[2])
            elif kind == "scope":
                self.on_scope_data(ev[1])
            elif kind == "log":
                self.log_line("board: " + ev[1])
            elif kind == "disconnected":
                if self.connected():
                    self.disconnect_port("Connection lost: %s" % ev[1])
                    QtWidgets.QMessageBox.warning(self, APP_TITLE, "Connection lost: %s" % ev[1])

    def on_ack(self, cmd, ok, msg):
        word = cmd.split(" ", 1)[0]
        if word in ("stream", "wd", "get", "params"):
            if ok is False:
                self.log_line("%s -> %s" % (cmd, msg))
            return
        reason = msg.split(",", 1)[1] if (msg and msg.startswith("ERR,")) else msg
        if ok is None:
            self.log_line("%s -> no reply from the board" % cmd)
            return
        if word == "set":
            if ok:
                self.dirty = True
            else:
                self.log_line("%s -> rejected (%s)" % (cmd, reason))
                self.set_status("Rejected: %s  %s" % (cmd, ERR_TEXT.get(reason, reason)))
                self.link.send("get")                       # put the widgets back to what the board really has
        elif word == "save":
            if ok:
                self.dirty = False
                self.set_status("Settings written to flash.")
                self.log_line("Settings written to flash")
            else:
                QtWidgets.QMessageBox.warning(self, APP_TITLE, "Write failed: " + ERR_TEXT.get(reason, reason))
        elif word == "load":
            if ok:
                self.dirty = False
                self.link.send("get")
                self.set_status("Loaded the settings saved in flash.")
            else:
                self.set_status("Load failed: " + ERR_TEXT.get(reason, reason))
        elif word == "factory":
            if ok:
                self.dirty = True
                self.link.send("get")
                self.set_status("Factory defaults loaded into RAM, write them to flash to keep them.")
        elif word == "zero":
            if ok:
                self.set_status("Current sensors zeroed and stored in flash.")
            else:
                self.set_status("Zero failed: " + ERR_TEXT.get(reason, reason))
        elif word == "setname":
            if not ok:
                self.log_line("%s -> %s" % (cmd, msg))
        elif word == "start":
            if not ok:
                self.set_status("Start refused: " + ERR_TEXT.get(reason, reason))
                self.log_line("start refused: %s" % reason)
                self.on_step_stop(False)
            elif getattr(self, "start_mode", 0) == P.MODE_POSITION and self.sample and self.sample.get("angle") is not None:
                self.pos_spin.blockSignals(True)
                self.pos_spin.setValue(float(self.sample["angle"]))      # the firmware holds the present position at start
                self.pos_spin.blockSignals(False)
                self._spin_to_dial(self.pos_spin.value())
        elif word in ("mode", "tq", "pos"):
            if not ok:
                self.log_line("%s -> %s" % (cmd, ERR_TEXT.get(reason, reason)))
        elif word == "tune":
            if not ok:
                self.tune_label.setText("Auto-tune refused: " + ERR_TEXT.get(reason, reason))
                self.set_status("Auto-tune refused: " + ERR_TEXT.get(reason, reason))
        elif word == "scope":
            if not ok:
                self.scope_timer.stop()
                self.scope_label.setText("Scope: " + reason)
        elif word == "calibrate":
            if not ok:
                self.cal_label.setText("Calibration refused: " + ERR_TEXT.get(reason, reason))
                self.set_status("Calibration refused: " + ERR_TEXT.get(reason, reason))
        elif word in ("stop", "clear"):
            if not ok:
                self.log_line("%s -> %s" % (cmd, msg))
        self.update_enabled()

    def on_telemetry(self, s):
        self.last_telem_mono = time.monotonic()
        if getattr(self, "telem_lost", False):
            self.telem_lost = False
            self.log_line("Telemetry is back.")
        if self.last_t is not None and s["t"] < self.last_t - 0.5:       # board rebooted: its clock restarted
            self.clear_buffers()
        self.last_t = s["t"]
        if self.t_base is None:
            self.t_base = s["t"]
        t = s["t"] - self.t_base
        self.sample = s
        self.times.append(t)
        for k in PLOT_KEYS:
            v = s.get(k)
            self.data[k].append(float("nan") if v is None else float(v))
        self.data["step_target"][-1] = float(self.step_value) if (self.step_active and self.step_value is not None) else float("nan")
        row = dict(s)
        row["t"] = t
        self.hist.append(row)
        if self.rec_writer is not None:
            self.rec_writer.writerow(P.telemetry_csv_row(row))
            self.rec_rows += 1
        if s["state"] != self.state or s["fault"] != self.fault:
            was_running = self.running()
            self.state, self.fault = s["state"], s["fault"]
            self.update_enabled()
            if self.running() != was_running:
                self.apply_rate()
            if self.state == P.ST_FAULT:
                self.log_line("FAULT: %s" % P.FAULT_NAMES[self.fault] if self.fault < len(P.FAULT_NAMES) else "FAULT")

    # ================================================================== state of the controls
    def update_enabled(self):
        con = self.connected()
        run = self.running()
        self.tabs.setEnabled(con)
        self.btn_stop_top.setEnabled(con)
        for b in self.stop_btns:
            b.setEnabled(con)
        for b in self.start_btns:
            b.setEnabled(con and not run and self.state != P.ST_FAULT)
        self.btn_calib.setEnabled(con and not run and self.state != P.ST_FAULT)
        self.btn_tune.setEnabled(con and not run and self.state != P.ST_FAULT)
        self.btn_step.setEnabled(con and not self.step_active and self.state != P.ST_FAULT)
        self.btn_step_stop.setEnabled(self.step_active)
        for w in (self.rb_ol, self.rb_foc):
            w.setEnabled(not run)
        self.btn_clear.setEnabled(con and self.state == P.ST_FAULT)
        for b in (self.btn_write, self.btn_loadflash, self.btn_factory, self.btn_preset_apply, self.btn_zero):
            b.setEnabled(con and not run)
        self.btn_reload.setEnabled(con)
        self.btn_raw.setEnabled(con)
        for w in (self.dir_fwd, self.dir_rev, self.pw["pp"], self.pw["six"], self.step_kind):
            w.setEnabled(not run)
        self.btn_write.setText("Write settings to flash" + ("   *" if self.dirty else ""))
        if con:
            name = P.STATE_NAMES[self.state] if self.state < len(P.STATE_NAMES) else str(self.state)
            if self.state == P.ST_FAULT and self.fault < len(P.FAULT_NAMES):
                name += ": " + P.FAULT_NAMES[self.fault]
            self.state_label.setText(name)
            color = {P.ST_IDLE: "#2e7d32", P.ST_FAULT: "#c62828"}.get(self.state, "#1565c0")
            self.state_label.setStyleSheet("color: %s;" % color)
        else:
            self.state_label.setStyleSheet("")

    # ================================================================== plots and readouts
    def on_window_changed(self):
        self.window_s = self.win_combo.currentData()
        self.refresh_plots()

    def on_pause(self, paused):
        self.paused = paused

    def clear_buffers(self):
        self.times.clear()
        for d in self.data.values():
            d.clear()
        self.hist.clear()
        self.t_base = None
        self.last_t = None

    def refresh_ui(self):
        s = self.sample
        if s is not None and self.connected():
            self.ro["state"].setText(P.STATE_NAMES[s["state"]] if s["state"] < len(P.STATE_NAMES) else "?")
            self.ro["fault"].setText(P.FAULT_NAMES[s["fault"]] if s["fault"] < len(P.FAULT_NAMES) else "?")
            self.ro["vbus"].setText("%.2f V" % s["vbus"])
            self.ro["vll"].setText("%.2f V" % s["vll"])
            for k in ("ia", "ib", "ic"):
                self.ro[k].setText("%.2f A" % s[k])
            self.ro["imag"].setText("%.3f A" % s["imag"])
            self.ro["id"].setText("%.3f A" % s["id"])
            self.ro["iq"].setText("%.3f A" % s["iq"])
            self.tq_live.setText("Iq measured: %.3f A  (|I| %.3f A)" % (s["iq"], s["imag"]))
            self.pos_sp_live.setText("Profile set point: ---" if s["possp"] is None else "Profile set point: %.1f deg" % s["possp"])
            self.ro["rpm_cmd"].setText("%.1f rpm" % s["rpm_cmd"])
            self.ro["rpm_spi"].setText("%.1f rpm" % s["rpm_spi"])
            self.ro["angle"].setText("---" if s["angle"] is None else "%.1f deg" % s["angle"])
            self.ro["tdrv"].setText("---" if s["tdrv"] is None else "%.1f C" % s["tdrv"])
            self.ro["tmot"].setText("---" if s["tmot"] is None else "%.1f C" % s["tmot"])
            self.pos_live.setText("Shaft angle: ---" if s["angle"] is None else "Shaft angle: %.1f deg" % s["angle"])
            if self.params:
                self.refresh_derived()
        if self.rec_writer is not None:
            self.btn_rec.setText("Stop recording (%d rows)" % self.rec_rows)
        if not self.paused:
            self.refresh_plots()

    def refresh_plots(self):
        if not self.times:
            return
        t = np.fromiter(self.times, dtype=float, count=len(self.times))
        tmax = t[-1]
        i0 = int(np.searchsorted(t, tmax - self.window_s))
        xs = t[i0:] - tmax
        for key, curve in self.curves.items():
            y = np.fromiter(self.data[key], dtype=float, count=len(self.data[key]))[i0:]
            curve.setData(xs, y)
        first = next(iter(self.plots.values()))
        first.setXRange(-self.window_s, 0, padding=0)
        self.refresh_step_plot(t)

    def refresh_step_plot(self, t):
        k = STEP_KINDS[self.step_kind.currentIndex()]
        span = max(2.0, 3.0 * self.step_half.value())
        tmax = t[-1]
        i0 = int(np.searchsorted(t, tmax - span))
        xs = t[i0:] - tmax
        for name, key in (("target", "step_target"), ("ref", k[8]), ("meas", k[9])):
            y = np.fromiter(self.data[key], dtype=float, count=len(self.data[key]))[i0:]
            self.step_curves[name].setData(xs, y)

    # ================================================================== CSV
    def on_record(self):
        if self.rec_writer is None:
            default = time.strftime("esc_log_%Y%m%d_%H%M%S.csv")
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Record telemetry to CSV", default, "CSV files (*.csv)")
            if not path:
                return
            try:
                self.rec = open(path, "w", newline="")
            except OSError as e:
                QtWidgets.QMessageBox.critical(self, APP_TITLE, "Cannot write %s:\n%s" % (path, e))
                return
            self.rec_writer = csv.writer(self.rec)
            self.rec_writer.writerow(P.CSV_COLUMNS)
            self.rec_rows = 0
            self.log_line("Recording to %s" % path)
        else:
            self.stop_recording()

    def stop_recording(self):
        if self.rec is not None:
            try:
                self.rec.close()
            except OSError:
                pass
            self.log_line("Recording stopped (%d rows)" % self.rec_rows)
        self.rec, self.rec_writer = None, None
        self.btn_rec.setText("Record CSV...")

    def on_export(self):
        if not self.hist:
            QtWidgets.QMessageBox.information(self, APP_TITLE, "The buffer is empty.")
            return
        default = time.strftime("esc_buffer_%Y%m%d_%H%M%S.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export buffered data", default, "CSV files (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(P.CSV_COLUMNS)
                for row in list(self.hist):
                    w.writerow(P.telemetry_csv_row(row))
        except OSError as e:
            QtWidgets.QMessageBox.critical(self, APP_TITLE, "Cannot write %s:\n%s" % (path, e))
            return
        self.log_line("Exported %d rows to %s" % (len(self.hist), path))

    # ================================================================== shutdown
    def closeEvent(self, event):
        self.stop_recording()
        if self.connected():
            self.link.send("stop")
            self.link.send("stream 0")
            time.sleep(0.05)
            self.heartbeat.stop()
            self.link.close()
        event.accept()


class WheelGuard(QtCore.QObject):
    """Application-wide: the mouse wheel changes spin boxes, combo boxes, sliders and dials only after they were clicked
    (have the focus). Otherwise the wheel event goes on to the scroll area, so scrolling never changes a value by accident.
    Also covers widgets created later (preset dialog)."""
    TYPES = (QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox, QtWidgets.QAbstractSlider)

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == QtCore.QEvent.Type.Polish and isinstance(obj, self.TYPES) and not isinstance(obj, QtWidgets.QScrollBar):
            obj.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)          # the wheel alone must not give them the focus
        elif t == QtCore.QEvent.Type.Wheel and isinstance(obj, QtWidgets.QWidget):
            w = obj if isinstance(obj, self.TYPES) else obj.parentWidget()  # e.g. the line edit inside a spin box
            if isinstance(w, self.TYPES) and not isinstance(w, QtWidgets.QScrollBar) and not w.hasFocus():
                area = w.parentWidget()                     # scroll the page instead of changing the value
                while area is not None and not isinstance(area, QtWidgets.QAbstractScrollArea):
                    area = area.parentWidget()
                if area is not None:
                    QtWidgets.QApplication.sendEvent(area.verticalScrollBar(), ev)
                return True
        return False


def install_crash_log():
    """Python exceptions, hard crashes and hangs of the GUI thread (> 5 s) are written to %APPDATA%/ESC_Control/crash.log
    with the stack traces of all threads. Returns the open file (keep it open) and its path."""
    import faulthandler
    import os
    import traceback
    d = os.path.join(os.environ.get("APPDATA", os.path.expanduser("~")), "ESC_Control")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, "crash.log")
    f = open(path, "a", buffering=1)
    f.write("\n===== GUI started %s =====\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
    faulthandler.enable(f, all_threads=True)

    def hook(et, ev, tb):
        f.write("\n--- exception %s ---\n" % time.strftime("%H:%M:%S"))
        traceback.print_exception(et, ev, tb, file=f)
        sys.__excepthook__(et, ev, tb)
    sys.excepthook = hook
    return f, path


def main():
    log_file, log_path = install_crash_log()
    app = QtWidgets.QApplication(sys.argv)
    wheel_guard = WheelGuard(app)
    app.installEventFilter(wheel_guard)
    pg.setConfigOptions(antialias=False)
    win = MainWindow()
    win.resize(1450, 920)
    win.show()
    # hang detector: re-armed every second by the GUI thread; if that thread is blocked for 5 s, all stacks go to crash.log
    import faulthandler
    hang = QtCore.QTimer()
    hang.timeout.connect(lambda: faulthandler.dump_traceback_later(5, repeat=False, file=log_file))
    hang.start(1000)
    win.log_line("Crash / hang log: %s" % log_path)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
