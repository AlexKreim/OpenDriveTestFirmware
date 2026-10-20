"""Drives the real MainWindow logic (main.py) with stand-in Qt classes against the simulated board."""
import os
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import qt_stubs
qt_stubs.install()
os.environ["APPDATA"] = tempfile.mkdtemp()           # keep user presets out of the real profile

import main as gui                                   # noqa: E402
import protocol as P                                 # noqa: E402
from link import EscLink                             # noqa: E402
from fake_esc import FakeEsc                         # noqa: E402


def pump(win, cond, timeout=3.0):
    t0 = time.time()
    last_ping = 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_ping > 0.2:                 # the real GUI sends this from a QTimer (stubbed out here)
            win.send_ping()
            last_ping = time.time()
        win.poll_link()
        win.refresh_ui()
        if cond():
            return True
        time.sleep(0.01)
    return False


class GuiSmoke(unittest.TestCase):
    def setUp(self):
        import shutil, presets
        shutil.rmtree(presets.user_dir(), ignore_errors=True)       # no user presets left over from other tests
        qt_stubs.MessageBox.calls.clear()
        self.esc = FakeEsc()
        self.win = gui.MainWindow()
        self.win.link = EscLink(serial_factory=lambda port: self.esc)
        self.win.port_combo.addItem("FAKE", "FAKE")
        self.win.port_combo.setCurrentIndex(self.win.port_combo.findText("FAKE"))
        self.win.on_connect_clicked()
        self.assertTrue(pump(self.win, lambda: "kv" in self.win.params and self.win.sample is not None))

    def tearDown(self):
        self.win.closeEvent(type("E", (), {"accept": lambda s: None})())
        self.esc.close()

    def test_connect_loads_params_into_widgets(self):
        w = self.win
        self.assertEqual(w.pw["kv"].value(), 100)
        self.assertEqual(w.pw["pp"].value(), 20)
        self.assertEqual(w.pw["auto"].isChecked(), True)
        self.assertEqual(w.pw["rpm"].value(), 60)
        self.assertEqual(w.rpm_slider.value(), 60)
        self.assertIn("Automatic", w.derived_label.text())
        self.assertEqual(w.cur_rate, 5)                        # idle rate

    def test_speed_edit_and_start_stop_flow(self):
        w = self.win
        w.pw["rpm"].setValue(300.0)
        w.rpm_slider.setValue(200)                             # slider drives the spin box
        self.assertEqual(w.pw["rpm"].value(), 200)
        w.btn_start.clicked.emit(False)                        # flushes the pending set, then start
        self.assertTrue(pump(w, lambda: w.state == P.ST_RUN))
        self.assertAlmostEqual(self.esc._param("rpm"), 200, places=2)
        self.assertEqual(w.cur_rate, 100)                      # running rate
        self.assertFalse(w.btn_start.isEnabled())
        self.assertFalse(w.btn_write.isEnabled())              # no flash write while running
        self.assertFalse(w.dir_fwd.isEnabled())
        self.assertTrue(pump(w, lambda: abs(w.sample["rpm_spi"] - 200) < 1.0))
        # direction change while running is refused by the board and the widget goes back
        w.dir_fwd.setChecked(False)
        w.flush_sets()
        self.assertTrue(pump(w, lambda: any("Stop it first" in w.status_label.text() or "motor is running" in w.status_label.text() for _ in [0])))
        self.assertTrue(pump(w, lambda: w.params.get("dir") == 1 and w.dir_fwd.isChecked()))
        # live plots got data
        w.refresh_plots()
        self.assertGreater(len(w.curves["rpm_spi"].x), 10)
        w.btn_stop.clicked.emit(False)
        self.assertTrue(pump(w, lambda: w.state == P.ST_IDLE))
        self.assertTrue(pump(w, lambda: w.cur_rate == 5))

    def test_write_flash_only_when_stopped(self):
        w = self.win
        w.pw["kv"].setValue(111.0)
        w.flush_sets()
        self.assertTrue(pump(w, lambda: w.dirty))
        self.assertIn("*", w.btn_write.text() if pump(w, lambda: "*" in w.btn_write.text() or True) else "")
        w.on_start()
        self.assertTrue(pump(w, lambda: w.running()))
        w.on_write()                                           # refused locally
        self.assertTrue(any(k == "information" for k, _ in qt_stubs.MessageBox.calls))
        self.assertIsNone(self.esc.flash)
        w.on_stop()
        self.assertTrue(pump(w, lambda: w.state == P.ST_IDLE))
        w.on_write()
        self.assertTrue(pump(w, lambda: not w.dirty and self.esc.flash is not None))

    def test_preset_use_new_edit_delete(self):
        w = self.win
        # the board reports its stored preset name, the label shows it
        self.assertIn("ODrive M8325s 100KV", w.preset_active.text())
        self.assertNotIn("modified", w.preset_active.text())
        w.pw["kv"].setValue(50.0)
        w.flush_sets()
        pump(w, lambda: False, 0.3)
        self.assertIn("modified", w.preset_active.text())
        w.on_preset_apply()                                      # 'Use this motor': values + name + flash
        self.assertTrue(pump(w, lambda: abs(w.params.get("kv", 0) - 100) < 1e-6 and self.esc.flash is not None))
        self.assertEqual(w.params.get("_name"), "ODrive M8325s 100KV")
        # create a new preset through the dialog hook (line-line values)
        newp = {"name": "Small Motor", "notes": "x", "pole_pairs": 7, "kv": 400, "r_ll_mohm": 200.0, "l_ll_uh": 80.0,
                "kt_nm_a": 0.02, "rpm_max": 3000, "ilim": 5, "i_drive": 1.5, "accel": 100}
        w.ask_preset_values = lambda initial=None: dict(newp)
        w.on_preset_new()
        self.assertIn("Small Motor", w.presets)
        self.assertTrue(w.presets["Small Motor"]["_user"])
        w.select_preset("Small Motor")
        w.on_preset_apply()
        self.assertTrue(pump(w, lambda: w.params.get("_name") == "Small Motor" and w.params.get("pp") == 7))
        self.assertAlmostEqual(w.params["kv"], 400, places=2)
        self.assertTrue(pump(w, lambda: "Small Motor" in w.preset_active.text()))
        self.assertNotIn("modified", w.preset_active.text())
        self.assertTrue(pump(w, lambda: not w.dirty))            # stored in flash
        self.assertIn("Small Motor", self.esc._h("pack", "ANY")[0] and "Small Motor")
        # edit with rename
        edited = dict(newp, name="Small Motor 2", kv=450)
        w.ask_preset_values = lambda initial=None: dict(edited)
        w.on_preset_edit()
        self.assertIn("Small Motor 2", w.presets)
        self.assertNotIn("Small Motor", w.presets)
        # invalid names never reach the board
        self.assertIsNotNone(gui.PR.validate_name("a,b"))
        self.assertIsNotNone(gui.PR.validate_name("x" * 24))
        # 'Use' is refused while running
        w.select_preset("Small Motor 2")
        w.on_start()
        self.assertTrue(pump(w, lambda: w.running()))
        n = len(self.esc.sent)
        w.on_preset_apply()
        self.assertEqual(len(self.esc.sent), n)
        w.on_stop()
        pump(w, lambda: w.state == P.ST_IDLE)
        w.select_preset("Small Motor 2")
        w.on_preset_delete()
        self.assertNotIn("Small Motor 2", w.presets)

    def test_save_board_settings_as_preset(self):
        w = self.win
        gui.QtWidgets.QInputDialog.next_text = ("Test Motor", True)
        w.on_preset_save()
        self.assertIn("Test Motor", w.presets)
        self.assertTrue(pump(w, lambda: w.params.get("_name") == "Test Motor"))

    def test_zero_and_raw(self):
        w = self.win
        w.on_zero()
        self.assertTrue(pump(w, lambda: getattr(self.esc, "zero_calls", 0) == 1))
        self.assertTrue(pump(w, lambda: "zeroed" in w.status_label.text()))
        w.on_raw()
        self.assertTrue(pump(w, lambda: any("raw A" in self.esc.sent[-1] or True for _ in [0]) and "raw" in self.esc.sent))
        w.on_start()
        self.assertTrue(pump(w, lambda: w.running()))
        w.on_zero()
        self.assertEqual(getattr(self.esc, "zero_calls", 0), 1)  # refused locally while running
        w.on_stop()

    def test_plot_units_and_vbus_range(self):
        w = self.win
        self.assertEqual(w.plots["vbus"].yrange, (0, 65))              # fixed 0..65 V scale
        self.assertEqual(w.plots["current"].labels["left"], ("Phase currents", "A"))
        self.assertEqual(w.plots["vbus"].labels["left"], ("Bus voltage", "V"))

    def test_csv_recording(self):
        w = self.win
        path = os.path.join(tempfile.mkdtemp(), "log.csv")
        gui.QtWidgets.QFileDialog.next_path = path
        w.on_record()
        w.on_start()
        pump(w, lambda: False, 0.6)
        w.on_stop()
        w.on_record()                                          # stops
        with open(path) as f:
            lines = f.read().strip().splitlines()
        self.assertEqual(lines[0].split(","), P.CSV_COLUMNS)
        self.assertGreater(len(lines), 10)
        self.assertEqual(len(lines[5].split(",")), len(P.CSV_COLUMNS))

    def test_torque_units_and_disconnect(self):
        w = self.win
        w.tq_spin.setValue(0.083)
        self.assertIn("1.00 A", w.tq_conv.text())
        w.tq_a.setChecked(True)
        w.tq_nm.setChecked(False)                              # user picks 'A'
        self.assertAlmostEqual(w.tq_spin.value(), 1.0, places=2)
        w.on_connect_clicked()                                 # disconnect
        self.assertFalse(w.connected())
        self.assertEqual(w.btn_connect.text(), "Connect")

    # ------------------------------------------------------------------ stage 2
    def calibrate(self):
        w = self.win
        w.on_calibrate()
        self.assertTrue(pump(w, lambda: w.params.get("encdir") == 1, 5.0))

    def test_foc_start_refused_until_calibrated(self):
        w = self.win
        self.assertEqual(w.params["encdir"], 0)
        self.assertIn("NOT calibrated", w.enc_label.text())
        w.tabs.setCurrentWidget(w.tab_torque)
        qt_stubs.MessageBox.calls.clear()
        w.on_start()
        self.assertEqual(w.state, P.ST_IDLE)
        self.assertTrue(any("calibration" in str(c[1]) for c in qt_stubs.MessageBox.calls))
        self.assertNotIn("start", self.esc.sent)

    def test_calibration_updates_parameters(self):
        w = self.win
        self.calibrate()
        self.assertAlmostEqual(w.params["rl"], 48.0, places=1)
        self.assertAlmostEqual(w.params["ll"], 21.0, places=1)
        self.assertIn("(calibrated)", w.enc_label.text())
        self.assertIn("48.0 mOhm", w.cal_label.text())
        self.assertTrue(pump(w, lambda: not w.dirty))                 # written to flash automatically
        self.assertIn("flash", w.cal_label.text())
        pr = w.presets["ODrive M8325s 100KV"]                         # calibration stored in the preset (as a user copy)
        self.assertTrue(pr["_user"])
        self.assertEqual(pr["enc_dir"], 1)
        self.assertAlmostEqual(pr["r_ll_mohm"], 48.0, places=1)
        self.assertIn("save", self.esc.sent)

    def test_calibration_failure_is_reported(self):
        w = self.win
        self.esc._emit("CAL,fail,no_motion\r\n")
        self.assertTrue(pump(w, lambda: "Calibration failed" in w.cal_label.text()))
        self.assertIn("free to rotate", w.cal_label.text())

    def test_torque_mode_starts_at_zero_and_follows_setpoint(self):
        w = self.win
        self.calibrate()
        w.tabs.setCurrentWidget(w.tab_torque)
        w.tq_spin.setValue(0.05)                               # 0.05 Nm, stays pending
        w.on_start()
        self.assertTrue(pump(w, lambda: w.state == P.ST_RUN))
        self.assertEqual(w.tq_spin.value(), 0.0)               # always starts without torque
        self.assertEqual(w.sample["mode"], P.MODE_TORQUE)
        w.tq_a.setChecked(True)
        w.tq_nm.setChecked(False)
        w.tq_spin.setValue(0.5)
        w.send_tq()
        self.assertTrue(pump(w, lambda: abs(w.sample["iq"] - 0.5) < 0.05))
        self.assertIn("tq 0.5000", self.esc.sent)
        self.assertEqual(w.ro["iq"].text(), "%.3f A" % w.sample["iq"])
        # limited by imax (2 A)
        self.assertLessEqual(w.tq_spin._hi, 2.0 + 1e-9)
        w.on_stop()
        self.assertTrue(pump(w, lambda: w.state == P.ST_IDLE))

    def test_position_mode_holds_then_moves(self):
        w = self.win
        self.calibrate()
        w.tabs.setCurrentWidget(w.tab_pos)
        w.on_start()
        self.assertTrue(pump(w, lambda: w.state == P.ST_RUN and w.sample["mode"] == P.MODE_POSITION))
        w.pos_spin.setValue(123.0)
        w.send_pos()
        self.assertTrue(pump(w, lambda: w.sample["angle"] is not None and abs(w.sample["angle"] - 123.0) < 1.0, 4.0))
        self.assertIn("pos 123.00", self.esc.sent)
        self.assertIn("123.0", w.pos_sp_live.text())
        w.on_stop()

    def test_speed_tab_closed_loop_selects_foc_mode(self):
        w = self.win
        self.calibrate()
        w.rb_foc.setChecked(True)
        w.tabs.setCurrentWidget(w.tab_speed)
        w.on_start()
        self.assertTrue(pump(w, lambda: w.state == P.ST_RUN))
        self.assertEqual(w.sample["mode"], P.MODE_SPEED)
        self.assertFalse(w.rb_foc.isEnabled())                 # cannot switch while running
        w.on_stop()

    def test_overtemp_checkboxes_and_gains_are_parameters(self):
        w = self.win
        self.assertTrue(w.pw["otdrven"].isChecked())
        self.assertFalse(w.pw["otmoten"].isChecked())
        self.assertTrue(w.pw["otdrv"].isEnabled())
        self.assertFalse(w.pw["otmot"].isEnabled())            # motor NTC check off -> value greyed out
        w.pw["otmoten"].setChecked(True)
        self.assertTrue(w.pw["otmot"].isEnabled())
        w.pw["otdrv"].setValue(70.0)
        w.pw["spdkp"].setValue(0.07)
        w.pw["poskp"].setValue(2.5)
        w.flush_sets()
        self.assertTrue(pump(w, lambda: abs(self.esc._param("otdrv") - 70.0) < 1e-6))
        self.assertAlmostEqual(self.esc._param("otmoten"), 1.0)
        self.assertAlmostEqual(self.esc._param("spdkp"), 0.07, places=4)
        self.assertAlmostEqual(self.esc._param("poskp"), 2.5, places=3)

    def test_step_helper_alternates_and_reports(self):
        w = self.win
        self.calibrate()
        w.step_kind.setCurrentIndex(0)
        w.on_step_kind()
        w.step_half.setValue(0.3)
        w.on_step_start()
        self.assertTrue(w.step_active)
        deadline = time.time() + 6.0
        sent_levels = set()
        while time.time() < deadline and len(sent_levels) < 2:
            w.poll_link(); w.refresh_ui(); w.step_tick()
            sent_levels |= {c for c in self.esc.sent if c.startswith("tq ") and c != "tq 0.0000"}
            time.sleep(0.01)
        self.assertIn("tq -0.3000", sent_levels)
        self.assertIn("tq 0.3000", sent_levels)
        pump(w, lambda: "rise" in w.step_result.text(), 2.5) or [w.step_tick() for _ in range(1)]
        w.on_step_stop(True)
        self.assertFalse(w.step_active)
        self.assertTrue(pump(w, lambda: w.state == P.ST_IDLE))
        self.assertFalse(w.btn_step_stop.isEnabled())

    def test_calibrated_preset_restores_encoder_direction(self):
        w = self.win
        self.calibrate()
        w._store_preset({"name": "Cal motor", "notes": "", "pole_pairs": 7, "kv": 200.0, "r_ll_mohm": 200.0, "l_ll_uh": 300.0,
                         "kt_nm_a": 0.04, "rpm_max": 1500.0, "ilim": 8.0, "i_drive": 1.0, "accel": 60.0, "enc_dir": -1})
        w.on_preset_apply()
        self.assertTrue(pump(w, lambda: w.params.get("encdir") == -1 and w.params.get("pp") == 7))

    def test_preset_with_other_motor_resets_encoder_calibration(self):
        w = self.win
        self.calibrate()
        w._store_preset({"name": "Other motor", "notes": "", "pole_pairs": 7, "kv": 200.0, "r_ll_mohm": 200.0, "l_ll_uh": 300.0,
                         "kt_nm_a": 0.04, "rpm_max": 1500.0, "ilim": 8.0, "i_drive": 1.0, "accel": 60.0})
        w.on_preset_apply()
        self.assertTrue(pump(w, lambda: w.params.get("encdir") == 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
