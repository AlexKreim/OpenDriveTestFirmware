"""Protocol definitions shared by the GUI: telemetry parsing, parameter table, derived-value model.

The firmware speaks plain text lines over the USB virtual COM port:
    host -> board : "set rpm 120", "get", "start", "stop", "save", "stream 50", "ping", ...
    board -> host : "OK" / "ERR,<reason>", "P,<name>,<value>" lines after "get",
                    "V,ESC,<fw>,<params version>,<count>" after "ver",
                    "T,..." telemetry lines while streaming.
"""

EXPECTED_PARAMS_VERSION = 5

STATE_NAMES = ["IDLE", "PRECHARGE", "ALIGN", "RUN", "FAULT", "CALIB"]
FAULT_NAMES = ["none", "overcurrent", "overvoltage", "undervoltage", "overtemp", "encoder", "overspeed", "estop (CAN)", "CAN timeout"]
ST_IDLE, ST_PRECHARGE, ST_ALIGN, ST_RUN, ST_FAULT, ST_CALIB = range(6)
RUNNING_STATES = (ST_PRECHARGE, ST_ALIGN, ST_RUN, ST_CALIB)

MODE_OPEN, MODE_TORQUE, MODE_SPEED, MODE_POSITION = range(4)
MODE_NAMES = ["Open loop", "Torque (FOC)", "Speed (FOC)", "Position (FOC)"]

# T,<ms>,<state>,<fault>,<vbus mV>,<ia mA>,<ib mA>,<ic mA>,<rpm cmd x100>,<rpm spi x100>,<rpm abi x100>,
#   <angle x100 deg, -1 = error>,<driver temp x10 C>,<motor temp x10 C>,<applied line-line mV>,
#   <mode>,<Id mA>,<Iq mA>,<|I| mA>,<Iq reference mA>,<position set point x100 deg, -1 = none>,<Vd mV>,<Vq mV>
TELEM_FIELDS = ["ms", "state", "fault", "vbus_mv", "ia_ma", "ib_ma", "ic_ma",
                "rpm_cmd_c", "rpm_spi_c", "rpm_abi_c", "angle_c", "tdrv_d", "tmot_d", "vll_mv",
                "mode", "id_ma", "iq_ma", "imag_ma", "iqref_ma", "possp_c", "vd_mv", "vq_mv"]

# optional fields appended by newer firmware (older firmware sends only the 22 above):
#   <multi-turn position x100 deg, user frame>,<position-mode profile set point x100 deg, -2147483647 = none>,
#   <control owner: 0 none, 1 USB, 2 CAN>,<bus current mA (CC6905)>
TELEM_EXTRA = ["pos_mt_c", "possp_mt_c", "owner", "ibus_ma"]
OWNER_NAMES = ["none", "USB", "CAN"]

CSV_COLUMNS = ["t_s", "state", "fault", "vbus_V", "ia_A", "ib_A", "ic_A", "rpm_cmd", "rpm_spi",
               "rpm_abi", "angle_deg", "t_driver_C", "t_motor_C", "v_ll_applied_V",
               "mode", "id_A", "iq_A", "i_mag_A", "iq_ref_A", "pos_setpoint_deg", "vd_V", "vq_V"]


def parse_telemetry(line):
    """Return a dict with SI units, or None if the line is not a valid telemetry line."""
    parts = line.strip().split(",")
    n = len(parts) - 1
    if parts[0] != "T" or n < len(TELEM_FIELDS) or n > len(TELEM_FIELDS) + len(TELEM_EXTRA):
        return None
    try:
        v = dict(zip(TELEM_FIELDS + TELEM_EXTRA, (int(x) for x in parts[1:])))
    except ValueError:
        return None
    return {
        "t": v["ms"] / 1000.0,
        "state": v["state"],
        "fault": v["fault"],
        "vbus": v["vbus_mv"] / 1000.0,
        "ia": v["ia_ma"] / 1000.0,
        "ib": v["ib_ma"] / 1000.0,
        "ic": v["ic_ma"] / 1000.0,
        "rpm_cmd": v["rpm_cmd_c"] / 100.0,
        "rpm_spi": v["rpm_spi_c"] / 100.0,
        "rpm_abi": v["rpm_abi_c"] / 100.0,
        "angle": None if v["angle_c"] < 0 else v["angle_c"] / 100.0,
        "tdrv": None if v["tdrv_d"] < -900 else v["tdrv_d"] / 10.0,
        "tmot": None if v["tmot_d"] < -900 else v["tmot_d"] / 10.0,
        "vll": v["vll_mv"] / 1000.0,
        "mode": v["mode"],
        "id": v["id_ma"] / 1000.0,
        "iq": v["iq_ma"] / 1000.0,
        "imag": v["imag_ma"] / 1000.0,
        "iqref": v["iqref_ma"] / 1000.0,
        "possp": None if v["possp_c"] < 0 else v["possp_c"] / 100.0,
        "vd": v["vd_mv"] / 1000.0,
        "vq": v["vq_mv"] / 1000.0,
        "pos_mt": (v["pos_mt_c"] / 100.0) if "pos_mt_c" in v else None,
        "possp_mt": (v["possp_mt_c"] / 100.0) if ("possp_mt_c" in v and v["possp_mt_c"] != -2147483647) else None,
        "owner": v.get("owner"),
        "ibus": (v["ibus_ma"] / 1000.0) if "ibus_ma" in v else None
    }


def telemetry_csv_row(s):
    def f(x, nd):
        return "" if x is None else ("%.*f" % (nd, x))
    return [f(s["t"], 3), s["state"], s["fault"], f(s["vbus"], 3), f(s["ia"], 3), f(s["ib"], 3), f(s["ic"], 3),
            f(s["rpm_cmd"], 2), f(s["rpm_spi"], 2), f(s["rpm_abi"], 2), f(s["angle"], 2),
            f(s["tdrv"], 1), f(s["tmot"], 1), f(s["vll"], 3),
            s["mode"], f(s["id"], 3), f(s["iq"], 3), f(s["imag"], 3), f(s["iqref"], 3), f(s["possp"], 2), f(s["vd"], 3), f(s["vq"], 3)]


# name, label, kind, min, max, step, decimals, unit, tooltip
PARAM_SPECS = [
    ("pp",      "Pole pairs",                 "int",   1,     50,    1,     0, "",      "Motor pole pairs (magnets / 2)"),
    ("kv",      "Kv",                         "float", 1,     5000,  1,     2, "rpm/V", "Speed constant: rpm per volt of peak line-line back-EMF"),
    ("rl",      "Resistance (line-line)",     "float", 1,     10000, 1,     1, "mOhm",  "Line-line winding resistance (2 x phase-neutral)"),
    ("ll",      "Inductance (line-line)",     "float", 0,     10000, 0.1,   1, "uH",    "Line-line inductance (2 x phase-neutral), used by the torque/position stage"),
    ("kt",      "Torque constant",            "float", 0.001, 10,    0.001, 4, "Nm/A",  "Torque per ampere"),
    ("rpmmax",  "Speed limit",                "float", 1,     20000, 10,    0, "rpm",   "The firmware never commands more than this"),
    ("ilim",    "Overcurrent trip",           "float", 0.5,   100,   0.5,   1, "A",     "Per-phase current that stops the motor (software)"),
    ("idrive",  "Drive current (auto mode)",  "float", 0.1,   100,   0.5,   2, "A",     "Peak phase current used for holding and as boost in automatic mode"),
    ("accel",   "Acceleration",               "float", 1,     10000, 10,    0, "rpm/s", "Speed ramp"),
    ("rpm",     "Target speed",               "float", 0,     10000, 10,    1, "rpm",   "Mechanical rpm"),
    ("dir",     "Direction",                  "dir",   -1,    1,     1,     0, "",      "+1 forward / -1 reverse (cannot change while running)"),
    ("six",     "6-step commutation",         "bool",  0,     1,     1,     0, "",      "Off = sinusoidal"),
    ("auto",    "Automatic parameters",       "bool",  0,     1,     1,     0, "",      "Derive boost, align voltage and modulation limit from the motor constants"),
    ("boost",   "Boost (manual mode)",        "float", 0,     30,    0.05,  3, "V",     "Extra line-line volts on top of rpm/Kv"),
    ("maxmod",  "Modulation limit (manual)",  "float", 0.01,  0.95,  0.01,  3, "",      "Caps the voltage at maxmod * Vbus / 1.15"),
    ("alignv",  "Align voltage (manual)",     "float", 0,     30,    0.05,  3, "V",     "Voltage used to pull the rotor to a known angle before spinning"),
    ("alignms", "Align time",                 "int",   0,     10000, 100,   0, "ms",    "Duration of the align step"),
    # ---- FOC / closed loops ----
    ("curbw",   "Current loop bandwidth",     "float", 10,    5000,  50,    0, "Hz",    "Auto gains: Kp = 2 pi bw L, Ki = 2 pi bw R. 1000 Hz is a safe start at 20 kHz PWM"),
    ("curauto", "Automatic current gains",    "bool",  0,     1,     1,     0, "",      "Derive the current loop gains from R, L and the bandwidth"),
    ("curkp",   "Current Kp (manual)",        "float", 0,     100,   0.01,  4, "V/A",   "Proportional gain of the d and q current controllers"),
    ("curki",   "Current Ki (manual)",        "float", 0,     100000, 10,   1, "V/(A s)", "Integral gain of the d and q current controllers"),
    ("spdkp",   "Speed Kp",                   "float", 0,     10,    0.005, 4, "A/rpm", "Speed loop proportional gain (output: q current)"),
    ("spdki",   "Speed Ki",                   "float", 0,     1000,  0.01,  3, "A/(rpm s)", "Speed loop integral gain"),
    ("poskp",   "Position Kp",                "float", 0,     1000,  0.1,   2, "rpm/deg", "Position loop proportional gain (output: speed set point)"),
    ("poski",   "Position Ki",                "float", 0,     1000,  0.1,   2, "rpm/(deg s)", "Position loop integral gain (usually 0)"),
    ("poskd",   "Position Kd",                "float", 0,     100,   0.01,  3, "rpm s/deg", "Position loop derivative gain (usually 0, the speed loop already damps)"),
    ("posvmax", "Position move speed",        "float", 1,     20000, 10,    0, "rpm",   "Speed limit of a position move (motion profile)"),
    ("posacc",  "Position move acceleration", "float", 1,     100000, 50,   0, "rpm/s", "Acceleration of a position move (motion profile)"),
    ("imax",    "FOC current limit",          "float", 0.1,   100,   0.5,   2, "A",     "Maximum q-axis current of torque, speed and position mode"),
    ("ialign",  "Align current",              "float", 0,     100,   0.1,   2, "A",     "D-axis current that pulls the rotor to the electrical zero at every FOC start"),
    ("ical",    "Calibration current",        "float", 0.2,   100,   0.5,   2, "A",     "Current used to measure R, L and the encoder direction"),
    ("otdrv",   "Driver overtemperature",     "float", 20,    150,   1,     0, "\u00b0C", "Driver NTC trip temperature"),
    ("otdrven", "Driver overtemperature on",  "bool",  0,     1,     1,     0, "",      "Enable the driver temperature trip (a missing or shorted NTC also trips)"),
    ("otmot",   "Motor overtemperature",      "float", 20,    200,   1,     0, "\u00b0C", "Motor NTC trip temperature"),
    ("otmoten", "Motor overtemperature on",   "bool",  0,     1,     1,     0, "",      "Enable the motor temperature trip. Only with a fitted NTC: a missing NTC reads -999 and trips"),
    ("encdir",  "Encoder direction",          "enc",   -1,    1,     1,     0, "",      "0 = not calibrated. Set by the calibration, needed before any FOC mode can start"),
]
PARAM_SPEC_BY_NAME = {s[0]: s for s in PARAM_SPECS}

# must match esc_config.h
DEADTIME_NS = 500.0
PWM_FREQ_HZ = 20000.0


def max_rpm_at_bus(p, vbus):
    vb = max(vbus, 5.0)
    boost = p["idrive"] * p["rl"] * 1e-3 + vb * DEADTIME_NS * 1e-9 * PWM_FREQ_HZ
    avail = 0.9 * vb / 1.1547
    return max(0.0, (avail - boost) * p["kv"])


def derive(p, vbus):
    """Mirror of params_derive() in esc_params.c (used for the on-screen estimates only)."""
    vb = max(vbus, 5.0)
    if p["auto"]:
        boost = p["idrive"] * p["rl"] * 1e-3 + vb * DEADTIME_NS * 1e-9 * PWM_FREQ_HZ
        rpm_t = min(abs(p["rpm"]), p["rpmmax"])
        v_need = rpm_t / p["kv"] + boost
        m = min(0.95, max(0.03, 1.1547 * v_need * 1.3 / vb))
        return {"boost": boost, "align": boost, "maxmod": m,
                "rpm_limit": min(p["rpmmax"], max_rpm_at_bus(p, vb))}
    return {"boost": p["boost"], "align": p["alignv"], "maxmod": p["maxmod"], "rpm_limit": p["rpmmax"]}
