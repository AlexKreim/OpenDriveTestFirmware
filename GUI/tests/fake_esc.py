"""A simulated ESC for testing the GUI without hardware.

Parameter handling (set/get/save/load/factory) is delegated to the REAL firmware code (esc_params.c)
through a small host-compiled harness, so range checks and formatting are the firmware's own.
Motor behaviour (state machine, telemetry) is a crude simulation.
"""
import os
import subprocess
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
def _find_core():
    """Firmware Core folder: $ESC_FW_CORE, else the CubeIDE project root (esc_GUI inside it), else a sibling esc_firmware folder."""
    if os.environ.get("ESC_FW_CORE"):
        return os.environ["ESC_FW_CORE"]
    for c in (os.path.join(HERE, "..", "..", "Core"), os.path.join(HERE, "..", "..", "esc_firmware", "Core")):
        if os.path.isfile(os.path.join(c, "Src", "esc_params.c")):
            return os.path.normpath(c)
    return os.path.normpath(os.path.join(HERE, "..", "..", "Core"))


FW_CORE = _find_core()
HARNESS = os.path.join(tempfile.gettempdir(), "esc_params_harness")


def build_harness():
    cmd = ["gcc", "-std=c11", "-I" + os.path.join(FW_CORE, "Inc"), os.path.join(HERE, "host_harness.c"),
           os.path.join(FW_CORE, "Src", "esc_params.c"), "-o", HARNESS, "-lm"]
    subprocess.check_call(cmd)


class FakeEsc:
    """Looks like a pyserial Serial object (read/write/close)."""

    def __init__(self, vbus=16.0):
        srcs = [os.path.join(FW_CORE, "Src", "esc_params.c"), os.path.join(FW_CORE, "Inc", "esc_params.h"),
                os.path.join(FW_CORE, "Inc", "esc_config.h"), os.path.join(HERE, "host_harness.c")]
        if not os.path.exists(HARNESS) or os.path.getmtime(HARNESS) < max(os.path.getmtime(p) for p in srcs):
            build_harness()                      # the firmware parameter code changed
        self.h = subprocess.Popen([HARNESS], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.out = bytearray()
        self.cv = threading.Condition()
        self.state = 0
        self.fault = 0
        self.vbus = vbus
        self.stream_ms = 0
        self.wd_ms = 0
        self.last_rx = time.monotonic()
        self.flash = None
        self.rpm = 0.0
        self.t_start = None
        self.closed = False
        self.sent = []
        self.mode = 0
        self.tq = 0.0
        self.pos_cmd = 0.0
        self.pos = 0.0
        self.iq = 0.0
        self.cal_at = None
        self.t0 = time.monotonic()
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    # --- harness helpers
    def _h(self, line, until):
        self.h.stdin.write(line + "\n")
        self.h.stdin.flush()
        res = []
        while True:
            l = self.h.stdout.readline().rstrip("\n")
            res.append(l)
            if l == until or (until == "ANY"):
                return res

    def _param(self, name):
        for l in self._h("get", "OK"):
            if l.startswith("P,%s," % name):
                return float(l.split(",")[2])

    # --- serial-like API
    def write(self, data):
        self.last_rx = time.monotonic()
        for ln in data.decode().splitlines():
            if ln.strip():
                self.sent.append(ln.strip())
                self._cmd(ln.strip())
        return len(data)

    def read(self, n):
        with self.cv:
            if not self.out:
                self.cv.wait(0.05)
            chunk = bytes(self.out[:n])
            del self.out[:n]
            return chunk

    def close(self):
        self.closed = True
        try:
            self.h.stdin.close()
            self.h.terminate()
        except Exception:
            pass

    def _emit(self, text):
        with self.cv:
            self.out += text.encode()
            self.cv.notify_all()

    # --- command handling
    def _cmd(self, line):
        w = line.split()
        c = w[0]
        running = self.state in (1, 2, 3, 5)
        if c == "ver":
            n = sum(1 for l in self._h("get", "OK") if l.startswith("P,"))
            self._emit("V,ESC,2.0,5,%d\r\n" % n)
        elif c == "ping":
            pass
        elif c in ("get", "params"):
            for l in self._h("get", "OK"):
                self._emit(l + "\r\n")
        elif c == "set" and len(w) == 3:
            if running and w[1] in ("pp", "six", "dir", "encdir"):
                self._emit("ERR,motor_running\r\n")
            else:
                r = self._h(line, "ANY")[0]
                self._emit("OK\r\n" if r == "OK" else "ERR,invalid\r\n")
        elif c == "setname" and len(w) >= 2:
            if running:
                self._emit("ERR,motor_running\r\n")
            else:
                r = self._h(line, "ANY")[0]
                self._emit("OK\r\n" if r == "OK" else "ERR,invalid\r\n")
        elif c == "zero":
            if running:
                self._emit("ERR,motor_running\r\n")
            else:
                self.zero_calls = getattr(self, "zero_calls", 0) + 1
                self._emit("zero: offsets A 1350.0, B 1351.0, C 1352.0 counts; noise p-p 6.0 / 5.0 / 7.0 counts\r\nOK\r\n")
        elif c == "raw":
            for i in range(3):
                self._emit("raw %c: mean 135%d, min 1348, max 1356, offset in use 1350.0 counts (0.002 A at mean)\r\n" % (65 + i, i))
            self._emit("expected idle ~1350 counts (1.09 V); a mean near 0 or 4095 means the chain is saturated or not connected\r\n")
        elif c == "mode" and len(w) == 2:
            if running:
                self._emit("ERR,motor_running\r\n")
            elif not w[1].isdigit() or int(w[1]) > 3:
                self._emit("ERR,range\r\n")
            else:
                self.mode = int(w[1])
                self._emit("OK\r\n")
        elif c == "tq" and len(w) == 2:
            v = float(w[1])
            if abs(v) > self._param("imax"):
                self._emit("ERR,range\r\n")
            else:
                self.tq = v
                self._emit("OK\r\n")
        elif c == "pos" and len(w) == 2:
            v = float(w[1])
            if not (0.0 <= v <= 360.0):
                self._emit("ERR,range\r\n")
            else:
                self.pos_cmd = v
                self._emit("OK\r\n")
        elif c == "calibrate":
            if self.state == 4:
                self._emit("ERR,fault\r\n")
            elif self.state != 0:
                self._emit("ERR,not_idle\r\n")
            else:
                self.state = 5
                self.cal_at = time.monotonic() + 0.4
                self._emit("OK\r\n")
        elif c == "tune":
            if self.state == 4:
                self._emit("ERR,fault\r\n")
            elif self.state != 0:
                self._emit("ERR,not_idle\r\n")
            elif self._param("encdir") == 0:
                self._emit("ERR,no_enc_cal\r\n")
            else:
                self.state = 3
                self.tune_at = time.monotonic() + 0.4
                self._emit("OK\r\n")
        elif c == "start":
            if self.state == 4:
                self._emit("ERR,fault\r\n")
            elif self.mode != 0 and self._param("encdir") == 0:
                self._emit("ERR,no_enc_cal\r\n")
            elif self.state != 0:
                self._emit("ERR,not_idle\r\n")
            elif not (9.0 <= self.vbus <= 62.0):
                self._emit("ERR,bus_voltage\r\n")
            else:
                self.state = 1
                self.t_start = time.monotonic()
                self.tq = 0.0
                self.iq = 0.0
                self.pos_cmd = self.pos
                self._emit("OK\r\n")
        elif c == "stop":
            if self.state != 4:
                self.state = 0
            self.rpm = 0.0
            self.tq = 0.0
            self.cal_at = None
            self._emit("OK\r\n")
        elif c == "clear":
            if self.state == 4:
                self.state, self.fault = 0, 0
            self._emit("OK\r\n")
        elif c == "save":
            if running:
                self._emit("ERR,motor_running\r\n")
            else:
                self.flash = self._h("pack", "ANY")[0]
                self._emit("OK\r\n")
        elif c == "load":
            if running:
                self._emit("ERR,motor_running\r\n")
            elif self.flash and self._h("unpack " + self.flash, "ANY")[0] == "VALID":
                self._emit("OK\r\n")
            else:
                self._emit("ERR,no_saved_data\r\n")
        elif c == "factory":
            if running:
                self._emit("ERR,motor_running\r\n")
            else:
                self._h("defaults", "ANY")
                self._emit("OK\r\n")
        elif c == "stream" and len(w) == 2:
            hz = int(w[1])
            if hz > 200:
                self._emit("ERR,range\r\n")
            else:
                self.stream_ms = 0 if hz == 0 else 1000 // hz
                self._emit("OK\r\n")
        elif c == "wd" and len(w) == 2:
            ms = int(w[1])
            if ms != 0 and not (200 <= ms <= 10000):
                self._emit("ERR,range\r\n")
            else:
                self.wd_ms = ms
                self._emit("OK\r\n")
        else:
            self._emit("unknown command, try 'help'\r\n")

    # --- simulation / telemetry
    def _loop(self):
        last = time.monotonic()
        next_t = last
        while not self.closed:
            time.sleep(0.002)
            now = time.monotonic()
            dt, last = now - last, now
            if self.state == 5 and self.cal_at and now >= self.cal_at:
                self._h("set rl 48.0", "ANY")
                self._h("set ll 21.0", "ANY")
                self._h("set encdir 1", "ANY")
                self.state = 0
                self.cal_at = None
                self._emit("CAL,ok,48.000,21.000,1,20.00\r\n")
            if self.state == 3 and getattr(self, "tune_at", None) and now >= self.tune_at:
                self._h("set spdkp 0.012", "ANY")
                self._h("set spdki 0.4", "ANY")
                self._h("set poskp 1.3", "ANY")
                self.state = 0
                self.tune_at = None
                self._emit("TUNE,ok,200.000,1.500,0.012000,0.400000,1.300000,5.000\r\n")
            if self.wd_ms and self.state in (1, 2, 3) and (now - self.last_rx) * 1000 > self.wd_ms:
                self.state = 0
                self.rpm = 0.0
            if self.state == 1 and now - self.t_start > 0.05:
                self.state = 2
            elif self.state == 2 and now - self.t_start > 0.3:
                self.state = 3
            if self.state == 3 and self.mode == 1:
                self.iq += (self.tq - self.iq) * min(1.0, dt / 0.005)         # crude first-order torque response
                self.rpm = max(-1500.0, min(1500.0, self.rpm + self.iq * 3000.0 * dt))
            elif self.state == 3 and self.mode == 3:
                self.pos += max(-300 * dt, min(300 * dt, self.pos_cmd - self.pos))
                self.rpm = 0.0
            elif self.state == 3:
                tgt = self._param("rpm") * self._param("dir")
                acc = self._param("accel") * dt * 0 + 600.0 * dt           # fast fake ramp
                self.rpm += max(-acc, min(acc, tgt - self.rpm))
            if self.stream_ms and now >= next_t:
                next_t = now + self.stream_ms / 1000.0
                ms = int((now - self.t0) * 1000)
                r = int(self.rpm * 100)
                ang = int(self.pos * 100) if self.mode == 3 else int((ms * 7) % 36000)
                possp = int(self.pos_cmd * 100) if (self.mode == 3 and self.state == 3) else -1
                foc = self.mode != 0 and self.state == 3
                self._emit("T,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n" % (
                    ms, self.state, self.fault, int(self.vbus * 1000), 1500, -700, -800, r, r, 0,
                    ang, 253, -9990, 1200 if self.state == 3 else 0,
                    self.mode, 20 if foc else 0, int(self.iq * 1000) if foc else 0, 1050 if self.state == 3 else 0,
                    int(self.tq * 1000) if foc else 0, possp, 50 if foc else 0, 400 if foc else 0))
