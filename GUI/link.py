"""Serial link to the ESC. Runs a reader thread and hands events to the GUI through a queue (no Qt here)."""
import collections
import queue
import threading
import time

from protocol import parse_telemetry

try:
    import serial
except ImportError:          # the GUI shows a clear message; tests inject their own serial factory
    serial = None

REPLY_COMMANDS = {"set", "start", "stop", "clear", "save", "load", "factory", "stream", "wd", "get", "params", "setname", "zero", "mode", "tq", "pos", "calibrate"}
PENDING_TIMEOUT_S = 2.0


class EscLink:
    """Events put on self.events (tuples):
         ("telemetry", dict)            one telemetry sample
         ("params", dict)               full parameter list after "get"
         ("ack", cmd, ok, message)      reply to a command ("OK" or "ERR,reason"; ok None = no reply in time)
         ("version", dict)              reply to "ver"
         ("calibration", dict)          result of "calibrate": {ok, r_ll_mohm, l_ll_uh, enc_dir, pp} or {ok False, reason}
         ("log", text)                  any other line from the board
         ("disconnected", text)         the port went away
    """

    def __init__(self, serial_factory=None):
        self.events = queue.Queue()
        self._factory = serial_factory or self._default_factory
        self._ser = None
        self._thread = None
        self._stop = threading.Event()
        self._wlock = threading.Lock()
        self._pending = collections.deque()      # (cmd, time) waiting for OK/ERR
        self._plock = threading.Lock()
        self._params = {}

    @staticmethod
    def _default_factory(port):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        return serial.Serial(port, 115200, timeout=0.05, write_timeout=0.5)

    # ------------------------------------------------------------------ open / close
    def is_open(self):
        return self._ser is not None

    def open(self, port):
        self.close()
        self._ser = self._factory(port)
        self._stop.clear()
        with self._plock:
            self._pending.clear()
        self._params = {}
        self._thread = threading.Thread(target=self._run, name="esc-reader", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.0)
        self._thread = None
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ sending
    def send(self, cmd):
        ser = self._ser
        if ser is None:
            return False
        word = cmd.split(" ", 1)[0]
        if word in REPLY_COMMANDS:
            with self._plock:
                self._pending.append((cmd, time.monotonic()))
        try:
            with self._wlock:
                ser.write((cmd + "\n").encode("ascii"))
            return True
        except Exception as e:
            self.events.put(("disconnected", "write failed: %s" % e))
            return False

    # ------------------------------------------------------------------ reading
    def _run(self):
        buf = b""
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                break
            try:
                data = ser.read(512)
            except Exception as e:
                self.events.put(("disconnected", str(e)))
                break
            if data:
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("ascii", "replace").strip()
                    if line:
                        self._handle(line)
            self._expire_pending()

    def _pop_pending(self):
        with self._plock:
            return self._pending.popleft()[0] if self._pending else None

    def _expire_pending(self):
        now = time.monotonic()
        while True:
            with self._plock:
                if not self._pending or now - self._pending[0][1] < PENDING_TIMEOUT_S:
                    return
                cmd = self._pending.popleft()[0]
            self.events.put(("ack", cmd, None, "no reply"))

    def _handle(self, line):
        if line.startswith("T,"):
            s = parse_telemetry(line)
            if s is not None:
                self.events.put(("telemetry", s))
            return
        if line.startswith("P,"):
            parts = line.split(",")
            if len(parts) == 3:
                try:
                    self._params[parts[1]] = float(parts[2])
                    return
                except ValueError:
                    pass
            self.events.put(("log", line))
            return
        if line.startswith("N,"):
            self._params["_name"] = line[2:].strip()
            return
        if line == "OK" or line.startswith("ERR,"):
            cmd = self._pop_pending()
            if cmd is not None and cmd.split(" ", 1)[0] in ("get", "params") and line == "OK":
                self.events.put(("params", dict(self._params)))
                self._params = {}
            self.events.put(("ack", cmd or "?", line == "OK", line))
            return
        if line.startswith("CAL,"):
            f = line.split(",")
            if len(f) >= 3 and f[1] == "fail":
                self.events.put(("calibration", {"ok": False, "reason": f[2]}))
                return
            if len(f) >= 6 and f[1] == "ok":
                try:
                    self.events.put(("calibration", {"ok": True, "r_ll_mohm": float(f[2]), "l_ll_uh": float(f[3]),
                                                     "enc_dir": int(f[4]), "pp": float(f[5])}))
                    return
                except ValueError:
                    pass
            self.events.put(("log", line))
            return
        if line.startswith("V,"):
            f = line.split(",")
            if len(f) >= 5:
                try:
                    self.events.put(("version", {"fw": f[2], "params_version": int(f[3]), "count": int(f[4])}))
                    return
                except ValueError:
                    pass
        self.events.put(("log", line))
