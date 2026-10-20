import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import protocol
from link import EscLink
from fake_esc import FakeEsc


def wait_for(link, pred, timeout=3.0):
    """Collect events until pred(event) is true; return (matching event, all events)."""
    seen = []
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            ev = link.events.get(timeout=0.05)
        except Exception:
            continue
        seen.append(ev)
        if pred(ev):
            return ev, seen
    return None, seen


def ack_for(link, cmd, timeout=3.0):
    ev, _ = wait_for(link, lambda e: e[0] == "ack" and e[1] == cmd, timeout)
    assert ev is not None, "no ack for %r" % cmd
    return ev


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.esc = FakeEsc()
        self.link = EscLink(serial_factory=lambda port: self.esc)
        self.link.open("FAKE")

    def tearDown(self):
        self.link.close()

    def get_params(self):
        self.link.send("get")
        ev, _ = wait_for(self.link, lambda e: e[0] == "params")
        self.assertIsNotNone(ev, "no params event")
        ack_for(self.link, "get")
        return ev[1]

    def test_telemetry_parse(self):
        s = protocol.parse_telemetry("T,12345,3,0,16065,-120,330,-210,6000,5980,0,12345,251,-9990,1175,2,-15,1234,1250,1300,-1,120,-640")
        self.assertEqual(s["mode"], 2)
        self.assertAlmostEqual(s["iq"], 1.234)
        self.assertAlmostEqual(s["imag"], 1.25)
        self.assertAlmostEqual(s["iqref"], 1.3)
        self.assertIsNone(s["possp"])
        self.assertAlmostEqual(s["vq"], -0.64)
        self.assertEqual(len(protocol.telemetry_csv_row(s)), len(protocol.CSV_COLUMNS))
        self.assertAlmostEqual(s["vbus"], 16.065)
        self.assertAlmostEqual(s["ia"], -0.12)
        self.assertAlmostEqual(s["rpm_spi"], 59.8)
        self.assertAlmostEqual(s["angle"], 123.45)
        self.assertIsNone(s["tmot"])
        self.assertAlmostEqual(s["tdrv"], 25.1)
        self.assertIsNone(protocol.parse_telemetry("T,1,2,3"))
        self.assertIsNone(protocol.parse_telemetry("T,a,b,c,d,e,f,g,h,i,j,k,l,m,n"))

    def test_calibration_event(self):
        self.esc._emit("CAL,ok,48.000,21.500,-1,19.98\r\n")
        ev, _ = wait_for(self.link, lambda e: e[0] == "calibration")
        self.assertEqual(ev[1], {"ok": True, "r_ll_mohm": 48.0, "l_ll_uh": 21.5, "enc_dir": -1, "pp": 19.98})
        self.esc._emit("CAL,fail,no_current\r\n")
        ev, _ = wait_for(self.link, lambda e: e[0] == "calibration")
        self.assertEqual(ev[1], {"ok": False, "reason": "no_current"})

    def test_params_roundtrip_and_ranges_match_firmware(self):
        p = self.get_params()
        names = [s[0] for s in protocol.PARAM_SPECS]
        self.assertEqual(sorted(k for k in p if not k.startswith("_")), sorted(names))
        self.assertEqual(p["_name"], "ODrive M8325s 100KV")
        # GUI limits must agree with the firmware table: just inside accepted, just outside rejected
        for name, label, kind, lo, hi, step, dec, unit, tip in protocol.PARAM_SPECS:
            if kind == "bool":
                continue
            for val, ok in ((lo, True), (hi, True), (lo - max(abs(lo) * 0.01, 0.001) if lo != -1 else -2, False),
                            (hi + max(abs(hi) * 0.01, 0.001), False)):
                if name == "dir" and val in (-1, 1):
                    pass
                cmd = "set %s %s" % (name, repr(float(val)))
                self.link.send(cmd)
                ev = ack_for(self.link, cmd)
                self.assertEqual(ev[2], ok, "param %s value %s expected ok=%s got %s" % (name, val, ok, ev))

    def test_derive_matches_firmware(self):
        # firmware side (real C code) vs the GUI mirror
        for vbus in (12.0, 16.0, 24.0, 48.0):
            for auto in (1, 0):
                self.esc._h("defaults", "ANY")
                self.esc._h("set auto %d" % auto, "ANY")
                self.esc._h("set rpm 300", "ANY")
                line = self.esc._h("derive %s" % vbus, "ANY")[0]
                fw = [float(x) for x in line.split(",")[1:]]
                p = {}
                for l in self.esc._h("get", "OK"):
                    if l.startswith("P,"):
                        a = l.split(",")
                        p[a[1]] = float(a[2])
                d = protocol.derive(p, vbus)
                self.assertAlmostEqual(d["boost"], fw[0], places=4)
                self.assertAlmostEqual(d["align"], fw[1], places=4)
                self.assertAlmostEqual(d["maxmod"], fw[2], places=4)
                self.assertAlmostEqual(d["rpm_limit"], fw[3], places=1)
                if auto:
                    self.assertAlmostEqual(protocol.max_rpm_at_bus(p, vbus), fw[4], places=1)

    def test_save_load_cycle(self):
        self.link.send("set kv 123.5")
        ack_for(self.link, "set kv 123.5")
        self.link.send("save")
        ev, _ = wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "save")
        self.assertTrue(ev[2])
        self.link.send("set kv 99")
        ack_for(self.link, "set kv 99")
        self.link.send("load")
        ev, _ = wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "load")
        self.assertTrue(ev[2])
        self.assertAlmostEqual(self.get_params()["kv"], 123.5, places=3)

    def test_start_stop_and_save_refused_while_running(self):
        self.link.send("start")
        ev, _ = wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "start")
        self.assertTrue(ev[2])
        self.link.send("save")
        ev, _ = wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "save")
        self.assertFalse(ev[2])
        self.assertEqual(ev[3], "ERR,motor_running")
        self.link.send("set pp 7")
        ev, _ = wait_for(self.link, lambda e: e[0] == "ack" and e[1].startswith("set pp"))
        self.assertEqual(ev[3], "ERR,motor_running")
        self.link.send("stop")
        ev, _ = wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "stop")
        self.assertTrue(ev[2])

    def test_stream_and_rate_change(self):
        self.link.send("stream 100")
        ev, seen = wait_for(self.link, lambda e: e[0] == "telemetry")
        self.assertIsNotNone(ev)
        n = sum(1 for _ in range(1))
        t0 = time.time()
        count = 0
        while time.time() - t0 < 1.0:
            try:
                e = self.link.events.get(timeout=0.05)
                count += e[0] == "telemetry"
            except Exception:
                pass
        self.assertGreater(count, 60)
        self.assertLess(count, 140)
        self.link.send("stream 0")
        time.sleep(0.3)
        while not self.link.events.empty():
            self.link.events.get()
        time.sleep(0.3)
        self.assertTrue(self.link.events.empty() or all(e[0] != "telemetry" for e in list(self.link.events.queue)))

    def test_watchdog_stops_motor(self):
        self.link.send("wd 300")
        wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "wd 300")
        self.link.send("start")
        wait_for(self.link, lambda e: e[0] == "ack" and e[1] == "start")
        self.assertIn(self.esc.state, (1, 2, 3))
        time.sleep(0.8)                      # no pings
        self.assertEqual(self.esc.state, 0)

    def test_version(self):
        self.link.send("ver")
        ev, _ = wait_for(self.link, lambda e: e[0] == "version")
        self.assertEqual(ev[1]["params_version"], protocol.EXPECTED_PARAMS_VERSION)


class PresetTests(unittest.TestCase):
    def test_bundled_preset_and_commands(self):
        import presets
        pr = presets.load_presets()
        self.assertIn("ODrive M8325s 100KV", pr)
        cmds, skipped = presets.preset_commands(pr["ODrive M8325s 100KV"])
        self.assertEqual(skipped, [])
        self.assertIn("set pp 20", cmds)
        self.assertIn("set kv 100", cmds)
        self.assertIn("set rl 48", cmds)
        back = presets.preset_from_params("x", {"pp": 20, "kv": 100.0, "rl": 48.0})
        self.assertEqual(back["pole_pairs"], 20)


if __name__ == "__main__":
    unittest.main(verbosity=2)
