"""The dashboard's emulator runner: starts the emulated ESP32 and its network inside the hub process (needs g++, about 10 s)."""
import os
import sys
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

import server

server.print = lambda *a, **k: None
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dashboard"))
from emu_runner import EmuRunner  # noqa: E402


@unittest.skipIf(os.environ.get("ROOBOT_SKIP_SLOW"), "slow")
class EmuRunnerTest(unittest.TestCase):
    def test_start_drive_link_stop(self):
        srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        base = "http://127.0.0.1:%d" % srv.server_address[1]
        cal = b'{"jointLimits": [[0,180],[0,180],[0,180],[0,180]], "jointHomes": [90,90,90,90]}'   # the hub pushes this before starting
        urllib.request.urlopen(urllib.request.Request(base + "/calibration", data=cal, method="POST", headers={"Content-Type": "application/json"}), timeout=3).read()
        runner = EmuRunner("http://127.0.0.1:%d" % srv.server_address[1])
        try:
            with self.assertRaises(ValueError):
                runner.start("nonsense")
            with self.assertRaises(RuntimeError):
                runner.set_link("esp", preset="poor")          # not running yet
            runner.start("stock", "lan")
            end = time.time() + 40
            while runner.state == "building" and time.time() < end:
                time.sleep(0.2)
            self.assertEqual(runner.state, "running", runner.error)
            runner.set_link("esp", preset="poor")
            runner.outage("ctl", 1)
            snap = runner.snapshot()
            self.assertEqual(snap["links"]["esp"]["preset"], "poor")
            self.assertEqual(snap["links"]["ctl"]["preset"], "lan")
            self.assertGreater(snap["links"]["ctl"]["outage_s"], 0)
            # a stick pushed through the controller link reaches the server
            req = urllib.request.Request(runner.ctl_url() + "/input", data=b'{"LSX":0,"LSY":0,"RSX":0.5,"RSY":0,"A":false,"B":false,"X":false,"Y":false}',
                                         method="POST", headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=5).read()
            self.assertGreaterEqual(server._state["count"], 1)
            end = time.time() + 15
            while runner.snapshot()["servo"] is None and time.time() < end:
                time.sleep(0.2)
            self.assertIsNotNone(runner.snapshot()["servo"])
        finally:
            runner.stop()
            srv.shutdown()
        self.assertEqual(runner.state, "off")
        self.assertIsNone(runner.ctl_url())


if __name__ == "__main__":
    unittest.main()
