"""Our firmware (stock + damper patch + sequences-and-magnet patch) built for this PC and run against the real server.

Slow (real time): needs g++ and git or patch. A sequence posted to the server is fetched by the firmware, run once with a
wait for arrival at every pose, and the magnet relay switches on the grab and release poses. The state the firmware posts
back is what the dashboard and an AI read.
"""
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

import server

server.print = lambda *a, **k: None
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "emu"))
import emulator  # noqa: E402

CAL = {"jointLimits": [[0, 180]] * 4, "jointHomes": [90, 90, 90, 90]}
IDLE = {"LSX": 0.0, "LSY": 0.0, "RSX": 0.0, "RSY": 0.0, "A": False, "B": False, "X": False, "Y": False}


def call(base, path, obj=None):
    req = urllib.request.Request(base + path, data=None if obj is None else json.dumps(obj).encode(),
                                 method="GET" if obj is None else "POST", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as r:
        body = r.read()
    return json.loads(body) if body else None


class SequenceFirmware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get("ROOBOT_SKIP_SLOW"):
            raise unittest.SkipTest("ROOBOT_SKIP_SLOW is set")
        if not (shutil.which("g++") and emulator.have_patch_tool()):
            raise unittest.SkipTest("needs g++ and git or patch")
        cls.tmp = tempfile.mkdtemp()
        cls.exe = emulator.build("ours", cls.tmp)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def setUp(self):
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        self.base = "http://127.0.0.1:%d" % self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        with server._lock:
            server._state.update(data=None, received_at=None, count=0)
            server._arm.update(status="", calibration=None, max_speed=None)
            server._seq.update(id=0, poses=[])
            server._armstate.update(data=None, received_at=None)
        call(self.base, "/calibration", CAL)
        self.feeding = True
        threading.Thread(target=self._feed, daemon=True).start()
        self.emu = emulator.Emulator(self.exe, self.base[len("http://"):])

    def _feed(self):
        while self.feeding:
            try:
                call(self.base, "/input", IDLE)
            except OSError:
                pass
            time.sleep(0.5)

    def tearDown(self):
        self.feeding = False
        self.emu.stop()
        self.srv.shutdown()
        self.srv.server_close()

    def state(self):
        s = call(self.base, "/armstate")
        return (s or {}).get("state") or {}

    def wait(self, cond, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            if cond(self.state()):
                return
            time.sleep(0.2)
        self.fail("timed out waiting for " + what + "; last state " + json.dumps(self.state()))

    def test_runs_a_sequence_once_with_magnet_actions(self):
        self.wait(lambda s: "x" in s, 15, "the first state report")
        self.assertEqual(self.state()["magnet"], 0)
        poses = [{"x": 0, "y": 200, "z": 120, "angle": 0, "action": None},
                 {"x": 0, "y": 230, "z": 60, "angle": 0, "action": "grab"},
                 {"x": 80, "y": 200, "z": 100, "angle": 0, "action": None},
                 {"x": 80, "y": 200, "z": 60, "angle": 0, "action": "release"}]
        sid = call(self.base, "/sequence", {"poses": poses})["id"]
        seen = []
        end = time.time() + 120
        while time.time() < end:
            s = self.state()
            if s.get("mode") == "sequence":
                if not seen or seen[-1] != (s["step"], s["magnet"]):
                    seen.append((s["step"], s["magnet"]))
            if s.get("seqDone") == sid:
                break
            time.sleep(0.15)
        else:
            self.fail("the sequence did not finish; last state " + json.dumps(self.state()))
        s = self.state()
        self.assertEqual(s["magnet"], 0)                                   # released at the end
        self.assertAlmostEqual(s["x"], 80, delta=4)
        self.assertAlmostEqual(s["z"], 60, delta=4)
        self.assertIn(1, [m for _, m in seen])                              # the magnet was on in between
        self.assertEqual(s["mode"], "idle")
        time.sleep(4)                                                       # and it does not start a second lap
        self.assertEqual(self.state()["mode"], "idle")
        self.assertEqual(self.state()["magnet"], 0)

    def test_a_sequence_queued_before_power_up_is_not_run(self):
        self.emu.stop()
        call(self.base, "/sequence", {"poses": [{"x": 0, "y": 200, "z": 120, "angle": 0, "action": "grab"}]})
        self.emu = emulator.Emulator(self.exe, self.base[len("http://"):])
        self.wait(lambda s: "x" in s, 15, "the first state report")
        time.sleep(3)
        self.assertEqual(self.state()["magnet"], 0)
        self.assertEqual(self.state()["mode"], "idle")

    def test_x_toggles_the_magnet_and_b_aborts_a_run(self):
        self.wait(lambda s: "x" in s, 15, "the first state report")
        call(self.base, "/input", dict(IDLE, X=True))
        self.wait(lambda s: s.get("magnet") == 1, 15, "magnet on after an X press")
        call(self.base, "/input", IDLE)
        time.sleep(1)
        call(self.base, "/input", dict(IDLE, X=True))
        self.wait(lambda s: s.get("magnet") == 0, 15, "magnet off after a second X press")

    def test_server_rejects_bad_sequences(self):
        for bad in ({"poses": [{"x": 1}]}, {"poses": "no"}, {"poses": [{"x": 0, "y": 0, "z": 0, "angle": 0, "action": "zap"}]},
                    {"poses": [{"x": 0, "y": 0, "z": 0, "angle": 0, "action": None}] * 21}):
            with self.assertRaises(urllib.error.HTTPError):
                call(self.base, "/sequence", bad)

    def test_server_passes_a_pose_speed_to_the_firmware(self):
        call(self.base, "/sequence", {"poses": [{"x": 0, "y": 200, "z": 120, "angle": 0, "action": None, "speed": 22.5},
                                                {"x": 0, "y": 200, "z": 100, "angle": 0, "action": None}]})
        self.assertEqual([p[5] for p in call(self.base, "/sequence")["poses"]], [22.5, 0.0])      # sixth number; 0 = the normal limit
        for bad in (-1, 5000, "fast"):
            with self.assertRaises(urllib.error.HTTPError):
                call(self.base, "/sequence", {"poses": [{"x": 0, "y": 0, "z": 0, "angle": 0, "action": None, "speed": bad}]})

    def restart_firmware(self):
        """A new power-up: forget the old report, so what is read next comes from the new firmware (which ignores a sequence queued before it started)."""
        self.emu.stop()
        with server._lock:
            server._armstate.update(data=None, received_at=None)
        self.emu = emulator.Emulator(self.exe, self.base[len("http://"):])
        self.wait(lambda s: "x" in s, 15, "the first state report after a restart")

    def run_timed(self, speed):
        """Seconds the firmware takes over the one big move of a sequence whose poses carry this speed (None = no speed)."""
        far = {"x": 90, "y": 190, "z": 70, "angle": 0, "action": None}
        poses = [{"x": 0, "y": 200, "z": 120, "angle": 0, "action": None}, dict(far)]
        if speed:
            poses[0]["speed"] = poses[1]["speed"] = speed
        self.wait(lambda s: "x" in s, 15, "the first state report")
        sid = call(self.base, "/sequence", {"poses": poses})["id"]
        started = None
        end = time.time() + 120
        while time.time() < end:
            s = self.state()
            if started is None and s.get("mode") == "sequence":
                started = time.time()
            if s.get("seqDone") == sid:
                return time.time() - (started or time.time())
            time.sleep(0.05)
        self.fail("the sequence did not finish")

    def test_a_pose_speed_changes_how_fast_the_arm_moves(self):
        self.wait(lambda s: "x" in s, 15, "the first state report")
        self.assertEqual(self.state().get("poseSpeeds"), 1)                 # this firmware says it reads a speed per pose
        slow = self.run_timed(8)
        self.assertGreater(slow, 2.5)
        self.restart_firmware()
        fast = self.run_timed(55)
        self.assertLess(fast, slow - 2.0)                                    # the same move, much quicker at 55 deg/s than at 8

    def test_the_normal_limit_is_back_after_a_sped_up_run(self):
        sped = self.run_timed(55)
        self.restart_firmware()
        plain = self.run_timed(None)                                         # no speeds: the 30 deg/s default again, not 55
        self.assertGreater(plain, sped + 0.4)


if __name__ == "__main__":
    unittest.main()
