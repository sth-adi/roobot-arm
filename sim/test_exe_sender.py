"""ExeSender: the real arm through Sam's exe (one run per step, his start/end-point JSON) with a virtual twin running beside it. The exe here is a fake."""
import json
import os
import sys
import tempfile
import threading
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import lowlevel as LL  # noqa: E402


def step(x, y, z, a=0.0, speed=30.0, action=None):
    return {"x": x, "y": y, "z": z, "angle": a, "speed": speed, "action": action, "label": "t"}


HOME = {"x": 0.0, "y": 250.0, "z": 200.0, "angle": 0.0}


class FakeTwin:
    def __init__(self):
        self.ran, self.stopped = [], False

    def run(self, steps, job="", on_step=None):
        self.ran.extend(steps)

    def stop(self):
        self.stopped = True


class Recorder:
    """A fake exe runner: remembers every JSON it was given (read from the temp file, as the real exe would)."""
    def __init__(self, code=0, out="", delay=0.0):
        self.calls, self.code, self.out, self.delay = [], code, out, delay

    def __call__(self, argv, timeout_s, stopped):
        with open(argv[1]) as f:
            self.calls.append((argv, json.load(f)))
        end = time.time() + self.delay
        while time.time() < end and not stopped():
            time.sleep(0.01)
        return self.code, self.out


def sender(runner, twin=None, **kw):
    return LL.ExeSender(["sam.exe", "{json_file}", "{baud}", "{port}"], lambda: dict(HOME), twin=twin, baud=115200, port="COM5", runner=runner, **kw)


class ExeSenderTests(unittest.TestCase):
    def test_each_step_is_one_run_of_the_exe_with_his_start_end_json_and_the_arguments_in_order(self):
        r = Recorder()
        s = sender(r)
        done = []
        s.run([step(10, 250, 200, 0), step(10, 250, 150, -90, speed=10)], "j", lambda i, st: done.append(i))
        self.assertEqual(done, [0, 1])
        argv, msg = r.calls[0]
        self.assertEqual((argv[0], argv[2], argv[3]), ("sam.exe", "115200", "COM5"))
        self.assertEqual(msg["StartPoint"], {"x": 0.0, "y": 250.0, "z": 200.0, "toolDegrees": 0.0})
        self.assertEqual(msg["EndPoint"], {"x": 10, "y": 250, "z": 200, "toolDegrees": 0.0})
        self.assertEqual(r.calls[1][1]["StartPoint"], {"x": 10, "y": 250, "z": 200, "toolDegrees": 0.0})      # each move starts where the last one ended
        self.assertEqual((r.calls[1][1]["EndPoint"]["toolDegrees"], r.calls[1][1]["speed"]), (-90, 10))
        self.assertEqual(sorted(msg), ["AngleLimits", "EndPoint", "SamplesPerMM", "StartPoint", "Tool", "speed"])
        self.assertEqual(msg["Tool"], 0)
        self.assertEqual(msg["AngleLimits"]["S3"], {"Min": 0, "Max": 0})

    def test_the_temp_file_is_removed_afterwards(self):
        paths = []

        def runner(argv, t, stopped):
            paths.append(argv[1])
            return 0, ""
        sender(runner).run([step(10, 250, 200)], "j")
        self.assertFalse(os.path.exists(paths[0]))

    def test_the_twin_runs_the_same_steps(self):
        twin = FakeTwin()
        sender(Recorder(), twin).run([step(10, 250, 200), step(10, 250, 150)], "j")
        self.assertEqual([(t["x"], t["z"]) for t in twin.ran], [(10, 200), (10, 150)])

    def test_the_magnet_goes_in_as_Tool_0_or_1_and_a_grab_in_place_is_still_a_run(self):
        r = Recorder()
        sender(r).run([step(10, 250, 200), step(10, 250, 150), step(10, 250, 150, action="grab"), step(10, 250, 200), step(10, 250, 150), step(10, 250, 150, action="release")], "j")
        self.assertEqual([m["Tool"] for _, m in r.calls], [0, 0, 1, 1, 1, 0])

    def test_a_failing_exe_ends_the_run_with_its_output_and_stops_the_twin(self):
        twin = FakeTwin()
        with self.assertRaises(LL.BackendError) as cm:
            sender(Recorder(code=2, out="could not open COM5"), twin).run([step(10, 250, 200)], "j")
        self.assertEqual(cm.exception.code, "EXE_FAILED")
        self.assertIn("COM5", cm.exception.detail)
        self.assertTrue(twin.stopped)

    def test_a_timeout_is_a_soft_not_reached(self):
        with self.assertRaises(LL.BackendError) as cm:
            sender(lambda argv, t, stopped: (None, "")).run([step(10, 250, 200)], "j")
        self.assertEqual(cm.exception.code, "NOT_REACHED")

    def test_a_missing_exe_says_so(self):
        def runner(argv, t, stopped):
            raise FileNotFoundError(argv[0])
        with self.assertRaises(LL.BackendError) as cm:
            sender(runner).run([step(10, 250, 200)], "j")
        self.assertEqual(cm.exception.code, "EXE_MISSING")

    def test_stop_ends_the_run_and_the_twin(self):
        twin = FakeTwin()
        s = sender(Recorder(delay=5.0), twin)
        got = []
        t = threading.Thread(target=lambda: got.append(self._catch(s)), daemon=True)
        t.start()
        time.sleep(0.2)
        s.stop()
        t.join(5)
        self.assertEqual(got[0].code, "STOPPED")
        self.assertTrue(twin.stopped)

    def _catch(self, s):
        try:
            s.run([step(10, 250, 200)], "j")
        except LL.BackendError as e:
            return e

    def test_the_real_process_runner_starts_the_exe_with_the_json_file_baud_and_port(self):
        with tempfile.TemporaryDirectory() as d:
            fake = os.path.join(d, "fake_exe.py")
            out = os.path.join(d, "seen.json")
            with open(fake, "w") as f:
                f.write("import json,sys\nj=json.load(open(sys.argv[1]))\njson.dump([j['EndPoint']['x'],sys.argv[2],sys.argv[3]],open(%r,'w'))\n" % out)
            s = LL.ExeSender([sys.executable, fake, "{json_file}", "{baud}", "{port}"], lambda: dict(HOME), baud=9600, port="COM7")
            s.run([step(42, 250, 200)], "j")
            with open(out) as f:
                self.assertEqual(json.load(f), [42, "9600", "COM7"])
            bad = os.path.join(d, "bad.py")
            with open(bad, "w") as f:
                f.write("import sys\nprint('port busy')\nsys.exit(3)\n")
            with self.assertRaises(LL.BackendError) as cm:
                LL.ExeSender([sys.executable, bad], lambda: dict(HOME)).run([step(42, 250, 200)], "j")
            self.assertEqual(cm.exception.code, "EXE_FAILED")
            self.assertIn("port busy", cm.exception.detail)

    def test_server_exe_mode_builds_a_front_door_whose_state_is_the_twins(self):
        import server
        saved = dict(server._front)
        self.addCleanup(lambda: server._front.update(saved))
        server._front["door"], server._front["mode"] = server.make_front("exe", exe={"cmd": ["x", "{json_file}"], "port": "COM1"}), "exe"
        code, out = server._door("GET", "/arm/state")
        self.assertEqual((code, out["sender"]), (200, "exe"))

    def test_the_stand_in_exe_accepts_a_good_move_and_fails_an_unreachable_one_like_the_real_one_should(self):
        fake = os.path.join(ROOT, "tools", "fake_sam_exe.py")
        cmd = [sys.executable, fake, "{json_file}", "{baud}", "{port}", "--speedup", "100"]
        done = []
        LL.ExeSender(cmd, lambda: dict(HOME), port="COM5").run([step(20, 250, 200)], "j", lambda i, st: done.append(i))
        self.assertEqual(done, [0])
        with self.assertRaises(LL.BackendError) as cm:
            LL.ExeSender(cmd, lambda: dict(HOME), port="COM5").run([step(900, 900, 900)], "j")
        self.assertEqual(cm.exception.code, "EXE_FAILED")
        self.assertIn("out of reach", cm.exception.detail)
        with self.assertRaises(LL.BackendError) as cm:                          # a port that cannot be opened
            LL.ExeSender(cmd + ["--fail-port", "COM9"], lambda: dict(HOME), port="COM9").run([step(20, 250, 200)], "j")
        self.assertIn("COM9", cm.exception.detail)


if __name__ == "__main__":
    unittest.main()
