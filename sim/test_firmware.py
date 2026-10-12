"""Differential test: the C++ motion code in firmware/rarm/src/ArmMotion.h against sim/armsim.py.

The same scripted inputs go to both; joint angles, mode, servo state and saved-position count must agree every tick.
Needs g++ (skipped without it). The ESP32 itself is not involved: Arduino/WiFi/servo headers are stubbed in
firmware/rarm/test/stubs, so this checks the logic, not the board.
"""
import os
import shutil
import subprocess
import tempfile
import unittest

from armsim import ArmSim

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "..", "firmware", "rarm")
MODES = ["idle", "manual", "home", "replay"]


def script():
    """(seconds, LSX, LSY, RSX, RSY, A, B) segments."""
    return [
        (0.5, 0, 0, 0, 0, 0, 0),
        (3.0, 0.8, 0, 0, 0, 0, 0),       # jog +x
        (0.1, 0, 0, 0, 0, 1, 0), (0.3, 0, 0, 0, 0, 0, 0),    # tap A: save
        (3.0, 0, -1.0, 0, 0.6, 0, 0),    # jog y and z
        (0.1, 0, 0, 0, 0, 1, 0), (0.3, 0, 0, 0, 0, 0, 0),    # save
        (2.0, 0, 0, 0.7, 0, 0, 0),       # tool angle
        (0.1, 0, 0, 0, 0, 1, 0), (0.3, 0, 0, 0, 0, 0, 0),    # save
        (6.0, 0, 0, 0, 0, 1, 0),         # hold A: replay
        (0.5, 0, 0, 0, 0, 0, 0),
        (0.1, 0, 0, 0, 0, 0, 1), (30.0, 0, 0, 0, 0, 0, 0),   # B: go home, then servos release
        (2.0, -0.6, 0.6, 0, 0, 0, 0),    # jog from home (attached again)
        (30.0, 0, 1.0, 0, 0, 0, 0),      # push out past the reach of the arm (IK fails, arm holds)
    ]


def ticks(dt=0.02):
    for seg in script():
        for _ in range(int(round(seg[0] / dt))):
            yield dt, seg[1:]


class FirmwareMatchesSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cxx = shutil.which("g++") or shutil.which("c++")
        if not cxx:
            raise unittest.SkipTest("no C++ compiler")
        cls.dir = tempfile.mkdtemp()
        cls.exe = os.path.join(cls.dir, "harness" + (".exe" if os.name == "nt" else ""))
        r = subprocess.run([cxx, "-std=c++17", "-Wall", "-Wextra", "-I" + os.path.join(FW, "test", "stubs"),
                            "-I" + os.path.join(FW, "src"), os.path.join(FW, "test", "motion_harness.cpp"),
                            "-o", cls.exe], capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def run_cpp(self, lines):
        r = subprocess.run([self.exe], input="\n".join(lines) + "\n", capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        return [l.split() for l in r.stdout.strip().splitlines()]

    def test_main_cpp_compiles_against_the_stubs(self):
        cxx = shutil.which("g++") or shutil.which("c++")
        r = subprocess.run([cxx, "-std=c++17", "-fsyntax-only", "-Wall", "-I" + os.path.join(FW, "test", "stubs"),
                            "-I" + os.path.join(FW, "src"), os.path.join(FW, "test", "syntax_main.cpp")],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_selftests_pass(self):
        r = subprocess.run([self.exe, "--selftest"], capture_output=True, text=True)
        self.assertEqual(r.stdout.strip(), "OK", r.stdout)

    def compare(self, rate, label):
        sim = ArmSim(tool_mm=63.0, rate=rate)
        lines, expected = [], []
        for n, (dt, (lsx, lsy, rsx, rsy, a, b)) in enumerate(ticks()):
            fresh = 0 if 400 <= n < 450 else 1               # 1 s with no fresh input in the middle of the first jog
            lines.append("%g %g %g %g %g %d %d %g %d" % (dt, lsx, lsy, rsx, rsy, a, b, rate, fresh))
            inp = {"LSX": lsx, "LSY": lsy, "RSX": rsx, "RSY": rsy, "A": bool(a), "B": bool(b)} if fresh else None
            sim.step(inp, dt)
            expected.append(([__import__("math").degrees(v) for v in sim.angles], sim.mode, sim.attached,
                             len(sim.saved)))
        got = self.run_cpp(lines)
        self.assertEqual(len(got), len(expected))
        worst = 0.0
        for n, (g, (ang, mode, att, saved)) in enumerate(zip(got, expected)):
            err = max(abs(float(g[i]) - ang[i]) for i in range(4))
            worst = max(worst, err)
            ctx = "%s tick %d: C++ %s vs sim %s" % (label, n, g, (ang, mode, att, saved))
            self.assertLess(err, 0.05, ctx)
            self.assertEqual(MODES[int(g[4])], mode, ctx)
            self.assertEqual(int(g[5]), 1 if att else 0, ctx)
            self.assertEqual(int(g[8]), saved, ctx)
        return worst

    def test_matches_at_30_deg_s(self):
        self.compare(30.0, "30 deg/s")

    def test_matches_when_slow(self):
        self.compare(8.0, "8 deg/s")

    def test_matches_unlimited(self):
        self.compare(0.0, "no limit")

    def test_scenario_exercises_everything(self):
        """Guard against the scenario silently not doing anything."""
        sim = ArmSim(tool_mm=63.0, rate=30.0)
        modes, saved_max, released, ik_failed = set(), 0, False, False
        for dt, (lsx, lsy, rsx, rsy, a, b) in ticks():
            sim.step({"LSX": lsx, "LSY": lsy, "RSX": rsx, "RSY": rsy, "A": bool(a), "B": bool(b)}, dt)
            modes.add(sim.mode)
            saved_max = max(saved_max, len(sim.saved))
            released = released or not sim.attached
            ik_failed = ik_failed or not sim.ik_ok
        self.assertEqual(modes, set(MODES))
        self.assertEqual(saved_max, 4)      # three taps, plus the press that starts the replay saves one too (as in the original)
        self.assertTrue(released)
        self.assertTrue(ik_failed)


if __name__ == "__main__":
    unittest.main()
