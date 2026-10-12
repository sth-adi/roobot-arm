"""The stock UWE-RArm firmware, and the damper/failsafe patch, against the Python model of them.

firmware/stock/src is his firmware untouched. This test builds its real main.cpp (setup() and loop(), with fake time, a
fake controller packet and recording servos), does the same with the patch applied, and checks that every servo
command and servo on/off state matches ArmSim(model="stock") tick for tick. Needs g++ and patch (skipped without).
"""
import math
import os
import shutil
import subprocess
import tempfile
import unittest

from armsim import ArmSim, forward_kin
from test_firmware import ticks

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "..", "firmware")
STUBS = os.path.join(FW, "rarm", "test", "stubs")
HARNESS = os.path.join(FW, "rarm", "test", "loop_harness.cpp")
PATCH = os.path.join(FW, "stock", "patches", "damper-and-failsafes.patch")


def build(src_dir, out):
    r = subprocess.run(["g++", "-std=c++17", "-I" + STUBS, "-I" + src_dir, HARNESS, "-o", out],
                       capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr)


class StockFirmware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (shutil.which("g++") and shutil.which("patch")):
            raise unittest.SkipTest("needs g++ and patch")
        cls.dir = tempfile.mkdtemp()
        cls.stock = os.path.join(cls.dir, "stock")
        cls.patched = os.path.join(cls.dir, "patched")
        build(os.path.join(FW, "stock", "src"), cls.stock)
        tree = os.path.join(cls.dir, "tree")
        shutil.copytree(os.path.join(FW, "stock", "src"), os.path.join(tree, "src"))
        r = subprocess.run(["patch", "-p1", "-d", tree, "-i", PATCH], capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError("patch does not apply to the stock firmware:\n" + r.stdout + r.stderr)
        build(os.path.join(tree, "src"), cls.patched)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def check(self, exe, patch, rate, script_ticks, skip_fresh=lambda n: False):
        sim = ArmSim(tool_mm=53.0, rate=rate, model="stock", patch=patch)
        lines, expected = [], []
        for n, (dt, (lsx, lsy, rsx, rsy, a, b)) in enumerate(script_ticks):
            fresh = 0 if skip_fresh(n) else 1
            lines.append("%g %g %g %g %g %d %d %g %d" % (dt, lsx, lsy, rsx, rsy, a, b, rate if patch else 0, fresh))
            sim.step({"LSX": lsx, "LSY": lsy, "RSX": rsx, "RSY": rsy, "A": bool(a), "B": bool(b)} if fresh else None, dt)
            expected.append((sim.servo_commands(), [1 if sim.attached else 0] * 4))
        r = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        got = [list(map(int, l.split())) for l in r.stdout.strip().splitlines()]
        self.assertEqual(len(got), len(expected))
        for n, (g, (servo, att)) in enumerate(zip(got, expected)):
            ctx = "tick %d: firmware %s vs model %s %s" % (n, g, servo, att)
            for i in range(4):
                self.assertLessEqual(abs(g[i] - servo[i]), 1, ctx)
            self.assertEqual(g[4:], att, ctx)

    def scenario(self):
        return list(ticks())

    def gap(self, n):
        return 400 <= n < 450             # 1 s with no packet while a stick is held

    def test_stock_firmware_matches_the_stock_model(self):
        self.check(self.stock, False, 0, self.scenario(), self.gap)

    def test_patched_firmware_matches_the_patched_model_at_30(self):
        self.check(self.patched, True, 30.0, self.scenario(), self.gap)

    def test_patched_firmware_matches_the_patched_model_slow(self):
        self.check(self.patched, True, 8.0, self.scenario(), self.gap)


class StockQuirksAndFailsafes(unittest.TestCase):
    """What differs, and that the patch fixes it. Runs on the Python model, which the tests above tie to the real code."""

    def sticks(self, sim, seconds, **kw):
        s = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False}
        s.update(kw)
        for _ in range(int(seconds / 0.02)):
            sim.step(s, 0.02)

    def home_then_nudge(self, patch):
        sim = ArmSim(tool_mm=53.0, rate=30.0, model="stock", patch=patch)
        self.sticks(sim, 4.0, LSX=1.0)                       # out to the side
        out = sim.pose()
        self.sticks(sim, 0.1, B=True)
        home = forward_kin(sim._home_angles(), sim.tool)
        self.sticks(sim, 8.0)                                # long enough for either firmware to finish going home
        self.sticks(sim, 0.06, LSX=1.0)                      # one small nudge afterwards
        return out, home, sim.pose()

    def test_stock_leaves_home_again_by_itself(self):
        """Stock writes its XYZ target to the servos every packet once "go home" is over, even with the sticks centred,
        and the target still holds the pre-home pose: 3 s after B the arm swings back to where it was."""
        out, home, after = self.home_then_nudge(False)
        self.assertAlmostEqual(after[0], out[0], delta=3)
        self.assertGreater(abs(after[0] - home[0]), 40)

    def test_patch_stays_at_home_and_released(self):
        out, home, after = self.home_then_nudge(True)
        self.assertLess(abs(after[0] - home[0]), 3)

    def test_stock_integrates_a_long_gap_in_one_go_patch_caps_it(self):
        for patch in (False, True):
            sim = ArmSim(tool_mm=53.0, rate=0, model="stock", patch=patch)
            s = {"LSX": 1.0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False}
            sim.step(s, 0.02)
            sim.step(None, 1.0)                              # a second with no packet
            sim.step(s, 0.02)
            moved = abs(sim._t_pos[0])                       # target x came from 0
            if patch:
                self.assertLessEqual(moved, 2.5)             # 100 ms at most
            else:
                self.assertGreater(moved, 19)                # the whole second, in one step

    def test_stock_gives_up_on_home_after_3s_patch_waits_for_arrival(self):
        for patch, gave_up in ((False, True), (True, False)):
            sim = ArmSim(tool_mm=53.0, rate=10.0, model="stock", patch=patch)
            self.sticks(sim, 4.0, LSX=1.0, LSY=0.5)          # a long way from home
            self.sticks(sim, 0.06, B=True)
            self.sticks(sim, 3.2)
            self.assertEqual(not sim._moving_home, gave_up, patch)

    def test_patch_limits_joint_speed(self):
        sim = ArmSim(tool_mm=53.0, rate=20.0, model="stock", patch=True)
        self.sticks(sim, 3.0, LSX=1.0)
        prev, worst = sim.angles, 0.0
        for _ in range(100):
            sim.step({"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": True}, 0.02)
            worst = max(worst, max(abs(math.degrees(sim.angles[i] - prev[i])) for i in range(4)) / 0.02)
            prev = sim.angles
        self.assertLessEqual(worst, 20.5)
        sim2 = ArmSim(tool_mm=53.0, rate=20.0, model="stock", patch=False)       # stock ignores the limit
        self.sticks(sim2, 3.0, LSX=1.0)
        before = sim2.angles
        sim2.step({"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": True}, 0.02)
        self.assertGreater(max(abs(math.degrees(sim2.angles[i] - before[i])) for i in range(4)) / 0.02, 100)

    def test_patch_pauses_sticks_while_the_arm_lags(self):
        sim = ArmSim(tool_mm=53.0, rate=5.0, model="stock", patch=True)         # very slow limit, fast target
        self.sticks(sim, 10.0, LSX=1.0, LSY=1.0)
        lag = max(abs(math.degrees(sim._t_ang[i] - sim.angles[i])) for i in range(4))
        self.assertLess(lag, 30)                                                  # bounded near the hold threshold
        nolimit = ArmSim(tool_mm=53.0, rate=5.0, model="stock", patch=False)
        self.sticks(nolimit, 10.0, LSX=1.0, LSY=1.0)
        self.assertGreater(abs(nolimit._t_pos[1]), abs(sim._t_pos[1]))


if __name__ == "__main__":
    unittest.main()
