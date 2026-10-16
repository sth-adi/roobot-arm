"""The sketch's mirror-mode inverse kinematics (arduino/arm_receiver/arm_ik.h) against the model in sim/armsim.py.

The board runs Sam's own IK, unchanged: the three headers copied into the sketch folder must stay identical to firmware/stock/src, and
a pose sent as "#x,y,z,angle" must give the same servo degrees as the model, with the real arm's calibration (joint 3 reversed).
Needs g++ (skipped without).
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from emu.emulator import EXE, compiler  # noqa: E402
import sim.sequences as SEQ  # noqa: E402
from sim.armsim import ArmSim, inverse_kin  # noqa: E402

SKETCH = os.path.join(ROOT, "arduino", "arm_receiver")
STOCK = os.path.join(ROOT, "firmware", "stock", "src")
STUBS = os.path.join(ROOT, "firmware", "rarm", "test", "stubs")
HARNESS = os.path.join(HERE, "fixtures", "ik_board_harness.cpp")
REAL_LIMS = [[11, 180], [0, 115], [161, 21], [0, 180]]          # the real arm's calibration (joint 3 reversed); homes below
REAL_HOMES = [95, 115, 27, 127]
POINTS = [(-100, 200, 100), (0, 200, 100), (0, 200, 300), (0, 200, 100), (100, 200, 100)]   # Sam's demo points A to E


IDLE = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}


def points_loop_trace():
    """The virtual arm, calibrated like the real one, runs Sam's five default points once the way the hub does. Returns [(tool-tip pose, servo degrees)] per 20 ms tick."""
    sim = ArmSim(tool_mm=53.0, model="stock", patch=True)
    sim.set_calibration(REAL_LIMS, REAL_HOMES)
    sim.set_move_speed(100.0)
    seq = SEQ.validate({"format": SEQ.FORMAT, "poses": [{"x": x, "y": y, "z": z, "angle": 0.0, "action": None} for x, y, z in POINTS]})
    saved, _ = SEQ.to_saved(seq)
    sim.stop_replay()
    sim.saved[:] = saved
    sim.saved_speed[:] = SEQ.speeds_of(seq)
    sim.start_replay(wait_arrival=True, once=True)
    trace, t, quiet = [], 0.0, 0.0
    while t < 120.0:
        sim.step(IDLE, 0.02)
        t += 0.02
        trace.append((tuple(sim.pose()), sim.servo_degrees()))
        done = sim.replay_idx >= len(saved) and sim._lin is None and max(abs(math.degrees(sim._t_ang[i] - sim.angles[i])) for i in range(4)) < 0.6
        quiet = quiet + 0.02 if done else 0.0
        if quiet > 0.5:
            break
    return trace


def expected(poses, lims, homes):
    """What the model says for the same poses in order: servo degrees, or None when unreachable (the elbow choice follows the last solution)."""
    prev = [math.radians(h) for h in homes]
    out = []
    for x, y, z, a in poses:
        r = inverse_kin((x, y, z, a), prev, lims, 53.0)
        if r is None:
            out.append(None)
            continue
        prev = list(r)
        out.append([180.0 - min(180.0, max(0.0, math.degrees(v))) if lims[i][0] > lims[i][1] else min(180.0, max(0.0, math.degrees(v)))
                    for i, v in enumerate(r)])
    return out


class SameAsSam(unittest.TestCase):
    def test_the_sketch_uses_sams_files_unchanged(self):
        for name in ("InverseKinTransform.h", "SticksToXYZ.h", "RobotCharacteristics.h"):
            with open(os.path.join(SKETCH, name), "rb") as a, open(os.path.join(STOCK, name), "rb") as b:
                self.assertEqual(a.read(), b.read(), name + " in the sketch has drifted from firmware/stock/src: copy it again")


class OnBoardIk(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cc = compiler()
        except RuntimeError:
            raise unittest.SkipTest("needs g++")
        cls.dir = tempfile.mkdtemp()
        cls.exe = os.path.join(cls.dir, "ik_board" + EXE)
        r = subprocess.run([cc, "-std=c++17", "-I" + STUBS, "-I" + SKETCH, HARNESS, "-o", cls.exe], capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def run_board(self, lims, homes, poses, cal=True):
        lines = []
        if cal:
            lines.append("cal " + " ".join(str(v) for p in lims for v in p) + " " + " ".join(str(h) for h in homes))
        lines += ["pose %g %g %g %g" % p for p in poses]
        r = subprocess.run([self.exe], input="\n".join(lines) + "\n", capture_output=True, text=True)
        out = r.stdout.splitlines()
        if cal:
            self.assertEqual(out[0], "cal ok")
            out = out[1:]
        return [None if l == "fail" else [float(v) for v in l.split()] for l in out]

    def check(self, lims, homes, poses):
        got, want = self.run_board(lims, homes, poses), expected(poses, lims, homes)
        self.assertEqual(len(got), len(want))
        for g, w, p in zip(got, want, poses):
            if w is None:
                self.assertIsNone(g, "board solved %s, the model says unreachable" % (p,))
            else:
                self.assertIsNotNone(g, "board says unreachable for %s, the model solved it" % (p,))
                for a, b in zip(g, w):
                    self.assertAlmostEqual(a, b, delta=0.01, msg=str(p))

    def test_default_points_with_the_real_calibration(self):
        poses = [(x, y, z, 0.0) for x, y, z in POINTS]
        self.check(REAL_LIMS, REAL_HOMES, poses)
        got = self.run_board(REAL_LIMS, REAL_HOMES, poses)
        self.assertTrue(all(g is not None for g in got), "every default point must be reachable")
        self.assertAlmostEqual(got[0][2], 118.1, delta=0.1)       # joint 3 is reversed: 61.9 logical, 118.1 on the servo

    def test_the_board_follows_the_whole_default_points_loop_exactly_as_the_virtual_arm_does(self):
        """The hub streams the tip pose of the virtual arm; the board's own IK must land on the joint angles the virtual arm has, every tick."""
        trace = points_loop_trace()
        self.assertGreater(len(trace), 500)
        got = self.run_board(REAL_LIMS, REAL_HOMES, [tuple(round(v, 4) for v in pose) for pose, _ in trace])
        worst = 0.0
        for g, (pose, servo) in zip(got, trace):
            self.assertIsNotNone(g, "board could not solve %s" % (pose,))
            worst = max(worst, max(abs(a - b) for a, b in zip(g, servo)))
        self.assertLess(worst, 0.06, "the board and the virtual arm differ by %.3f degrees somewhere in the loop" % worst)
        self.assertGreater(max(s[2] for _, s in trace) - min(s[2] for _, s in trace), 60)       # joint 3 (reversed) really swings through the loop

    def test_a_walk_in_small_steps_agrees_throughout(self):
        poses = [(0.0, 200.0, 100.0 + 2.0 * i, 0.0) for i in range(100)] + [(-60.0 + 1.5 * i, 200.0, 300.0, 0.0) for i in range(80)]
        self.check(REAL_LIMS, REAL_HOMES, poses)

    def test_with_the_default_calibration_too(self):
        self.check([[0, 180]] * 4, [90] * 4, [(x, y, z, 0.0) for x, y, z in POINTS] + [(0.0, 220.0, 150.0, 20.0)])

    def test_unreachable_and_out_of_limits_poses_fail(self):
        far = [(0.0, 500.0, 100.0, 0.0), (0.0, 200.0, 900.0, 0.0), (0.0, 0.0, 0.0, 0.0)]
        self.assertEqual(self.run_board(REAL_LIMS, REAL_HOMES, far), [None, None, None])
        self.check(REAL_LIMS, REAL_HOMES, far)

    def test_no_pose_is_followed_before_a_calibration_arrives(self):
        self.assertEqual(self.run_board(REAL_LIMS, REAL_HOMES, [(0.0, 200.0, 100.0, 0.0)], cal=False), [None])

    def test_a_bad_calibration_is_refused(self):
        for bad in ("cal 0 0 0 180 0 180 0 180 90 90 90 90", "cal 0 181 0 180 0 180 0 180 90 90 90 90", "cal 0 180 0 180 0 180 0 180 90 90 90 200"):
            r = subprocess.run([self.exe], input=bad + "\n", capture_output=True, text=True)
            self.assertEqual(r.stdout.strip(), "cal refused", bad)


if __name__ == "__main__":
    unittest.main()
