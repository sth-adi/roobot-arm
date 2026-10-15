"""Straight-line moves between sequence poses (UWE-RArm's CommandJointAnglesLinear, ported as a non-blocking move): the maths of the
move itself, and what the arm does with it."""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from sim import armsim as A  # noqa: E402

IDLE = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}


def run_sequence(poses, move_speed, rate=60.0, limit_s=120.0):
    """Replay `poses` once the way the hub does. Returns (arm, trace of (replay_idx, tip xyz, tip speed mm/s), seconds)."""
    sim = A.ArmSim(model="stock", patch=True, rate=rate)
    if move_speed:
        sim.set_move_speed(move_speed)
    sim.saved[:] = poses
    sim.start_replay(wait_arrival=True, once=True)
    trace, t, prev, quiet = [], 0.0, sim.pose()[:3], 0.0
    while t < limit_s:
        sim.step(IDLE, 0.02)
        t += 0.02
        p = sim.pose()[:3]
        trace.append((sim.replay_idx, p, math.dist(p, prev) / 0.02))
        prev = p
        done = sim.replay_idx >= len(poses) and sim._lin is None and \
            max(abs(math.degrees(sim._t_ang[i] - sim.angles[i])) for i in range(4)) < 0.6
        quiet = quiet + 0.02 if done else 0.0
        if quiet > 0.5:
            break
    return sim, trace, t


class LinearMoveTest(unittest.TestCase):
    def test_easing_has_zero_speed_at_both_ends_and_the_peak_slope_is_1_875(self):
        self.assertEqual((A.quintic(0.0), A.quintic(1.0)), (0.0, 1.0))
        h = 1e-6
        self.assertAlmostEqual((A.quintic(h) - A.quintic(0)) / h, 0.0, places=3)
        self.assertAlmostEqual((A.quintic(1) - A.quintic(1 - h)) / h, 0.0, places=3)
        self.assertAlmostEqual(max((A.quintic(i / 1e4 + 1e-4) - A.quintic(i / 1e4)) / 1e-4 for i in range(10000)), 1.875, places=2)

    def test_duration_makes_the_requested_speed_the_peak(self):
        m = A.LinearMove((0, 200, 100, 0), (300, 200, 100, 0), 100.0, 0)
        self.assertAlmostEqual(m.duration_ms, 1.875 * 300 * 10, places=3)        # 5625 ms for 300 mm at 100 mm/s peak
        pts = [m.at(t)[0][0] for t in range(0, 5626, 5)]
        peak = max((b - a) / 0.005 for a, b in zip(pts, pts[1:]))
        self.assertAlmostEqual(peak, 100.0, delta=1.0)

    def test_short_and_turn_only_moves_get_a_gentle_minimum(self):
        self.assertEqual(A.LinearMove((0, 200, 100, 0), (1, 200, 100, 0), 100.0, 0).duration_ms, 500.0)
        turn = A.LinearMove((0, 200, 100, 0), (0, 200, 100, 60), 100.0, 0)        # 60 degrees at a 30 deg/s peak
        self.assertAlmostEqual(turn.duration_ms, 1.875 * 60 * 1000 / 30.0, places=3)

    def test_it_starts_and_ends_exactly_on_the_poses(self):
        m = A.LinearMove((10, 200, 100, 5), (-80, 180, 60, 40), 100.0, 1000)
        self.assertEqual(m.at(1000)[0], (10, 200, 100, 5))
        pose, done = m.at(1000 + m.duration_ms + 50)
        self.assertEqual((pose, done), ((-80, 180, 60, 40), True))

    def test_the_path_is_a_straight_line(self):
        a, b = (0, 200, 100, 0), (200, 120, 300, 0)
        m = A.LinearMove(a, b, 100.0, 0)
        for t in range(0, int(m.duration_ms), 50):
            p = m.at(t)[0]
            to_start = [p[i] - a[i] for i in range(3)]
            ab = [b[i] - a[i] for i in range(3)]
            u = sum(to_start[i] * ab[i] for i in range(3)) / sum(v * v for v in ab)
            self.assertLess(math.dist(p[:3], [a[i] + u * ab[i] for i in range(3)]), 1e-9)


class ArmFollowsTheLineTest(unittest.TestCase):
    POSES = [(0.0, 246.7, 219.95, 0.0), (-200.0, 200.0, 180.0, 0.0), (150.0, 150.0, 60.0, 0.0), (0.0, 246.7, 219.95, 0.0)]

    def test_the_tool_stays_on_the_straight_line_between_poses(self):
        _, trace, _ = run_sequence(self.POSES, 100)
        worst = 0.0
        for j in range(1, len(self.POSES)):
            a, b = self.POSES[j - 1][:3], self.POSES[j][:3]
            ab = [b[i] - a[i] for i in range(3)]
            L2 = sum(v * v for v in ab)
            for k, p, _ in trace:
                if k != j + 1:
                    continue
                u = max(0.0, min(1.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / L2))
                worst = max(worst, math.dist(p, [a[i] + u * ab[i] for i in range(3)]))
        self.assertLess(worst, 4.0)                    # mm: the damper may lag the line a little, it never cuts a corner

    def test_the_old_joint_space_move_is_much_faster_at_the_tool(self):
        _, old, _ = run_sequence(self.POSES, 0)
        _, new, _ = run_sequence(self.POSES, 100)
        self.assertGreater(max(v for _, _, v in old), 2 * max(v for _, _, v in new))
        self.assertLess(max(v for _, _, v in new), 112.0)                    # the peak tool speed is what was asked for (plus damper lag catching up)

    def test_it_ends_on_the_last_pose(self):
        sim, _, _ = run_sequence(self.POSES, 100)
        for got, want in zip(sim.pose()[:3], self.POSES[-1][:3]):
            self.assertAlmostEqual(got, want, delta=1.5)

    def test_the_damper_still_caps_the_joints_underneath(self):
        sim = A.ArmSim(model="stock", patch=True, rate=10.0)             # a 10 deg/s damper under a fast 300 mm/s line
        sim.set_move_speed(300)
        sim.saved[:] = self.POSES
        sim.start_replay(wait_arrival=True, once=True)
        prev, worst = sim.servo_degrees(), 0.0
        for _ in range(1500):
            sim.step(IDLE, 0.02)
            cur = sim.servo_degrees()
            worst = max(worst, max(abs(a - b) for a, b in zip(cur, prev)) / 0.02)
            prev = cur
        self.assertLess(worst, 10.0 + 1.5)

    def test_home_and_stop_cancel_a_move_in_progress(self):
        sim = A.ArmSim(model="stock", patch=True, rate=60.0)
        sim.set_move_speed(100)
        sim.saved[:] = self.POSES
        sim.start_replay(wait_arrival=True, once=True)
        for _ in range(60):
            sim.step(IDLE, 0.02)
        self.assertIsNotNone(sim._lin)
        sim.stop_replay()
        self.assertIsNone(sim._lin)
        sim.start_replay(wait_arrival=True, once=True)
        for _ in range(60):
            sim.step(IDLE, 0.02)
        sim.go_home()
        self.assertIsNone(sim._lin)

    def test_a_line_that_leaves_the_workspace_goes_straight_to_the_pose_instead_of_stalling(self):
        # across the base: the straight line passes where the wrist cannot go
        poses = [(-230.0, 120.0, 100.0, 0.0), (230.0, 120.0, 100.0, 0.0)]
        sim, _, t = run_sequence(poses, 100, limit_s=60.0)
        self.assertLess(t, 59.0)                                        # it finished
        self.assertAlmostEqual(sim.pose()[0], 230.0, delta=2.0)

    def test_the_firmware_default_is_unchanged(self):
        self.assertEqual(A.ArmSim().move_mm_s, 0.0)                    # as shipped: every joint straight to the pose; the hub switches the line on
        with self.assertRaises(ValueError):
            A.ArmSim().set_move_speed(5)
        A.ArmSim().set_move_speed(0)


class WholeDegreeLimitTest(unittest.TestCase):
    def test_an_exactly_trained_limit_is_not_rejected_by_float_noise(self):
        limits = [[0, 180], [0, 180], [0, 180], [0, 115]]
        ok = tuple(math.radians(v) for v in (90, 90, 90, 115.00001))
        self.assertTrue(A._within_limits(ok, limits))                  # raw float comparison said 115.00001 > 115
        self.assertFalse(A._within_limits(tuple(math.radians(v) for v in (90, 90, 90, 116.0)), limits))
        self.assertEqual((A._lround(2.5), A._lround(-2.5), A._lround(2.4)), (3, -3, 2))      # lroundf: halves go away from zero


if __name__ == "__main__":
    unittest.main()
