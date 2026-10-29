"""dashboard/planner.py: Sam's inverse kinematics and packet format, checked before anything moves."""
import math
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import planner as PL  # noqa: E402



def _load_sam():
    """Sam's kinematics and packet code (firmware/sam/backend), imported under a temporary path so its package names (models, motion) do not linger."""
    import types
    path = os.path.join(ROOT, "firmware", "sam", "backend")
    saved = {k: sys.modules.pop(k) for k in list(sys.modules) if k.split(".")[0] in ("models", "motion", "GlobalVariables")}
    sys.path.insert(0, path)
    try:
        from models.robot import Coordinates
        from motion.kinematics import InverseKim
        from motion.packets import CompileAnglePacket
    finally:
        sys.path.remove(path)
        for k in [k for k in sys.modules if k.split(".")[0] in ("models", "motion", "GlobalVariables")]:
            del sys.modules[k]
        sys.modules.update(saved)
    return types.SimpleNamespace(Coordinates=Coordinates, InverseKim=InverseKim, CompileAnglePacket=CompileAnglePacket)


SAM = _load_sam()

HOME = (0.0, 250.0, 250.0, 0.0)


def fk(q, tool):
    """Tool tip (x, y, z, pitch degrees) for joint angles in degrees; the maths of firmware/sam ForwardKinTransform.h with a chosen tool length."""
    s1, s2, s3, s4 = [math.radians(v) for v in q]
    base, forearm, tool_a = s1 - math.pi / 2, s2 + s3 - math.pi, s2 + s3 + s4 - 1.5 * math.pi
    er, ez = 145.95 * math.cos(s2), 145.95 * math.sin(s2) + 74.0
    wr, wz = er + 183.70 * math.cos(forearm), ez + 183.70 * math.sin(forearm)
    tr = wr + tool * math.cos(tool_a)
    return tr * math.sin(base), tr * math.cos(base), wz + tool * math.sin(tool_a), math.degrees(tool_a)


def parse(lines):
    return [[float(v) for v in l.split("/")] for l in lines]


class SameAsSam(unittest.TestCase):
    def test_inverse_kinematics_matches_his_code(self):
        rnd, n = random.Random(7), 0
        for _ in range(4000):
            x, y, z, a = rnd.uniform(-200, 200), rnd.uniform(20, 330), rnd.uniform(0, 300), rnd.uniform(-1.0, 1.0)
            try:
                mine = PL.inverse_kim(x, y, z, a)
            except PL.PlanError:
                with self.assertRaises((ValueError, ZeroDivisionError)) if math.hypot(x, y) >= PL.AXIS_MIN_MM else self.assertRaises(Exception):
                    SAM.InverseKim(SAM.Coordinates(x, y, z, a))                      # where mine refuses, his fails too (or the point is on the axis)
                continue
            t = SAM.InverseKim(SAM.Coordinates(x, y, z, a))
            for got, want in zip(mine, (t.S1, t.S2, t.S3, t.S4)):
                self.assertAlmostEqual(got, math.degrees(want), places=9)
            n += 1
        self.assertGreater(n, 500)

    def test_packet_format_is_his(self):
        out = PL.plan(HOME, (0.0, 250.0, 150.0, 0.0))
        first = parse(out["lines"])[0]
        t = SAM.InverseKim(SAM.Coordinates(*HOME))
        self.assertEqual(first[0], 0)
        for got, want in zip(first[1:], (t.S1, t.S2, t.S3, t.S4)):
            self.assertAlmostEqual(got, math.degrees(want), places=2)
        for l in out["lines"]:
            self.assertEqual(len(l.split("/")), 5)


class PlanQuality(unittest.TestCase):
    def test_ends_where_asked_and_time_only_goes_forward(self):
        out = PL.plan(HOME, (60.0, 250.0, 150.0, 0.0))
        rows = parse(out["lines"])
        end = SAM.InverseKim(SAM.Coordinates(60.0, 250.0, 150.0, 0.0))
        for got, want in zip(rows[-1][1:], (end.S1, end.S2, end.S3, end.S4)):
            self.assertAlmostEqual(got, math.degrees(want), places=2)
        self.assertTrue(all(b[0] >= a[0] for a, b in zip(rows, rows[1:])))
        self.assertEqual(rows[-1][0], round(out["duration"] * 1000))

    def test_live_packets_are_short_and_constant_speed_and_say_where_they_end(self):
        goal = (100.0, 250.0, 250.0, 0.0)
        out = PL.plan(HOME, goal, speed_mm_s=35, live=True, chunk_s=0.25)
        self.assertAlmostEqual(out["duration"], 0.25, places=2)
        self.assertAlmostEqual(math.dist(HOME[:3], out["end"][:3]), 35 * 0.25, places=3)      # a quarter of a second of the way, on the straight line
        steps = [b[1] - a[1] for a, b in zip(parse(out["lines"]), parse(out["lines"])[1:])]
        self.assertLess(max(steps) - min(steps), 0.2 * (sum(steps) / len(steps)))           # no ease in or out
        whole = PL.plan(HOME, goal, speed_mm_s=35)
        self.assertGreater(whole["duration"], 2.0)                                           # the default is unchanged
        near = PL.plan(HOME, (3.0, 250.0, 250.0, 0.0), speed_mm_s=35, live=True)
        self.assertEqual(near["end"], (3.0, 250.0, 250.0, 0.0))                              # a target closer than a chunk is reached in one packet

    def test_fixed_update_rate(self):
        out = PL.plan(HOME, (0.0, 250.0, 150.0, 0.0), rate_hz=50)
        gaps = {round(b[0] - a[0]) for a, b in zip(parse(out["lines"]), parse(out["lines"])[1:])}
        self.assertTrue(gaps <= {19, 20, 21}, gaps)

    def test_joint_speed_never_above_the_limit_and_whole_move_slows_down(self):
        fast = PL.plan((-100, 250, 150, 0), (100, 250, 150, 0), speed_mm_s=500, max_joint_dps=30)
        rows = parse(fast["lines"])
        for a, b in zip(rows, rows[1:]):
            dt = (b[0] - a[0]) / 1000.0
            for j in range(1, 5):
                self.assertLessEqual(abs(b[j] - a[j]) / dt, 30 * 1.06 + 1.0)       # rounding of the time stamp
        slow = PL.plan((-100, 250, 150, 0), (100, 250, 150, 0), speed_mm_s=500, max_joint_dps=10)
        self.assertGreater(slow["duration"], fast["duration"] * 2.5)             # a lower joint limit stretches the move
        self.assertGreater(fast["duration"], 200 / 500.0)                          # and 500 mm/s was not kept

    def test_cartesian_speed_is_kept_when_joints_allow(self):
        out = PL.plan(HOME, (0.0, 250.0, 200.0, 0.0), speed_mm_s=10, max_joint_dps=60)
        self.assertAlmostEqual(out["duration"], 5.0, delta=0.3)

    def test_path_stays_straight(self):
        start, end = (-60.0, 250.0, 150.0, 0.0), (60.0, 250.0, 100.0, 0.0)
        rows = parse(PL.plan(start, end, speed_mm_s=60)["lines"])
        d = [e - s_ for s_, e in zip(start[:3], end[:3])]
        n = math.sqrt(sum(c * c for c in d))
        for r in rows:
            tip = fk(r[1:], 63.0)
            v = [t - s_ for t, s_ in zip(tip[:3], start[:3])]
            along = sum(a * b for a, b in zip(v, d)) / n
            off = math.sqrt(max(0.0, sum(c * c for c in v) - along * along))
            self.assertLess(off, 0.05)                          # the tip never leaves the line
            self.assertTrue(-0.05 <= along <= n + 0.05)

    def test_starts_and_stops_gently(self):
        rows = parse(PL.plan(HOME, (0.0, 250.0, 100.0, 0.0), speed_mm_s=30)["lines"])
        mid = max(abs(b[j] - a[j]) for a, b in zip(rows[len(rows) // 2 - 1:], rows[len(rows) // 2:]) for j in range(1, 5))
        first = max(abs(rows[1][j] - rows[0][j]) for j in range(1, 5))
        last = max(abs(rows[-1][j] - rows[-2][j]) for j in range(1, 5))
        self.assertLess(first, mid * 0.15)
        self.assertLess(last, mid * 0.15)

    def test_no_movement(self):
        self.assertEqual(PL.plan(HOME, HOME)["lines"], [])


class Refusals(unittest.TestCase):
    def code(self, *a, **k):
        with self.assertRaises(PL.PlanError) as c:
            PL.plan(*a, **k)
        return c.exception.code

    def test_unreachable_end(self):
        self.assertEqual(self.code(HOME, (0, 600, 250, 0)), "UNREACHABLE")

    def test_unreachable_in_the_middle_of_the_line(self):
        wide = [[-720, 720]] * 4
        self.assertEqual(self.code((-100, 100, 150, 0), (100, -100, 150, 0), limits=wide), "AXIS_SINGULARITY")     # the line passes through the base axis

    def test_outside_the_calibrated_range(self):
        tight = [[80, 100], [0, 131], [154, 21], [0, 180]]
        self.assertEqual(self.code(HOME, (120, 250, 250, 0), limits=tight), "JOINT_LIMIT")

    def test_base_axis(self):
        self.assertEqual(self.code(HOME, (0, 0, 250, 0)), "AXIS_SINGULARITY")
        self.assertEqual(self.code((0, 250, 150, 0), (0, 5, 150, 0)), "AXIS_SINGULARITY")

    def test_reversed_limits_are_an_inverted_servo_not_an_empty_range(self):
        out = PL.plan(HOME, (0.0, 250.0, 150.0, 0.0), limits=[[15, 180], [0, 131], [154, 21], [0, 180]])
        self.assertTrue(out["lines"])

    def test_bad_inputs(self):
        self.assertEqual(self.code(HOME, (float("nan"), 250, 150, 0)), "BAD_POINT")
        self.assertEqual(self.code(HOME, (0, 250, 150, 0), speed_mm_s=0), "BAD_SETTING")
        self.assertEqual(self.code(HOME, (0, 250, 150, 0), max_joint_dps=-5), "BAD_SETTING")
        self.assertEqual(self.code(HOME, (0, 250, 150)), "BAD_POINT")

    def test_errors_carry_the_dashboard_fields(self):
        try:
            PL.plan(HOME, (0, 600, 250, 0))
        except PL.PlanError as e:
            d = e.as_dict()
            self.assertEqual(set(d), {"code", "title", "detail", "hint"})
            self.assertTrue(d["title"] and d["hint"])


if __name__ == "__main__":
    unittest.main()
