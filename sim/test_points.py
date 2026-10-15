"""The inverse kinematics explainer behind the Points page: both elbow solutions, why one is refused, which one the arm takes."""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from sim import armsim as A  # noqa: E402


def arm():
    return A.ArmSim(model="stock", patch=True)


class IkReportTest(unittest.TestCase):
    def test_a_reachable_point_has_a_solution_that_lands_on_it(self):
        r = arm().ik_report((100, 200, 100, 0))
        self.assertTrue(r["reachable"])
        self.assertLess(r["fk_error_mm"], 1e-3)                       # run forwards again, the solution lands on the point
        up = r["branches"][0]
        self.assertEqual((up["name"], up["ok"], r["chosen"]), ("elbow up", True, 1))

    def test_the_reported_angles_are_the_ones_the_arm_actually_uses(self):
        sim = arm()
        r = sim.ik_report((-100, 200, 100, 0))
        goal = A.inverse_kin((-100, 200, 100, 0), sim._t_ang, sim.limits, sim.tool)
        chosen = next(b for b in r["branches"] if b["branch"] == r["chosen"])
        for got, want in zip(chosen["angles"], goal):
            self.assertAlmostEqual(got, math.degrees(want), delta=0.06)

    def test_elbow_down_is_refused_by_the_joint_limits_and_says_which(self):
        down = arm().ik_report((0, 200, 250, 0))["branches"][1]
        self.assertFalse(down["ok"])
        self.assertIn("joint 3", down["why"])
        self.assertIn("limit is 0 to 180", down["why"])

    def test_too_far_is_refused_with_the_reach_in_the_message(self):
        r = arm().ik_report((0, 400, 100, 0))
        self.assertFalse(r["reachable"])
        self.assertIsNone(r["chosen"])
        self.assertIn("only allow 38 to 330 mm", r["branches"][0]["why"])

    def test_behind_the_base_is_refused_by_the_base_joint_limit(self):
        r = arm().ik_report((0, -100, 100, 0))
        self.assertFalse(r["reachable"])
        self.assertIn("joint 1", r["branches"][0]["why"])

    def test_narrower_limits_change_the_answer(self):
        sim = arm()
        sim.limits = [[0, 180], [0, 180], [0, 100], [0, 180]]                 # joint 3 may not bend past 100 degrees
        r = sim.ik_report((100, 200, 100, 0))                                # elbow up needs joint 3 at 57.8: still fine
        self.assertTrue(r["reachable"])
        sim.limits = [[0, 180], [0, 180], [0, 40], [0, 180]]
        r = sim.ik_report((100, 200, 100, 0))
        self.assertFalse(r["reachable"])
        self.assertIn("joint 3", r["branches"][0]["why"])

    def test_the_move_in_progress_is_reported(self):
        sim = arm()
        sim.set_move_speed(100)
        idle = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}
        self.assertIsNone(sim.snapshot()["linear"])
        sim.saved[:] = [(100.0, 200.0, 100.0, 0.0)]
        sim.start_replay(wait_arrival=True, once=True)
        for _ in range(60):
            sim.step(idle, 0.02)
        lin = sim.snapshot()["linear"]
        self.assertEqual(lin["to"][:3], [100.0, 200.0, 100.0])
        self.assertTrue(0.0 < lin["progress"] < 1.0)


if __name__ == "__main__":
    unittest.main()
