"""dashboard/routes.py: routes calculated from the stored coordinates, checked against box sizes and the planner."""
import math
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import boxes as B  # noqa: E402
import routes as R  # noqa: E402

HOME = {"x": 0, "y": 250, "z": 250, "angle": 0}
SIZE = {"w": 40, "d": 40, "h": 50}


def store():
    d = tempfile.mkdtemp()
    s = B.Store(os.path.join(d, "b.json"))
    s.set_box(1, -60, 250, 120, SIZE)
    s.set_box(2, 0, 250, 120, SIZE)
    s.set_box(3, 60, 250, 120, SIZE)
    s.set_tray(0, 180, 60, {"w": 80, "d": 60, "h": 30})
    return s


class Geometry(unittest.TestCase):
    def test_segment_box(self):
        box = (0, 0, 0, 10, 10, 10)
        self.assertTrue(R.segment_hits_box((-5, 5, 5), (15, 5, 5), box))
        self.assertFalse(R.segment_hits_box((-5, 20, 5), (15, 20, 5), box))
        self.assertTrue(R.segment_hits_box((5, 5, 20), (5, 5, 5), box))          # straight down into it
        self.assertFalse(R.segment_hits_box((5, 5, 20), (5, 5, 11), box))        # stops above it
        self.assertTrue(R.segment_hits_box((5, 5, 5), (5, 5, 5), box))           # a point inside


class FetchAndReturn(unittest.TestCase):
    def test_fetch_goes_up_across_down_grabs_carries_releases_and_goes_home(self):
        s = store()
        r = R.fetch_route(s, 2, HOME)
        acts = [(i, st["action"]) for i, st in enumerate(r["steps"]) if "action" in st]
        self.assertEqual([a for _, a in acts], ["grab", "release"])
        grab, rel = (r["steps"][i] for i, _ in acts)
        self.assertEqual((grab["x"], grab["y"], grab["z"]), (0.0, 250.0, 120.0))        # exactly the stored coordinate
        self.assertEqual((rel["x"], rel["y"]), (0.0, 180.0))                            # over the tray
        self.assertEqual(r["steps"][-1]["label"], "down to home")
        self.assertGreater(r["seconds"], 0)

    def test_return_is_the_other_way(self):
        r = R.return_route(store(), 1, HOME)
        acts = [st for st in r["steps"] if "action" in st]
        self.assertEqual([(a["action"], a["x"], a["y"]) for a in acts], [("grab", 0.0, 180.0), ("release", -60.0, 250.0)])

    def test_all_sideways_movement_is_above_every_box(self):
        s = store()
        r = R.fetch_route(s, 3, {"x": 0, "y": 250, "z": 160, "angle": 0})
        tops = max(p["z"] for p in s.data["boxes"].values()) + s.constants()["safe_clearance_mm"]
        prev = (0, 250, 160)
        for st in r["steps"]:
            cur = (st["x"], st["y"], st["z"])
            if math.hypot(cur[0] - prev[0], cur[1] - prev[1]) > 1:
                self.assertGreaterEqual(min(cur[2], prev[2]), tops - 1e-6, st["label"])
            prev = cur

    def test_a_box_in_the_way_of_the_descent_is_refused(self):
        s = store()
        s.data["boxes"]["2"]["size"] = {"w": 200, "d": 40, "h": 50}     # box 2 is so wide that box 3's column goes through it
        with self.assertRaises(R.RouteError) as c:
            R.fetch_route(s, 3, HOME)
        self.assertEqual(c.exception.code, "COLLISION")
        self.assertIn("box 2", c.exception.title)

    def test_starting_inside_a_box_is_refused_for_lifting(self):
        s = store()
        with self.assertRaises(R.RouteError):
            R.lift_route(s, {"x": 0, "y": 250, "z": 100, "angle": 0})   # inside box 2's body: lifting would scrape it

    def test_missing_teaching_is_a_clear_error(self):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        s = B.Store(os.path.join(d.name, "b.json"))
        with self.assertRaises(R.RouteError) as c:
            R.fetch_route(s, 1, HOME)
        self.assertEqual(c.exception.code, "NOT_TAUGHT")

    def test_unknown_position_is_refused(self):
        with self.assertRaises(R.RouteError) as c:
            R.fetch_route(store(), 1, None)
        self.assertEqual(c.exception.code, "NO_POSITION")

    def test_unreachable_leg_is_found_before_moving(self):
        s = store()
        s.data["boxes"]["2"]["y"] = 600.0               # bypass the save-time check: the dry run must still catch it
        with self.assertRaises(R.RouteError) as c:
            R.fetch_route(s, 2, HOME)
        self.assertIn(c.exception.code, ("UNREACHABLE", "JOINT_LIMIT"))


class Recovery(unittest.TestCase):
    def test_lift_is_straight_up_only(self):
        s = store()
        r = R.lift_route(s, {"x": 0, "y": 250, "z": 130, "angle": 0})
        self.assertEqual(len(r["steps"]), 1)
        st = r["steps"][0]
        self.assertEqual((st["x"], st["y"]), (0.0, 250.0))
        self.assertGreater(st["z"], 130)

    def test_lift_when_already_clear_does_nothing(self):
        self.assertEqual(R.lift_route(store(), HOME)["steps"], [])

    def test_home_route_ends_at_home(self):
        r = R.home_route(store(), {"x": 0, "y": 180, "z": 90, "angle": 0})
        self.assertEqual([(s["x"], s["y"], s["z"]) for s in r["steps"]][-1], (0.0, 250.0, 250.0))


if __name__ == "__main__":
    unittest.main()
