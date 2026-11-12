"""dashboard/boxes.py: the only things we save about the workspace."""
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import boxes as B  # noqa: E402


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = os.path.join(self.dir.name, "boxes.json")
        self.s = B.Store(self.path)

    def test_starts_empty_and_says_what_is_missing(self):
        self.assertEqual(self.s.missing(), ["box 1", "box 2", "box 3", "tray"])
        with self.assertRaises(B.StoreError) as c:
            self.s.box(2)
        self.assertEqual(c.exception.code, "NOT_TAUGHT")
        self.assertTrue(c.exception.hint)

    def test_a_fresh_store_assumes_every_key_is_in_its_box(self):
        self.assertEqual(self.s.get_keys(), {"1": "box", "2": "box", "3": "box"})

    def test_where_a_key_is_survives_a_restart(self):
        self.s.set_key(2, "tray")
        self.s.set_key("3", "unknown")
        t = B.Store(self.path)
        self.assertEqual(t.get_keys(), {"1": "box", "2": "tray", "3": "unknown"})

    def test_an_older_file_with_no_keys_section_defaults_to_box(self):
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({"version": 1, "boxes": {k: None for k in B.KEYS}, "tray": None, "home": list(B.DEFAULT_HOME), "constants": {}}, f)
        t = B.Store(self.path)
        self.assertEqual(t.get_keys(), {"1": "box", "2": "box", "3": "box"})

    def test_bad_key_or_place_is_refused(self):
        with self.assertRaises(B.StoreError) as c:
            self.s.set_key(9, "tray")
        self.assertEqual(c.exception.code, "BAD_KEY")
        with self.assertRaises(B.StoreError) as c:
            self.s.set_key(1, "shelf")
        self.assertEqual(c.exception.code, "BAD_WHERE")

    def test_saved_values_survive_a_restart(self):
        self.s.set_box(2, 60, 250, 120, {"w": 40, "d": 40, "h": 50})
        self.s.set_tray(-80, 250, 100)
        self.s.set_constant("hover_mm", 25)
        t = B.Store(self.path)
        self.assertEqual(t.box(2)["x"], 60.0)
        self.assertEqual(t.box(2)["size"], {"w": 40.0, "d": 40.0, "h": 50.0})
        self.assertEqual(t.tray()["x"], -80.0)
        self.assertEqual(t.constants()["hover_mm"], 25.0)
        self.assertEqual(t.missing(), ["box 1", "box 3"])

    def test_teach_takes_the_pose_the_arm_reports(self):
        self.s.teach(1, {"x": 20.0, "y": 240.0, "z": 110.0, "a": 0.0, "age_s": 0.1})
        self.s.teach("tray", {"x": -50, "y": 230, "z": 100})
        self.assertEqual((self.s.box(1)["y"], self.s.tray()["z"]), (240.0, 100.0))

    def test_unreachable_or_nonsense_is_refused_and_nothing_is_saved(self):
        for args in ((1, 0, 900, 100), (1, 0, 0, 200), (1, float("nan"), 250, 100), (1, 0, 250, -5), (1, "5", 250, 100)):
            with self.assertRaises(B.StoreError, msg=repr(args)):
                self.s.set_box(*args)
        with self.assertRaises(B.StoreError):
            self.s.set_box(1, 60, 250, 120, {"w": 1, "d": 40, "h": 50})
        with self.assertRaises(B.StoreError):
            self.s.set_box(4, 60, 250, 120)
        self.assertFalse(os.path.exists(self.path))
        self.assertEqual(B.Store(self.path).missing()[0], "box 1")

    def test_the_hover_point_above_must_be_reachable_too(self):
        import planner as PL
        PL.plan((0, 250, 250, 0), (0, 330, 250, 0))                         # the point itself is reachable (this would raise otherwise)
        with self.assertRaises(B.StoreError) as c:
            self.s.set_box(1, 0, 330, 250)                                  # but not 30 mm higher, where the arm has to hover first
        self.assertEqual(c.exception.code, "UNREACHABLE")

    def test_constants_are_range_checked(self):
        for name, v in (("speed_mm_s", 500), ("hover_mm", -1), ("nope", 1), ("lift_mm", float("inf"))):
            with self.assertRaises(B.StoreError):
                self.s.set_constant(name, v)

    def test_obstacles_from_sizes(self):
        self.s.set_box(1, 60, 250, 120, {"w": 40, "d": 20, "h": 50})
        self.s.set_box(2, -60, 250, 120)                                    # no size: a point, not an obstacle
        self.assertEqual(self.s.obstacles(), [(40.0, 240.0, 70.0, 80.0, 260.0, 120.0)])

    def test_a_broken_file_is_reported_not_silently_replaced(self):
        with open(self.path, "w") as f:
            f.write("{not json")
        with self.assertRaises(B.StoreError) as c:
            B.Store(self.path)
        self.assertEqual(c.exception.code, "BAD_FILE")
        with open(self.path, "w") as f:
            json.dump({"version": 2}, f)
        with self.assertRaises(B.StoreError):
            B.Store(self.path)

    def test_the_write_is_atomic_no_temp_files_are_left(self):
        self.s.set_box(1, 60, 250, 120)
        self.assertEqual(os.listdir(self.dir.name), ["boxes.json"])


if __name__ == "__main__":
    unittest.main()
