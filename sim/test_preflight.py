import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
import preflight as P  # noqa: E402

FWD = {"requires": {"key_in": "box2"}, "ends": {"key_on": "tray"}}
REV = {"requires": {"key_on": "tray"}, "ends": {"key_in": "box2"}}
PLAN = {"sequence": "box2_to_tray"}


def st(**keys):
    return {"keys": {k[1:]: v for k, v in keys.items()}, "holding": False}


class PreflightTest(unittest.TestCase):
    def code(self, req, seq, state):
        r = P.check(req, PLAN, seq, state)
        return r[0]["code"] if r else None

    def test_fine(self):
        self.assertIsNone(self.code("key 2 to the tray", FWD, st(k1="box 1", k2="box 2", k3="box 3")))

    def test_key_already_moved(self):
        self.assertEqual(self.code("key 2 from the box to the tray", FWD, st(k1="box 1", k2="tray", k3="box 3")), "ALREADY_DONE")

    def test_key_in_wrong_box(self):
        self.assertEqual(self.code("key 2 from box 2 to the tray", FWD, st(k1="box 1", k2="box 3", k3="box 2")), "KEY_NOT_AT_SOURCE")

    def test_no_key_at_source(self):
        self.assertEqual(self.code("fetch the key", FWD, st(k1="box 1", k2="tray", k3="box 3")), "KEY_NOT_AT_SOURCE")

    def test_unknown_key(self):
        self.assertEqual(self.code("key 9 to the tray", FWD, st(k1="box 1", k2="box 2", k3="box 3")), "UNKNOWN_KEY")

    def test_floor_and_magnet(self):
        self.assertEqual(self.code("key 2 to the tray", FWD, st(k1="box 1", k2="floor", k3="box 3")), "KEY_ON_FLOOR")
        self.assertEqual(self.code("key 2 to the tray", FWD, st(k1="box 1", k2="on the magnet", k3="box 3")), "KEY_ON_MAGNET")

    def test_reverse_needs_key_on_tray(self):
        self.assertEqual(self.code("put key 2 back", REV, st(k1="box 1", k2="box 2", k3="box 3")), "ALREADY_DONE")
        self.assertIsNone(self.code("put key 2 back", REV, st(k1="box 1", k2="tray", k3="box 3")))

    def test_holding(self):
        s = st(k1="box 1", k2="box 2", k3="box 3"); s["holding"] = True
        self.assertEqual(self.code("fetch the key", FWD, s), "HOLDING")

    def test_scope(self):
        self.assertEqual(P.no_sequence("make me a coffee", {})["code"], "OUT_OF_SCOPE")
        self.assertEqual(P.no_sequence("put the key in box 3", {})["code"], "NOT_TAUGHT")

    def test_verify(self):
        before = st(k1="box 1", k2="box 2", k3="box 3")
        self.assertIsNone(P.verify(FWD, before, st(k1="box 1", k2="tray", k3="box 3"), "key 2 to the tray", PLAN))
        self.assertEqual(P.verify(FWD, before, st(k1="box 1", k2="floor", k3="box 3"), "key 2 to the tray", PLAN)["code"], "MOVE_FAILED")
        self.assertEqual(P.verify(FWD, before, before, "", PLAN)["code"], "MOVE_FAILED")


class QuickCommandTest(unittest.TestCase):
    def act(self, text):
        r = P.keyword_action(text)
        return (r["action"], r.get("on")) if r else None

    def test_commands(self):
        self.assertEqual(self.act("go to home position"), ("go_home", None))
        self.assertEqual(self.act("return to the default pose"), ("go_home", None))
        self.assertEqual(self.act("stop"), ("stop", None))
        self.assertEqual(self.act("STOP!! halt now"), ("stop", None))
        self.assertEqual(self.act("turn the magnet on"), ("magnet", True))
        self.assertEqual(self.act("switch magnet off"), ("magnet", False))
        self.assertEqual(self.act("release it"), ("magnet", False))
        self.assertEqual(self.act("where is key 2?"), ("status", None))

    def test_key_moves_are_not_quick_commands(self):
        self.assertIsNone(self.act("put key 2 on the tray"))
        self.assertIsNone(self.act("take the key from box 1 back home"))     # a key move, not "go home"
        self.assertIsNone(self.act("bring me the key?"))

    def test_stop_is_decided_locally(self):
        self.assertTrue(P.is_stop("please stop"))
        self.assertFalse(P.is_stop("go home"))

    def test_describe_uses_only_the_state(self):
        s = P.describe({"at_home": True, "magnet": False, "holding": False, "keys": {"1": "box 1", "2": "tray"}})
        self.assertIn("At home", s)
        self.assertIn("key 2 on the tray", s)

    def test_moving_the_real_arm_needs_the_output_armed(self):
        self.assertEqual(P.check_action({"action": "go_home"}, {}, False)["code"], "OUTPUT_SAFE")
        self.assertIsNone(P.check_action({"action": "go_home"}, {}, True))
        self.assertIsNone(P.check_action({"action": "status"}, {}, False))


class FreeMovementTest(unittest.TestCase):
    POSE = {"x": 0.0, "y": 200.0, "z": 100.0, "angle": 0.0}

    def test_parse(self):
        self.assertEqual(P.parse_move("move up 2 cm"), {"dx": 0.0, "dy": 0.0, "dz": 20.0})
        self.assertEqual(P.parse_move("go left 5mm and down 1 cm"), {"dx": -5.0, "dy": 0.0, "dz": -10.0})
        self.assertEqual(P.parse_move("raise 30 millimetres"), {"dx": 0.0, "dy": 0.0, "dz": 30.0})

    def test_needs_a_number_and_unit_and_no_key_words(self):
        self.assertIsNone(P.parse_move("go back"))
        self.assertIsNone(P.parse_move("move up a bit"))
        self.assertIsNone(P.parse_move("move up 2"))                                   # no unit: ambiguous, never guessed
        self.assertIsNone(P.parse_move("put the key back"))
        self.assertIsNone(P.parse_move("move the key 2 cm to the left"))               # keys only ever use taught sequences
        self.assertIsNone(P.parse_move("bring key 2 back from the tray, up 3 cm"))

    def test_checks(self):
        ok = lambda *a: True
        self.assertIsNone(P.check_move({"dx": 0, "dy": 0, "dz": 20}, self.POSE, ok))
        self.assertEqual(P.check_move({"dx": 0, "dy": 0, "dz": 0.2}, self.POSE, ok)["code"], "NOTHING_TO_MOVE")
        self.assertEqual(P.check_move({"dx": 90, "dy": 90, "dz": 0}, self.POSE, ok)["code"], "MOVE_TOO_FAR")
        self.assertEqual(P.check_move({"dx": 0, "dy": 0, "dz": -95}, self.POSE, ok)["code"], "MOVE_TOO_LOW")
        self.assertEqual(P.check_move({"dx": 0, "dy": 50, "dz": 0}, self.POSE, lambda x, y, z, a: y < 230)["code"], "MOVE_UNREACHABLE")   # the end is out, so is the line
        self.assertEqual(P.check_move({"dx": "x", "dy": 0, "dz": 5}, self.POSE, ok)["code"], "MOVE_UNKNOWN")
        self.assertEqual(P.check_move({"dx": float("nan"), "dy": 0, "dz": 0}, self.POSE, ok)["code"], "NOTHING_TO_MOVE")

    def test_midline_obstacle_is_caught(self):
        self.assertEqual(P.check_move({"dx": 60, "dy": 0, "dz": 0}, self.POSE, lambda x, y, z, a: not (25 < x < 35))["code"], "MOVE_UNREACHABLE")

    def test_through_keyword_action(self):
        self.assertEqual(P.keyword_action("move up 2 cm")["action"], "move_by")
        self.assertEqual(P.keyword_action("put key 2 back")["action"] if P.keyword_action("put key 2 back") else None, None)

    def test_moving_needs_the_output_armed(self):
        self.assertEqual(P.check_action({"action": "move_by"}, {}, False)["code"], "OUTPUT_SAFE")


if __name__ == "__main__":
    unittest.main()
