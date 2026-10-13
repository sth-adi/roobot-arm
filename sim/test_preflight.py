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


if __name__ == "__main__":
    unittest.main()
