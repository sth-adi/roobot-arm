import tempfile
import unittest

from sim import sequences as S

SAVED = [(0, 100, 80, 0), (0, 240, 60, 0), (0, 240, 20, 0), (222, 60, 60, 0), (222, 60, 20, 0), (0, 100, 80, 0)]
STATES = [False, False, True, True, False, False]       # magnet on at the box, off at the tray


class SequenceTest(unittest.TestCase):
    def test_actions_roundtrip(self):
        acts = S.actions_from_states(STATES)
        self.assertEqual(acts, [None, None, "grab", None, "release", None])
        self.assertEqual(S.states_from_actions(acts), STATES)

    def test_reverse_swaps_grab_and_release_and_conditions(self):
        seq = S.make_sequence(SAVED, STATES, "box_to_tray", "key from box to tray", {"key_in": "box2"}, {"key_on": "tray"})
        rev = S.reverse(seq)
        self.assertEqual(rev["name"], "box_to_tray_reverse")
        self.assertEqual([p["y"] for p in rev["poses"]], [100, 60, 60, 240, 240, 100])
        self.assertEqual([p["action"] for p in rev["poses"]], [None, "grab", None, "release", None, None])
        self.assertEqual(rev["requires"], {"key_on": "tray"})
        self.assertEqual(rev["ends"], {"key_in": "box2"})
        self.assertEqual(S.reverse(rev)["poses"], seq["poses"])
        self.assertEqual(S.reverse(rev)["name"], "box_to_tray")

    def test_reversed_magnet_state_is_on_between_tray_and_box(self):
        rev = S.reverse(S.make_sequence(SAVED, STATES, "x"))
        _, states = S.to_saved(rev)
        self.assertEqual(states, [False, True, True, False, False, False])

    def test_validate(self):
        with self.assertRaises(ValueError):
            S.validate({"format": "nope"})
        with self.assertRaises(ValueError):
            S.validate({"format": S.FORMAT, "poses": [{"x": 0, "y": 0, "z": 0, "angle": 0, "action": "zap"}]})
        with self.assertRaises(ValueError):
            S.validate({"format": S.FORMAT, "name": "../evil", "poses": [{"x": 0, "y": 0, "z": 0, "angle": 0}]})
        v1 = S.validate({"format": S.FORMAT, "positions": [{"x": 0, "y": 1, "z": 2, "angle": 3, "magnet": True}, {"x": 0, "y": 1, "z": 2, "angle": 3, "magnet": False}]})
        self.assertEqual([p["action"] for p in v1["poses"]], ["grab", "release"])

    def test_library(self):
        with tempfile.TemporaryDirectory() as d:
            lib = S.Library(d)
            self.assertEqual(lib.list(), [])
            lib.save(S.make_sequence(SAVED, STATES, "a1", "first"))
            self.assertEqual([s["name"] for s in lib.list()], ["a1"])
            self.assertEqual(lib.get("a1")["count"], 6)
            with self.assertRaises(KeyError):
                lib.get("missing")
            with self.assertRaises(ValueError):
                lib.get("../x")


if __name__ == "__main__":
    unittest.main()
