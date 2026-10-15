import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
import agent  # noqa: E402

LIB = [{"name": "box2_to_tray", "description": "move the key from box 2 to the tray", "requires": {}, "ends": {}},
       {"name": "box1_to_tray", "description": "move the key from box 1 to the tray", "requires": {}, "ends": {}}]


class AgentTest(unittest.TestCase):
    def test_keywords_pick_by_description(self):
        c = agent.keyword_choice("take the key out of box 1 and put it on the tray", LIB)
        self.assertEqual((c["sequence"], c["reverse"]), ("box1_to_tray", False))

    def test_reverse_words_flip(self):
        c = agent.keyword_choice("put the key from the tray back to box 2", LIB)
        self.assertEqual((c["sequence"], c["reverse"]), ("box2_to_tray", True))

    def test_undo_uses_last_run(self):
        c = agent.keyword_choice("put it back", LIB, {"name": "box2_to_tray", "reverse": False})
        self.assertEqual((c["sequence"], c["reverse"]), ("box2_to_tray", True))
        c = agent.keyword_choice("put it back", LIB, {"name": "box2_to_tray", "reverse": True})
        self.assertFalse(c["reverse"])

    def test_undo_a_nudge(self):
        state = {"last_move": {"dx": 0.0, "dy": 30.0, "dz": 0.0}, "last_run": {"name": "box2_to_tray", "reverse": False}}
        with mock.patch.object(agent, "api_key", return_value="k"), mock.patch.object(agent, "groq_choice", side_effect=AssertionError("model not needed")):
            c = agent.choose("move it back", LIB, state)
        self.assertEqual((c["action"], c["dy"], c["by"]), ("move_by", -30.0, "keywords"))
        self.assertEqual(agent.keyword_choice("undo", LIB, state["last_run"], state["last_move"])["action"], "move_by")
        self.assertEqual(agent.keyword_choice("undo", LIB, state["last_run"], None)["reverse"], True)   # no nudge: undo the sequence as before

    def test_nothing_matches(self):
        self.assertIsNone(agent.keyword_choice("paint the wall", LIB)["sequence"])

    def test_model_answer_is_checked_against_the_library(self):
        bad = {"sequence": "rm_rf", "reverse": False, "reason": "x", "by": "groq"}
        with mock.patch.object(agent, "api_key", return_value="k"), mock.patch.object(agent, "groq_choice", return_value=bad):
            c = agent.choose("take the key out of box 1 and put it on the tray", LIB, {})
        self.assertEqual(c["by"], "keywords")
        good = {"sequence": "box1_to_tray", "reverse": True, "reason": "x", "by": "groq"}
        with mock.patch.object(agent, "api_key", return_value="k"), mock.patch.object(agent, "groq_choice", return_value=good):
            self.assertEqual(agent.choose("anything", LIB, {})["sequence"], "box1_to_tray")

    def test_network_failure_falls_back(self):
        with mock.patch.object(agent, "api_key", return_value="k"), mock.patch.object(agent, "groq_choice", side_effect=OSError("offline")):
            self.assertEqual(agent.choose("key from box 2 to the tray", LIB, {})["by"], "keywords")


if __name__ == "__main__":
    unittest.main()
