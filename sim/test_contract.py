"""dashboard/contract.py and dashboard/assistant.py: the page's requests, the AI's menu, and what is refused."""
import io
import json
import os
import sys
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import assistant as AS  # noqa: E402
import contract as C  # noqa: E402
import fake_page as FP  # noqa: E402


class Requests(unittest.TestCase):
    def test_every_request_the_fake_page_makes_is_understood(self):
        for name, d in FP.REQUESTS.items():
            r = C.parse_request(d)
            self.assertEqual(set(r), {"id", "text", "action"}, name)
        self.assertIsNone(C.parse_request(FP.REQUESTS["typed"])["action"])               # free text: the AI chooses
        self.assertEqual(C.parse_request(FP.REQUESTS["button_fetch"])["action"], {"action": "fetch_key", "key": 2})
        self.assertEqual(C.parse_request(FP.REQUESTS["button_stop"])["action"], {"action": "stop"})

    def test_the_pages_current_buttons_ask_which_key_instead_of_guessing(self):
        a = C.parse_request({"command": "request_key"})["action"]
        self.assertEqual(a["action"], "ask")
        self.assertEqual(C.parse_request({"command": "stop"})["action"], {"action": "stop"})

    def test_refused_requests(self):
        for bad in (None, [], "text", {}, {"text": ""}, {"text": "x" * 501}, {"command": "dance"}, {"action": "fetch", "key": 4},
                    {"action": "fetch", "key": True}, {"action": "fetch"}, {"action": "explode"}):
            with self.assertRaises(C.ContractError, msg=repr(bad)):
                C.parse_request(bad)


class Actions(unittest.TestCase):
    def test_valid_actions_are_cleaned(self):
        self.assertEqual(C.validate_action({"action": "fetch_key", "key": "2", "x": 9, "y": 1}), {"action": "fetch_key", "key": 2})
        self.assertEqual(C.validate_action({"action": "magnet", "on": False, "reason": "r"}), {"action": "magnet", "on": False, "reason": "r"})
        self.assertEqual(C.validate_action({"action": "move_by", "dz": 20})["dz"], 20.0)

    def test_everything_else_is_refused(self):
        for bad in ({"action": "run_sequence"}, {"action": "fetch_key"}, {"action": "fetch_key", "key": 0}, {"action": "fetch_key", "key": 2.5},
                    {"action": "magnet"}, {"action": "magnet", "on": "yes"}, {"action": "move_by", "dx": 90, "dy": 90},
                    {"action": "move_by", "dx": float("nan")}, {"action": "move_by", "dx": "5"}, "fetch", {}):
            with self.assertRaises(C.ContractError, msg=repr(bad)):
                C.validate_action(bad)

    def test_job_status_shape(self):
        packets = FP.lifecycle()
        self.assertEqual([p["status"] for p in packets], ["queued", "running", "running", "running", "done"])
        for p in packets:
            self.assertEqual(set(p), {"v", "job", "status", "step", "of", "message", "error", "led"})
        with self.assertRaises(ValueError):
            C.job_status("x", "exploded")
        with self.assertRaises(ValueError):
            C.job_status("x", "failed", error={"title": "no code"})
        self.assertEqual(C.job_status("x", "failed", error={"code": "UNREACHABLE", "title": "t"})["error"]["code"], "UNREACHABLE")


class Decide(unittest.TestCase):
    def setUp(self):
        p = mock.patch.object(AS.AG, "api_key", return_value="")                         # no key: the keyword matcher decides
        p.start()
        self.addCleanup(p.stop)

    def test_keywords(self):
        d = AS.decide("get me the second key")
        self.assertEqual((d["action"], d["key"], d["by"]), ("fetch_key", 2, "keywords"))
        self.assertEqual(AS.decide("put key 3 back")["action"], "return_key")
        self.assertEqual(AS.decide("put key 3 back")["key"], 3)
        self.assertEqual(AS.decide("go home")["action"], "go_home")
        self.assertEqual(AS.decide("magnet off")["on"], False)

    def test_unclear_requests_get_a_question_not_a_guess(self):
        for text in ("get me a key", "return it", "get key 1 and 2", "what is the weather", "fetch"):
            self.assertEqual(AS.decide(text)["action"], "ask", text)

    def test_stop_words_win_and_need_no_network(self):
        with mock.patch.object(AS.urllib.request, "urlopen", side_effect=AssertionError("network used")):
            self.assertEqual(AS.decide("stop")["action"], "stop")
            self.assertEqual(AS.decide("stop, get key 2")["action"], "stop")

    def _groq(self, answer):
        resp = mock.MagicMock()
        resp.read.return_value = json.dumps({"choices": [{"message": {"content": json.dumps(answer) if not isinstance(answer, str) else answer}}]}).encode()
        resp.__enter__.return_value = resp
        return mock.patch.object(AS.urllib.request, "urlopen", return_value=resp)

    def test_the_model_chooses_inside_the_menu(self):
        with mock.patch.object(AS.AG, "api_key", return_value="k"), self._groq({"action": "fetch_key", "key": 3, "reason": "asked"}):
            d = AS.decide("bring the last one")
        self.assertEqual((d["action"], d["key"], d["by"]), ("fetch_key", 3, "groq"))

    def test_a_bad_model_answer_is_refused_and_keywords_decide(self):
        for answer in ({"action": "run_sequence", "sequence": "x"}, {"action": "fetch_key", "key": 9}, {"action": "move_by", "dx": 500}, "not json",
                       {"action": "fetch_key", "key": 2, "x": 100, "y": 200, "z": 5}):
            with mock.patch.object(AS.AG, "api_key", return_value="k"), self._groq(answer):
                d = AS.decide("put key 1 back")
            if isinstance(answer, dict) and answer.get("action") == "fetch_key" and answer.get("key") == 2:
                self.assertEqual(d["by"], "groq")                                         # extra coordinate fields are dropped, the action is fine
                self.assertEqual(set(d) - {"by", "model", "ms", "raw"}, {"action", "key"})
            else:
                self.assertEqual((d["action"], d["key"], d["by"]), ("return_key", 1, "keywords"), answer)
                self.assertTrue(d["ai_error"])


if __name__ == "__main__":
    unittest.main()
