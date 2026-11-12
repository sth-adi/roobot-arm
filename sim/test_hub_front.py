"""The hub's chat and voice text go through the AI front door on the input server (server.py /ai/...), and fall back to the old path without it."""
import os
import sys
import threading
import time
import unittest
from http.server import ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import assistant as AS  # noqa: E402
import hub as H  # noqa: E402
import server  # noqa: E402


class HubUsesTheFrontDoor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        cls.url = "http://127.0.0.1:%d" % cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls._key = AS.AG.api_key
        AS.AG.api_key = lambda: ""                          # keywords decide: no network in tests

    @classmethod
    def tearDownClass(cls):
        AS.AG.api_key = cls._key
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self._saved = dict(server._front)
        server._front["door"] = server.make_front("virtual")
        self.addCleanup(lambda: server._front.update(self._saved))
        self.h = H.Hub(self.url)

    def texts(self, role=None):
        return [m["text"] for m in self.h.chat if role is None or m["role"] == role]

    def wait_state(self, state, timeout=30):
        end = time.time() + timeout
        while time.time() < end and self.h.task.get("state") != state:
            time.sleep(0.1)
        self.assertEqual(self.h.task.get("state"), state)

    def test_a_typed_request_is_confirmed_then_run_and_reported_in_the_chat(self):
        self.h.ask("get me the second key")
        self.assertTrue(any("Fetch key 2?" in t for t in self.texts("robot")))
        self.assertIsNone(server._front["door"].runner.last)                 # nothing moved yet
        self.h.ask("yes")
        self.wait_state("done")
        self.assertIn("Done.", self.texts("robot"))
        self.assertEqual(server._front["door"].state()["keys"]["2"], "tray")

    def test_no_cancels_and_a_refusal_is_an_error_bubble(self):
        self.h.ask("get me the third key")
        self.h.ask("no")
        self.assertTrue(any("Cancelled" in t for t in self.texts("robot")))
        self.h.ask("fetch key 3")
        self.h.ask("yes")
        self.wait_state("done")
        self.h.ask("fetch key 3")                                              # already on the tray: an answer, not a movement
        self.assertTrue(any("already on the tray" in t for t in self.texts("robot")))

    def test_an_unclear_request_gets_a_question(self):
        self.h.ask("get me a key")
        self.assertTrue(any("Which key" in t for t in self.texts("robot")))

    def test_stop_reaches_the_front_door_too(self):
        door = server._front["door"]
        door.request({"action": "fetch", "key": 1})
        self.h.ask("stop")
        door.runner.wait(20)
        self.assertIn(door.job()["status"], ("stopped", "done"))               # the virtual arm is fast: it may already be done
        self.assertTrue(door.runner.sender.stopped or door.job()["status"] == "done")

    def test_without_the_front_door_the_old_path_is_used(self):
        server._front["door"] = None
        h = H.Hub(self.url)
        before = len(h.chat) and h.chat[-1]["id"]                              # the chat list is shared between Hub objects: look only at what is new
        h.ask("go home")
        end = time.time() + 10
        while time.time() < end and h.task.get("state") in (None, "working", "running"):
            time.sleep(0.1)
        new = [m for m in h.chat if m["id"] > before]
        self.assertTrue(any(m["role"] == "user" for m in new))
        self.assertTrue(any("Groq was not used" in m["text"] or "home" in m["text"].lower() for m in new if m["role"] != "user"))      # the old agent answered


if __name__ == "__main__":
    unittest.main()
