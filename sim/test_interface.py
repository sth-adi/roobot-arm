"""dashboard/interface.py: the front door between the page and the AI/job layers (placeholder contract)."""
import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import assistant as AS  # noqa: E402
import interface as I  # noqa: E402
import test_jobs as TJ  # noqa: E402  (its Arm and store helpers)


def door(arm=None, decide=None, **kw):
    arm = arm or TJ.Arm()
    return I.Frontdoor(TJ.store(), arm, lambda: dict(arm.pose), decide=decide, **kw), arm


def said(text):
    return {"v": 1, "id": "r1", "text": text}


def finish(f):
    return f.runner.wait(10)


class Buttons(unittest.TestCase):
    def test_a_button_runs_without_the_ai_or_a_confirmation(self):
        called = []
        f, arm = door(decide=lambda *a: called.append(1))
        r = f.request({"v": 1, "id": "b", "action": "fetch", "key": 2})
        self.assertEqual((r["kind"], r["by"], called), ("started", "button", []))
        self.assertEqual(finish(f)["status"], "done")
        self.assertEqual(f.state()["keys"]["2"], "tray")

    def test_stop_is_immediate_even_with_nothing_running(self):
        f, arm = door()
        r = f.request({"action": "stop"})
        self.assertEqual(r["kind"], "stopped")
        self.assertGreaterEqual(arm.stops, 1)

    def test_busy_arm_refuses_a_second_job_with_the_reason(self):
        import threading
        gate = threading.Event()
        f, arm = door(arm=TJ.Arm(block=gate))
        f.request({"action": "fetch", "key": 1})
        r = f.request({"action": "fetch", "key": 2})
        self.assertEqual((r["kind"], r["error"]["code"]), ("error", "BUSY"))
        gate.set()
        finish(f)

    def test_key_already_where_it_would_go_is_an_answer_not_a_movement(self):
        f, arm = door()
        f.request({"action": "fetch", "key": 2})
        finish(f)
        r = f.request({"action": "fetch", "key": 2})
        self.assertEqual((r["kind"], arm.ran.count("down to the key:grab")), ("answer", 1))
        self.assertIn("already on the tray", r["message"])
        r = f.request({"action": "return", "key": 3})
        self.assertIn("already in its box", r["message"])

    def test_a_box_not_taught_says_what_to_do_and_nothing_moves(self):
        f, arm = door()
        f.store.data["boxes"]["3"] = None
        r = f.request({"action": "fetch", "key": 3})
        self.assertEqual((r["kind"], r["error"]["code"], arm.ran), ("error", "NOT_TAUGHT", []))
        self.assertIn("Box 3", r["message"])

    def test_a_failed_job_marks_the_key_unknown(self):
        f, arm = door(arm=TJ.Arm(fail={"across to tray": ["BACKEND_OFFLINE"]}))
        f.request({"action": "fetch", "key": 1})
        self.assertEqual(finish(f)["status"], "failed")
        self.assertEqual(f.state()["keys"]["1"], "unknown")
        self.assertIn("somewhere unknown", f.describe())


class FreeText(unittest.TestCase):
    def decide_fetch2(self, text, state):
        return {"action": "fetch_key", "key": 2, "by": "groq", "model": "m", "ms": 5, "raw": "{}"}

    def test_text_motion_is_confirmed_first_and_nothing_moves_yet(self):
        f, arm = door(decide=self.decide_fetch2)
        r = f.request(said("get me the second key"))
        self.assertEqual((r["kind"], r["by"], r["heard"]), ("confirm", "groq", "get me the second key"))
        self.assertTrue(r["token"])
        self.assertEqual(arm.ran, [])
        self.assertNotIn("model", r["action"])

    def test_yes_by_token_or_spoken_word_starts_the_job(self):
        for how in ("token", "word"):
            f, arm = door(decide=self.decide_fetch2)
            c = f.request(said("fetch two"))
            r = f.request({"confirm": c["token"]}) if how == "token" else f.request(said("yes please"))
            self.assertEqual(r["kind"], "started", how)
            self.assertEqual(finish(f)["status"], "done")

    def test_no_cancels(self):
        f, arm = door(decide=self.decide_fetch2)
        f.request(said("fetch two"))
        r = f.request(said("no"))
        self.assertEqual((r["kind"], arm.ran), ("answer", []))
        self.assertIn("Cancelled", r["message"])

    def test_the_wrong_token_or_a_late_answer_is_refused(self):
        now = [0.0]
        f, arm = door(decide=self.decide_fetch2, clock=lambda: now[0])
        c = f.request(said("fetch two"))
        self.assertEqual(f.request({"confirm": "zzzzzz"})["error"]["code"], "NOTHING_PENDING")
        now[0] = 100.0
        self.assertEqual(f.request({"confirm": c["token"]})["error"]["code"], "NOTHING_PENDING")
        self.assertEqual(arm.ran, [])

    def test_yes_with_nothing_pending_goes_to_the_ai_like_any_text(self):
        seen = []
        f, arm = door(decide=lambda t, s: seen.append(t) or {"action": "ask", "question": "Which key?", "by": "keywords"})
        r = f.request(said("yes"))
        self.assertEqual((r["kind"], seen), ("question", ["yes"]))

    def test_stop_by_text_needs_no_confirmation(self):
        f, arm = door(decide=lambda t, s: {"action": "stop", "by": "keywords"})
        self.assertEqual(f.request(said("stop"))["kind"], "stopped")

    def test_confirmation_can_be_turned_off(self):
        f, arm = door(decide=self.decide_fetch2, confirm_text=False)
        self.assertEqual(f.request(said("fetch two"))["kind"], "started")
        finish(f)

    def test_the_ai_gets_the_arm_state(self):
        seen = []
        f, arm = door(decide=lambda t, s: seen.append(s) or {"action": "status", "by": "keywords"})
        f.request(said("what is going on"))
        self.assertEqual(set(seen[0]) >= {"busy", "at_home", "keys"}, True)
        self.assertTrue(seen[0]["at_home"])

    def test_real_decide_without_a_key_still_works_by_words(self):
        saved = AS.AG.api_key
        AS.AG.api_key = lambda: ""
        try:
            f, arm = door()
            r = f.request(said("get me the second key"))
        finally:
            AS.AG.api_key = saved
        self.assertEqual((r["kind"], r["action"], r["by"]), ("confirm", {"action": "fetch_key", "key": 2, "reason": "keyword match"}, "keywords"))

    def test_an_unclear_request_is_a_question(self):
        saved = AS.AG.api_key
        AS.AG.api_key = lambda: ""
        try:
            f, arm = door()
            r = f.request(said("get me a key"))
        finally:
            AS.AG.api_key = saved
        self.assertEqual(r["kind"], "question")


class Page(unittest.TestCase):
    def test_the_pages_current_buttons_get_a_question_not_a_guess(self):
        f, arm = door()
        r = f.request({"command": "request_key"})
        self.assertEqual((r["kind"], arm.ran), ("question", []))

    def test_bad_requests_are_error_replies(self):
        f, arm = door()
        for bad in (None, {}, {"text": ""}, {"action": "explode"}, {"confirm": 5}):
            r = f.request(bad)
            self.assertEqual((r["kind"], r["error"]["code"]), ("error", "BAD_REQUEST"), repr(bad))

    def test_status_describes_the_arm_in_words(self):
        f, arm = door()
        r = f.request({"action": "status"})
        self.assertEqual(r["kind"], "answer")
        self.assertIn("key 2 in its box", r["message"])

    def test_http_shapes(self):
        f, arm = door()
        code, r = I.handle_http(f, "POST", "/request", json.dumps({"id": "h", "action": "fetch", "key": 1}).encode())
        self.assertEqual((code, r["kind"]), (200, "started"))
        finish(f)
        code, j = I.handle_http(f, "GET", "/job")
        self.assertEqual((code, j["status"], j["led"]), (200, "done", "done"))
        self.assertEqual(I.handle_http(f, "GET", "/state")[1]["keys"]["1"], "tray")
        self.assertEqual(I.handle_http(f, "POST", "/stop", b"{}")[1]["kind"], "stopped")
        self.assertEqual(I.handle_http(f, "POST", "/request", b"not json")[0], 400)
        self.assertEqual(I.handle_http(f, "POST", "/request", b'{"text": ""}')[0], 400)
        self.assertEqual(I.handle_http(f, "GET", "/nope")[0], 404)
        self.assertEqual(I.handle_http(door()[0], "GET", "/job")[0], 404)


class Teaching(unittest.TestCase):
    """Setting up the stored coordinates through the front door. Nothing here ever moves the arm."""
    def setUp(self):
        import boxes as B
        import tempfile
        self.arm = TJ.Arm()
        self.f = I.Frontdoor(B.Store(os.path.join(tempfile.mkdtemp(), "b.json")), self.arm, lambda: dict(self.arm.pose))

    def at(self, x, y, z):
        self.arm.pose = {"x": x, "y": y, "z": z, "angle": 0.0}

    def test_an_empty_setup_lists_what_is_missing(self):
        r = self.f.teach({"action": "list"})
        self.assertEqual((r["ready"], r["missing"]), (False, ["box 1", "box 2", "box 3", "tray"]))

    def test_where_and_save_use_the_tip_position_and_never_move_the_arm(self):
        self.assertEqual(self.f.teach({"action": "where"})["pose"]["y"], 250.0)
        self.at(-60, 250, 120)
        r = self.f.teach({"action": "save", "which": 1, "size": {"w": 40, "d": 40, "h": 50}})
        self.assertTrue(r["ok"], r)
        self.assertEqual((r["place"]["x"], r["place"]["z"], r["missing"]), (-60.0, 120.0, ["box 2", "box 3", "tray"]))
        self.assertEqual(self.arm.ran, [])

    def test_everything_taught_makes_it_ready_and_a_fetch_possible(self):
        for which, x, y, z in ((1, -60, 250, 120), (2, 0, 250, 120), (3, 60, 250, 120), ("tray", 0, 180, 60)):
            self.at(x, y, z)
            self.assertTrue(self.f.teach({"action": "save", "which": which, "size": {"w": 40, "d": 40, "h": 40}})["ok"])
        self.at(0, 250, 250)
        self.assertTrue(self.f.teach({"action": "list"})["ready"])
        self.assertEqual(self.f.request({"action": "fetch", "key": 2})["kind"], "started")
        finish(self.f)

    def test_typed_numbers_work_without_a_position_report(self):
        f = I.Frontdoor(self.f.store, TJ.Arm(), lambda: None)
        self.assertTrue(f.teach({"action": "save", "which": 2, "x": 0, "y": 250, "z": 120})["ok"])
        self.assertEqual(f.teach({"action": "save", "which": 3})["error"]["code"], "NO_POSITION")

    def test_bad_places_are_refused_with_the_reason(self):
        self.at(0, 600, 120)
        self.assertIn(self.f.teach({"action": "save", "which": 1})["error"]["code"], ("UNREACHABLE", "OUT_OF_RANGE"))
        self.at(0, 250, 120)
        for bad in ({"which": 9}, {"which": "shelf"}, {"which": 1, "size": {"w": 1}}, {"which": 1, "size": {"w": 1, "d": 1, "h": 1}}):
            self.assertEqual(self.f.teach(dict(bad, action="save"))["ok"], False, repr(bad))
        self.assertEqual(self.f.teach({"action": "dance"})["error"]["code"], "BAD_REQUEST")
        self.assertEqual(self.f.teach(None)["error"]["code"], "BAD_REQUEST")

    def test_saving_is_refused_while_a_job_runs(self):
        import threading
        gate = threading.Event()
        arm = TJ.Arm(block=gate)
        f = I.Frontdoor(self.f.store, arm, lambda: dict(arm.pose))
        f.store.set_box(1, -60, 250, 120)
        f.store.set_tray(0, 180, 60)
        f.request({"action": "fetch", "key": 1})
        self.assertEqual(f.teach({"action": "save", "which": 2, "x": 0, "y": 250, "z": 120})["error"]["code"], "BUSY")
        self.assertTrue(f.teach({"action": "list"})["ok"])                           # reading is always fine
        gate.set()
        finish(f)

    def test_preview_shows_the_route_without_moving(self):
        self.assertEqual(self.f.teach({"action": "preview", "kind": "fetch", "key": 2})["error"]["code"], "NOT_TAUGHT")
        self.f.store.set_box(2, 0, 250, 120, {"w": 40, "d": 40, "h": 50})
        self.f.store.set_tray(0, 180, 60, {"w": 80, "d": 60, "h": 30})
        r = self.f.teach({"action": "preview", "kind": "fetch", "key": 2})
        self.assertTrue(r["ok"])
        self.assertEqual((len(r["steps"]), r["steps"][3]["action"], self.arm.ran), (11, "grab", []))
        self.assertGreater(r["seconds"], 0)
        self.assertFalse(self.f.teach({"action": "preview", "kind": "wave", "key": 2})["ok"])

    def test_home_and_constants(self):
        self.at(0, 240, 240)
        self.assertEqual(self.f.teach({"action": "home"})["home"][:3], [0.0, 240.0, 240.0])
        self.assertEqual(self.f.teach({"action": "constant", "name": "hover_mm", "value": 25})["constants"]["hover_mm"], 25.0)
        self.assertEqual(self.f.teach({"action": "constant", "name": "hover_mm", "value": 5000})["error"]["code"], "OUT_OF_RANGE")
        self.assertEqual(self.f.teach({"action": "constant", "name": "luck", "value": 1})["error"]["code"], "BAD_CONSTANT")

    def test_http_shapes(self):
        self.at(-60, 250, 120)
        code, r = I.handle_http(self.f, "POST", "/teach", json.dumps({"action": "save", "which": 1}).encode())
        self.assertEqual((code, r["ok"]), (200, True))
        self.assertEqual(I.handle_http(self.f, "GET", "/boxes")[1]["missing"], ["box 2", "box 3", "tray"])
        self.assertEqual(I.handle_http(self.f, "POST", "/teach", b"{")[0], 400)
        self.assertEqual(I.handle_http(self.f, "POST", "/teach", b'{"action": "save", "which": 9}')[0], 400)


if __name__ == "__main__":
    unittest.main()
