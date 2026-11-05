"""dashboard/jobs.py: the job runner, its statuses, stop, and the two-menu failure handling (AI choice, rule table, hard faults, caps)."""
import os
import sys
import tempfile
import threading
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import assistant as AS  # noqa: E402
import backend_client as BC  # noqa: E402
import boxes as B  # noqa: E402
import contract as C  # noqa: E402
import fake_receiver as FR  # noqa: E402
import jobs as J  # noqa: E402
import lowlevel as LL  # noqa: E402

HOME = {"x": 0.0, "y": 250.0, "z": 250.0, "angle": 0.0}
SIZE = {"w": 40, "d": 40, "h": 50}


def store():
    s = B.Store(os.path.join(tempfile.mkdtemp(), "b.json"))
    for k, x in ((1, -60), (2, 0), (3, 60)):
        s.set_box(k, x, 250, 120, SIZE)
    s.set_tray(0, 180, 60, {"w": 80, "d": 60, "h": 30})
    return s


class Arm:
    """A sender that moves instantly, remembers where the arm is, and fails on demand: fail = {label: [error codes for successive attempts]}."""
    def __init__(self, fail=None, block=None):
        self.pose, self.fail, self.ran, self.stops, self.block = dict(HOME), fail or {}, [], 0, block

    def run(self, steps, job="", on_step=None):
        for i, s in enumerate(steps):
            if self.block:
                self.block.wait(5)
            if self.stops:
                raise BC.BackendError("STOPPED", "Stopped")
            codes = self.fail.get(s["label"])
            if codes:
                code = codes.pop(0)
                if isinstance(code, tuple):                       # (code, where the arm is when it fails)
                    code, self.pose = code[0], dict(code[1])
                if code:
                    raise BC.BackendError(code, "fault: " + code, "scripted")
            self.pose = {k: s[k] for k in ("x", "y", "z", "angle")}
            self.ran.append(s["label"] + (":" + s["action"] if "action" in s else ""))
            if on_step:
                on_step(i, s)

    def stop(self):
        self.stops += 1


def runner(arm, recover=None, **kw):
    return J.JobRunner(store(), arm, lambda: dict(arm.pose), recover=recover, **kw)


def go(r, action):
    r.start(action)
    return r.wait()


class Happy(unittest.TestCase):
    def test_fetch_reports_every_step_and_finishes(self):
        arm = Arm()
        seen = []
        r = runner(arm, on_status=seen.append)
        last = go(r, {"action": "fetch_key", "key": 2})
        self.assertEqual(last["status"], "done")
        self.assertEqual([s["status"] for s in seen][:2], ["queued", "running"])
        steps = [s["step"] for s in seen if s["status"] == "running" and s["step"]]
        self.assertEqual(steps, sorted(steps))
        self.assertEqual([x for x in arm.ran if ":" in x], ["down to the key:grab", "down over tray:release"])
        self.assertEqual(last["led"], "done")

    def test_an_unreachable_route_never_moves_the_arm(self):
        arm = Arm()
        r = runner(arm)
        r.store.data["boxes"]["2"]["y"] = 600.0
        last = go(r, {"action": "fetch_key", "key": 2})
        self.assertEqual((last["status"], arm.ran), ("failed", []))
        self.assertIn(last["error"]["code"], ("UNREACHABLE", "JOINT_LIMIT"))

    def test_a_box_not_taught_is_reported(self):
        arm = Arm()
        r = J.JobRunner(B.Store(os.path.join(tempfile.mkdtemp(), "b.json")), arm, lambda: dict(arm.pose))
        last = go(r, {"action": "fetch_key", "key": 1})
        self.assertEqual((last["status"], last["error"]["code"], arm.ran), ("failed", "NOT_TAUGHT", []))

    def test_unknown_position_never_moves(self):
        arm = Arm()
        r = J.JobRunner(store(), arm, lambda: None)
        self.assertEqual(go(r, {"action": "go_home"})["error"]["code"], "NO_POSITION")

    def test_only_one_job_at_a_time(self):
        gate = threading.Event()
        arm = Arm(block=gate)
        r = runner(arm)
        r.start({"action": "go_home"})
        with self.assertRaises(J.JobError) as c:
            r.start({"action": "go_home"})
        self.assertEqual(c.exception.code, "BUSY")
        gate.set()
        r.wait()

    def test_not_everything_in_the_menu_is_a_job(self):
        for a in ({"action": "ask"}, {"action": "status"}, {"action": "stop"}):
            with self.assertRaises(J.JobError):
                runner(Arm()).start(a)

    def test_stop_ends_the_job_stopped(self):
        gate = threading.Event()
        arm = Arm(block=gate)
        r = runner(arm)
        r.start({"action": "fetch_key", "key": 1})
        r.stop()
        gate.set()
        last = r.wait()
        self.assertEqual(last["status"], "stopped")
        self.assertGreaterEqual(arm.stops, 1)

    def test_a_nudge_and_the_magnet(self):
        arm = Arm()
        r = runner(arm)
        self.assertEqual(go(r, {"action": "move_by", "dx": 10, "dy": 0, "dz": -20})["status"], "done")
        self.assertEqual(arm.pose["z"], 230.0)
        self.assertEqual(go(r, {"action": "magnet", "on": True})["status"], "done")
        self.assertEqual(arm.ran[-1], "magnet on:grab")


class Failures(unittest.TestCase):
    def test_hard_faults_are_never_given_to_the_ai(self):
        asked = []
        arm = Arm(fail={"across to box 2": ["BACKEND_OFFLINE"]})
        last = go(runner(arm, recover=lambda rep: asked.append(rep)), {"action": "fetch_key", "key": 2})
        self.assertEqual((last["status"], last["error"]["code"], asked), ("failed", "BACKEND_OFFLINE", []))

    def test_a_soft_fault_is_retried_and_the_job_finishes(self):
        arm = Arm(fail={"down to the key": ["NOT_REACHED"]})
        seen = []
        last = go(runner(arm, on_status=seen.append), {"action": "fetch_key", "key": 3})
        self.assertEqual(last["status"], "done")
        self.assertTrue(any("Retrying" in s["message"] for s in seen))
        self.assertEqual(arm.ran.count("down to the key:grab"), 1)

    def test_retries_are_capped_whatever_the_ai_says(self):
        arm = Arm(fail={"down to the key": ["NOT_REACHED"] * 10})
        reports = []

        def always_retry(rep):
            reports.append(rep)
            return {"action": "retry_step", "by": "groq"}

        last = go(runner(arm, recover=always_retry, max_retries=2), {"action": "fetch_key", "key": 1})
        self.assertEqual(last["status"], "failed")
        self.assertEqual([x["retries_left"] for x in reports], [2, 1, 0])
        self.assertEqual(last["error"].get("recovery"), "go_home")     # the rule table took over once the cap was hit; nothing is held yet, so home

    def test_the_report_tells_the_ai_what_it_needs(self):
        reports = []
        arm = Arm(fail={"lift the key": ["ARM_STALLED"]})
        r = runner(arm, recover=lambda rep: reports.append(rep) or {"action": "stop"})
        last = go(r, {"action": "fetch_key", "key": 2})
        rep = reports[0]
        self.assertEqual((rep["label"], rep["holding"], rep["pose_known"], rep["over"]), ("lift the key", True, True, "box 2"))
        self.assertEqual(rep["error"]["code"], "ARM_STALLED")
        self.assertEqual(last["status"], "failed")

    def test_the_ai_can_choose_to_lift_and_the_lift_is_straight_up(self):
        arm = Arm(fail={"lift the key": ["ARM_STALLED"]})
        r = runner(arm, recover=lambda rep: {"action": "lift_to_safe_height", "reason": "holding the key"})
        last = go(r, {"action": "fetch_key", "key": 2})
        self.assertEqual(last["status"], "failed")
        self.assertEqual(last["error"]["recovery"], "lift_to_safe_height")
        self.assertIn("lifted clear", last["message"])
        self.assertEqual(arm.pose["x"], 0.0)
        self.assertEqual(arm.pose["y"], 250.0)
        self.assertGreater(arm.pose["z"], 120)

    def test_release_is_refused_high_above_the_box_even_with_the_right_box(self):
        arm = Arm(fail={"across to tray": ["ARM_STALLED"] * 10})   # holding the key at travel height above box 2: a release would be a drop
        r = runner(arm, recover=lambda rep: {"action": "release_magnet"})
        last = go(r, {"action": "fetch_key", "key": 2})
        self.assertEqual(last["status"], "failed")
        self.assertFalse(any(x.endswith(":release") for x in arm.ran))
        self.assertNotEqual(last["error"].get("recovery"), "release_magnet")

    def test_release_is_allowed_low_over_the_tray_when_holding(self):
        low = {"x": 0, "y": 180, "z": 92, "angle": 0}              # 32 mm above the tray: within the hover height
        arm = Arm(fail={"down over tray": [("ARM_STALLED", low)]})
        r = runner(arm, recover=lambda rep: {"action": "release_magnet"})
        last = go(r, {"action": "fetch_key", "key": 2})
        self.assertEqual((last["status"], last["error"].get("recovery")), ("failed", "release_magnet"))
        self.assertTrue(arm.ran[-1].endswith(":release"))

    def test_a_garbage_answer_falls_back_to_the_rules(self):
        arm = Arm(fail={"down to the key": ["NOT_REACHED"]})
        last = go(runner(arm, recover=lambda rep: {"action": "launch_missiles"}), {"action": "fetch_key", "key": 2})
        self.assertEqual(last["status"], "done")                   # rules: retry while retries are left

    def test_ask_person_ends_the_job_with_the_question_and_led_attention(self):
        arm = Arm(fail={"down to the key": ["ARM_STALLED"]})
        last = go(runner(arm, recover=lambda rep: {"action": "ask_person", "question": "Is the key stuck?"}), {"action": "fetch_key", "key": 2})
        self.assertEqual((last["status"], last["error"]["code"], last["led"]), ("failed", "NEEDS_PERSON", "attention"))
        self.assertIn("stuck", last["message"])

    def test_a_failing_recovery_move_ends_the_job_without_another_round(self):
        arm = Arm(fail={"down to the key": ["ARM_STALLED"], "straight up to travel height": ["ARM_STALLED"]})
        calls = []
        last = go(runner(arm, recover=lambda rep: calls.append(1) or {"action": "lift_to_safe_height"}), {"action": "fetch_key", "key": 2})
        self.assertEqual((last["status"], len(calls)), ("failed", 1))
        self.assertIn("Recovery failed", last["message"])

    def test_an_ai_that_raises_does_not_stop_recovery(self):
        def boom(rep):
            raise RuntimeError("network")
        arm = Arm(fail={"down to the key": ["NOT_REACHED"]})
        self.assertEqual(go(runner(arm, recover=boom), {"action": "fetch_key", "key": 1})["status"], "done")


class RuleTable(unittest.TestCase):
    def test_rules(self):
        base = {"error": {"title": "x"}, "holding": False, "pose_known": True, "retries_left": 1}
        self.assertEqual(AS.rule_recovery(base)["action"], "retry_step")
        self.assertEqual(AS.rule_recovery(dict(base, retries_left=0, holding=True))["action"], "lift_to_safe_height")
        self.assertEqual(AS.rule_recovery(dict(base, retries_left=0))["action"], "go_home")
        self.assertEqual(AS.rule_recovery(dict(base, retries_left=0, pose_known=False))["action"], "ask_person")

    def test_recovery_validation(self):
        self.assertEqual(C.validate_recovery({"action": "retry_step", "x": 5, "reason": "r"}), {"action": "retry_step", "reason": "r"})
        for bad in ({"action": "fetch_key"}, {"action": "move_to", "x": 1}, "go_home", None):
            with self.assertRaises(C.ContractError):
                C.validate_recovery(bad)

    def test_no_key_means_the_rules_answer_and_say_why(self):
        saved = AS.AG.api_key
        AS.AG.api_key = lambda: ""
        try:
            a = AS.recover({"error": {"title": "x"}, "retries_left": 2, "holding": False, "pose_known": True})
        finally:
            AS.AG.api_key = saved
        self.assertEqual((a["action"], a["by"]), ("retry_step", "rules"))
        self.assertIn("GROQ_API_KEY", a["ai_error"])


class OverTheJsonSender(unittest.TestCase):
    def test_whole_fetch_through_the_json_sender_and_fake_receiver(self):
        rx = FR.FakeReceiver()
        r = J.JobRunner(store(), LL.JsonSender(http=rx, poll_s=0, sleep=lambda s: None), lambda: dict(HOME))
        last = go(r, {"action": "fetch_key", "key": 1})
        self.assertEqual(last["status"], "done")
        self.assertEqual(len(rx.steps), 11)

    def test_a_refused_step_is_a_hard_failure(self):
        rx = FR.FakeReceiver(refuse_at=3)
        r = J.JobRunner(store(), LL.JsonSender(http=rx, poll_s=0, sleep=lambda s: None), lambda: dict(HOME))
        last = go(r, {"action": "fetch_key", "key": 1})
        self.assertEqual((last["status"], last["error"]["code"]), ("failed", "UNREACHABLE"))


if __name__ == "__main__":
    unittest.main()
