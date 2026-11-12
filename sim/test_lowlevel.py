"""dashboard/lowlevel.py: the sender layer, JsonSender (the placeholder JSON hand-over) and the shared BackendError."""
import os
import sys
import threading
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import contract as C  # noqa: E402
import fake_receiver as FR  # noqa: E402
import lowlevel as LL  # noqa: E402

STEPS = [{"x": 0, "y": 250, "z": 200, "angle": 0, "speed": 30}, {"x": 40, "y": 250, "z": 120, "angle": 0, "action": "grab"},
         {"x": 40, "y": 250, "z": 200, "angle": 0, "speed": 30}]


def sender(rx, **kw):
    return LL.JsonSender(http=rx, poll_s=0, sleep=lambda s: None, **kw)


class StepMessage(unittest.TestCase):
    def test_shape_and_units(self):
        m = C.step_message("j", 2, 3, STEPS[1])
        self.assertEqual((m["step"], m["of"], m["magnet"], m["target"]["pitch"]), (2, 3, "on", 0.0))
        self.assertEqual(C.step_message("j", 1, 3, STEPS[0])["magnet"], None)
        self.assertEqual(C.step_message("j", 3, 3, dict(STEPS[2], action="release"))["magnet"], "off")

    def test_bad_steps_are_refused(self):
        for bad in ({"x": 0, "y": 250, "z": 200}, {"x": float("nan"), "y": 250, "z": 200, "angle": 0}, dict(STEPS[0], action="throw"), dict(STEPS[0], x="5")):
            with self.assertRaises(C.ContractError, msg=repr(bad)):
                C.step_message("j", 1, 1, bad)


class JsonSenderTests(unittest.TestCase):
    def test_runs_every_step_in_order_and_reports_each(self):
        rx, seen = FR.FakeReceiver(), []
        sender(rx).run(STEPS, job="7f3a", on_step=lambda i, s: seen.append(i))
        self.assertEqual(seen, [0, 1, 2])
        self.assertEqual([m["step"] for m in rx.steps], [1, 2, 3])
        self.assertTrue(all(m["job"] == "7f3a" and m["of"] == 3 for m in rx.steps))

    def test_a_refused_step_ends_the_run_with_the_reason_and_stops_the_arm(self):
        rx = FR.FakeReceiver(refuse_at=2)
        with self.assertRaises(LL.BackendError) as c:
            sender(rx).run(STEPS)
        self.assertEqual((c.exception.code, len(rx.steps)), ("UNREACHABLE", 1))
        self.assertGreaterEqual(rx.stops, 1)

    def test_an_error_while_moving_is_reported(self):
        rx = FR.FakeReceiver(fail_at=1)
        with self.assertRaises(LL.BackendError) as c:
            sender(rx).run(STEPS)
        self.assertEqual(c.exception.code, "ARM_STALLED")

    def test_a_step_that_never_finishes_times_out(self):
        now = [0.0]
        def clock():
            now[0] += 1.0
            return now[0]
        rx = FR.FakeReceiver(never_finish=True)
        with self.assertRaises(LL.BackendError) as c:
            LL.JsonSender(http=rx, poll_s=0, sleep=lambda s: None, clock=clock, step_timeout_s=5).run(STEPS)
        self.assertEqual(c.exception.code, "NOT_REACHED")

    def test_stop_from_another_thread(self):
        rx, s = FR.FakeReceiver(busy_polls=10 ** 6), None
        s = sender(rx)
        threading.Timer(0.05, s.stop).start()
        with self.assertRaises(LL.BackendError) as c:
            LL.JsonSender.run(s, STEPS)
        self.assertEqual(c.exception.code, "STOPPED")

    def test_malformed_answers(self):
        for ack in (None, [], {}, {"accepted": "yes"}):
            with self.assertRaises(LL.BackendError):
                sender(lambda m, p, b=None, a=ack: a if p == "/step" else {}).run(STEPS)

    def test_nothing_to_run_and_a_bad_step(self):
        with self.assertRaises(LL.BackendError) as c:
            sender(FR.FakeReceiver()).run([])
        self.assertEqual(c.exception.code, "NO_POSES")
        with self.assertRaises(LL.BackendError) as c:
            sender(FR.FakeReceiver()).run([{"x": 1}])
        self.assertEqual(c.exception.code, "BAD_REQUEST")


if __name__ == "__main__":
    unittest.main()
