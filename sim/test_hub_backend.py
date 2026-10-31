"""The hub with --backend: the virtual arm stays the brain, and when ARMED the real arm follows the same poses through Sam's backend (+ our patch)."""
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
from backend_harness import BackendTestCase, FakeEsp  # noqa: E402
import sim.sequences as SEQ  # noqa: E402

START = (0, 250, 250, 0)


def seq(poses, name="t"):
    return SEQ.make_sequence([(x, y, z, a) for x, y, z, a, _ in poses], [m for *_, m in poses], name, "test")


class HubWithBackend(BackendTestCase):
    def make_hub(self, armed=True, ignore_magnet=False):
        import hub as H
        self.H = H
        self.hub = H.Hub("http://127.0.0.1:1", backend_url=self.url, backend_speed=40, backend_ignore_magnet=ignore_magnet)
        self.hub.armed = armed
        self.hub.start()
        self.esp = FakeEsp(self, START)
        self.esp.start()
        self.addCleanup(self.esp.stop)
        self.addCleanup(self.hub.stop.set)

    def wait(self, cond, timeout=40):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(0.1)
        return False

    def idle(self):
        return self.hub._run_once is None and not self.hub._be_busy()

    def test_armed_the_real_arm_follows_the_same_poses(self):
        self.make_hub()
        self.hub.load_sequence(seq([(0, 250, 150, 0, False), (60, 250, 150, 0, False)]), False, True)
        self.assertTrue(self.wait(self.idle), "did not finish")
        self.assertIsNone(self.hub._be_error)
        end = self.IK(self.C(60, 250, 150, 0))
        for got, want in zip(self.esp.q, (end.S1, end.S2, end.S3, end.S4)):
            self.assertAlmostEqual(got, math.degrees(want), places=1)
        x, y, z, _ = self.hub.sim.pose()
        self.assertAlmostEqual(x, 60, delta=2)                           # and the virtual arm got there too

    def test_safe_only_the_virtual_arm_moves(self):
        self.make_hub(armed=False)
        before = list(self.esp.q)
        self.hub.load_sequence(seq([(0, 250, 150, 0, False), (60, 250, 150, 0, False)]), False, True)
        self.assertTrue(self.wait(lambda: self.hub._run_once is None))
        time.sleep(0.5)
        self.assertEqual(self.esp.q, before)
        self.assertEqual(self.esp.packets, [])

    def test_the_magnet_cannot_be_switched_so_it_is_refused_before_anything_moves(self):
        self.make_hub()
        with self.assertRaises(ValueError) as c:
            self.hub.load_sequence(seq([(0, 250, 150, 0, False), (0, 250, 120, 0, True)]), False, True)
        self.assertIn("magnet", str(c.exception).lower())
        self.assertIsNone(self.hub._run_once)
        self.assertEqual(self.esp.packets, [])

    def test_ignore_magnet_flag_moves_the_real_arm_anyway(self):
        self.make_hub(ignore_magnet=True)
        self.hub.load_sequence(seq([(0, 250, 150, 0, False), (0, 250, 120, 0, True), (60, 250, 150, 0, False), (60, 250, 120, 0, False)]), False, True)
        self.assertTrue(self.wait(self.idle, 60))
        self.assertIsNone(self.hub._be_error)
        self.assertGreaterEqual(len(self.esp.packets), 3)

    def test_a_pose_the_backend_refuses_ends_both_runs_with_the_reason(self):
        self.make_hub()
        self.hub.ask("move left 3 cm")                                    # a nudge goes through the same path
        self.assertTrue(self.wait(lambda: self.hub.task.get("state") in ("done", "failed")), self.hub.task)
        self.assertEqual(self.hub.task["state"], "done", self.hub.task)
        self.assertGreaterEqual(len(self.esp.packets), 1)                  # the real arm moved too, not just the virtual one
        # now a nudge the backend will refuse but the virtual arm would allow: far left of the calibrated base range
        self.hub.sim.stop_replay()
        self.hub._be_error = None
        self.hub._be_start([{"x": 0, "y": 250, "z": 150, "angle": 0, "action": None}, {"x": -30, "y": 60, "z": 150, "angle": 0, "action": None}])
        self.assertTrue(self.wait(lambda: not self.hub._be_busy(), 30))
        self.assertEqual(self.hub._be_error["code"], "REFUSED")
        self.assertIn("joint", self.hub._be_error["title"].lower())

    def test_stop_stops_the_real_arm_and_holds(self):
        self.make_hub()
        self.hub.load_sequence(seq([(-100, 250, 150, 0, False), (100, 250, 150, 0, False), (-100, 250, 150, 0, False), (0, 250, 100, 0, False)]), False, True)
        time.sleep(1.5)
        self.hub._stop_now("stop")
        self.assertTrue(self.wait(lambda: not self.hub._be_busy(), 15))
        self.assertEqual(self.hub._be_error["code"], "STOPPED")
        time.sleep(0.4)
        # the arm's target is where it stopped, not the last pose of the sequence
        self.assertNotAlmostEqual(self.state.TargetCords.Z, 100, delta=1.0)

    def test_disarming_stops_the_real_arm(self):
        self.make_hub()
        self.hub.load_sequence(seq([(-100, 250, 150, 0, False), (100, 250, 150, 0, False), (0, 250, 100, 0, False)]), False, True)
        time.sleep(1.2)
        self.hub.set_armed(False)
        self.assertTrue(self.wait(lambda: not self.hub._be_busy(), 15))
        self.assertEqual(self.hub._be_error["code"], "STOPPED")

    def test_state_packet_reports_the_backend(self):
        self.make_hub()
        self.assertTrue(self.wait(lambda: self.hub.state_packet().get("backend", {}).get("online") is True, 10))
        b = self.hub.state_packet()["backend"]
        self.assertEqual(b["url"], self.url)
        self.assertIn("position", b)

    def move_virtual_arm_to(self, x, y, z, a):
        ang = self.IK(self.C(x, y, z, a))
        with self.hub.sim_lock:
            self.hub.sim.angles = (ang.S1, ang.S2, ang.S3, ang.S4)

    def test_moving_the_virtual_arm_by_hand_moves_the_real_arm_live(self):
        """The sticks and the Home button only move the virtual arm; while ARMED the follower posts its position to the backend as the target."""
        self.make_hub()
        self.hub.set_backend_speed(33)
        time.sleep(0.6)                                                   # the first tick after arming only records where the virtual arm is
        self.move_virtual_arm_to(60, 250, 150, 0)
        end = self.IK(self.C(60, 250, 150, 0))
        want = [math.degrees(v) for v in (end.S1, end.S2, end.S3, end.S4)]
        self.assertTrue(self.wait(lambda: all(abs(g - w) < 0.5 for g, w in zip(self.esp.q, want)), 20), self.esp.q)
        self.assertEqual(self.state.TargetSpeed, 33)                       # at the speed set from the dashboard
        self.assertTrue(self.hub.state_packet()["backend"]["follow"])

    def test_nothing_is_sent_when_arming_only_when_the_virtual_arm_moves(self):
        self.make_hub()
        time.sleep(1.0)
        self.assertEqual(self.esp.packets, [])

    def test_safe_the_virtual_arm_moves_alone(self):
        self.make_hub(armed=False)
        self.move_virtual_arm_to(60, 250, 150, 0)
        time.sleep(1.2)
        self.assertEqual(self.esp.packets, [])

    def test_a_refused_follow_target_does_not_kill_the_loop(self):
        self.make_hub()
        time.sleep(0.6)
        self.move_virtual_arm_to(-30, 60, 150, 0)                         # the backend refuses this one (outside the calibrated base range)
        time.sleep(0.8)
        self.move_virtual_arm_to(60, 250, 150, 0)
        end = self.IK(self.C(60, 250, 150, 0))
        want = [math.degrees(v) for v in (end.S1, end.S2, end.S3, end.S4)]
        self.assertTrue(self.wait(lambda: all(abs(g - w) < 0.5 for g, w in zip(self.esp.q, want)), 20), self.esp.q)

    def test_real_arm_speed_is_limited_to_what_his_backend_allows(self):
        self.make_hub()
        self.hub.set_backend_speed(50)
        self.assertEqual(self.hub.backend_speed, 50)
        for bad in (4, 51, "x", None):
            with self.assertRaises((ValueError, TypeError)):
                self.hub.set_backend_speed(bad)
        self.assertEqual(self.hub.backend_speed, 50)
        self.assertEqual(self.hub.state_packet()["backend"]["max_speed"], 50.0)

    def test_offline_backend_is_reported_not_fatal(self):
        import hub as H
        h = H.Hub("http://127.0.0.1:1", backend_url="http://127.0.0.1:9")
        h.armed = True
        h.start()
        self.addCleanup(h.stop.set)
        self.assertTrue(self.wait(lambda: h.state_packet()["backend"].get("online") is False, 10))
        h.load_sequence(seq([(0, 250, 150, 0, False)]), False, True)
        self.assertTrue(self.wait(lambda: not h._be_busy() and h._run_once is None, 30))
        self.assertEqual(h._be_error["code"], "BACKEND_OFFLINE")


if __name__ == "__main__":
    unittest.main()
