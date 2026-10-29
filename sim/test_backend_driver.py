"""dashboard/backend_client.py driving Sam's backend (+ our patch) while a fake ESP32 behaves like his firmware."""
import math
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
from backend_harness import BackendTestCase, FakeEsp  # noqa: E402
import backend_client as BC  # noqa: E402

START = (0, 250, 250, 0)


def pose(x, y, z, a=0, action=None):
    return {"x": x, "y": y, "z": z, "angle": a, "action": action}


class DriverTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.esp = FakeEsp(self, START)
        self.esp.start()
        self.client = BC.BackendClient(self.url, timeout=3)
        self.runner = None

    def tearDown(self):
        self.esp.stop()
        super().tearDown()

    def run_poses(self, poses, **kw):
        self.runner = BC.PoseRunner(self.client, speed=40, poll_s=0.05, **kw)
        self.runner.run(poses)

    def test_runs_poses_in_order_and_ends_on_the_last(self):
        self.run_poses([pose(0, 250, 150), pose(60, 250, 150), pose(60, 250, 120)])
        self.assertEqual(self.runner.reached, 3)
        end = self.IK(self.C(60, 250, 120, 0))
        for got, want in zip(self.esp.q, (end.S1, end.S2, end.S3, end.S4)):
            self.assertAlmostEqual(got, math.degrees(want), places=1)
        self.assertGreaterEqual(len(self.esp.packets), 3)
        pos = self.client.position()
        self.assertAlmostEqual(pos["z"], 120, delta=0.5)

    def test_every_packet_keeps_to_the_joint_speed_limit(self):
        self.run_poses([pose(-100, 250, 150), pose(100, 250, 150)])
        for rows in self.esp.packets:
            for a, b in zip(rows, rows[1:]):
                for j in range(1, 5):
                    self.assertLessEqual(abs(b[j] - a[j]) / ((b[0] - a[0]) / 1000.0), self.state.MAX_JOINT_DPS * 1.07 + 1.0)

    def test_a_refused_pose_ends_the_run_and_holds(self):
        with self.assertRaises(BC.BackendError) as c:
            self.run_poses([pose(0, 250, 150), pose(-30, 60, 150), pose(0, 250, 100)])
        self.assertEqual(c.exception.code, "REFUSED")
        self.assertIn("joint", c.exception.title.lower())
        self.assertEqual(self.runner.reached, 1)                       # the first was done, the third never sent
        time.sleep(0.3)
        self.assertAlmostEqual(self.state.TargetCords.Z, 150, delta=1.0)   # the arm was told to hold where it is, not left chasing a target

    def test_stop_from_another_thread(self):
        errors = []

        def go():
            try:
                self.run_poses([pose(0, 250, 150), pose(100, 250, 150), pose(-100, 250, 150), pose(0, 250, 100)])
            except BC.BackendError as e:
                errors.append(e)
        t = threading.Thread(target=go)
        t.start()
        time.sleep(1.2)
        self.runner.stop()
        t.join(10)
        self.assertEqual([e.code for e in errors], ["STOPPED"])
        self.assertLess(self.runner.reached, 4)

    def test_actions_need_a_way_to_switch_the_magnet(self):
        with self.assertRaises(BC.BackendError) as c:
            self.run_poses([pose(0, 250, 150, action="grab")])
        self.assertEqual(c.exception.code, "MAGNET_UNSUPPORTED")
        self.assertEqual(self.state.TargetCords.Z, 250)                # nothing was sent

    def test_magnet_actions_happen_after_arriving(self):
        calls = []
        self.run_poses([pose(0, 250, 150), pose(0, 250, 120, action="grab"), pose(60, 250, 150), pose(60, 250, 120, action="release")], magnet=lambda on: calls.append((on, round(self.client.position()["z"]))))
        self.assertEqual(calls, [(True, 120), (False, 120)])

    def test_nothing_to_run(self):
        with self.assertRaises(BC.BackendError) as c:
            BC.PoseRunner(self.client).run([])
        self.assertEqual(c.exception.code, "NO_POSES")


class OfflineAndSilent(unittest.TestCase):
    def test_backend_offline(self):
        with self.assertRaises(BC.BackendError) as c:
            BC.BackendClient("http://127.0.0.1:9", timeout=0.5).movement()
        self.assertEqual(c.exception.code, "BACKEND_OFFLINE")
        self.assertTrue(c.exception.hint)

    def test_an_arm_that_never_answers_is_a_timeout_not_a_hang(self):
        class Silent(BC.BackendClient):
            def set_target(self, *a, **k):
                pass

            def movement(self):
                return {"status": "idle"}

            def position(self):
                return None

            def hold(self):
                self.held = True
        now = [0.0]
        client = Silent()
        runner = BC.PoseRunner(client, clock=lambda: now[0], sleep=lambda s: now.__setitem__(0, now[0] + s))
        with self.assertRaises(BC.BackendError) as c:
            runner.run([pose(0, 250, 150)])
        self.assertEqual(c.exception.code, "NOT_REACHED")
        self.assertTrue(client.held)
        self.assertGreater(now[0], 15)


if __name__ == "__main__":
    unittest.main()
