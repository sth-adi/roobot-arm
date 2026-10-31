import http.client
import json
import math
import threading
import unittest
from http.server import HTTPServer

from api import handlers
import GlobalVariables as state
from models.robot import Coordinates
from motion.kinematics import InverseKim


class MovementApiTests(unittest.TestCase):
    def setUp(self):
        self.old_speed = state.TargetSpeed
        state.TargetSpeed = 10.0
        self.old_target = state.TargetCords
        self.old_moving = state.arm_moving
        state.TargetCords = Coordinates(0, 250, 250, 0)
        state.arm_moving = False
        self.server = HTTPServer(("127.0.0.1", 0), handlers.Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        state.TargetSpeed = self.old_speed
        state.TargetCords = self.old_target
        state.arm_moving = self.old_moving

    def request(self, method, path, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port)
        try:
            connection.request(method, path, headers=headers or {})
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            connection.close()

    def packet(self, point=None):
        angles = InverseKim(point or Coordinates(0, 240, 240, 0))
        query = "&".join(f"{key}={math.degrees(value):.3f}" for key, value in zip(
            ("s1", "s2", "s3", "s4"),
            (angles.S1, angles.S2, angles.S3, angles.S4),
        ))
        return self.request("GET", "/UpdateMovement?" + query)

    def test_completion_and_retry(self):
        status, _, body = self.packet()
        self.assertEqual(status, 200)
        self.assertTrue(body)
        self.assertTrue(state.arm_moving)
        for _ in range(2):
            self.assertEqual(self.request("POST", "/movement-complete")[0], 204)
        status, _, body = self.request("GET", "/movement")
        self.assertEqual(status, 200)
        self.assertFalse(json.loads(body))

    def test_target_query_parameters(self):
        self.assertEqual(self.request("POST", "/target?X=200&Y=200&Z=200&A=0")[0], 204)
        self.assertEqual((state.TargetCords.X, state.TargetCords.Y, state.TargetCords.Z),
                         (200, 200, 200))
        self.assertEqual(self.request("POST", "/target?X=0&Y=250&Z=250&A=10&Speed=20")[0], 204)
        self.assertAlmostEqual(state.TargetCords.A, math.radians(10))
        self.assertEqual(state.TargetSpeed, 20)

    def test_invalid_target_query_preserves_state(self):
        original = state.TargetCords
        for query in ("X=0", "X=&Y=250&Z=250&A=0", "X=nan&Y=250&Z=250&A=0",
                      "X=0&X=1&Y=250&Z=250&A=0", "X=0&Y=250&Z=250&A=0&Speed=0"):
            self.assertEqual(self.request("POST", "/target?" + query)[0], 400)
            self.assertIs(state.TargetCords, original)
            self.assertEqual(state.TargetSpeed, 10)

    def test_target_headers_take_precedence(self):
        self.assertEqual(self.request("POST", "/target?X=invalid&Y=250&Z=250&A=0",
                                     {"X": "0"})[0], 204)
        self.assertEqual(state.TargetCords.X, 0)

    def test_completion_needs_no_identifier(self):
        self.assertEqual(self.request("POST", "/movement-complete")[0], 204)

    def test_targets_blocked_until_completion(self):
        self.packet()
        original = state.TargetCords
        self.assertTrue(state.arm_moving)
        for path, target_headers in (
            ("/target", {"X": "0", "Y": "230", "Z": "230", "A": "10", "Speed": "20"}),
            ("/target?X=0&Y=230&Z=230&A=10&Speed=20", {}),
        ):
            self.assertEqual(self.request("POST", path, target_headers)[0], 409)
            self.assertIs(state.TargetCords, original)
            self.assertEqual(state.TargetSpeed, 10)
        self.assertEqual(self.packet()[0], 204)
        self.assertEqual(self.request("POST", "/movement-complete")[0], 204)
        self.assertFalse(state.arm_moving)
        self.assertEqual(self.request("POST", "/target?X=0&Y=230&Z=230&A=10")[0], 204)
        self.packet(original)
        self.assertTrue(state.arm_moving)

    def test_invalid_target_leaves_target_unchanged(self):
        original = state.TargetCords
        for headers, expected in [
            ({"X": "0"}, 400),
            ({"X": "nan", "Y": "250", "Z": "250", "A": "0"}, 400),
            ({"X": "0", "Y": "1000", "Z": "1000", "A": "0"}, 422),
        ]:
            self.assertEqual(self.request("POST", "/target", headers)[0], expected)
            self.assertIs(state.TargetCords, original)

    def test_speed_changes_packet_timing(self):
        _, _, slow = self.packet()
        self.request("POST", "/movement-complete")
        self.assertEqual(self.request("POST", "/target",
                         {"X": "0", "Y": "250", "Z": "250", "A": "0", "Speed": "20"})[0], 204)
        _, _, fast = self.packet()
        self.request("POST", "/movement-complete")
        slow_lines, fast_lines = slow.splitlines(), fast.splitlines()
        self.assertEqual(len(slow_lines), len(fast_lines))
        for first, second in zip(slow_lines, fast_lines):
            self.assertEqual(first.split(b"/")[1:], second.split(b"/")[1:])
        self.assertAlmostEqual(float(slow_lines[-1].split(b"/")[0]),
                               2 * float(fast_lines[-1].split(b"/")[0]), delta=1)
        self.assertEqual(self.request("POST", "/target",
                         {"X": "0", "Y": "250", "Z": "250", "A": "0"})[0], 204)
        self.assertEqual(state.TargetSpeed, 20)

    def test_maximum_speed_is_accepted(self):
        self.assertEqual(self.request("POST", "/target",
                         {"X": "0", "Y": "250", "Z": "250", "A": "0",
                          "Speed": str(state.MAX_SPEED)})[0], 204)
        self.assertEqual(state.TargetSpeed, state.MAX_SPEED)

    def test_invalid_speed_leaves_target_and_speed_unchanged(self):
        original = state.TargetCords
        for speed in ("0", "-1", "nan", "inf", "bad", "", str(state.MAX_SPEED + 1)):
            self.assertEqual(self.request("POST", "/target",
                             {"X": "0", "Y": "230", "Z": "230", "A": "0", "Speed": speed})[0], 400)
            self.assertIs(state.TargetCords, original)
            self.assertEqual(state.TargetSpeed, 10)
        self.assertEqual(self.request("POST", "/target",
                         {"X": "0", "Y": "1000", "Z": "1000", "A": "0", "Speed": "20"})[0], 422)
        self.assertEqual(state.TargetSpeed, 10)

    def test_already_at_target_issues_no_movement(self):
        status, headers, _ = self.packet(state.TargetCords)
        self.assertEqual(status, 204)
        self.assertFalse(state.arm_moving)


if __name__ == "__main__":
    unittest.main()
