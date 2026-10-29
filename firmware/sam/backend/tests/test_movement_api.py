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
        self.old_state = state.movement_state.copy()
        state.TargetCords = Coordinates(0, 250, 250, 0)
        state.movement_state.update(id=None, status="idle", target=None)
        self.server = HTTPServer(("127.0.0.1", 0), handlers.Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        state.TargetSpeed = self.old_speed
        state.TargetCords = self.old_target
        state.movement_state.clear()
        state.movement_state.update(self.old_state)

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
        status, headers, body = self.packet()
        self.assertEqual(status, 200)
        self.assertTrue(body)
        movement_id = headers["Movement-ID"]
        self.assertEqual(state.movement_state["status"], "in_progress")
        for _ in range(2):
            self.assertEqual(self.request("POST", "/movement-complete",
                             {"Movement-ID": movement_id})[0], 204)
        status, _, body = self.request("GET", "/movement")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["status"], "complete")

    def test_bad_completion_ids(self):
        self.assertEqual(self.request("POST", "/movement-complete")[0], 400)
        self.assertEqual(self.request("POST", "/movement-complete",
                         {"Movement-ID": "unknown"})[0], 409)

    def test_target_during_move_keeps_old_completion_separate(self):
        _, headers, _ = self.packet()
        old_id = headers["Movement-ID"]
        self.assertEqual(self.request("POST", "/target",
                         {"X": "0", "Y": "230", "Z": "230", "A": "10"})[0], 204)
        self.assertEqual(state.movement_state["target"]["y"], 250)
        self.assertEqual(self.request("POST", "/movement-complete",
                         {"Movement-ID": old_id})[0], 204)
        self.assertEqual(state.TargetCords.Y, 230)
        self.assertAlmostEqual(state.TargetCords.A, math.radians(10))
        _, new_headers, _ = self.packet()
        self.assertNotEqual(new_headers["Movement-ID"], old_id)
        self.assertEqual(self.request("POST", "/movement-complete",
                         {"Movement-ID": old_id})[0], 409)

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
        self.assertEqual(self.request("POST", "/target",
                         {"X": "0", "Y": "250", "Z": "250", "A": "0", "Speed": "20"})[0], 204)
        _, _, fast = self.packet()
        slow_lines, fast_lines = slow.splitlines(), fast.splitlines()
        self.assertEqual(len(slow_lines), len(fast_lines))
        for first, second in zip(slow_lines, fast_lines):
            self.assertEqual(first.split(b"/")[1:], second.split(b"/")[1:])
        self.assertAlmostEqual(float(slow_lines[-1].split(b"/")[0]),
                               2 * float(fast_lines[-1].split(b"/")[0]), delta=1)
        self.assertEqual(state.movement_state["target"]["speed"], 20)
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
        self.assertNotIn("Movement-ID", headers)
        self.assertIsNone(state.movement_state["id"])


if __name__ == "__main__":
    unittest.main()
