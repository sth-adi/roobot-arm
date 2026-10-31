"""Sam's backend (firmware/sam/backend) + our patch and planner (firmware/ours): built into a temp folder, then run over HTTP."""
import http.client
import math
import os
import subprocess
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from backend_harness import BackendTestCase, parse, header  # noqa: E402


class OursBackendTest(BackendTestCase):
    def test_his_own_tests_pass_on_the_built_backend(self):
        r = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests"], cwd=self.dir, capture_output=True, text=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr[-1500:])

    def test_a_stream_of_targets_makes_short_constant_speed_packets(self):
        for i in range(4):                                                 # the dashboard following a controller: a new target every moment
            self.target(60 + i, 250, 250, 0, speed=35)
        code, _, headers, body = self.report((0, 250, 250, 0))
        self.assertEqual(code, 200)
        rows = parse(body)
        self.assertLessEqual(rows[-1][0], 520)                             # half a second of the 1.7 s way, not all of it
        steps = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
        self.assertLess(max(steps) - min(steps), 0.3 * (sum(steps) / len(steps)))         # constant speed: no ease in or out of each little packet
        self.assertGreater(self.state.TargetCords.X, 59)                   # the target itself is unchanged
        # asked again from where that packet ended, it carries on toward the target
        end = rows[-1]
        code2, _, _, body2 = self.req("GET", "/UpdateMovement?s1=%.3f&s2=%.3f&s3=%.3f&s4=%.3f" % tuple(end[1:]))
        self.assertEqual(code2, 200)
        self.assertGreater(parse(body2)[-1][1], end[1])

    def test_a_single_target_is_still_one_long_eased_packet(self):
        self.assertEqual(self.target(60, 250, 250, 0, speed=35)[0], 204)
        code, _, _, body = self.report((0, 250, 250, 0))
        rows = parse(body)
        self.assertGreater(rows[-1][0], 1500)                              # all of the 60 mm at 35 mm/s
        self.assertLess(abs(rows[1][1] - rows[0][1]), 0.3 * abs(rows[len(rows) // 2][1] - rows[len(rows) // 2 - 1][1]) + 0.01)    # starts gently

    def test_when_the_targets_stop_the_last_packet_finishes_the_move(self):
        for i in range(4):
            self.target(60 + i, 250, 250, 0, speed=35)
        time.sleep(self.state.LIVE_WINDOW_S + 0.2)
        code, _, _, body = self.report((0, 250, 250, 0))
        self.assertGreater(parse(body)[-1][0], 1500)

    def test_the_boards_connection_stays_open_and_does_not_block_others(self):
        board = http.client.HTTPConnection("127.0.0.1", self.server.server_port)
        try:
            for _ in range(3):                                             # three requests on one connection
                board.request("GET", "/movement")
                r = board.getresponse()
                self.assertEqual(r.status, 200)
                r.read()
            self.assertEqual(self.req("GET", "/movement")[0], 200)         # while that connection is still open, another client is served at once
            board.request("GET", "/UpdateMovement?s1=90&s2=90&s3=90&s4=90")
            r = board.getresponse()
            self.assertIn(r.status, (200, 204, 422))
            r.read()
        finally:
            board.close()

    def test_planner_copy_is_the_dashboards(self):
        with open(os.path.join(self.dir, "motion", "safe_planner.py"), "rb") as f1, open(os.path.join(ROOT, "dashboard", "planner.py"), "rb") as f2:
            self.assertEqual(f1.read(), f2.read())

    def test_packet_is_checked_smooth_and_joint_speed_limited(self):
        self.target(0, 250, 250, 0, speed=50)                  # reaching it from the start below is a 100 mm move at 50 mm/s
        code, _, headers, body = self.report((0, 250, 150, 0))
        self.assertEqual(code, 200)
        rows = parse(body)
        steps, interval = header(body)
        self.assertEqual(steps, len(rows) - 1)                  # what his ExecuteMovementPacket requires: step count, interval, then steps + 1 rows
        self.assertEqual(interval, 20)
        start = self.IK(self.C(0, 250, 150, 0))
        self.assertAlmostEqual(rows[0][1], math.degrees(start.S1), places=2)
        self.assertAlmostEqual(rows[0][2], math.degrees(start.S2), places=2)
        gaps = {round(b[0] - a[0]) for a, b in zip(rows, rows[1:])}
        self.assertTrue(gaps <= {19, 20, 21}, gaps)             # a fixed 50 Hz
        for a, b in zip(rows, rows[1:]):
            for j in range(1, 5):
                self.assertLessEqual(abs(b[j] - a[j]) / ((b[0] - a[0]) / 1000.0), self.state.MAX_JOINT_DPS * 1.07 + 1.0)
        end = self.IK(self.C(0, 250, 250, 0))
        self.assertAlmostEqual(rows[-1][2], math.degrees(end.S2), places=2)

    def test_pitch_only_move_is_no_longer_ignored(self):
        self.target(0, 250, 250, 20)
        code, _, _, body = self.report((0, 250, 250, 0))
        self.assertEqual(code, 200)
        self.assertGreater(len(parse(body)), 3)

    def test_target_the_arm_could_not_reach_from_here_is_refused_with_a_reason(self):
        self.report((0, 250, 250, 0))                           # the arm says where it is
        before = self.state.TargetCords
        self.assertEqual(self.target(0, 5, 150)[0], 422)        # his own check (wrist behind the base) still applies
        self.assertIs(self.state.TargetCords, before)
        code, reason, _, _ = self.target(-30, 60, 150)          # passes his checks, but a joint would leave its calibrated range
        self.assertEqual(code, 422)
        self.assertIn("joint", reason.lower())
        self.assertIs(self.state.TargetCords, before)
        self.assertEqual(self.target(60, 250, 200)[0], 204)     # a sensible one still goes through

    def test_hold(self):
        self.assertEqual(self.req("POST", "/hold")[0], 409)     # nothing reported yet
        self.report((0, 250, 150, 0))
        self.target(60, 250, 150)
        self.assertEqual(self.req("POST", "/hold")[0], 204)
        self.assertAlmostEqual(self.state.TargetCords.X, 0.0, places=3)
        self.assertEqual(self.report((0, 250, 150, 0))[0], 204)  # nothing left to do

    def test_while_a_move_is_played_a_new_target_is_refused_but_hold_works(self):
        self.assertEqual(self.target(60, 250, 150)[0], 204)
        self.assertEqual(self.report((0, 250, 150, 0))[0], 200)       # now the arm is moving
        self.assertEqual(self.req("GET", "/movement")[3], b"true")
        self.assertEqual(self.target(0, 250, 200)[0], 409)            # his rule
        self.assertEqual(self.report((0, 250, 150, 0))[0], 204)       # and no second packet
        self.assertEqual(self.req("POST", "/hold")[0], 204)           # ours: the target can still be cancelled
        self.assertAlmostEqual(self.state.TargetCords.X, 0.0, places=3)
        self.req("POST", "/movement-complete")
        self.assertEqual(self.req("GET", "/movement")[3], b"false")
        self.assertEqual(self.target(0, 250, 200)[0], 204)

    def test_every_angle_is_inside_what_his_firmware_accepts(self):
        self.target(0, 250, 250, 0, speed=50)
        for pose in ((0, 250, 150, 0), (-100, 250, 200, 0)):
            self.req("POST", "/movement-complete")
            code, _, _, body = self.report(pose)
            if code == 200:
                for row in parse(body):
                    self.assertTrue(all(0.0 <= v <= 180.0 for v in row[1:]), row)

    def test_a_move_that_goes_wrong_midway_is_refused_whole(self):
        self.report((0, 250, 250, 0))
        self.state.TargetCords = self.C(0, 5, 150, 0)            # set around the /target checks, as a stale target would be
        code, reason, _, body = self.report((0, 250, 250, 0))
        self.assertEqual(code, 422)
        self.assertIn("base axis", reason)
        self.assertNotRegex(body, rb"\d+/-?\d+\.\d+/")             # an error page, no part of a packet


if __name__ == "__main__":
    unittest.main()
