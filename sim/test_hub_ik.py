"""The hub side of "the board runs the inverse kinematics": reading the real arm's calibration from the end server (read-only), giving it to the
virtual arm, the local server and the board, and choosing between pose frames and joint-angle frames. No board and no network are needed."""
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import endcal  # noqa: E402
import serial_link  # noqa: E402

REAL = {"jointLimits": [[11, 180], [0, 115], [161, 21], [0, 180]], "joint0s": [95, 115, 27, 127]}      # what the end server returned (old key name for the homes)
REAL_PAIR = ([[11, 180], [0, 115], [161, 21], [0, 180]], [95, 115, 27, 127])


class FakeSerial:
    def __init__(self):
        self.out, self.in_waiting = [], 0

    def write(self, b):
        self.out.append(b.decode("ascii"))

    def read(self, n):
        return b""

    def close(self):
        pass


def linked():
    link = serial_link.SerialLink(enabled=True)
    link.enabled = True
    link.ser, link.device = FakeSerial(), "COMFAKE"
    return link


class NormaliseTest(unittest.TestCase):
    def test_the_end_servers_answer_is_read_including_the_old_home_key(self):
        self.assertEqual(endcal.normalise(REAL), REAL_PAIR)
        self.assertEqual(endcal.normalise({"jointLimits": REAL["jointLimits"], "jointHomes": REAL["joint0s"]}), REAL_PAIR)
        self.assertEqual(endcal.normalise({"jointLimits": [[0.0, 180.0]] * 4, "jointHomes": [90.0] * 4}), ([[0, 180]] * 4, [90] * 4))

    def test_anything_else_is_refused(self):
        good = dict(jointLimits=[[0, 180]] * 4, jointHomes=[90] * 4)
        for bad in (None, [], {}, dict(good, jointLimits=[[0, 180]] * 3), dict(good, jointLimits=[[0, 181]] * 4), dict(good, jointLimits=[[-1, 180]] * 4),
                    dict(good, jointLimits=[[90, 90]] * 4), dict(good, jointLimits=[[0.5, 180]] * 4), dict(good, jointHomes=[90, 90, 90]),
                    dict(good, jointHomes=[90, 90, 90, 200]), dict(good, jointHomes=[True, 90, 90, 90]), dict(good, jointLimits=[["0", "180"]] * 4),
                    dict(good, jointLimits=[[float("nan"), 180]] * 4)):
            with self.assertRaises(ValueError, msg=str(bad)):
                endcal.normalise(bad)

    def test_the_address_is_the_server_not_the_input_path(self):
        self.assertEqual(endcal.base_url("https://example.org/input"), "https://example.org")
        self.assertEqual(endcal.base_url(" http://10.0.0.5:8000/input "), "http://10.0.0.5:8000")
        for bad in ("", None, "example.org/input", "ftp://x/y"):
            with self.assertRaises(ValueError):
                endcal.base_url(bad)

    def test_the_cache_round_trips_and_a_damaged_one_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "cal.json")
            endcal.save_cache(*REAL_PAIR, path=p)
            self.assertEqual(endcal.load_cache(p), REAL_PAIR)
            with open(p, "w") as f:
                f.write("{not json")
            with self.assertRaises(ValueError):
                endcal.load_cache(p)


class FetchTest(unittest.TestCase):
    def test_it_only_reads(self):
        seen = []

        class H(BaseHTTPRequestHandler):
            def do_GET(self):
                seen.append(("GET", self.path))
                body = json.dumps(REAL).encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                seen.append(("POST", self.path))
                self.send_response(500)
                self.end_headers()

            def log_message(self, *a):
                pass

        srv = HTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            got = endcal.fetch("http://127.0.0.1:%d/input" % srv.server_address[1], timeout=3)
        finally:
            srv.shutdown()
            srv.server_close()
        self.assertEqual(got, REAL_PAIR)
        self.assertEqual(seen, [("GET", "/calibration")])


class HubCalibrationTest(unittest.TestCase):
    def hub(self):
        import hub as H
        h = H.Hub("http://127.0.0.1:1")
        h.link = linked()
        h.push_calibration = mock.Mock(return_value=True)          # the local server is not running here
        h.env_forward = lambda: "https://end.example/input"
        return h

    def test_loading_the_real_calibration_sets_the_virtual_arm_the_local_server_and_the_board(self):
        h = self.hub()
        with mock.patch.object(endcal, "fetch", return_value=REAL_PAIR), mock.patch.object(endcal, "save_cache") as save:
            ok, msg = h.load_end_calibration()
        self.assertTrue(ok, msg)
        self.assertEqual((h.sim.limits, h.sim.homes), REAL_PAIR)
        self.assertEqual(h.sim.tool, 53.0)                                  # the board's IK has Sam's 53 mm built in
        self.assertEqual(h.cal_source, "end server")
        h.push_calibration.assert_called_once()
        self.assertEqual(h.link.ser.out[-1], "!cal 11,180,0,115,161,21,0,180,95,115,27,127\n")
        save.assert_called_once()

    def test_a_server_that_cannot_be_reached_falls_back_to_the_last_one_saved_else_says_so(self):
        h = self.hub()
        with mock.patch.object(endcal, "fetch", side_effect=OSError("no route")), mock.patch.object(endcal, "load_cache", return_value=REAL_PAIR):
            ok, _ = h.load_end_calibration()
        self.assertTrue(ok)
        self.assertEqual((h.cal_source, h.sim.limits), ("cache", REAL_PAIR[0]))
        h2 = self.hub()
        with mock.patch.object(endcal, "fetch", side_effect=OSError("no route")), mock.patch.object(endcal, "load_cache", side_effect=OSError("none")):
            ok, msg = h2.load_end_calibration()
        self.assertFalse(ok)
        self.assertIn("none is saved", msg)
        self.assertEqual(h2.cal_source, "default")
        self.assertEqual(h2.sim.limits, [[0, 180]] * 4)                     # nothing changed

    def test_it_is_refused_while_the_real_arm_is_following(self):
        h = self.hub()
        h.armed, h.mirror = True, {"on": True}
        with mock.patch.object(endcal, "fetch", return_value=REAL_PAIR) as fetch:
            ok, msg = h.load_end_calibration()
        self.assertFalse(ok)
        self.assertIn("SAFE", msg)
        fetch.assert_not_called()
        self.assertEqual(h.sim.limits, [[0, 180]] * 4)


class StreamChoiceTest(unittest.TestCase):
    """Poses go to a board that runs the IK and has the calibration; angles go to everything else."""
    def hub(self, ik, tool=53.0):
        import hub as H
        h = H.Hub("http://127.0.0.1:1")
        h.link = linked()
        h.link.board_ik = ik
        h.sim.set_tool(tool)
        h.mirror = {"on": True}
        h.armed = True
        return h

    def first_frame(self, h):
        t = threading.Thread(target=h._pump, daemon=True)
        t.start()
        try:
            end = time.time() + 3
            while time.time() < end and not any(o[0] in "@#" for o in h.link.ser.out):
                time.sleep(0.02)
        finally:
            h.stop.set()
            t.join(2)
        return next(o for o in h.link.ser.out if o[0] in "@#")

    def test_a_board_with_the_calibration_gets_the_tip_pose(self):
        h = self.hub("set")
        self.assertEqual(self.first_frame(h)[0], "#")
        self.assertEqual(h.mirror_status()["ik"], "board")

    def test_an_older_sketch_or_a_board_without_the_calibration_gets_joint_angles(self):
        for ik in (None, "missing"):
            h = self.hub(ik)
            self.assertEqual(self.first_frame(h)[0], "@", ik)
            self.assertEqual(h.mirror_status()["ik"], "hub")

    def test_a_different_tool_length_keeps_to_joint_angles_so_the_poses_cannot_mean_something_else(self):
        h = self.hub("set", tool=63.0)
        self.assertEqual(self.first_frame(h)[0], "@")

    def test_the_joint_tests_use_joint_angles(self):
        h = self.hub("set")
        h._mirror_angle = (1, 60.0)
        self.assertEqual(self.first_frame(h)[0], "@")


if __name__ == "__main__":
    unittest.main()
