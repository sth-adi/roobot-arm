"""dashboard/movement_service.py against a small model of Sam's firmware: poll, play the packet, report the position with his 53 mm forward kinematics."""
import importlib.util
import os
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import movement_service as MS  # noqa: E402
import planner as PL  # noqa: E402
from test_planner import fk, parse  # noqa: E402

_spec = importlib.util.spec_from_file_location("sam_apc", os.path.join(ROOT, "firmware", "sam", "backend", "AnglePacketCompilation.py"))
SAM = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(SAM)


def f32(v):
    return struct.unpack("f", struct.pack("f", v))[0]


class FakeEsp:
    """Starts at a pose, reports its position like ForwardKinTransform.h (float32, 3 decimals in the query), and plays packets to their last line."""
    def __init__(self, tool, start):
        self.tool = tool
        self.q = PL.inverse_kim(*start)

    def report(self):
        x, y, z, a = fk(self.q, self.tool)
        return float("%.3f" % f32(x)), float("%.3f" % f32(y)), float("%.3f" % f32(z)), float("%.3f" % f32(a))

    def play(self, body):
        rows = parse(body.strip().split("\n"))
        for a, b in zip(rows, rows[1:]):
            assert b[0] >= a[0]
        self.q = rows[-1][1:]
        return rows


class Basics(unittest.TestCase):
    def test_disarmed_never_moves(self):
        s = MS.MovementService()
        s.set_target({"x": 0, "y": 250, "z": 150, "angle": 0})
        self.assertEqual(s.update(0, 240, 250, 0), (204, ""))
        self.assertEqual(s.snapshot()["packets_sent"], 0)

    def test_arrives_and_then_stays(self):
        s = MS.MovementService(fw_tool_mm=53.0)
        s.set_armed(True)
        s.set_target({"x": 0, "y": 250, "z": 150, "angle": 0})
        esp = FakeEsp(53.0, MS.HOME_TARGET)
        codes = []
        for _ in range(6):
            code, body = s.update(*esp.report())
            codes.append(code)
            if code == 200:
                esp.play(body)
        self.assertEqual(codes, [200, 204, 204, 204, 204, 204])
        self.assertEqual(s.snapshot()["poses_reached"], 1)

    def test_a_refused_move_clears_the_queue_and_says_why(self):
        s = MS.MovementService()
        s.set_armed(True)
        s.set_queue([{"x": 0, "y": 250, "z": 150, "angle": 0}, {"x": 0, "y": 700, "z": 150, "angle": 0}, {"x": 0, "y": 250, "z": 100, "angle": 0}])
        esp = FakeEsp(53.0, MS.HOME_TARGET)
        got = []
        for _ in range(8):
            code, body = s.update(*esp.report())
            got.append(code)
            if code == 200:
                esp.play(body)
        self.assertIn(422, got)
        self.assertEqual(s.snapshot()["queued"], 0)
        self.assertIn(s.snapshot()["error"]["code"], ("UNREACHABLE", "JOINT_LIMIT"))
        self.assertEqual(got[-1], 422)                                    # it keeps refusing; it does not skip to the third pose

    def test_queue_runs_in_order_and_applies_magnet_actions_on_arrival(self):
        s = MS.MovementService()
        s.set_armed(True)
        s.set_queue([{"x": 0, "y": 250, "z": 120, "angle": 0, "action": None}, {"x": 0, "y": 250, "z": 100, "angle": 0, "action": "grab"},
                     {"x": 60, "y": 250, "z": 150, "angle": 0, "action": None}, {"x": 60, "y": 250, "z": 120, "angle": 0, "action": "release"}])
        esp = FakeEsp(53.0, MS.HOME_TARGET)
        seen_magnet = []
        for _ in range(14):
            code, body = s.update(*esp.report())
            if code == 200:
                esp.play(body)
            seen_magnet.append(s.snapshot()["magnet"])
        self.assertEqual(s.snapshot()["poses_reached"], 4)
        self.assertTrue(any(seen_magnet))
        self.assertFalse(s.snapshot()["magnet"])                           # released at the end
        self.assertEqual(seen_magnet.index(True) > 0, True)

    def test_disarming_drops_the_queue(self):
        s = MS.MovementService()
        s.set_armed(True)
        s.set_queue([{"x": 0, "y": 250, "z": 150, "angle": 0}, {"x": 0, "y": 250, "z": 100, "angle": 0}])
        s.set_armed(False)
        self.assertEqual(s.snapshot()["queued"], 0)
        self.assertEqual(s.update(0, 240, 250, 0), (204, ""))

    def test_bad_input_is_rejected(self):
        s = MS.MovementService()
        for bad in ([], "x", [{"x": 1}], [{"x": 0, "y": 250, "z": 1, "angle": 0, "action": "fly"}], [{"x": float("nan"), "y": 1, "z": 1, "angle": 0}], [{"x": 0, "y": 250, "z": 1e6, "angle": 0}]):
            with self.assertRaises(ValueError):
                s.set_queue(bad)
        with self.assertRaises(ValueError):
            s.set_calibration([[0, 200]] * 4, [1, 2, 3, 4])


class HuntingInSamsOwnBackend(unittest.TestCase):
    """Documents the problem in his code (firmware says 53 mm, backend plans 63 mm) by running his own CompileAnglePacket."""
    def test_his_backend_keeps_correcting_an_arm_that_is_already_on_target(self):
        target = SAM.Coordinates(0, 250, 250, 0)
        esp = FakeEsp(53.0, (0, 250, 250, 0))                            # sitting exactly on the target
        corrections, biggest_jump = 0, 0.0
        for _ in range(5):
            cur = SAM.Coordinates(*esp.report())
            if (cur.X, cur.Y, cur.Z, cur.A) == (target.X, target.Y, target.Z, target.A):
                continue
            rows = parse(SAM.CompileAnglePacket(cur, target, 10, 5))
            corrections += 1
            biggest_jump = max(biggest_jump, max(abs(rows[0][j + 1] - esp.q[j]) for j in range(4)))   # first line vs where the servos really are
            esp.q = rows[-1][1:]
        self.assertEqual(corrections, 5)                                  # never settles
        self.assertGreater(biggest_jump, 3.0)                              # and each time it starts with a jump of several degrees

    def test_ours_settles(self):
        s = MS.MovementService(fw_tool_mm=53.0)
        s.set_armed(True)
        s.set_target({"x": 0, "y": 250, "z": 250, "angle": 0})
        esp = FakeEsp(53.0, (0, 250, 250, 0))
        self.assertEqual([s.update(*esp.report())[0] for _ in range(5)], [204] * 5)


class OverHttp(unittest.TestCase):
    def setUp(self):
        import json
        import threading
        import urllib.request
        import urllib.error
        import sam_server
        self.json, self.rq, self.err = json, urllib.request, urllib.error
        self.svc = MS.MovementService(53.0)
        self.srv = sam_server.ThreadingHTTPServer(("127.0.0.1", 0), sam_server.make_handler(self.svc, "secret"))
        self.base = "http://127.0.0.1:%d" % self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def call(self, path, body=None, token="secret"):
        data = None if body is None else self.json.dumps(body).encode()
        req = self.rq.Request(self.base + path, data, {"X-Arm-Token": token, "Content-Type": "application/json"} if token is not None else {}, method="POST" if data is not None else "GET")
        try:
            with self.rq.urlopen(req, timeout=5) as r:
                return r.status, r.read().decode()
        except self.err.HTTPError as e:
            return e.code, e.read().decode()

    def test_control_needs_the_token_his_endpoints_do_not(self):
        self.assertEqual(self.call("/arm", {"armed": True}, token="nope")[0], 403)
        self.assertEqual(self.call("/state", token=None)[0], 403)
        self.assertEqual(self.call("/mode", token=None), (200, "true"))
        self.assertEqual(self.call("/UpdateMovement?x=0&y=240&z=250&a=0", token=None)[0], 204)          # disarmed
        self.assertEqual(self.call("/calibration", token=None)[0], 200)

    def test_full_move_over_http(self):
        self.assertEqual(self.call("/arm", {"armed": True})[0], 200)
        self.assertEqual(self.call("/queue", {"poses": [{"x": 0, "y": 250, "z": 150, "angle": 0}, {"x": 60, "y": 250, "z": 150, "angle": 0}]})[0], 200)
        esp = FakeEsp(53.0, MS.HOME_TARGET)
        codes = []
        for _ in range(6):
            x, y, z, a = esp.report()
            code, body = self.call("/UpdateMovement?x=%s&y=%s&z=%s&a=%s" % (x, y, z, a), token=None)
            codes.append(code)
            if code == 200:
                esp.play(body)
        self.assertEqual(codes, [200, 200, 204, 204, 204, 204])
        st = self.json.loads(self.call("/state")[1])
        self.assertEqual(st["poses_reached"], 2)
        self.assertAlmostEqual(PL.tool_tip(*esp.report())[0], 60.0, places=1)

    def test_refused_move_is_422_with_the_reason(self):
        self.call("/arm", {"armed": True})
        self.call("/target", {"x": 0, "y": 800, "z": 150, "angle": 0})
        code, body = self.call("/UpdateMovement?x=0&y=240&z=250&a=0", token=None)
        self.assertEqual(code, 422)
        self.assertIn(self.json.loads(body)["code"], ("UNREACHABLE", "JOINT_LIMIT"))

    def test_bad_requests(self):
        self.assertEqual(self.call("/UpdateMovement?x=1", token=None)[0], 400)
        self.assertEqual(self.call("/UpdateMovement?x=nan&y=1&z=1&a=0", token=None)[0], 400)
        self.assertEqual(self.call("/target", {"x": 1})[0], 400)
        self.assertEqual(self.call("/calibration", {"jointLimits": [[0, 999]] * 4, "jointHomes": [1, 2, 3, 4]}, token=None)[0], 400)
        self.assertEqual(self.call("/nope", token=None)[0], 404)


if __name__ == "__main__":
    unittest.main()
