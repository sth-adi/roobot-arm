"""Tests for the arm side channel on server.py (/status, /calibration, /mode). Run:  python -m unittest sim.test_server -v"""
import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import server


class SideChannel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def req(self, path, data=None, ctype="application/json"):
        r = urllib.request.Request(self.base + path, data=data, method="GET" if data is None else "POST",
                                   headers={"Content-Type": ctype})
        try:
            with urllib.request.urlopen(r, timeout=3) as resp:
                return resp.status, resp.read(), resp.headers
        except urllib.error.HTTPError as e:
            return e.code, e.read(), e.headers

    def test_status_roundtrip(self):
        code, _, _ = self.req("/status", b"Position 1 saved", "text/plain")
        self.assertEqual(code, 204)
        code, body, h = self.req("/status")
        self.assertEqual((code, body), (200, b"Position 1 saved"))
        self.assertTrue(h["Content-Type"].startswith("text/plain"))

    def test_status_size_limit(self):
        self.assertEqual(self.req("/status", b"x" * 5000, "text/plain")[0], 400)

    def test_calibration(self):
        good = {"jointLimits": [[0, 180], [180, 0], [10, 170], [0, 180]], "jointHomes": [90, 100, 80, 90]}
        self.assertEqual(self.req("/calibration", json.dumps(good).encode())[0], 204)
        code, body, _ = self.req("/calibration")
        self.assertEqual((code, json.loads(body)), (200, good))
        for bad in ({"jointLimits": [[0, 180]] * 3, "jointHomes": [90] * 4},
                    {"jointLimits": [[0, 181]] * 4, "jointHomes": [90] * 4},
                    {"jointLimits": [[0, 180]] * 4, "jointHomes": [90, 90, 90, 90.5]},
                    {"jointLimits": [[0, 180]] * 4, "jointHomes": [True, 90, 90, 90]},
                    {"jointLimits": [[0, 180]] * 4}):
            self.assertEqual(self.req("/calibration", json.dumps(bad).encode())[0], 400, bad)
        self.assertEqual(json.loads(self.req("/calibration")[1]), good)    # a rejected post changes nothing

    def test_mode_header(self):
        code, body, h = self.req("/mode")
        self.assertEqual((code, body, h["Mode"]), (200, b"true", "Normal"))

    def test_input_still_works_with_the_wire_format(self):
        pkt = {"LSX": 0.1, "LSY": -0.2, "RSX": 0, "RSY": 0, "A": True, "B": False, "X": False, "Y": False}
        self.assertEqual(self.req("/input", json.dumps(pkt).encode())[0], 200)
        state = json.loads(self.req("/state")[1])
        self.assertEqual(state["state"], pkt)


class _EndServer:
    """Stands in for the UWE-RArm backend: records every packet forwarded to it."""

    def __init__(self):
        got = self.got = []

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                got.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
                self.send_response(204)
                self.end_headers()

            def log_message(self, *a):
                pass

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_address[1]}/controller"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()


HELD = {"LSX": 0.9, "LSY": -0.4, "RSX": 0.0, "RSY": 0.0, "A": True, "B": False, "X": False, "Y": False}
NEUTRAL = {"LSX": 0.0, "LSY": 0.0, "RSX": 0.0, "RSY": 0.0, "A": False, "B": False, "X": False, "Y": False}


class DeadMan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        server.STALE_S = 0.3
        threading.Thread(target=server._forward_worker, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    _base = 0

    def setUp(self):
        self.end = _EndServer()
        DeadMan._base += 1000        # the worker remembers the last count it forwarded, so every test starts higher
        with server._lock:
            server._state.update(data=None, received_at=None, count=DeadMan._base)
            server._fwd.update(url=self.end.url, ok=None, error=None, sent=0, failed=0, enabled=True,
                               neutrals=0, flush_neutral=False)

    def tearDown(self):
        self.end.srv.shutdown()
        self.end.srv.server_close()

    def post(self, path, obj):
        r = urllib.request.Request(self.base + path, data=json.dumps(obj).encode(), method="POST",
                                   headers={"Content-Type": "application/json"})
        urllib.request.urlopen(r, timeout=3).read()

    def test_dashboard_buttons_post_a_command(self):
        self.post("/controller", {"command": "home"})
        self.assertEqual(server._cmd["command"], "home")
        for bad in ({"command": "dance"}, {}):
            with self.assertRaises(urllib.error.HTTPError) as c:
                self.post("/controller", bad)
            self.assertEqual(c.exception.code, 400)
        self.assertEqual(server._cmd["command"], "home")

    def wait_for(self, cond, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(0.05)
        return False

    def test_neutral_is_all_zero_and_released(self):
        self.assertEqual(server._neutral_of(HELD), NEUTRAL)
        self.assertEqual(server._neutral_of({"lx": 0.5, "buttons": {"a": True}, "dpad": [1, -1], "name": "x"}),
                         {"lx": 0.0, "buttons": {"a": False}, "dpad": [0, 0], "name": "x"})

    def test_stale_input_sends_neutral_once(self):
        self.post("/input", HELD)
        self.assertTrue(self.wait_for(lambda: len(self.end.got) >= 2), self.end.got)
        self.assertEqual(self.end.got[0], HELD)
        self.assertEqual(self.end.got[1], NEUTRAL)
        time.sleep(1.2)
        self.assertEqual(len(self.end.got), 2)               # one neutral, not a stream of them

    def test_live_input_is_not_neutralised(self):
        for _ in range(8):
            self.post("/input", HELD)
            time.sleep(0.1)
        self.assertNotIn(NEUTRAL, self.end.got)

    def test_pausing_forwarding_stops_the_arm(self):
        self.post("/input", HELD)
        self.assertTrue(self.wait_for(lambda: len(self.end.got) >= 1))
        self.post("/control", {"forward": False})
        self.assertTrue(self.wait_for(lambda: NEUTRAL in self.end.got), self.end.got)
        n = len(self.end.got)
        self.post("/input", HELD)                             # paused: dropped, not forwarded
        time.sleep(0.8)
        self.assertEqual(len(self.end.got), n)

    def test_max_speed_goes_with_every_packet_including_neutral(self):
        self.post("/control", {"maxSpeed": 25})
        self.post("/input", HELD)
        self.assertTrue(self.wait_for(lambda: len(self.end.got) >= 2), self.end.got)
        self.assertEqual(self.end.got[0], {**HELD, "maxSpeed": 25})
        self.assertEqual(self.end.got[1], {**NEUTRAL, "maxSpeed": 25})   # neutral keeps the limit
        self.assertEqual(json.loads(urllib.request.urlopen(self.base + "/controller", timeout=3).read())["maxSpeed"], 25)
        with server._lock:
            server._arm["max_speed"] = None

    def test_max_speed_validation(self):
        for bad in (0, 61, -1, "fast", True, None):
            with self.assertRaises(urllib.error.HTTPError, msg=repr(bad)) as cm:
                self.post("/control", {"maxSpeed": bad})
            self.assertEqual(cm.exception.code, 400)
        with self.assertRaises(urllib.error.HTTPError):
            self.post("/control", {})
        self.post("/control", {"maxSpeed": 60, "forward": True})        # both at once, upper bound allowed
        with server._lock:
            self.assertEqual(server._arm["max_speed"], 60)
            server._arm["max_speed"] = None

    def test_controller_endpoint(self):
        self.assertEqual(json.loads(urllib.request.urlopen(self.base + "/controller", timeout=3).read()), {})
        self.post("/input", HELD)
        self.assertEqual(json.loads(urllib.request.urlopen(self.base + "/controller", timeout=3).read()), HELD)
        time.sleep(0.5)                                       # stale: the firmware must see neutral
        self.assertEqual(json.loads(urllib.request.urlopen(self.base + "/controller", timeout=3).read()), NEUTRAL)


if __name__ == "__main__":
    unittest.main()


class StretchTest(unittest.TestCase):
    def setUp(self):
        server._pressed_at.clear()

    def test_quick_tap_is_kept_pressed_then_released(self):
        with server._lock:
            d, wait = server._stretch_presses({"A": True, "B": False}, 100.0)
            self.assertEqual((d["A"], wait), (True, 0.0))
            d, wait = server._stretch_presses({"A": False, "B": False}, 100.05)     # released after 50 ms
            self.assertTrue(d["A"])
            self.assertAlmostEqual(wait, server.MIN_PRESS_S - 0.05, places=3)
            d, wait = server._stretch_presses({"A": False, "B": False}, 100.4)      # a later packet: long enough, released
            self.assertFalse(d["A"])
            self.assertEqual(wait, 0.0)

    def test_long_press_is_not_changed(self):
        with server._lock:
            server._stretch_presses({"A": True}, 10.0)
            d, wait = server._stretch_presses({"A": False}, 11.0)
        self.assertFalse(d["A"])
        self.assertEqual(wait, 0.0)


class PageAndArm(unittest.TestCase):
    """The page (/, /Noah.png) and the arm's front door routes (/arm/...) - buttons and typed text both go through them, only typed text reaches the AI."""
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self._saved = dict(server._front)
        self.addCleanup(lambda: server._front.update(self._saved))

    def call(self, path, body=None, headers=None):
        r = urllib.request.Request(self.base + path, data=None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode()),
                                   method="GET" if body is None else "POST", headers=headers or {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(r, timeout=5) as resp:
                return resp.status, resp.read(), resp.headers
        except urllib.error.HTTPError as e:
            return e.code, e.read(), e.headers

    def test_the_page_and_its_picture_are_served(self):
        code, body, h = self.call("/")
        self.assertEqual(code, 200)
        self.assertTrue(h["Content-Type"].startswith("text/html"))
        self.assertIn(b"<html", body.lower())
        code, body, h = self.call("/Noah.png")
        self.assertEqual((code, h["Content-Type"], body[:4]), (200, "image/png", b"\x89PNG"))

    def test_routes_say_so_when_the_front_door_is_off(self):
        server._front["door"] = None
        self.assertEqual(self.call("/arm/state")[0], 503)
        self.assertEqual(self.call("/arm/request", {"action": "stop"})[0], 503)

    def test_a_button_and_the_status_go_through_the_virtual_arm(self):
        server._front["door"] = server.make_front("virtual")
        code, body, _ = self.call("/arm/request", {"v": 1, "id": "t", "action": "fetch", "key": 2})
        r = json.loads(body)
        self.assertEqual((code, r["kind"], r["by"]), (200, "started", "button"))
        server._front["door"].runner.wait(20)
        code, body, _ = self.call("/arm/job")
        self.assertEqual((code, json.loads(body)["status"]), (200, "done"))
        self.assertEqual(json.loads(self.call("/arm/state")[1])["keys"]["2"], "tray")

    def test_typed_text_is_confirmed_before_anything_moves(self):
        door = server._front["door"] = server.make_front("virtual")
        door.decide = lambda text, state: {"action": "fetch_key", "key": 3, "by": "keywords"}
        r = json.loads(self.call("/arm/request", {"v": 1, "id": "t", "text": "get key three"})[1])
        self.assertEqual(r["kind"], "confirm")
        self.assertEqual(door.runner.last, None)
        r = json.loads(self.call("/arm/request", {"confirm": r["token"]})[1])
        self.assertEqual(r["kind"], "started")
        door.runner.wait(20)

    def test_bad_input_is_a_clear_error_and_big_bodies_are_refused(self):
        server._front["door"] = server.make_front("virtual")
        self.assertEqual(self.call("/arm/request", b"not json")[0], 400)
        self.assertEqual(self.call("/arm/request", {"text": ""})[0], 400)
        self.assertEqual(self.call("/arm/request", b"x" * 9000)[0], 413)
        self.assertEqual(self.call("/arm/nowhere")[0], 404)

    def test_the_old_controller_routes_still_work(self):
        self.assertEqual(self.call("/controller", {"command": "stop"})[0], 200)
        self.assertEqual(self.call("/health")[0], 200)
