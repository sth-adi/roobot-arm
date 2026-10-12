"""Tests for the arm side channel on server.py (/status, /calibration, /mode). Run:  python -m unittest sim.test_server -v"""
import json
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

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


if __name__ == "__main__":
    unittest.main()
