"""Shared test setup: Sam's backend + our patch (firmware/ours), built once into a temp folder and served in this process on a free port."""
import atexit
import http.client
import importlib.util
import math
import os
import sys
import tempfile
import threading
import unittest
from http.server import HTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_MODS = ("api", "api.handlers", "models", "models.robot", "motion", "motion.kinematics", "motion.packets", "motion.safe_planner", "GlobalVariables")
_built = []


def built_dir():
    if not _built:
        spec = importlib.util.spec_from_file_location("build_backend", os.path.join(ROOT, "firmware", "ours", "build_backend.py"))
        bb = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bb)
        tmp = tempfile.TemporaryDirectory()
        atexit.register(tmp.cleanup)
        _built.append(str(bb.build(os.path.join(tmp.name, "backend"))))
    return _built[0]


def parse(body):
    """The rows of a packet: "time_ms/j1/j2/j3/j4". The first two lines (step count, interval in ms) are left out; see header()."""
    return [[float(v) for v in l.split("/")] for l in body.decode().strip().split("\n") if "/" in l]


def header(body):
    """(step count, interval ms) from the first two lines of a packet."""
    lines = body.decode().strip().split("\n")
    return int(lines[0]), int(lines[1])


class BackendTestCase(unittest.TestCase):
    def setUp(self):
        self.dir = built_dir()
        for m in _MODS:
            sys.modules.pop(m, None)
        sys.path.insert(0, self.dir)
        from api import handlers
        import GlobalVariables as state
        from motion.kinematics import InverseKim
        from models.robot import Coordinates
        self.state, self.IK, self.C = state, InverseKim, Coordinates
        self.server = HTTPServer(("127.0.0.1", 0), handlers.Handler)
        self.url = "http://127.0.0.1:%d" % self.server.server_port
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        sys.path.remove(self.dir)
        for m in _MODS:
            sys.modules.pop(m, None)

    def req(self, method, path, headers=None):
        c = http.client.HTTPConnection("127.0.0.1", self.server.server_port)
        try:
            c.request(method, path, headers=headers or {})
            r = c.getresponse()
            return r.status, r.reason, dict(r.getheaders()), r.read()
        finally:
            c.close()

    def report(self, pose):
        a = self.IK(self.C(*pose))
        q = "&".join("s%d=%.3f" % (i + 1, math.degrees(v)) for i, v in enumerate((a.S1, a.S2, a.S3, a.S4)))
        return self.req("GET", "/UpdateMovement?" + q)

    def target(self, x, y, z, a=0, speed=None):
        h = {"X": str(x), "Y": str(y), "Z": str(z), "A": str(a)}
        if speed is not None:
            h["Speed"] = str(speed)
        return self.req("POST", "/target", h)


class FakeEsp(threading.Thread):
    """Behaves like Sam's firmware: report joint angles, play the packet (instantly, in this test), confirm, repeat."""
    def __init__(self, harness, start_pose, gap=0.03):
        super().__init__(daemon=True)
        self.h, self.gap, self.running = harness, gap, True
        a = harness.IK(harness.C(*start_pose))
        self.q = [math.degrees(v) for v in (a.S1, a.S2, a.S3, a.S4)]
        self.packets = []

    def run(self):
        import time
        while self.running:
            try:
                path = "/UpdateMovement?" + "&".join("s%d=%.3f" % (i + 1, v) for i, v in enumerate(self.q))
                code, _, headers, body = self.h.req("GET", path)
                if code == 200:
                    rows = parse(body)
                    self.packets.append(rows)
                    self.q = rows[-1][1:]
                    self.h.req("POST", "/movement-complete")
            except Exception:
                pass
            time.sleep(self.gap)

    def stop(self):
        self.running = False
        self.join(2)
