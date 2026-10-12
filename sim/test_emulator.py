"""The ESP32 firmware running on this PC in real time against the real server, through the network proxy.

Slow (real time, about 40 s): builds the stock and patched firmware, then runs each against a server on localhost with a
link of known quality. Needs g++ and patch. These check the emulator and proxy work and that the effects of the network
the firmware is exposed to (handshake cost per poll, an outage during a held stick) show up as the model predicts.
"""
import asyncio
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

import server

server.print = lambda *a, **k: None      # the server logs every packet; keep the test output readable

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "emu"))
import emulator  # noqa: E402
import netproxy  # noqa: E402

CAL = {"jointLimits": [[0, 180]] * 4, "jointHomes": [90, 90, 90, 90]}


def post(url, obj):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), method="POST", headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=3).read()


class Rig:
    """Server + proxy + a controller feeding a held stick, all on localhost."""

    def __init__(self, preset):
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        self.srv_port = self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        with server._lock:
            server._state.update(data=None, received_at=None, count=0)
            server._arm.update(status="", calibration=None, max_speed=None)
        post("http://127.0.0.1:%d/calibration" % self.srv_port, CAL)
        self.loop = asyncio.new_event_loop()
        ports = [self._free() for _ in range(3)]
        args = netproxy.parser().parse_args(["--upstream", "127.0.0.1:%d" % self.srv_port, "--esp-port", str(ports[0]),
                                             "--ctl-port", str(ports[1]), "--control-port", str(ports[2]), "--preset", preset])
        ready = threading.Event()
        self.proxy_thread = threading.Thread(target=lambda: self._run_proxy(args, ready), daemon=True)
        self.proxy_thread.start()
        assert ready.wait(5)
        self.esp_host, self.control = "127.0.0.1:%d" % ports[0], "http://127.0.0.1:%d" % ports[2]
        self.stick = {"LSX": 0.0, "LSY": 0.0, "RSX": 0.0, "RSY": 0.0, "A": False, "B": False, "X": False, "Y": False}
        self.feeding = True
        threading.Thread(target=self._feed, daemon=True).start()

    @staticmethod
    def _free():
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        return p

    def _run_proxy(self, args, ready):
        asyncio.set_event_loop(self.loop)
        self.task = self.loop.create_task(netproxy.serve(args, ready))
        try:
            self.loop.run_until_complete(self.task)
        except asyncio.CancelledError:
            pass
        finally:
            pending = [t for t in asyncio.all_tasks(self.loop) if not t.done()]
            for t in pending:
                t.cancel()
            self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self.loop.close()

    def _feed(self):
        while self.feeding:
            try:
                post("http://127.0.0.1:%d/input" % self.srv_port, self.stick)
            except OSError:
                pass
            time.sleep(1 / 30)

    def link(self, **kw):
        post(self.control + "/link", dict(path="esp", **kw))

    def outage(self, seconds):
        post(self.control + "/outage", {"path": "esp", "seconds": seconds})

    def close(self):
        self.feeding = False
        time.sleep(0.1)
        with server._lock:
            server._state.update(data=None, received_at=None)      # leave the shared server module as found
        self.srv.shutdown()
        self.srv.server_close()
        self.loop.call_soon_threadsafe(self.task.cancel)
        self.proxy_thread.join(timeout=3)


class EmulatedEsp32(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get("ROOBOT_SKIP_SLOW"):
            raise unittest.SkipTest("ROOBOT_SKIP_SLOW is set")
        if not (shutil.which("g++") and emulator.have_patch_tool()):
            raise unittest.SkipTest("needs g++ and git or patch")
        cls.tmp = tempfile.mkdtemp()
        cls.exe = {k: emulator.build(k, cls.tmp) for k in ("stock", "patched")}

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def run_for(self, kind, preset, seconds, script=None):
        rig = Rig(preset)
        emu = emulator.Emulator(self.exe[kind], rig.esp_host)
        try:
            t0 = time.monotonic()
            for at, fn in sorted(script or [], key=lambda x: x[0]):
                time.sleep(max(0, t0 + at - time.monotonic()))
                fn(rig)
            time.sleep(max(0, t0 + seconds - time.monotonic()))
            emu.stop()
            return rig, emu
        finally:
            rig.close()
            emu.stop()

    def test_emulated_board_follows_the_sticks_over_a_lan(self):
        rig, emu = self.run_for("stock", "lan", 5.0, [(0.5, lambda r: r.stick.update(LSX=1.0))])
        base = [h[1]["servo"][0] for h in emu.history]
        self.assertGreater(len(base), 20)
        self.assertGreater(max(base) - min(base), 5)                  # joint 1 swung while x was held
        self.assertGreater(emu.stats.get("polls", 0), 50)
        self.assertEqual(emu.stats.get("poll_fails", 1), 0)

    def test_polling_a_plain_server_pays_a_handshake_every_time(self):
        """Behind a TLS front end the connection is reused; straight to a plain HTTP/1.0 server (this repo's, and UWE-RArm's)
        every poll is a new connection, so with the handshake cost the firmware polls a few times a second."""
        with_keepalive = self.run_for("stock", "home_wifi", 6.0)[1].stats
        direct = self.run_for("stock", "direct_http", 6.0)[1].stats
        self.assertLess(with_keepalive["poll_ms_mean"], 120)
        self.assertGreater(direct["poll_ms_mean"], 300)
        self.assertGreater(with_keepalive["polls"], 3 * direct["polls"])

    def test_outage_while_a_stick_is_held(self):
        """The link drops for 2 s with the stick held. Stock adds the whole gap in one step when it comes back; the patch caps it."""
        jump = {}
        for kind in ("stock", "patched"):
            script = [(0.3, lambda r: r.stick.update(LSX=1.0)), (3.0, lambda r: r.outage(2.0))]
            rig, emu = self.run_for(kind, "lan", 8.0, script)
            base = [h[1]["servo"][0] for h in emu.history]
            jump[kind] = max(abs(b - a) for a, b in zip(base, base[1:]))
        self.assertGreaterEqual(jump["stock"], 5)                     # ~40 mm of x in one tick is about 9 degrees of base
        self.assertLessEqual(jump["patched"], 2)


if __name__ == "__main__":
    unittest.main()
