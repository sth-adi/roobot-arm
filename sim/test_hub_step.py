"""The end-user page and the dashboard move the SAME arm: the front door's steps run on the hub's virtual arm (HubSender -> /api/step), which is what the
real arm's board follows in mirror mode. Real hub, real HTTP, no hardware."""
import os
import sys
import threading
import time
import unittest
from http.server import ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import hub as H  # noqa: E402
import interface  # noqa: E402
import lowlevel as LL  # noqa: E402
import virtual  # noqa: E402


class HubSteps(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hub = H.Hub("http://127.0.0.1:1")
        threading.Thread(target=cls.hub._sim_loop, daemon=True).start()
        cls._old = H.HUB
        H.HUB = cls.hub
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), H.Handler)
        cls.url = "http://127.0.0.1:%d" % cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.hub.stop.set()
        cls.srv.shutdown()
        cls.srv.server_close()
        H.HUB = cls._old

    def setUp(self):
        self.sender = LL.HubSender(self.url, poll_s=0.05, step_timeout_s=60)
        self.pose = LL.hub_pose(self.url)

    def step(self, x, y, z, a=0.0, action=None):
        return {"x": x, "y": y, "z": z, "angle": a, "speed": 30.0, "action": action, "label": "t"}

    def test_the_front_door_reads_the_hubs_arm_pose(self):
        p = self.pose()
        self.assertEqual(sorted(p), ["angle", "x", "y", "z"])

    def test_a_step_runs_on_the_hubs_virtual_arm_and_each_one_is_reported(self):
        start = self.pose()
        a = (start["x"] + 10, start["y"], start["z"], start["angle"])
        done = []
        self.sender.run([self.step(*a), self.step(start["x"], start["y"], start["z"], start["angle"])], "j", lambda i, s: done.append(i))
        self.assertEqual(done, [0, 1])
        end = self.pose()
        for k in ("x", "y", "z"):
            self.assertAlmostEqual(end[k], start[k], delta=3.0)

    def test_an_unreachable_step_is_refused_with_the_reason_and_nothing_moves(self):
        with self.assertRaises(LL.BackendError) as cm:
            self.sender.run([self.step(900, 900, 900)], "j")
        self.assertEqual(cm.exception.code, "UNREACHABLE")

    def test_a_second_step_while_one_runs_is_busy_and_stop_ends_the_run(self):
        p = self.pose()
        far = self.step(p["x"] + 40, p["y"], p["z"], p["angle"])
        err = []
        t = threading.Thread(target=lambda: err.append(self._run_catch([far, far])), daemon=True)
        t.start()
        time.sleep(0.3)
        ack = LL._urllib_http(self.url, 3.0, {"X-Hub": "1"})("POST", "/api/step", {"v": 1, "step": 1, "of": 1, "target": {"x": p["x"], "y": p["y"], "z": p["z"], "pitch": p["angle"]}, "magnet": None})
        self.assertFalse(ack["accepted"])
        self.assertEqual(ack["error"]["code"], "BUSY")
        self.sender.stop()
        t.join(30)
        self.assertEqual(err[0].code, "STOPPED")

    def _run_catch(self, steps):
        try:
            self.sender.run(steps, "j")
        except LL.BackendError as e:
            return e

    def test_a_bad_message_is_refused_not_a_crash(self):
        ack = LL._urllib_http(self.url, 3.0, {"X-Hub": "1"})("POST", "/api/step", {"target": {"x": "a"}})
        self.assertEqual((ack["accepted"], ack["error"]["code"]), (False, "BAD_REQUEST"))

    def test_the_hub_needs_its_header(self):
        with self.assertRaises(LL.BackendError):
            LL._urllib_http(self.url, 3.0)("POST", "/api/step", {})

    def test_a_button_on_the_front_door_moves_the_hubs_arm_end_to_end(self):
        store = virtual.demo_store()
        door = interface.Frontdoor(store, self.sender, self.pose)
        r = door.request({"v": 1, "id": "b1", "action": "fetch", "key": 2})
        self.assertEqual(r["kind"], "started", r)
        self.assertIsNotNone(door.runner.wait(120))
        self.assertEqual(door.runner.last["status"], "done", door.runner.last)
        self.assertEqual(door.state()["keys"]["2"], "tray")

    def test_server_hub_mode_builds_the_door_and_says_so_in_its_state(self):
        import server
        saved = dict(server._front)
        self.addCleanup(lambda: server._front.update(saved))
        server._front["door"], server._front["mode"] = server.make_front("hub", self.url), "hub"
        code, out = server._door("GET", "/arm/state")
        self.assertEqual((code, out["sender"]), (200, "hub"))
        self.assertIsNotNone(out["pose"])                                  # the pose is the hub's own arm


if __name__ == "__main__":
    unittest.main()
