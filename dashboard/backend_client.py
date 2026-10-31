"""The dashboard's side of Sam's backend: set a target, ask where the arm is, run a list of poses one after the other.

Talks to the backend over its own API (POST /target, GET /movement, and from our build GET /position and POST /hold), so the dashboard drives the real arm
the same way his tools do. Nothing here moves anything by itself: it only asks, and the backend's own checks decide.
"""
import json
import math
import time
import urllib.error
import urllib.request


class BackendError(Exception):
    """Same fields as the hub's structured errors."""
    def __init__(self, code, title, detail="", hint=""):
        super().__init__(title)
        self.code, self.title, self.detail, self.hint = code, title, detail, hint

    def as_dict(self):
        return {"code": self.code, "title": self.title, "detail": self.detail, "hint": self.hint}


class BackendClient:
    def __init__(self, base_url="http://127.0.0.1:8000", timeout=3.0):
        self.base = base_url.rstrip("/")
        self.timeout = timeout

    def _call(self, method, path, headers=None):
        req = urllib.request.Request(self.base + path, method=method, headers=headers or {}, data=b"" if method == "POST" else None)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                body = r.read()
                return r.status, body
        except urllib.error.HTTPError as e:
            reason = e.reason if isinstance(e.reason, str) else str(e.reason)
            if e.code == 422:
                raise BackendError("REFUSED", reason or "The backend refused that move", "The backend checked the move and said no.", "Pick a different point.")
            if e.code == 409 and method == "POST" and path.startswith("/target"):
                raise BackendError("BUSY", "The arm is still finishing its last move", "The backend does not take a new target while a move is being played.", "Wait a moment.")
            if e.code == 400:
                raise BackendError("BAD_REQUEST", reason or "The backend did not accept that", "", "")
            raise BackendError("BACKEND_ERROR", "The backend answered %d %s" % (e.code, reason), "", "Check the backend window for the reason.")
        except (urllib.error.URLError, OSError) as e:
            raise BackendError("BACKEND_OFFLINE", "Cannot reach the arm's backend", "%s: %s" % (self.base, getattr(e, "reason", e)),
                               "Start the backend (python main.py in the built backend folder) and check the address.")

    def set_target(self, x, y, z, angle_deg, speed=None):
        h = {"X": repr(float(x)), "Y": repr(float(y)), "Z": repr(float(z)), "A": repr(float(angle_deg))}
        if speed is not None:
            h["Speed"] = repr(float(speed))
        self._call("POST", "/target", h)

    def hold(self):
        """Stop wanting to go anywhere (a packet already being played still finishes)."""
        self._call("POST", "/hold")

    def movement(self):
        """{"status": "in_progress" | "idle"}. Sam's backend answers true/false (arm_moving); the older one answered a dict with a status."""
        v = json.loads(self._call("GET", "/movement")[1])
        if isinstance(v, bool):
            return {"status": "in_progress" if v else "idle"}
        return v

    def position(self):
        """{x, y, z, a, age_s}, or None when the arm has not reported yet."""
        code, body = self._call("GET", "/position")
        return json.loads(body) if code == 200 and body else None


class PoseRunner:
    """Runs poses (dicts x, y, z, angle in degrees, optional action grab/release/None) through a BackendClient, one at a time.

    A pose is reached when the arm's reported position is within tol_mm of it, is recent, and no move is in progress. A pose that is not reached in time,
    a refused target, or a lost backend ends the run with a BackendError and the arm is told to hold. stop() does the same from another thread."""
    def __init__(self, client, speed=20.0, tol_mm=1.5, magnet=None, clock=time.time, sleep=time.sleep, poll_s=0.2, max_age_s=2.0, on_pose=None):
        self.client, self.speed, self.tol, self.magnet = client, speed, tol_mm, magnet
        self.clock, self.sleep, self.poll, self.max_age, self.on_pose = clock, sleep, poll_s, max_age_s, on_pose
        self._stop = False
        self.reached = 0

    def stop(self):
        self._stop = True

    def run(self, poses):
        if not poses:
            raise BackendError("NO_POSES", "There is nothing to run", "", "")
        if any(p.get("action") for p in poses) and self.magnet is None:
            raise BackendError("MAGNET_UNSUPPORTED", "This sequence uses the magnet, but there is no way to switch it",
                               "Sam's firmware has no magnet switch.", "Use our firmware build, or run a sequence without grab/release.")
        self.reached = 0
        try:
            last = self.client.position()
            for i, p in enumerate(poses):
                self._check_stopped()
                self._set_target(p)
                dist = math.dist((last["x"], last["y"], last["z"]), (p["x"], p["y"], p["z"])) if last else 300.0
                self._wait_for(p, timeout=max(15.0, 3.0 * dist / max(self.speed, 1.0) + 10.0))
                last = {"x": p["x"], "y": p["y"], "z": p["z"]}
                self.reached = i + 1
                if self.on_pose:
                    self.on_pose(i, p)
                if p.get("action") and self.magnet is not None:
                    self.magnet(p["action"] == "grab")
        except BackendError:
            try:
                self.client.hold()
            except BackendError:
                pass
            raise

    def _set_target(self, p, wait_s=8.0):
        """The backend refuses a new target while the last packet is still being played (409). The previous pose can look reached a moment before the arm
        confirms, so wait a little for it rather than failing."""
        end = self.clock() + wait_s
        while True:
            self._check_stopped()
            try:
                self.client.set_target(p["x"], p["y"], p["z"], p["angle"], p.get("speed") or self.speed)
                return
            except BackendError as e:
                if e.code != "BUSY" or self.clock() >= end:
                    raise
                self.sleep(self.poll)

    def _check_stopped(self):
        if self._stop:
            raise BackendError("STOPPED", "Stopped", "The run was stopped.", "")

    def _wait_for(self, pose, timeout):
        end = self.clock() + timeout
        seen_busy = False
        started = self.clock()
        while self.clock() < end:
            self._check_stopped()
            self.sleep(self.poll)
            mv, pos = self.client.movement(), self.client.position()
            if mv.get("status") == "in_progress":
                seen_busy = True
            near = pos is not None and pos["age_s"] <= self.max_age and math.dist((pos["x"], pos["y"], pos["z"]), (pose["x"], pose["y"], pose["z"])) <= self.tol
            # at the target and not moving: reached. (An arm that was already there never shows a busy move: give the firmware a moment to confirm.)
            if near and mv.get("status") != "in_progress" and (seen_busy or self.clock() - started >= 1.0):
                return
        raise BackendError("NOT_REACHED", "The arm did not get to the pose in time", "Pose x %.0f, y %.0f, z %.0f was not reached within %.0f s." % (pose["x"], pose["y"], pose["z"], timeout),
                           "Check the arm is powered and connected to the backend, then try again.")
