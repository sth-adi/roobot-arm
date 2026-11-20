"""The sender: how the high-level logic gets a list of steps to the low-level side (the board, which does its own IK now - mirror mode) and waits until
each is done.

A step is a pose dict {"x", "y", "z", "angle" (degrees), optional "speed" (mm/s), optional "action": "grab" | "release"}.
Every sender has the same two methods, so the job runner does not care which one is behind it:
    run(steps, job="", on_step=None)   one step at a time, blocking; on_step(i, step) after each is reached; raises BackendError (code, title, detail, hint)
    stop()                             from another thread: the run ends with a STOPPED error and the arm is told to hold

JsonSender  the JSON hand-over in the architecture sketch. PLACEHOLDER: Sam has not defined it. The shapes are in contract.step_message and below.
            POST <url>/step {step message} -> {"accepted": true} or {"accepted": false, "error": {"code", "title", "detail"}}
            GET  <url>/status -> {"state": "idle" | "moving" | "error", "step": n, "error": {...}}
            POST <url>/stop {} -> {}
HubSender   the same three calls, but to the dashboard hub (dashboard/hub.py: /api/step, /api/step/status, /api/step/stop). The hub's virtual arm is the
            brain; with mirror mode ARMED the real arm's board follows it, so the end-user page moves the real arm exactly as the dashboard does.
            hub_pose(url) gives the front door the arm's pose from the same place.

(There used to be a SamSender here that drove the real arm through Sam's HTTP backend - removed now that the board does its own IK in mirror mode and
nothing talks to that backend any more. BackendError lives here, not in a "backend_client" module, because every sender raises it, not just that one did.)
"""
import json
import time
import urllib.error
import urllib.request

try:
    import contract as C
except ImportError:                                    # imported as dashboard.lowlevel
    from dashboard import contract as C


class BackendError(Exception):
    """Same fields as the hub's structured errors. The name predates mirror mode (it used to mean "Sam's HTTP backend said no"); now it is just
    "the low-level side (whatever it is) said no", raised by every sender."""
    def __init__(self, code, title, detail="", hint=""):
        super().__init__(title)
        self.code, self.title, self.detail, self.hint = code, title, detail, hint

    def as_dict(self):
        return {"code": self.code, "title": self.title, "detail": self.detail, "hint": self.hint}


def _urllib_http(base, timeout, extra_headers=None):
    def call(method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(base.rstrip("/") + path, data, dict({"Content-Type": "application/json", "Accept": "application/json"}, **(extra_headers or {})), method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                raw = r.read()
        except urllib.error.HTTPError as e:
            raise BackendError("REFUSED", "The low-level side refused the request", "HTTP %d on %s" % (e.code, path), "")
        except (urllib.error.URLError, OSError):
            raise BackendError("BACKEND_OFFLINE", "The low-level side cannot be reached", base, "Check that Sam's code is running and the address is right.")
        try:
            return json.loads(raw or b"{}")
        except ValueError:
            raise BackendError("BAD_ANSWER", "The low-level side answered with something that is not JSON", path, "")
    return call


class JsonSender:
    PATHS = ("/step", "/status", "/stop")              # (send one step, ask how it is going, stop)

    def __init__(self, url=None, http=None, timeout=3.0, poll_s=0.1, step_timeout_s=60.0, clock=time.time, sleep=time.sleep):
        self.http = http or _urllib_http(url, timeout)
        self.poll, self.step_timeout, self.clock, self.sleep = poll_s, step_timeout_s, clock, sleep
        self._stop = False

    def stop(self):
        self._stop = True
        try:
            self.http("POST", self.PATHS[2], {})
        except BackendError:
            pass

    def _check(self):
        if self._stop:
            raise BackendError("STOPPED", "Stopped", "The run was stopped.", "")

    def run(self, steps, job="", on_step=None):
        if not steps:
            raise BackendError("NO_POSES", "There is nothing to run", "", "")
        self._stop = False
        try:
            for i, step in enumerate(steps):
                self._check()
                try:
                    msg = C.step_message(job, i + 1, len(steps), step)
                except C.ContractError as e:
                    raise BackendError("BAD_REQUEST", "A step does not fit the message format", str(e), "")
                ack = self.http("POST", self.PATHS[0], msg)
                if not isinstance(ack, dict) or ack.get("accepted") is not True:
                    e = (ack or {}).get("error") if isinstance(ack, dict) else None
                    e = e if isinstance(e, dict) else {}
                    raise BackendError(str(e.get("code") or "REFUSED"), str(e.get("title") or "The step was refused"), str(e.get("detail") or ""), "")
                self._wait(i + 1)
                if on_step:
                    on_step(i, step)
        except BackendError:
            self.stop_quietly()
            raise

    def stop_quietly(self):
        try:
            self.http("POST", self.PATHS[2], {})
        except BackendError:
            pass

    def _wait(self, n):
        end = self.clock() + self.step_timeout
        while self.clock() < end:
            self._check()
            self.sleep(self.poll)
            st = self.http("GET", self.PATHS[1])
            state = st.get("state") if isinstance(st, dict) else None
            if state == "error":
                e = st.get("error") if isinstance(st.get("error"), dict) else {}
                raise BackendError(str(e.get("code") or "ARM_ERROR"), str(e.get("title") or "The arm reported an error"), str(e.get("detail") or ""), "")
            if state == "idle" and st.get("step", n) >= n:
                return
        raise BackendError("NOT_REACHED", "The step did not finish in time", "Step %d was not done within %.0f s." % (n, self.step_timeout), "")


class HubSender(JsonSender):
    """Steps go to the dashboard hub, whose virtual arm runs them (and the real arm follows in mirror mode when ARMED). url: e.g. http://127.0.0.1:8765."""
    PATHS = ("/api/step", "/api/step/status", "/api/step/stop")

    def __init__(self, url, http=None, **kw):
        super().__init__(url, http=http or _urllib_http(url, kw.pop("timeout", 3.0), {"X-Hub": "1"}), **kw)


def hub_pose(url, timeout=1.5, http=None):
    """A get_pose() for the front door: the hub's virtual arm tip {"x","y","z","angle"}, or None when the hub is not reachable."""
    call = http or _urllib_http(url, timeout, {"X-Hub": "1"})

    def get_pose():
        try:
            p = (call("GET", "/api/state") or {}).get("pose")
        except BackendError:
            return None
        return {k: p[k] for k in ("x", "y", "z", "angle")} if isinstance(p, dict) and all(isinstance(p.get(k), (int, float)) for k in ("x", "y", "z", "angle")) else None
    return get_pose
