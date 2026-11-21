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
ExeSender   Sam's exe: one run per step. His three inputs are the instructions JSON, the baud rate and the port; the JSON we send is his "start point and
            end point" form (StartPoint / EndPoint {x, y, z, toolDegrees}, speed, SamplesPerMM, AngleLimits). A "twin" (a virtual arm) runs the same
            steps alongside so the dashboard can show what the AI is doing (server.py --ai exe; the hub copies the twin's pose onto its viewer).
HubSender   the same three calls, but to the dashboard hub (dashboard/hub.py: /api/step, /api/step/status, /api/step/stop). The hub's virtual arm is the
            brain; with mirror mode ARMED the real arm's board follows it, so the end-user page moves the real arm exactly as the dashboard does.
            hub_pose(url) gives the front door the arm's pose from the same place.

(There used to be a SamSender here that drove the real arm through Sam's HTTP backend - removed now that the board does its own IK in mirror mode and
nothing talks to that backend any more. BackendError lives here, not in a "backend_client" module, because every sender raises it, not just that one did.)
"""
import json
import math
import os
import shlex
import subprocess
import sys
import tempfile
import threading
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


class ExeSender:
    """Drive the real arm through Sam's exe, one run of the exe per step. UNCONFIRMED against the real exe: how it is started, what it prints and how it is
    stopped are assumptions (see the questions in the README) and all live in ONE place: cmd, the argument template.
      cmd           the command as a list; "{json_file}" (a temp file holding the instructions), "{json}" (the same text), "{baud}" and "{port}" are filled in.
                    e.g. ["C:/arm/sam.exe", "{json_file}", "{baud}", "{port}"]. Exit code 0 = the move is done; anything else = it failed (its output is the detail).
      twin          optional: a sender (VirtualArm) that runs the same steps beside the real arm; it is also where the arm's position comes from (start_pose)
      magnet        Sam: the magnet is one more top-level field in the JSON, "Tool": 0 (off) or 1 (on). Each run carries the state it should be in AFTER that move
                    (grab -> 1, release -> 0, starts at 0); a grab/release in place is still a run (start = end). His hardware is not built yet.
      runner        runner(argv, timeout_s, stopped) -> (returncode, output); the default starts the process and kills it on stop() (tests pass a fake)
    The first StartPoint is where start_pose() says the arm is: the real arm must really be there (home) when the first job starts."""
    LIMITS_UNSET = {k: {"Min": 0, "Max": 0} for k in ("S1", "S2", "S3", "S4")}      # exactly as in Sam's example: 0/0

    def __init__(self, cmd, start_pose, twin=None, baud=115200, port="", samples_per_mm=1000, limits=None, step_timeout_s=180.0, runner=None):
        self.cmd, self.start_pose, self.twin, self.baud, self.port = list(cmd), start_pose, twin, baud, port
        self.samples_per_mm, self.step_timeout = samples_per_mm, step_timeout_s
        self.tool = 0                                  # the magnet state we last asked for (0 off, 1 on)
        self.limits = limits or self.LIMITS_UNSET
        self.runner = runner or self._run_process
        self._stop, self._proc = False, None
        self._lock = threading.Lock()

    # ---- what we send ----
    def instruction(self, start, step, tool=0):
        """Sam's start/end-point JSON for one move. start: {"x","y","z","angle"}; step: a route step (x, y, z, angle, speed ...)."""
        return {"StartPoint": {"x": start["x"], "y": start["y"], "z": start["z"], "toolDegrees": start["angle"]},
                "EndPoint": {"x": step["x"], "y": step["y"], "z": step["z"], "toolDegrees": step["angle"]},
                "speed": step.get("speed"), "SamplesPerMM": self.samples_per_mm, "AngleLimits": self.limits, "Tool": tool}

    @staticmethod
    def _still(a, b):
        return math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"])) < 1e-6 and abs(a["angle"] - b["angle"]) < 1e-6

    # ---- run / stop ----
    def stop(self):
        self._stop = True
        with self._lock:
            if self._proc and self._proc.poll() is None:
                self._proc.kill()
        if self.twin:
            self.twin.stop()

    def run(self, steps, job="", on_step=None):
        if not steps:
            raise BackendError("NO_POSES", "There is nothing to run", "", "")
        pose = self.start_pose()
        if not isinstance(pose, dict):
            raise BackendError("NO_POSITION", "The arm's position is not known", "", "Wait for the arm's first position report.")
        self._stop = False
        for i, step in enumerate(steps):
            if self._stop:
                raise BackendError("STOPPED", "Stopped", "", "")
            twin_err, twin_thread = [], None
            if self.twin:
                def work(step=step):
                    try:
                        self.twin.run([step], job, None)
                    except BackendError as e:
                        twin_err.append(e)
                twin_thread = threading.Thread(target=work, daemon=True)
                twin_thread.start()
            try:
                tool = {"grab": 1, "release": 0}.get(step.get("action"), self.tool)
                if tool != self.tool or not self._still(pose, step):
                    self._run_exe(self.instruction(pose, step, tool))
                self.tool = tool
            except BackendError:
                if self.twin:
                    self.twin.stop()
                raise
            finally:
                if twin_thread:
                    twin_thread.join(self.step_timeout)
            if twin_err and twin_err[0].code != "STOPPED":
                raise twin_err[0]
            pose = {k: step[k] for k in ("x", "y", "z", "angle")}
            if on_step:
                on_step(i, step)

    def _run_exe(self, instruction):
        text = json.dumps(instruction)
        fd, path = tempfile.mkstemp(suffix=".json", prefix="arm_")
        try:
            with os.fdopen(fd, "w") as f:
                f.write(text)
            argv = [a.format(json_file=path, json=text, baud=self.baud, port=self.port) for a in self.cmd]
            try:
                code, out = self.runner(argv, self.step_timeout, lambda: self._stop)
            except FileNotFoundError:
                raise BackendError("EXE_MISSING", "Sam's exe was not found", argv[0], "Check the path to the exe.")
        finally:
            try:
                os.remove(path)
            except OSError:
                pass
        if self._stop:
            raise BackendError("STOPPED", "Stopped", "", "")
        if code is None:
            raise BackendError("NOT_REACHED", "Sam's exe did not finish in time", "No answer within %.0f s." % self.step_timeout, "Check the arm and the cable.")
        if code != 0:
            raise BackendError("EXE_FAILED", "Sam's exe reported an error", ("exit code %s: %s" % (code, (out or "").strip()[-300:])), "Check the port, the baud rate and that nothing else is using the port.")

    def _run_process(self, argv, timeout_s, stopped):
        with self._lock:
            self._proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            proc = self._proc
        try:
            out, _ = proc.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            return None, ""
        finally:
            with self._lock:
                self._proc = None
        return proc.returncode, out
