"""The job runner: one job at a time, step by step, with statuses, a stop that always works, and a second AI menu for when a step fails.

    runner = JobRunner(store, sender, get_pose=..., recover=assistant.recover, on_status=send_to_page)
    runner.start({"action": "fetch_key", "key": 2})     returns the job id at once; the work happens in a thread
    runner.stop()                                       any time; the arm is told to hold and the job ends "stopped"

What a job does: the route generator (routes.py) builds and checks the whole route first; if it is refused nothing moves. Then the steps go to the
sender (lowlevel.py) one list at a time, and every step reached is reported as a status packet (contract.job_status).

When a step fails:
  * hard faults (the link is down, a stop, a planner refusal, a joint limit ...) are NEVER handed to the AI: the job fails with the reason
  * soft faults (a step not reached in time, a stall, the low-level side busy) become a failure report. The AI (assistant.recover) picks ONE action from
    contract.RECOVERY_MENU; if there is no AI, no network or its answer does not fit, a fixed rule table (assistant.rule_recovery) picks.
    Retries are capped per step (max_retries); the AI cannot raise the cap. Code checks the choice is allowed right now and does the maths:
        retry_step            run the failed step and the rest again
        lift_to_safe_height   routes.lift_route: straight up, no sideways movement
        go_home               routes.home_route
        release_magnet        only when the key is held low (within the hover height) over the tray or the job's own box
        ask_person / stop     the job fails with the question or the reason; nothing else moves
    A recovery move that itself fails ends the job: there is no second round of recovery.
"""
import threading
import uuid

try:
    import assistant as AS
    import contract as C
    import lowlevel as LL
    import routes as R
except ImportError:                                    # imported as dashboard.jobs
    from dashboard import assistant as AS, contract as C, lowlevel as LL, routes as R

SOFT = {"NOT_REACHED", "ARM_STALLED", "BUSY", "BAD_ANSWER"}      # the only faults the AI is asked about
NEAR_MM = 15.0                                                   # "over the tray" means within this distance in x and y
JOB_ACTIONS = ("fetch_key", "return_key", "go_home", "move_by", "magnet")


class JobError(ValueError):
    def __init__(self, code, title, detail="", hint=""):
        super().__init__(title)
        self.code, self.title, self.detail, self.hint = code, title, detail, hint

    def as_dict(self):
        return {"code": self.code, "title": self.title, "detail": self.detail, "hint": self.hint}


class JobRunner:
    def __init__(self, store, sender, get_pose, recover=None, on_status=None, max_retries=2, limits=None):
        """get_pose() -> {"x","y","z","angle"} of the arm now, or None when unknown. recover(report) -> contract recovery action (default: the rule table)."""
        self.store, self.sender, self.get_pose, self.limits = store, sender, get_pose, limits
        self.recover = recover or (lambda report: dict(AS.rule_recovery(report), by="rules"))
        self.on_status, self.max_retries = on_status, max_retries
        self.lock = threading.Lock()
        self.thread, self.job, self.last, self.history, self._stop = None, None, None, [], False

    # ---- public ----
    def busy(self):
        return bool(self.thread and self.thread.is_alive())

    def start(self, action):
        """Validate, then run in a thread. Returns the job id. Raises JobError (BUSY, NOT_A_JOB) or C.ContractError."""
        action = C.validate_action(action)
        if action["action"] not in JOB_ACTIONS:
            raise JobError("NOT_A_JOB", "%s is not something the arm runs as a job" % action["action"])
        with self.lock:
            if self.busy():
                raise JobError("BUSY", "The arm is already busy with a job", "", "Wait for it to finish, or stop it first.")
            self.job, self._stop, self.history = uuid.uuid4().hex[:8], False, []
            self.thread = threading.Thread(target=self._run, args=(self.job, action), daemon=True)
            job = self.job
        self._emit(job, "queued", message=self._describe(action))
        self.thread.start()
        return job

    def stop(self):
        self._stop = True
        self.sender.stop()

    def wait(self, timeout=30.0):
        t = self.thread
        if t:
            t.join(timeout)
        return self.last

    # ---- status ----
    def _emit(self, job, status, step=None, of=None, message="", error=None):
        p = C.job_status(job, status, step, of, message, error)
        self.last = p
        self.history.append(p)
        if self.on_status:
            try:
                self.on_status(p)
            except Exception:                                  # a broken page connection must never stop the arm
                pass
        return p

    @staticmethod
    def _describe(a):
        return {"fetch_key": "Fetch key %s" % a.get("key"), "return_key": "Return key %s" % a.get("key"), "go_home": "Go home",
                "move_by": "Nudge the arm", "magnet": "Magnet %s" % ("on" if a.get("on") else "off")}[a["action"]]

    # ---- the work ----
    def _pose(self):
        try:
            p = self.get_pose()
        except Exception:
            p = None
        return p if isinstance(p, dict) and all(k in p for k in ("x", "y", "z", "angle")) else None

    def _plan(self, a):
        pose = self._pose()
        if pose is None:
            raise R.RouteError("NO_POSITION", "The arm's position is not known", "", "Wait for the arm's first position report.")
        kind = a["action"]
        if kind == "fetch_key":
            return R.fetch_route(self.store, a["key"], pose, self.limits), "box %s" % a["key"]
        if kind == "return_key":
            return R.return_route(self.store, a["key"], pose, self.limits), "box %s" % a["key"]
        if kind == "go_home":
            return R.home_route(self.store, pose, self.limits), None
        if kind == "move_by":
            v = self.store.constants()["speed_mm_s"]
            step = R._step(pose["x"] + a["dx"], pose["y"] + a["dy"], pose["z"] + a["dz"], pose["angle"], v, "nudge")
            return R._finish(self.store, [step], pose, None, self.limits), None
        step = R._step(pose["x"], pose["y"], pose["z"], pose["angle"], self.store.constants()["speed_mm_s"], "magnet " + ("on" if a["on"] else "off"),
                       "grab" if a["on"] else "release")
        return {"steps": [step], "seconds": 0.0}, None

    def _run(self, job, action):
        try:
            route, own_box = self._plan(action)
        except (R.RouteError, JobError) as e:
            self._emit(job, "failed", message=e.title, error={"code": e.code, "title": e.title, "detail": e.detail, "hint": e.hint})
            return
        except Exception as e:
            self._emit(job, "failed", message="The route could not be built", error={"code": "INTERNAL", "title": "The route could not be built", "detail": repr(e)})
            return
        steps, n = route["steps"], len(route["steps"])
        if not steps:
            self._emit(job, "done", 0, 0, "Nothing to do")
            return
        state = {"done": 0, "holding": False}
        retries = {}

        def reached(base):
            def cb(i, step):
                state["done"] = base + i + 1
                if step.get("action") == "grab":
                    state["holding"] = True
                elif step.get("action") == "release":
                    state["holding"] = False
                self._emit(job, "running", state["done"], n, step.get("label", ""))
            return cb

        self._emit(job, "running", 0, n, "Starting (%d steps, about %.0f s)" % (n, route["seconds"]))
        while True:
            try:
                self.sender.run(steps[state["done"]:], job, reached(state["done"]))
                self._emit(job, "done", n, n, "Done")
                return
            except LL.BackendError as e:
                if self._stop or e.code == "STOPPED":
                    self._emit(job, "stopped", state["done"], n, "Stopped")
                    return
                err = {"code": e.code, "title": e.title, "detail": e.detail, "hint": getattr(e, "hint", "")}
                if e.code not in SOFT:
                    self._emit(job, "failed", state["done"], n, e.title, err)
                    return
                if not self._recover(job, e, err, steps, state, retries, own_box, n):
                    return
            except Exception as e:                                # nothing unexpected may leave the arm moving
                try:
                    self.sender.stop()
                except Exception:
                    pass
                self._emit(job, "failed", state["done"], n, "Unexpected error", {"code": "INTERNAL", "title": "Unexpected error", "detail": repr(e)})
                return

    # ---- failure ----
    def _over(self, pose):
        """(name of the place the tip is above, height above its top in mm): ("tray", 32.0), or ("elsewhere", None) / ("unknown", None)."""
        if not pose:
            return "unknown", None
        places = [("tray", self.store.data["tray"])] + [("box %s" % k, p) for k, p in self.store.data["boxes"].items()]
        for name, p in places:
            if p and abs(p["x"] - pose["x"]) <= NEAR_MM and abs(p["y"] - pose["y"]) <= NEAR_MM:
                return name, pose["z"] - p["z"]
        return "elsewhere", None

    def _recover(self, job, exc, err, steps, state, retries, own_box, n):
        """Returns True to run the remaining steps again (retry), False when the job has ended (its final status has been sent)."""
        i = state["done"]
        pose = self._pose()
        over, height = self._over(pose)
        low = height is not None and height <= self.store.constants()["hover_mm"] + 5.0      # near enough to put a key down, not drop it
        left = max(0, self.max_retries - retries.get(i, 0))
        report = {"error": {"code": err["code"], "title": err["title"], "detail": err["detail"][:200]}, "step": i + 1, "of": n,
                  "label": steps[i].get("label", "") if i < len(steps) else "", "holding": state["holding"], "retries_left": left,
                  "pose_known": pose is not None, "over": over, "height_mm": None if height is None else round(height)}
        try:
            choice = self.recover(report)
            choice = C.validate_recovery(choice) | {k: choice[k] for k in ("by", "ai_error") if k in choice}
        except Exception:
            choice = dict(AS.rule_recovery(report), by="rules")
        act = choice["action"]
        allowed = {"retry_step": left > 0, "lift_to_safe_height": pose is not None, "go_home": pose is not None,
                   "release_magnet": state["holding"] and low and over in ("tray", own_box), "ask_person": True, "stop": True}
        if not allowed[act]:                                         # code has the final say on what is allowed right now
            choice = dict(AS.rule_recovery(report), by="rules", overruled=act)
            act = choice["action"]
            if not allowed[act]:
                choice, act = {"action": "ask_person", "question": "The arm stopped (%s). What should it do?" % err["title"], "by": "rules"}, "ask_person"
        why = choice.get("reason", "")
        if act == "retry_step":
            retries[i] = retries.get(i, 0) + 1
            self._emit(job, "running", i, n, "Retrying step %d (%d of %d): %s" % (i + 1, retries[i], self.max_retries, err["title"]))
            return True
        if act == "ask_person":
            q = choice.get("question") or "The arm stopped (%s). What should it do?" % err["title"]
            self._emit(job, "failed", i, n, q, {"code": "NEEDS_PERSON", "title": q, "detail": "%s: %s" % (err["code"], err["detail"])})
            return False
        if act == "stop":
            self._emit(job, "failed", i, n, "Stopped after a fault: " + err["title"], dict(err, recovery="stop", reason=why))
            return False
        # a recovery move: built from the arm's real position, checked like any route, and run once
        try:
            if act == "release_magnet":
                extra = [R._step(pose["x"], pose["y"], pose["z"], pose["angle"], self.store.constants()["speed_mm_s"], "release the key", "release")]
                extra = R._finish(self.store, extra, pose, None, self.limits)["steps"]
                then = "released the key over " + over
            else:
                r = R.lift_route(self.store, pose, self.limits) if act == "lift_to_safe_height" else R.home_route(self.store, pose, self.limits)
                extra, then = r["steps"], "lifted clear" if act == "lift_to_safe_height" else "went home"
            self._emit(job, "running", i, n, "Recovering: " + act.replace("_", " "))
            if extra:
                self.sender.run(extra, job, None)
        except (R.RouteError, LL.BackendError) as e2:
            self._emit(job, "failed", i, n, "Recovery failed: " + e2.title, dict(err, recovery=act, recovery_error=e2.title))
            return False
        except Exception as e2:
            self._emit(job, "failed", i, n, "Recovery failed", dict(err, recovery=act, recovery_error=repr(e2)))
            return False
        self._emit(job, "failed", i, n, "%s; the arm %s and has stopped." % (err["title"], then), dict(err, recovery=act, reason=why))
        return False
