"""The front door: the ONE place a request from the user's page (typed, spoken or a button) turns into something the arm does, and the answers go back.

    page --request--> Frontdoor.request() --parse (contract)--> [AI chooses from the menu: assistant.decide] --> checks --> job runner (jobs.py)
    page <--reply---- immediate: started | confirm | answer | question | stopped | error          page <--status-- Frontdoor.job() (job_status packets)

What it adds around the AI's choice (the AI only ever picks an action from contract.MENU; everything below is code):
  * Buttons (an explicit action) skip the AI. Free text and speech go through it, and a movement it chose is CONFIRMED first ("Fetch key 2?"): a mis-heard
    word must not move the arm. confirm_text=False turns that off. A confirmation lasts 30 s; "yes" / "no" spoken or typed answers it without the AI.
  * Stop is never delayed, never confirmed and never needs the AI.
  * It keeps where each key is (box / tray / unknown) so "fetch key 2" when key 2 is already on the tray is an answer, not a movement.
  * Nothing moves if the box or tray was not taught, or the arm is busy: the person gets the reason, in words.
The HTTP shapes are in handle_http(); every shape is the PLACEHOLDER contract (contract.py): when Terra and Sam say what theirs are, change that file.
"""
import json
import secrets
import time
import urllib.parse

try:
    import assistant as AS
    import boxes as B
    import contract as C
    import jobs as J
except ImportError:                                    # imported as dashboard.interface
    from dashboard import assistant as AS, boxes as B, contract as C, jobs as J

CONFIRM_TTL_S = 30.0
YES = {"yes", "yeah", "yep", "confirm", "confirmed", "ok", "okay", "go", "proceed", "sure", "affirmative"}
NO = {"no", "nope", "cancel", "abort", "nevermind", "don't", "dont", "negative"}
KEY_HOME = {"1": "box", "2": "box", "3": "box"}


def _words(text):
    import re
    return re.findall(r"[a-z']+", text.lower())


class Frontdoor:
    def __init__(self, store, sender, get_pose, decide=None, recover=None, on_status=None, confirm_text=True, clock=time.time, max_retries=2, limits=None):
        self.store, self.get_pose, self.decide = store, get_pose, decide or AS.decide
        self.confirm_text, self.clock = confirm_text, clock
        self.on_status_cb = on_status
        self.runner = J.JobRunner(store, sender, get_pose, recover=recover, on_status=self._status, max_retries=max_retries, limits=limits)
        self.keys = dict(KEY_HOME)
        self.pending = None                            # {"token", "action", "heard", "expires"}
        self.current = None                            # {"job", "action"}
        self.last_move = None

    # ---- the page's requests ----
    def request(self, payload):
        """payload: what the page sent (a dict). Returns a reply (contract.reply). Never raises for a bad request: that is an error reply."""
        try:
            req = C.parse_request(payload)
        except C.ContractError as e:
            return C.reply("", "error", "I did not understand that request: %s." % e, error={"code": "BAD_REQUEST", "title": str(e), "detail": "", "hint": ""})
        rid = req["id"]
        if "confirm" in req or "cancel" in req:
            return self._answer_pending(rid, req.get("confirm") or req.get("cancel"), "confirm" in req)
        if req["action"] is not None:                                          # a button: no AI, no confirmation (the press is the confirmation)
            return self._dispatch(rid, req["action"], "button", "", confirm=False)
        text = req["text"]
        if self.pending and self._clean_pending():
            ws = set(_words(text))
            if ws and len(ws) <= 3 and ws <= YES | {"please", "go", "ahead", "do", "it"}:
                return self._answer_pending(rid, self.pending["token"], True)
            if ws and ws & NO and len(ws) <= 4:
                return self._answer_pending(rid, self.pending["token"], False)
        d = self.decide(text, self._state())
        by = d.pop("by", "keywords")
        for k in ("ai_error", "model", "ms", "raw"):
            d.pop(k, None)
        return self._dispatch(rid, d, by, text, confirm=self.confirm_text)

    def job(self):
        """The latest job_status packet (or None before the first job)."""
        return self.runner.last

    def state(self):
        return self._state()

    # ---- decisions ----
    def _state(self):
        pose = None
        try:
            pose = self.get_pose()
        except Exception:
            pass
        home = self.store.home()
        at_home = bool(pose) and abs(pose["x"] - home[0]) < 5 and abs(pose["y"] - home[1]) < 5 and abs(pose["z"] - home[2]) < 5
        return {"busy": self.runner.busy(), "at_home": at_home, "holding": any(v == "unknown" for v in self.keys.values()) and self.runner.busy(),
                "keys": dict(self.keys), "last_move": self.last_move, "ready": not self.store.missing()}

    def _clean_pending(self):
        if self.pending and self.clock() > self.pending["expires"]:
            self.pending = None
        return self.pending

    def _answer_pending(self, rid, token, yes):
        p = self._clean_pending()
        if not p or p["token"] != token:
            return C.reply(rid, "error", "There is nothing waiting for an answer (it may have timed out). Please ask again.",
                           error={"code": "NOTHING_PENDING", "title": "No matching question", "detail": "", "hint": ""})
        self.pending = None
        if not yes:
            return C.reply(rid, "answer", "Cancelled. The arm has not moved.", action=p["action"], heard=p["heard"])
        return self._dispatch(rid, p["action"], p["by"], p["heard"], confirm=False)

    def _describe(self, a):
        return {"fetch_key": "Fetch key %s", "return_key": "Return key %s"}.get(a["action"], "").replace("%s", str(a.get("key", ""))) or J.JobRunner._describe(a)

    def _dispatch(self, rid, a, by, heard, confirm):
        kind = a["action"]
        if kind == "stop":                                                     # never delayed, never confirmed
            self.pending = None
            self.runner.stop()
            return C.reply(rid, "stopped", "Stopping. The arm will hold where it is.", action=a, by=by, heard=heard)
        if kind == "ask":
            return C.reply(rid, "question", a["question"], action=a, by=by, heard=heard)
        if kind == "status":
            return C.reply(rid, "answer", self.describe(), action=a, by=by, heard=heard)
        if self.runner.busy():
            return C.reply(rid, "error", "The arm is busy. Wait for it to finish, or say stop.", action=a, by=by, heard=heard,
                           error={"code": "BUSY", "title": "The arm is busy", "detail": "", "hint": "Wait, or stop the current job."})
        if kind in ("fetch_key", "return_key"):
            k = str(a["key"])
            where = self.keys.get(k)
            if kind == "fetch_key" and where == "tray":
                return C.reply(rid, "answer", "Key %s is already on the tray." % k, action=a, by=by, heard=heard)
            if kind == "return_key" and where == "box":
                return C.reply(rid, "answer", "Key %s is already in its box." % k, action=a, by=by, heard=heard)
            try:
                self.store.box(k)
                self.store.tray()
            except B.StoreError as e:
                return C.reply(rid, "error", "%s. %s" % (e.title, e.hint), action=a, by=by, heard=heard,
                               error={"code": e.code, "title": e.title, "detail": e.detail, "hint": e.hint})
        if confirm and (kind != "go_home" or not self._state()["at_home"]):
            tok = secrets.token_hex(3)
            self.pending = {"token": tok, "action": a, "by": by, "heard": heard, "expires": self.clock() + CONFIRM_TTL_S}
            return C.reply(rid, "confirm", "%s? Say yes to go ahead, or no to cancel." % self._describe(a), action=a, by=by, token=tok, heard=heard)
        try:
            job = self.runner.start(a)
        except (J.JobError, C.ContractError) as e:
            return C.reply(rid, "error", e.args[0], action=a, by=by, heard=heard,
                           error={"code": getattr(e, "code", "BAD_REQUEST"), "title": e.args[0], "detail": getattr(e, "detail", ""), "hint": getattr(e, "hint", "")})
        self.current = {"job": job, "action": a}
        if kind == "move_by":
            self.last_move = {k: a[k] for k in ("dx", "dy", "dz")}
        return C.reply(rid, "started", "%s: started." % self._describe(a), action=a, job=job, by=by, heard=heard)

    def describe(self):
        st = self._state()
        keys = ", ".join("key %s %s" % (k, {"box": "in its box", "tray": "on the tray"}.get(v, "somewhere unknown")) for k, v in sorted(st["keys"].items()))
        last = self.runner.last
        if st["busy"] and last:
            return "Working: %s (step %s of %s). %s." % (last.get("message") or last["status"], last.get("step"), last.get("of"), keys.capitalize())
        tail = ""
        if last and last["status"] in ("failed", "stopped"):
            tail = " The last job %s: %s." % (last["status"], last["message"])
        missing = self.store.missing()
        if missing:
            tail += " Not set up yet: %s." % ", ".join(missing)
        return "%s, %s.%s" % ("At home" if st["at_home"] else "Idle", keys, tail)

    # ---- what happened to the job (called from the job thread) ----
    def _status(self, p):
        cur = self.current
        if cur and p["job"] == cur["job"] and p["status"] in ("done", "failed", "stopped"):
            a = cur["action"]
            if a["action"] in ("fetch_key", "return_key"):
                k = str(a["key"])
                if p["status"] == "done":
                    self.keys[k] = "tray" if a["action"] == "fetch_key" else "box"
                else:                                                          # the key may be anywhere: do not guess
                    self.keys[k] = "unknown"
        if self.on_status_cb:
            try:
                self.on_status_cb(p)
            except Exception:
                pass


def handle_http(front, method, path, body=b""):
    """The placeholder HTTP face, as a pure function so any server can mount it: returns (status code, dict).
        POST /request   {contract request} -> reply            GET /job    -> latest job_status (404 before the first job)
        POST /stop      {}                 -> reply            GET /state  -> {"busy", "at_home", "keys", "ready", ...}"""
    route = urllib.parse.urlsplit(path).path
    if (method, route) == ("POST", "/request"):
        try:
            data = json.loads(body or b"{}")
        except ValueError:
            return 400, C.reply("", "error", "The request is not valid JSON.", error={"code": "BAD_JSON", "title": "Not JSON", "detail": "", "hint": ""})
        r = front.request(data)
        return (400 if r["kind"] == "error" and (r["error"] or {}).get("code") == "BAD_REQUEST" else 200), r
    if (method, route) == ("POST", "/stop"):
        return 200, front.request({"action": "stop"})
    if (method, route) == ("GET", "/job"):
        j = front.job()
        return (200, j) if j else (404, {"error": "no job yet"})
    if (method, route) == ("GET", "/state"):
        return 200, front.state()
    return 404, {"error": "not found"}
