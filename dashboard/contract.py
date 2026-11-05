"""The wire contract between the user's page (index.html, Terra's) and the AI layer, in ONE place. PLACEHOLDER: nobody has told us the real format.

Everything inside the hub uses the shapes made here; if the page's real format turns out different, change parse_request() (and job_status()
if its status format differs) and nothing else.

Inbound (page -> us), the placeholder:
    {"v": 1, "id": "client-made id", "text": "get me the second key"}          free text, typed or transcribed: the AI chooses
    {"v": 1, "id": "...", "action": "fetch", "key": 2}                         a button press: no AI is involved
    {"command": "request_key"}                                                  what the page sends today (see LEGACY)
Outbound (us -> page), the placeholder:
    {"v": 1, "job": "7f3a", "status": "queued|running|done|failed|stopped", "step": 3, "of": 8, "message": "...", "error": null, "led": "moving"}

The AI's answer, and the only things it can ask for (validate_action refuses everything else):
    fetch_key(key 1-3), return_key(key 1-3), go_home, stop, magnet(on), status, move_by(dx, dy, dz; small, never for keys), ask(question)
It never writes coordinates, angles or packets: code does the maths after this point.
"""
import math

VERSION = 1
STATUSES = ("queued", "running", "done", "failed", "stopped")
KEYS = (1, 2, 3)
MOVE_MAX_MM = 100.0                        # same limit as dashboard/preflight.py: per command, total distance
MAX_TEXT = 500
MENU = ("fetch_key", "return_key", "go_home", "stop", "magnet", "status", "move_by", "ask")
# The SECOND menu: what the AI may pick when a step fails (see jobs.py). Code does the maths for each one; the AI never writes a coordinate.
RECOVERY_MENU = ("retry_step", "lift_to_safe_height", "go_home", "release_magnet", "ask_person", "stop")
# PLACEHOLDER hint for the LEDs (Sam's side): which state to show. Nobody has said whether the LEDs follow our status or the arm's own state.
LED = {"queued": "waiting", "running": "moving", "done": "done", "failed": "error", "stopped": "stopped"}

# The page as it is today sends these (names from index.html and server.py disagree: it sends request_key, the server accepts collect_keys).
# Both spellings mean "fetch" / "return" without saying which key, so the answer is a question, not a guess.
LEGACY = {"request_key": "fetch_key", "collect_keys": "fetch_key", "return_key": "return_key", "return_keys": "return_key", "home": "go_home", "stop": "stop"}


class ContractError(ValueError):
    """A request or an AI answer that does not fit the contract. The text is safe to show to the person."""


def _key(v):
    if isinstance(v, str) and v.strip().isdigit():
        v = int(v)
    if isinstance(v, float) and v.is_integer():
        v = int(v)
    if isinstance(v, bool) or not isinstance(v, int) or v not in KEYS:
        raise ContractError("key must be 1, 2 or 3")
    return v


def _num(v, name):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ContractError("%s must be a number" % name)
    return float(v)


def validate_action(a):
    """A clean copy of an action dict, or ContractError. Only the fields each action needs are kept (anything else the AI added is dropped)."""
    if not isinstance(a, dict):
        raise ContractError("the answer is not an object")
    act = a.get("action")
    if act not in MENU:
        raise ContractError("unknown action %r" % (act,))
    out = {"action": act}
    if act in ("fetch_key", "return_key"):
        out["key"] = _key(a.get("key"))
    elif act == "magnet":
        if not isinstance(a.get("on"), bool):
            raise ContractError("magnet needs on: true or false")
        out["on"] = a["on"]
    elif act == "move_by":
        d = [_num(a.get(k, 0), k) for k in ("dx", "dy", "dz")]
        if math.sqrt(sum(v * v for v in d)) > MOVE_MAX_MM:
            raise ContractError("a nudge is at most %d mm in total" % MOVE_MAX_MM)
        out.update(dx=d[0], dy=d[1], dz=d[2])
    elif act == "ask":
        q = a.get("question")
        out["question"] = q.strip()[:200] if isinstance(q, str) and q.strip() else "Which key do you mean: 1, 2 or 3?"
    if isinstance(a.get("reason"), str):
        out["reason"] = a["reason"][:200]
    return out


def validate_recovery(a):
    """A clean copy of the AI's recovery choice, or ContractError. Only the action and a short reason (and a question for ask_person) survive."""
    if not isinstance(a, dict):
        raise ContractError("the answer is not an object")
    if a.get("action") not in RECOVERY_MENU:
        raise ContractError("unknown recovery action %r" % (a.get("action"),))
    out = {"action": a["action"]}
    if a["action"] == "ask_person":
        q = a.get("question")
        out["question"] = q.strip()[:200] if isinstance(q, str) and q.strip() else "Something went wrong. What should the arm do?"
    if isinstance(a.get("reason"), str):
        out["reason"] = a["reason"][:200]
    return out


def parse_request(d):
    """What the page sent -> {"id", "text", "action"}: action is None when the AI must choose (free text), else an already-validated action dict.
    Raises ContractError for anything that is not understood."""
    if not isinstance(d, dict):
        raise ContractError("the request must be a JSON object")
    rid = str(d.get("id") or "")[:64]
    if isinstance(d.get("command"), str) and "action" not in d and "text" not in d:        # LEGACY
        act = LEGACY.get(d["command"])
        if act is None:
            raise ContractError("unknown command %r" % d["command"][:40])
        if act in ("fetch_key", "return_key"):
            return {"id": rid, "text": "", "action": {"action": "ask", "question": "Which key: 1, 2 or 3?", "reason": "the page did not say which key"}}
        return {"id": rid, "text": "", "action": validate_action({"action": act})}
    if "action" in d:                                                                         # a button: fetch / return / home / stop, with a key where needed
        names = {"fetch": "fetch_key", "return": "return_key", "home": "go_home", "stop": "stop", "status": "status"}
        name = names.get(d["action"], d["action"])
        return {"id": rid, "text": "", "action": validate_action(dict(d, action=name))}
    text = d.get("text")
    if not isinstance(text, str) or not text.strip():
        raise ContractError("the request needs text or an action")
    if len(text) > MAX_TEXT:
        raise ContractError("the request is longer than %d characters" % MAX_TEXT)
    return {"id": rid, "text": text.strip(), "action": None}


def job_status(job, status, step=None, of=None, message="", error=None):
    """The status packet going back to the page. error: None or {"code", "title", "detail"} (the same fields the hub's structured errors have)."""
    if status not in STATUSES:
        raise ValueError("status must be one of %s" % (STATUSES,))
    if error is not None and not (isinstance(error, dict) and {"code", "title"} <= set(error)):
        raise ValueError("error needs code and title")
    led = "attention" if error and error.get("code") == "NEEDS_PERSON" else LED[status]
    return {"v": VERSION, "job": job, "status": status, "step": step, "of": of, "message": message, "error": error, "led": led}


def step_message(job, step, of, pose):
    """The message to the low-level side (Sam's Python code) for ONE step of a job. PLACEHOLDER: Sam has not defined the JSON yet.
    pose: {"x", "y", "z", "angle" (degrees), optional "speed" (mm/s), optional "action": "grab" | "release"}. Units are mm and degrees."""
    for k in ("x", "y", "z", "angle"):
        _num(pose.get(k), k)
    magnet = {"grab": "on", "release": "off", None: None}.get(pose.get("action"), "bad")
    if magnet == "bad":
        raise ContractError("action must be grab or release")
    return {"v": VERSION, "job": job, "type": "move", "step": step, "of": of,
            "target": {"x": float(pose["x"]), "y": float(pose["y"]), "z": float(pose["z"]), "pitch": float(pose["angle"]), "speed": pose.get("speed")},
            "magnet": magnet}
