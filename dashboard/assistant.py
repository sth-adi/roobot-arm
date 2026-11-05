"""The AI step of the coordinate design: a plain-language request -> ONE action from contract.MENU, or a question back to the person.

The model only CHOOSES from the menu. It never sees a coordinate and never writes an angle, a path or a packet; whatever it answers is checked by
contract.validate_action, and an answer that does not fit is refused (the keyword matcher then decides, and the reason is reported).
Stop is decided here by words, before any network call. With no GROQ_API_KEY, no network or a refused answer, keywords do the choosing.
Code after this point (the route generator, the planner checks, the hub) turns the action into movement.
"""
import json
import re
import time
import urllib.request

try:
    import agent as AG
    import contract as C
    from preflight import is_stop, keyword_action, has_topic
except ImportError:                                    # imported as dashboard.assistant
    from dashboard import agent as AG, contract as C
    from dashboard.preflight import is_stop, keyword_action, has_topic

WORD_KEYS = {"one": 1, "first": 1, "two": 2, "second": 2, "three": 3, "third": 3}
FETCH = {"fetch", "get", "bring", "give", "collect", "grab", "pick", "take", "need", "want", "hand", "request"}
RETURN = {"return", "put", "replace", "store", "restore", "back", "returning"}


def _key_in(request):
    found = set()
    for w in re.findall(r"[a-z0-9]+", request.lower()):
        if w in ("1", "2", "3"):
            found.add(int(w))
        elif w in WORD_KEYS:
            found.add(WORD_KEYS[w])
    return found.pop() if len(found) == 1 else None


def keyword_decide(request, last_move=None):
    """The decision by words alone. Always returns a valid action (ask when it is not sure)."""
    quick = keyword_action(request, last_move)
    if quick:
        a = {"action": {"home": "go_home"}.get(quick["action"], quick["action"]), **{k: v for k, v in quick.items() if k != "action"}}
        if a["action"] in C.MENU:
            try:
                return dict(C.validate_action(a), by="keywords")
            except C.ContractError:
                pass
    w = set(re.findall(r"[a-z]+", request.lower()))
    wants_return, wants_fetch = bool(w & RETURN), bool(w & FETCH)
    if wants_return != wants_fetch:
        a = {"action": "return_key" if wants_return else "fetch_key", "key": _key_in(request), "reason": "keyword match"}
        try:
            return dict(C.validate_action(a), by="keywords")
        except C.ContractError:
            return {"action": "ask", "question": "Which key: 1, 2 or 3?", "reason": "no single key named", "by": "keywords"}
    if has_topic(request):
        return {"action": "ask", "question": "Do you want to fetch a key or return one, and which: 1, 2 or 3?", "reason": "unclear", "by": "keywords"}
    return {"action": "ask", "question": "I can fetch or return a key (1, 2 or 3), go home, or stop. Which would you like?", "reason": "off topic", "by": "keywords"}


def _call(system, user, key, model, timeout=15):
    extra = {"reasoning_effort": "low", "max_completion_tokens": 600} if "gpt-oss" in model else {"max_tokens": 200}
    body = json.dumps({**extra, "model": model, "temperature": 0, "response_format": {"type": "json_object"},
                       "messages": [{"role": "system", "content": system}, {"role": "user", "content": user[:C.MAX_TEXT]}]}).encode()
    req = urllib.request.Request(AG.GROQ_URL, body, {"Content-Type": "application/json", "Authorization": "Bearer " + key,
                                                   "User-Agent": "roobot-arm/1.0", "Accept": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as r:
        out = json.loads(r.read())
    raw = out["choices"][0]["message"]["content"]
    return json.loads(raw), {"model": model, "ms": int((time.time() - t0) * 1000), "raw": raw[:500]}


def _choose(system, user, validate, timeout=15):
    """Ask Groq (model fallbacks as agent.py has them) and validate the answer. Returns (clean answer + meta, "") or (None, why it was not used)."""
    key = AG.api_key()
    if not key:
        return None, "no GROQ_API_KEY found (environment or .env in the repository folder)"
    err = ""
    try:
        import urllib.error
        order = list(dict.fromkeys([m for m in [AG._good_model[0], AG.GROQ_MODEL] + AG.FALLBACK_MODELS if m]))
        for m in order:
            try:
                ans, meta = _call(system, user, key, m, timeout)
            except urllib.error.HTTPError as e:
                err = AG._why(e)
                if e.code in (400, 403, 404):
                    continue
                break
            AG._good_model[0] = m
            try:
                return dict(validate(ans), by="groq", **meta), ""
            except C.ContractError as e:
                return None, "Groq's answer was refused: %s" % e
    except Exception as e:
        err = AG._why(e)
    return None, err


RECOVERY_RULES = ("You help a key-fetching robot arm recover from a failed step. You only CHOOSE one recovery action; you never write coordinates, angles or paths. "
                  'Answer ONLY with JSON: {"action": <"retry_step" | "lift_to_safe_height" | "go_home" | "release_magnet" | "ask_person" | "stop">, "question": <for ask_person>, "reason": <short>}. '
                  "retry_step runs the failed step again (the report says how many retries are left). lift_to_safe_height moves straight up, clear of every box. "
                  "go_home lifts, crosses and returns to the home position. release_magnet is only for when the key is held over the tray or its own box. "
                  "ask_person gives the person a question; stop leaves the arm where it is. Prefer retry_step for a timeout or a brief stall when retries are left; "
                  "prefer lift_to_safe_height when the arm is holding a key; never guess when a real fault is reported: ask_person.")


def recover(report):
    """report: the failure report jobs.py builds (error, step, of, label, holding, retries_left, pose_known, over). Returns a valid recovery action
    ({"action", ["question"], "by": "groq" | "rules", ["ai_error"]}). jobs.py has the final say: it checks that the action is allowed right now."""
    system = RECOVERY_RULES + " Failure report: " + json.dumps(report)
    a, err = _choose(system, "What should the arm do now?", C.validate_recovery, timeout=8)
    if a:
        return a
    return dict(rule_recovery(report), by="rules", ai_error=err)


def rule_recovery(report):
    """The fixed rule table used when the AI is not available: retry while retries are left, otherwise get clear and stop."""
    if report.get("retries_left", 0) > 0:
        return {"action": "retry_step", "reason": "retries left"}
    if report.get("holding") and report.get("pose_known"):
        return {"action": "lift_to_safe_height", "reason": "holding a key: get clear first"}
    if report.get("pose_known"):
        return {"action": "go_home", "reason": "no retries left"}
    return {"action": "ask_person", "question": "The arm stopped (%s). What should it do?" % report.get("error", {}).get("title", "a fault"), "reason": "position unknown"}


def _groq(request, state, key, model, timeout=15):
    system = ("You decide what a key-fetching robot arm should do. " + AG.SCOPE + " You only choose; you never write coordinates, angles or paths. "
              'Answer ONLY with JSON: {"action": <"fetch_key" | "return_key" | "go_home" | "stop" | "magnet" | "status" | "move_by" | "ask">, '
              '"key": <1, 2 or 3 for fetch_key/return_key>, "on": <true/false for magnet>, "dx": <mm>, "dy": <mm>, "dz": <mm>, "question": <for ask>, "reason": <short>}. '
              "fetch_key moves a key from its box to the tray; return_key moves it from the tray back to its box. If the person did not say which key, or the request is unclear, "
              "use ask with one short question. move_by is a small nudge (at most 100 mm total) and is NEVER used to fetch or return a key. status is a question, no motion. "
              "Arm state: " + json.dumps({k: state.get(k) for k in ("busy", "at_home", "holding", "keys")}))
    return _call(system, request, key, model, timeout)


def decide(request, state=None):
    """{"action": ..., fields, "by": "keywords" | "groq", ["ai_error": why Groq was not used]}; the action is always inside the contract."""
    state = state or {}
    if is_stop(request):                                         # safe even when it is a wrong guess, and needs no network
        return {"action": "stop", "reason": "stop word", "by": "keywords"}
    key = AG.api_key()
    err = ""
    if key:
        try:
            import urllib.error
            order = list(dict.fromkeys([m for m in [AG._good_model[0], AG.GROQ_MODEL] + AG.FALLBACK_MODELS if m]))
            for m in order:
                try:
                    ans, meta = _groq(request, state, key, m)
                except urllib.error.HTTPError as e:
                    err = AG._why(e)
                    if e.code in (400, 403, 404):
                        continue
                    break
                AG._good_model[0] = m
                try:
                    a = C.validate_action(ans)
                except C.ContractError as e:
                    err = "Groq's answer was refused: %s" % e
                    break
                return dict(a, by="groq", **meta)
        except Exception as e:
            err = AG._why(e)
    else:
        err = "no GROQ_API_KEY found (environment or .env in the repository folder)"
    d = keyword_decide(request, state.get("last_move"))
    d["ai_error"] = err
    return d
