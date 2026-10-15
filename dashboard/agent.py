"""Turns a plain-language request into "run this sequence, forwards or reversed".

The language model only CHOOSES from the sequences in the library; it never writes motion. Whatever it answers is checked against the
library before anything runs. With no GROQ_API_KEY (environment or .env) or no network, a keyword matcher does the choosing instead.
"""
import json
import os
import re
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
try:
    from preflight import ACTIONS, SCOPE, keyword_action
except ImportError:                                    # imported as dashboard.agent
    from dashboard.preflight import ACTIONS, SCOPE, keyword_action
GROQ_URL = "https://api.groq.com/openai/v1/chat/completions"
GROQ_MODEL = os.environ.get("GROQ_MODEL", "openai/gpt-oss-20b")
FALLBACK_MODELS = ["openai/gpt-oss-120b", "llama-3.1-8b-instant"]   # cheapest first (Groq list prices); tried in turn when Groq says model_not_found (a project can have models switched off)
_good_model = [None]                                                                      # the model that last worked, tried first next time
REVERSE_WORDS = {"back", "reverse", "undo", "return", "restore", "opposite", "revert"}
STOP = {"the", "a", "an", "to", "from", "into", "in", "on", "of", "put", "move", "please", "key", "and", "it", "me"}


def api_key():
    k = os.environ.get("GROQ_API_KEY", "").strip()
    if k:
        return k
    try:
        for line in (ROOT / ".env").read_text(encoding="utf-8").splitlines():
            m = re.match(r"\s*GROQ_API_KEY\s*=\s*(.*)", line)
            if m:
                return m.group(1).strip().strip('"\'')
    except OSError:
        pass
    return ""


def _words(s):
    return [w for w in re.findall(r"[a-z0-9]+", s.lower().replace("_", " ")) if w not in STOP]


def keyword_choice(request, library, last=None):
    """Best overlap between the request and each sequence's name + description; backwards words flip the direction."""
    quick = keyword_action(request)
    if quick:
        return dict({"sequence": None, "reverse": False, "by": "keywords"}, **quick)
    req = set(_words(request))
    rev = bool(req & REVERSE_WORDS)
    best, best_score = None, 0
    for s in library:
        base = s["name"][:-8] if s["name"].endswith("_reverse") else s["name"]
        score = len(req & set(_words(base + " " + s["description"])))
        if score > best_score:
            best, best_score = s, score
    if best is None and rev and last and any(s["name"] == last["name"] for s in library):
        return {"sequence": last["name"], "reverse": not last["reverse"], "reason": "undoing the last move", "by": "keywords"}
    if best is None:
        return {"sequence": None, "reverse": False, "reason": "no sequence matches that request", "by": "keywords"}
    return {"sequence": best["name"], "reverse": rev, "reason": "keyword match", "by": "keywords"}


def groq_choice(request, library, state, key, timeout=15, model=None):
    catalog = [{"name": s["name"], "description": s["description"], "requires": s["requires"], "ends": s["ends"]} for s in library]
    system = ("You decide what a robot arm should do. " + SCOPE + " Never invent a sequence and never write coordinates or angles. Answer ONLY with JSON: "
              '{"action": <"run_sequence" | "go_home" | "stop" | "magnet" | "status" | "move_by">, "sequence": <name from the list or null>, "reverse": <true to run it backwards for the opposite task>, '
              '"key": <key number the person named, else null>, "on": <true or false, for magnet only>, '
              '"dx": <mm>, "dy": <mm>, "dz": <mm>, "reason": <short>}. '
              'Use go_home for "go to home position" or "return to default"; stop to stop moving; magnet to switch the electromagnet on or off (release/drop means off); '
              'status for a question about the arm or keys (no motion); run_sequence for everything that moves a key. '
              'move_by nudges the tip by a small distance in mm (dx right, dy away from the base, dz up; at most 100 mm in total) and is NEVER used for fetching, returning or putting a key. '
              "Each sequence moves a key between places; running it reversed does the opposite move. Use null if nothing fits. "
              "Sequences: " + json.dumps(catalog) + " Arm state: " + json.dumps({k: state.get(k) for k in ("at_home", "holding", "keys", "last_run")}) + " last_run is the sequence that ran most recently; undoing it means running it with the opposite reverse flag.")
    extra = {"reasoning_effort": "low", "max_completion_tokens": 600} if "gpt-oss" in (model or GROQ_MODEL) else {"max_tokens": 200}   # a one-line JSON answer needs little thinking
    body = json.dumps({**extra, "model": model or GROQ_MODEL, "temperature": 0, "response_format": {"type": "json_object"},
                       "messages": [{"role": "system", "content": system}, {"role": "user", "content": request[:500]}]}).encode()
    req = urllib.request.Request(GROQ_URL, body, {"Content-Type": "application/json", "Authorization": "Bearer " + key,
                                                 "User-Agent": "roobot-arm/1.0", "Accept": "application/json"})   # Groq sits behind Cloudflare, which rejects the default Python-urllib agent
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as r:
        out = json.loads(r.read())
    raw = out["choices"][0]["message"]["content"]
    ans = json.loads(raw)
    act = ans.get("action") if ans.get("action") in ACTIONS else "run_sequence"
    return {"action": act, "on": ans.get("on") if isinstance(ans.get("on"), bool) else None,
            **{k: ans.get(k) for k in ("dx", "dy", "dz") if isinstance(ans.get(k), (int, float)) and not isinstance(ans.get(k), bool)}, "sequence": ans.get("sequence"), "reverse": bool(ans.get("reverse")), "key": ans.get("key"), "reason": str(ans.get("reason", ""))[:200], "by": "groq",
            "model": model or GROQ_MODEL, "raw": raw[:1000], "ms": int((time.time() - t0) * 1000), "tokens": (out.get("usage") or {}).get("total_tokens")}   # raw/ms/tokens are shown in the chat page


def _why(e):
    import urllib.error
    if isinstance(e, urllib.error.HTTPError):
        try:
            body = e.read().decode("utf-8", "replace")[:160]
        except Exception:
            body = ""
        return "Groq answered HTTP %s %s" % (e.code, body)
    return "%s: %s" % (type(e).__name__, str(e)[:160])


def choose(request, library, state):
    """{"sequence": name | None, "reverse": bool, "reason": str, "by": ...}; the name is always one that is in the library."""
    key = api_key()
    err = ""
    names = {s["name"] for s in library}
    if key:
        try:
            import urllib.error
            order = [m for m in [_good_model[0], GROQ_MODEL] + FALLBACK_MODELS if m]
            order = list(dict.fromkeys(order))
            for m in order:
                try:
                    c = groq_choice(request, library, state, key, model=m)
                except urllib.error.HTTPError as e:
                    err = _why(e)
                    if e.code in (400, 403, 404):       # this model is unavailable to the key: try the next one
                        continue
                    break
                _good_model[0] = m
                c.setdefault("action", "run_sequence")
                if c["action"] == "magnet" and c.get("on") is None:
                    err = "Groq asked for the magnet without saying on or off"
                    break
                if c["action"] != "run_sequence" or c["sequence"] is None or c["sequence"] in names:
                    return c
                err = "Groq named a sequence that is not in the library"
                break
        except Exception as e:                         # no network, bad key, bad JSON: fall back, but say why
            err = _why(e)
    else:
        err = "no GROQ_API_KEY found (environment or .env in the repository folder)"
    c = keyword_choice(request, library, state.get("last_run"))
    c["ai_error"] = err
    return c
