"""Turns a plain-language request into "run this sequence, forwards or reversed".

The language model only CHOOSES from the sequences in the library; it never writes motion. Whatever it answers is checked against the
library before anything runs. With no GROQ_API_KEY (environment or .env) or no network, a keyword matcher does the choosing instead.
"""
import json
import os
import re
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GROQ_URL = "https://api.groq.com/openai/v1/chat/completions"
GROQ_MODEL = os.environ.get("GROQ_MODEL", "llama-3.3-70b-versatile")
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


def groq_choice(request, library, state, key, timeout=15):
    catalog = [{"name": s["name"], "description": s["description"], "requires": s["requires"], "ends": s["ends"]} for s in library]
    system = ("You pick which taught robot-arm sequence to run. Answer ONLY with JSON: "
              '{"sequence": <name from the list or null>, "reverse": <true to run it backwards for the opposite task>, "reason": <short>}. '
              "Each sequence moves a key between places; running it reversed does the opposite move. Use null if nothing fits. "
              "Sequences: " + json.dumps(catalog) + " Arm state: " + json.dumps({k: state.get(k) for k in ("at_home", "holding", "keys", "last_run")}) + " last_run is the sequence that ran most recently; undoing it means running it with the opposite reverse flag.")
    body = json.dumps({"model": GROQ_MODEL, "temperature": 0, "response_format": {"type": "json_object"},
                       "messages": [{"role": "system", "content": system}, {"role": "user", "content": request[:500]}]}).encode()
    req = urllib.request.Request(GROQ_URL, body, {"Content-Type": "application/json", "Authorization": "Bearer " + key})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        out = json.loads(r.read())
    ans = json.loads(out["choices"][0]["message"]["content"])
    return {"sequence": ans.get("sequence"), "reverse": bool(ans.get("reverse")), "reason": str(ans.get("reason", ""))[:200], "by": "groq"}


def choose(request, library, state):
    """{"sequence": name | None, "reverse": bool, "reason": str, "by": ...}; the name is always one that is in the library."""
    key = api_key()
    names = {s["name"] for s in library}
    if key:
        try:
            c = groq_choice(request, library, state, key)
            if c["sequence"] is None or c["sequence"] in names:
                return c
        except Exception:                              # no network, bad key, bad JSON: fall back
            pass
    return keyword_choice(request, library, state.get("last_run"))
