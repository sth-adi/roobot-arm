"""Checks around an AI-chosen task, kept apart from the hub so they can be tested without hardware.

The model only CHOOSES a taught sequence. Before anything moves, check() compares that choice with what the arm can actually see
(the state packet: where each key is, whether it is busy). After the run, verify() checks the key really ended up where the
sequence says it should. Every failure is a structured error: {"code", "title", "detail", "hint"} so the dashboards can show it.
"""
import re

SCOPE = ("This is a key-fetching desk: 3 numbered keys start in boxes 1 to 3, there is one tray, and the arm moves a key between "
         "a box and the tray with an electromagnet. It cannot do anything else.")
KEY_IDS = ("1", "2", "3")
_TOPIC = re.compile(r"\b(keys?|box(es)?|tray|fetch|bring|return|put|take|move|pick|drop|give|hand)\b", re.I)


ACTIONS = ("run_sequence", "go_home", "stop", "magnet", "status", "move_by")   # the only things the AI can ask the arm to do besides running a taught sequence
STOP_RE = re.compile(r"\b(stop|halt|abort|freeze|e-?stop|emergency|cancel)\b", re.I)
_TOPIC_WORDS = {"key", "keys", "box", "boxes", "tray"}


# ---- free movement: small relative nudges only (x right, y away from the base, z up). Never used for fetching or returning keys. ----
MOVE_MAX_MM = 100.0        # per command; ask again for more
MOVE_MIN_Z = 10.0          # the tip is never sent lower than this above the table
_DIR = {"up": (0, 0, 1), "raise": (0, 0, 1), "lift": (0, 0, 1), "down": (0, 0, -1), "lower": (0, 0, -1), "left": (-1, 0, 0), "right": (1, 0, 0),
        "forward": (0, 1, 0), "forwards": (0, 1, 0), "back": (0, -1, 0), "backward": (0, -1, 0), "backwards": (0, -1, 0)}
_MOVE = re.compile(r"\b(up|down|raise|lift|lower|left|right|forwards?|backwards?|back)\b(?:\s+(?:by|about))?\s+(\d+(?:\.\d+)?)\s*(mm|millimet\w*|cm|centimet\w*)\b", re.I)


def has_topic(request):
    return bool(set(re.findall(r"[a-z]+", request.lower())) & _TOPIC_WORDS)


def parse_move(request):
    """'move up 2 cm and left 5 mm' -> {"dx","dy","dz"} in mm. Only with an explicit number and unit, and never in a request about keys, boxes or the tray."""
    if has_topic(request):
        return None
    d = [0.0, 0.0, 0.0]
    found = False
    for m in _MOVE.finditer(request):
        mm = float(m.group(2)) * (10.0 if m.group(3).lower().startswith("c") else 1.0)
        for i, c in enumerate(_DIR[m.group(1).lower()]):
            d[i] += c * mm
        found = True
    return {"dx": d[0], "dy": d[1], "dz": d[2]} if found else None


UNDO_WORDS = {"undo", "back", "reverse", "revert", "restore", "opposite", "return"}


def undo_move(request, last_move):
    """'move it back' / 'undo' right after a nudge -> the opposite nudge, else None. Only when the last thing done was a nudge,
    and never for a request about keys, boxes or the tray, or one that names a distance (that is a new move) or home."""
    if not last_move or has_topic(request) or parse_move(request):
        return None
    w = set(re.findall(r"[a-z]+", request.lower()))
    if not w & UNDO_WORDS or w & {"home", "default", "rest", "magnet"}:
        return None
    try:
        return {"action": "move_by", "reason": "undoing the last nudge", **{k: -float(last_move[k]) for k in ("dx", "dy", "dz")}}
    except (TypeError, ValueError, KeyError):
        return None


def check_move(delta, pose, reachable):
    """Errors that stop a nudge before the arm moves, else None. reachable(x, y, z, angle) -> bool is the inverse-kinematics check."""
    try:
        dx, dy, dz = (float(delta[k]) for k in ("dx", "dy", "dz"))
        x, y, z, a = (float(pose[k]) for k in ("x", "y", "z", "angle"))
    except (TypeError, ValueError, KeyError):
        return err("MOVE_UNKNOWN", "I could not read the move or the arm's position", "", "Say it like: move up 2 cm.")
    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
    if dist != dist or dist < 1.0:
        return err("NOTHING_TO_MOVE", "That is less than 1 mm", "", "Give a distance, like: move up 2 cm.")
    if dist > MOVE_MAX_MM:
        return err("MOVE_TOO_FAR", "That is %.0f mm: the limit is %.0f mm per command" % (dist, MOVE_MAX_MM), "", "Ask in smaller steps.")
    if z + dz < MOVE_MIN_Z:
        return err("MOVE_TOO_LOW", "That would put the tip %.0f mm above the table; the minimum is %.0f mm" % (z + dz, MOVE_MIN_Z), "", "Move less far down.")
    for i in range(1, 11):                                    # the whole straight line must be reachable, not only the end
        f = i / 10.0
        if not reachable(x + dx * f, y + dy * f, z + dz * f, a):
            return err("MOVE_UNREACHABLE", "The arm cannot reach there", "Part of that straight line is outside its reach or joint limits.", "Try a smaller move or another direction.")
    return None


def is_stop(request):
    """Stopping is safe even when the guess is wrong, so it is decided here, before any AI or network call."""
    return bool(STOP_RE.search(request))


def keyword_action(request, last_move=None):
    """A plan for the quick commands (stop, home, magnet, status) found by words alone, or None. Used without Groq and as a double check."""
    w = set(re.findall(r"[a-z]+", request.lower()))
    if is_stop(request):
        return {"action": "stop", "reason": "stop word"}
    on, off = w & {"on", "enable", "activate", "engage", "grab"}, w & {"off", "disable", "deactivate", "release", "drop"}
    if ("magnet" in w or w & {"release", "drop"}) and bool(on) != bool(off):
        return {"action": "magnet", "on": bool(on), "reason": "magnet words"}
    mv = parse_move(request)
    if mv:
        return dict({"action": "move_by", "reason": "explicit distance"}, **mv)
    undo = undo_move(request, last_move)
    if undo:
        return undo
    if w & {"home", "default", "rest"} and not w & _TOPIC_WORDS:
        return {"action": "go_home", "reason": "home words"}
    if (w & {"status", "where", "holding", "state"} or request.strip().endswith("?")) and not w & {"put", "move", "take", "bring", "fetch", "give", "return"}:
        return {"action": "status", "reason": "question"}
    return None


def describe(state):
    """One plain sentence about the arm, built only from the state packet (never from model text)."""
    place = lambda v: ("on the " + v) if v == "tray" else ("in " + v) if str(v).startswith("box") else str(v)
    keys = ", ".join("key %s %s" % (k, place(v)) for k, v in sorted((state.get("keys") or {}).items()))
    return "%s, magnet %s, %s. %s%s" % ("At home" if state.get("at_home") else "Not at home", "on" if state.get("magnet") else "off",
                                       "holding a key" if state.get("holding") else "not holding a key", keys.capitalize() + "." if keys else "",
                                       " Running a sequence." if state.get("mode") == "running" else "")


def check_action(plan, state, output_ok):
    """Stop is always allowed; moving the real arm needs the output armed. None when fine."""
    a = plan.get("action")
    if a in ("go_home", "magnet", "move_by") and not output_ok:
        return err("OUTPUT_SAFE", "The output is SAFE", "This would move the real arm.", "Arm the output (Escape returns to SAFE), then ask again.")
    return None


def err(code, title, detail="", hint=""):
    return {"code": code, "title": title, "detail": detail, "hint": hint}


def loc(name):
    """'box2' -> 'box 2', 'Tray' -> 'tray' (the spelling the state packet uses)."""
    n = str(name or "").strip().lower()
    m = re.fullmatch(r"box\s*(\d+)", n)
    return "box %s" % m.group(1) if m else n


def asked_key(request, plan=None):
    """Which key number the person named ('key 2', 'key #2', 'number 2 key'); the model's reading only fills a gap."""
    m = re.search(r"\bkey\s*(?:number|no\.?|#)?\s*(\d+)\b", request, re.I) or re.search(r"\b(?:number|no\.?|#)\s*(\d+)\s*key\b", request, re.I)
    if m:
        return m.group(1)
    k = (plan or {}).get("key")
    return str(k) if k not in (None, "", "null") else None


def where_all(keys):
    return ", ".join("key %s: %s" % (k, v) for k, v in sorted(keys.items()))


def no_sequence(request, plan):
    """The model (or the keywords) found nothing to run: say whether the request is out of scope or just not taught yet."""
    if plan.get("error_code") == "OUT_OF_SCOPE" or not _TOPIC.search(request):
        return err("OUT_OF_SCOPE", "I can only move keys between the boxes and the tray",
                   plan.get("reason") or "That request is outside what this arm does.", SCOPE)
    return err("NOT_TAUGHT", "No taught sequence does that yet", plan.get("reason") or "",
               "Teach it under Robot > Sequences, then ask again. Taught: see the list below the Ask box.")


def check(request, plan, seq, state):
    """Errors that stop the run before the arm moves ([] when it is fine)."""
    keys = state.get("keys") or {}
    src_loc = loc((seq.get("requires") or {}).get("key_in") or (seq.get("requires") or {}).get("key_on"))
    dst_loc = loc((seq.get("ends") or {}).get("key_on") or (seq.get("ends") or {}).get("key_in"))
    k = asked_key(request, plan)
    out = []
    if k is not None and k not in KEY_IDS:
        return [err("UNKNOWN_KEY", "There is no key %s" % k, "The keys are numbered 1 to %s." % KEY_IDS[-1], SCOPE)]
    if k is not None and k not in keys:
        return [err("STATE_MISSING", "Cannot see key %s" % k, "The arm's state packet does not list it.", "Check the ESP32 / simulator is connected.")]
    if k is not None:
        here = keys[k]
        if here == "on the magnet":
            return [err("KEY_ON_MAGNET", "Key %s is already on the magnet" % k, "The arm is holding it.", "Release it first (X switches the magnet off).")]
        if here == "falling":
            return [err("KEY_MOVING", "Key %s is still moving" % k, "", "Wait a moment and ask again.")]
        if here == "floor":
            return [err("KEY_ON_FLOOR", "Key %s is on the floor" % k, "The arm cannot reach it from there.", "Put it back in its box (Reset keys), then ask again.")]
        if dst_loc and here == dst_loc:
            return [err("ALREADY_DONE", "Key %s is already %s %s" % (k, "on the" if dst_loc == "tray" else "in", dst_loc), "Nothing to do.",
                        "To move it back, ask: put key %s back in its box." % k if dst_loc == "tray" else "")]
        if src_loc and here != src_loc:
            hint = "Key %s is %s %s." % (k, "on the" if here == "tray" else "in", here)
            if here == "tray" or here.startswith("box"):
                hint += " Ask for it from there instead."
            return [err("KEY_NOT_AT_SOURCE", "Key %s is not %s %s" % (k, "on the" if src_loc == "tray" else "in", src_loc),
                        "The move %s needs it there. Keys now: %s." % (plan.get("sequence"), where_all(keys)), hint)]
    elif src_loc and src_loc not in keys.values():
        return [err("KEY_NOT_AT_SOURCE", "No key is %s %s" % ("on the" if src_loc == "tray" else "in", src_loc),
                    "The move %s starts there. Keys now: %s." % (plan.get("sequence"), where_all(keys)), "Reset keys, or ask for a different move.")]
    if state.get("holding"):
        out.append(err("HOLDING", "The arm is holding a key", "It must be released before a new move.", "Switch the magnet off (X)."))
    return out


def verify(seq, before, after, request="", plan=None):
    """After the run: did the key really end up where the sequence says?  Returns an error or None."""
    dst = loc((seq.get("ends") or {}).get("key_on") or (seq.get("ends") or {}).get("key_in"))
    if not dst:
        return None
    k = asked_key(request, plan)
    ka, kb = after.get("keys") or {}, before.get("keys") or {}
    if k is not None:
        ok = ka.get(k) == dst
        who = "Key %s" % k
    else:
        moved = [i for i in ka if kb.get(i) != ka.get(i)]
        ok = any(ka[i] == dst for i in moved) or list(ka.values()).count(dst) > list(kb.values()).count(dst)
        who = "The key"
    if ok:
        return None
    return err("MOVE_FAILED", "%s did not end up %s %s" % (who, "on the" if dst == "tray" else "in", dst),
               "The arm finished the move, but the keys are now: %s." % where_all(ka),
               "The magnet may have missed it. Check the arm is calibrated, then try again.")
