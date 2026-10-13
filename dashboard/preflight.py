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
