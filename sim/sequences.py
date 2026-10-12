"""Move sequences as data: named lists of tool-tip poses with an optional magnet action (grab / release) on each.

A pose is {"x","y","z","angle","action"} in the arm's base frame (mm, degrees). The magnet is stored as an ACTION so that a sequence
can be played backwards: going forward you grab at the box and release at the tray, going back you grab at the tray and release
at the box. Reversing flips the order and swaps grab and release. The magnet state at each pose is derived by running the actions.
"""
import json
import math
import re
from pathlib import Path

FORMAT = "roobot-arm-sequence"
VERSION = 2
MAX_POSES = 20
ACTIONS = (None, "grab", "release")
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")


def actions_from_states(states):
    """Magnet state per pose (what was on when the pose was saved) -> action per pose (the change into that state)."""
    out, prev = [], False
    for on in states:
        on = bool(on)
        out.append("grab" if on and not prev else "release" if prev and not on else None)
        prev = on
    return out


def states_from_actions(actions):
    out, on = [], False
    for a in actions:
        on = True if a == "grab" else False if a == "release" else on
        out.append(on)
    return out


def make_sequence(saved, states, name="", description="", requires=None, ends=None):
    acts = actions_from_states(states)
    poses = [{"x": round(p[0], 2), "y": round(p[1], 2), "z": round(p[2], 2), "angle": round(p[3], 2), "action": acts[i] if i < len(acts) else None}
             for i, p in enumerate(saved)]
    return {"format": FORMAT, "version": VERSION, "name": name, "description": description,
            "requires": requires or {}, "ends": ends or {}, "count": len(poses), "poses": poses}


def validate(pkt):
    """Return a clean sequence dict (version 2) from a version 1 or 2 packet, or raise ValueError."""
    if not isinstance(pkt, dict) or pkt.get("format") != FORMAT:
        raise ValueError("not a roobot-arm-sequence packet")
    items = pkt.get("poses", pkt.get("positions"))
    if not isinstance(items, list) or not items:
        raise ValueError("a sequence needs at least one pose")
    if len(items) > MAX_POSES:
        raise ValueError("at most %d poses" % MAX_POSES)
    if pkt.get("poses") is None:                                  # version 1: magnet state per position
        items = [dict(it, action=a) for it, a in zip(items, actions_from_states([bool(i.get("magnet")) for i in items if isinstance(i, dict)]))]
    poses = []
    for it in items:
        try:
            v = [float(it[k]) for k in ("x", "y", "z", "angle")]
        except (KeyError, TypeError, ValueError):
            raise ValueError("each pose needs numeric x, y, z and angle")
        if not all(math.isfinite(c) and abs(c) < 1000 for c in v):
            raise ValueError("pose out of range")
        act = it.get("action")
        if act not in ACTIONS:
            raise ValueError("action must be grab, release or null")
        poses.append({"x": v[0], "y": v[1], "z": v[2], "angle": v[3], "action": act})
    name = str(pkt.get("name") or "")
    if name and not NAME_RE.match(name):
        raise ValueError("name: letters, digits, - and _ only")
    return {"format": FORMAT, "version": VERSION, "name": name, "description": str(pkt.get("description") or "")[:400],
            "requires": dict(pkt.get("requires") or {}), "ends": dict(pkt.get("ends") or {}), "count": len(poses), "poses": poses}


def reverse(seq):
    """The opposite task: poses backwards, grab and release swapped, start and end conditions swapped."""
    swap = {"grab": "release", "release": "grab", None: None}
    rev = [dict(p, action=swap[p["action"]]) for p in reversed(seq["poses"])]
    base = seq.get("name") or ""
    name = base[:-8] if base.endswith("_reverse") else (base + "_reverse" if base else "")
    return dict(seq, name=name, poses=rev, requires=dict(seq.get("ends") or {}), ends=dict(seq.get("requires") or {}),
                description=("Reverse of: " + seq["description"]) if seq.get("description") else "")


def to_saved(seq):
    """(positions as ArmSim.saved wants them, magnet state per position)."""
    return [(p["x"], p["y"], p["z"], p["angle"]) for p in seq["poses"]], states_from_actions([p["action"] for p in seq["poses"]])


class Library:
    def __init__(self, folder):
        self.folder = Path(folder)

    def _path(self, name):
        if not NAME_RE.match(name or ""):
            raise ValueError("name: letters, digits, - and _ only")
        return self.folder / (name + ".json")

    def save(self, seq):
        if not seq.get("name"):
            raise ValueError("give the sequence a name")
        self.folder.mkdir(parents=True, exist_ok=True)
        p = self._path(seq["name"])
        p.write_text(json.dumps(seq, indent=2) + "\n", encoding="utf-8")
        return p

    def get(self, name):
        p = self._path(name)
        if not p.exists():
            raise KeyError(name)
        return validate(json.loads(p.read_text(encoding="utf-8")))

    def list(self):
        out = []
        for p in sorted(self.folder.glob("*.json")) if self.folder.exists() else []:
            try:
                s = validate(json.loads(p.read_text(encoding="utf-8")))
            except (ValueError, OSError):
                continue
            out.append({"name": s["name"] or p.stem, "description": s["description"], "count": s["count"],
                        "requires": s["requires"], "ends": s["ends"]})
        return out
