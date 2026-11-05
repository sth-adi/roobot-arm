"""The box store (the "?" box in the architecture sketch): the ONLY things we save about the workspace. Everything the arm does for a fetch or a return is
calculated from these numbers and a few constants.

Saved in one JSON file (default .boxes.json in the repository folder, git-ignored; written atomically so a crash cannot leave half a file):
    boxes   "1".."3": {"x", "y", "z", "size": {"w", "d", "h"} or null, "taught": true}     the key's pick-up point in the arm's frame (mm)
    tray    the same shape                                                                    where keys are put
    home    [x, y, z, pitch_deg]
    constants   safe_clearance_mm, hover_mm, lift_mm, approach_pitch_deg, speed_mm_s        PLACEHOLDER values: tune them on the real arm
Positions are the arm's own frame (the one dashboard/planner.py uses: x right, y away from the base, z up, mm). To fill them in, jog the tip to the
spot and call teach(); the numbers come from the arm itself, so calibration error is already in them.
A position is refused at save time if the planner cannot reach it from home (the same checks that guard every move).
"""
import json
import math
import os
import tempfile

try:
    import planner as PL
except ImportError:                                    # imported as dashboard.boxes
    from dashboard import planner as PL

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PATH = os.path.join(ROOT, ".boxes.json")
KEYS = ("1", "2", "3")
DEFAULT_HOME = [0.0, 250.0, 250.0, 0.0]
DEFAULT_CONSTANTS = {"safe_clearance_mm": 40.0, "hover_mm": 30.0, "lift_mm": 50.0, "approach_pitch_deg": 0.0, "speed_mm_s": 30.0}
CONSTANT_RANGE = {"safe_clearance_mm": (10, 200), "hover_mm": (5, 150), "lift_mm": (10, 200), "approach_pitch_deg": (-90, 90), "speed_mm_s": (1, 50)}
SIZE_RANGE = (5.0, 300.0)                              # mm: a box smaller or larger than this is a typing mistake
WORKSPACE = {"x": (-400.0, 400.0), "y": (-400.0, 400.0), "z": (0.0, 400.0)}


class StoreError(ValueError):
    """Something not saved or not usable. Carries the same fields as the other structured errors (code, title, detail, hint)."""
    def __init__(self, code, title, detail="", hint=""):
        super().__init__(title)
        self.code, self.title, self.detail, self.hint = code, title, detail, hint

    def as_dict(self):
        return {"code": self.code, "title": self.title, "detail": self.detail, "hint": self.hint}


def _num(v, name, lo=None, hi=None):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise StoreError("BAD_NUMBER", "%s must be a number" % name)
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise StoreError("OUT_OF_RANGE", "%s is outside %g to %g" % (name, lo, hi), "got %g" % v)
    return float(v)


def _size(size):
    if size is None:
        return None
    if not isinstance(size, dict) or set(size) != {"w", "d", "h"}:
        raise StoreError("BAD_SIZE", "A size needs w, d and h (mm)")
    return {k: _num(size[k], "size " + k, *SIZE_RANGE) for k in ("w", "d", "h")}


class Store:
    def __init__(self, path=None):
        self.path = path or DEFAULT_PATH
        self.data = {"version": 1, "boxes": {k: None for k in KEYS}, "tray": None, "home": list(DEFAULT_HOME), "constants": dict(DEFAULT_CONSTANTS)}
        self.load()

    # ---- file ----
    def load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                d = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as e:
            raise StoreError("BAD_FILE", "The box file cannot be read", "%s: %s" % (self.path, e), "Fix or delete it; the boxes then have to be taught again.")
        if not isinstance(d, dict) or d.get("version") != 1:
            raise StoreError("BAD_FILE", "The box file is not in the format this version understands", self.path, "")
        boxes = d.get("boxes") if isinstance(d.get("boxes"), dict) else {}
        for k in KEYS:
            self.data["boxes"][k] = self._clean_place(boxes.get(k)) if boxes.get(k) else None
        self.data["tray"] = self._clean_place(d["tray"]) if d.get("tray") else None
        if isinstance(d.get("home"), list) and len(d["home"]) == 4:
            self.data["home"] = [_num(v, "home") for v in d["home"]]
        for k, v in (d.get("constants") or {}).items():
            if k in DEFAULT_CONSTANTS:
                self.data["constants"][k] = _num(v, k, *CONSTANT_RANGE[k])

    def save(self):
        folder = os.path.dirname(self.path) or "."
        fd, tmp = tempfile.mkstemp(dir=folder, prefix=".boxes-", suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self.data, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # ---- places ----
    @staticmethod
    def _clean_place(p):
        if not isinstance(p, dict):
            raise StoreError("BAD_PLACE", "A place needs x, y and z")
        return {"x": _num(p.get("x"), "x", *WORKSPACE["x"]), "y": _num(p.get("y"), "y", *WORKSPACE["y"]), "z": _num(p.get("z"), "z", *WORKSPACE["z"]),
                "size": _size(p.get("size")), "taught": True}

    def _reachable(self, pt):
        """Refuse a position the planner could not reach from home (limits, singularities, jumps): at save time, not halfway through a fetch."""
        h = self.data["home"]
        pitch = math.radians(self.data["constants"]["approach_pitch_deg"])
        try:
            for z_extra in (0.0, self.data["constants"]["hover_mm"]):               # the point itself and the hover point above it
                PL.plan((h[0], h[1], h[2], math.radians(h[3])), (pt["x"], pt["y"], pt["z"] + z_extra, pitch))
        except PL.PlanError as e:
            raise StoreError("UNREACHABLE", "The arm cannot get there: " + e.title, e.detail, e.hint)

    def _set(self, name, x, y, z, size):
        place = self._clean_place({"x": x, "y": y, "z": z, "size": size})
        self._reachable(place)
        if name == "tray":
            self.data["tray"] = place
        else:
            self.data["boxes"][name] = place
        self.save()
        return place

    def set_box(self, key, x, y, z, size=None):
        if str(key) not in KEYS:
            raise StoreError("BAD_KEY", "There is no box %r (1, 2 or 3)" % (key,))
        return self._set(str(key), x, y, z, size)

    def set_tray(self, x, y, z, size=None):
        return self._set("tray", x, y, z, size)

    def teach(self, which, pose, size=None):
        """Save where the arm's tip is now. which: 1, 2, 3 or "tray"; pose: {"x", "y", "z"} as the arm reports it."""
        if which == "tray":
            return self.set_tray(pose["x"], pose["y"], pose["z"], size)
        return self.set_box(which, pose["x"], pose["y"], pose["z"], size)

    def set_home(self, x, y, z, pitch_deg=0.0):
        self.data["home"] = [_num(x, "x"), _num(y, "y"), _num(z, "z"), _num(pitch_deg, "pitch", -90, 90)]
        self.save()

    def set_constant(self, name, value):
        if name not in DEFAULT_CONSTANTS:
            raise StoreError("BAD_CONSTANT", "Unknown setting %r" % (name,), "one of %s" % sorted(DEFAULT_CONSTANTS))
        self.data["constants"][name] = _num(value, name, *CONSTANT_RANGE[name])
        self.save()

    # ---- reading ----
    def box(self, key):
        b = self.data["boxes"].get(str(key))
        if not b:
            raise StoreError("NOT_TAUGHT", "Box %s has not been set up yet" % key, "", "Jog the arm to the key in box %s and save its position." % key)
        return dict(b)

    def tray(self):
        if not self.data["tray"]:
            raise StoreError("NOT_TAUGHT", "The tray has not been set up yet", "", "Jog the arm over the tray and save its position.")
        return dict(self.data["tray"])

    def constants(self):
        return dict(self.data["constants"])

    def home(self):
        return list(self.data["home"])

    def missing(self):
        """What still has to be taught before a fetch or a return can run: [] when ready."""
        return ["box %s" % k for k in KEYS if not self.data["boxes"][k]] + (["tray"] if not self.data["tray"] else [])

    def obstacles(self):
        """Axis-aligned boxes (x0, y0, z0, x1, y1, z1) for everything with a known size: what the route must keep clear of. A place without a size is a point."""
        out = []
        for p in list(self.data["boxes"].values()) + [self.data["tray"]]:
            if p and p["size"]:
                w, d, h = p["size"]["w"], p["size"]["d"], p["size"]["h"]
                out.append((p["x"] - w / 2, p["y"] - d / 2, p["z"] - h, p["x"] + w / 2, p["y"] + d / 2, p["z"]))    # z is the top: the key is picked from the top surface
        return out
