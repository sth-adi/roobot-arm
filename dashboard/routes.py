"""The route generator: a box coordinate in, a list of steps out. This is the "calculate it mathematically" part of the design; there is no saved sequence.

fetch_route(store, key, start)    from where the arm is: up, across, down to the key, grab, up, across to the tray, down a little, release, up, home
return_route(store, key, start)   the same the other way: tray -> box
home_route(store, start)          up to the travel height, across, down to home (used by recovery and by "go home")
lift_route(store, start)          straight up to the travel height (recovery: get clear of everything without sideways movement)

Every route is CHECKED before it is returned (RouteError otherwise, with the same fields as the other structured errors):
  * every leg is tested against the stored box sizes, grown by half the safe clearance on every side (a leg that straight-descends onto its own
    box is allowed to enter that one box's clearance zone, nothing else is)
  * every leg is dry-run through the planner (planner.plan), so a point the arm cannot reach, a joint limit or a singularity is found now, not halfway
A step is {"x","y","z","angle" (deg), "speed" (mm/s), optional "action": "grab" | "release", "label"}; this is what lowlevel's senders take.
Heights: the travel height is the top of the tallest stored place plus safe_clearance_mm (never lower than where the arm starts), so sideways movement
is always above everything. All constants are the placeholders in boxes.DEFAULT_CONSTANTS until they are measured on the real arm.
"""
import math

try:
    import boxes as B
    import planner as PL
except ImportError:                                    # imported as dashboard.routes
    from dashboard import boxes as B, planner as PL

SLOW = 0.5                                             # speed factor near contact (the grab, the release and the first lift)
EPS = 1e-6


class RouteError(ValueError):
    def __init__(self, code, title, detail="", hint=""):
        super().__init__(title)
        self.code, self.title, self.detail, self.hint = code, title, detail, hint

    def as_dict(self):
        return {"code": self.code, "title": self.title, "detail": self.detail, "hint": self.hint}


def _grow(box, m):
    x0, y0, z0, x1, y1, z1 = box
    return (x0 - m, y0 - m, z0 - m, x1 + m, y1 + m, z1 + m)


def segment_hits_box(p, q, box):
    """True when the straight segment p->q passes through the axis-aligned box (x0,y0,z0,x1,y1,z1), edges included. Slab method."""
    t0, t1 = 0.0, 1.0
    for i in range(3):
        d = q[i] - p[i]
        lo, hi = box[i], box[i + 3]
        if abs(d) < EPS:
            if p[i] < lo or p[i] > hi:
                return False
        else:
            a, b = (lo - p[i]) / d, (hi - p[i]) / d
            if a > b:
                a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
            if t0 > t1:
                return False
    return True


def _escapes_upward(a, b, box):
    """A leg that only goes straight up is always allowed to leave a box's clearance zone (the arm may start just above or beside one),
    as long as its start is not inside the box itself (standing on its top surface, where a key is picked, is not inside)."""
    if abs(b[0] - a[0]) > EPS or abs(b[1] - a[1]) > EPS or b[2] <= a[2]:
        return False
    return not (box[0] <= a[0] <= box[3] and box[1] <= a[1] <= box[4] and box[2] <= a[2] < box[5] - EPS)


def _places(store):
    """Every stored place as (name, x, y, z, size-or-None); missing ones are skipped (the route builders ask for what they need and fail on its own)."""
    out = [("box %s" % k, p) for k, p in store.data["boxes"].items() if p]
    if store.data["tray"]:
        out.append(("tray", store.data["tray"]))
    return out


def travel_height(store, start_z):
    c = store.constants()
    tops = [p["z"] for _, p in _places(store)]
    return max(max(tops, default=0.0) + c["safe_clearance_mm"], start_z)


def check_legs(store, points, owners=None):
    """points: [(x,y,z), ...] in order. A leg may not touch a (grown) stored box, except the box named by owners[i] for the leg ending at points[i+1].
    Raises RouteError naming the leg and the box."""
    margin = store.constants()["safe_clearance_mm"] / 2.0
    stored = []
    for name, p in _places(store):
        if p["size"]:
            w, d, h = p["size"]["w"], p["size"]["d"], p["size"]["h"]
            stored.append((name, (p["x"] - w / 2, p["y"] - d / 2, p["z"] - h, p["x"] + w / 2, p["y"] + d / 2, p["z"])))
    for i, (a, b) in enumerate(zip(points, points[1:])):
        owner = owners[i] if owners and i < len(owners) else None
        for name, box in stored:
            if name == owner or _escapes_upward(a, b, box):
                continue
            if segment_hits_box(a, b, _grow(box, margin)):
                raise RouteError("COLLISION", "The route would hit %s" % name,
                                 "Leg %d, from (%.0f, %.0f, %.0f) to (%.0f, %.0f, %.0f), passes within %.0f mm of it." % (i + 1, *a, *b, margin),
                                 "Check the stored size of %s, or move it further from the path." % name)


def _step(x, y, z, angle, speed, label, action=None):
    s = {"x": float(x), "y": float(y), "z": float(z), "angle": float(angle), "speed": float(speed), "label": label}
    if action:
        s["action"] = action
    return s


def dry_run(steps, start, limits=None):
    """Plan every leg with the real planner. start: {"x","y","z","angle"}. Returns the total time in seconds; raises RouteError on the first refused leg."""
    prev = (start["x"], start["y"], start["z"], math.radians(start["angle"]))
    total = 0.0
    for i, s in enumerate(steps):
        cur = (s["x"], s["y"], s["z"], math.radians(s["angle"]))
        try:
            p = PL.plan(prev, cur, limits, speed_mm_s=s["speed"])
        except PL.PlanError as e:
            raise RouteError(e.code, "Step %d (%s): %s" % (i + 1, s["label"], e.title), e.detail, e.hint)
        total += p["duration"]
        prev = cur
    return total


def _finish(store, steps, start, owners, limits):
    pts = [(start["x"], start["y"], start["z"])] + [(s["x"], s["y"], s["z"]) for s in steps]
    check_legs(store, pts, owners)
    return {"steps": steps, "seconds": dry_run(steps, start, limits)}


def _start(start):
    try:
        return {k: float(start[k]) for k in ("x", "y", "z", "angle")}
    except (KeyError, TypeError, ValueError):
        raise RouteError("NO_POSITION", "The arm's position is not known", "", "Send the arm home first, or wait for its first position report.")


def _route(store, key_or_none, direction, start, limits):
    c = store.constants()
    try:
        if direction == "fetch":
            pick, drop = store.box(key_or_none), store.tray()
            pick_name, drop_name = "box %s" % key_or_none, "tray"
        else:
            pick, drop = store.tray(), store.box(key_or_none)
            pick_name, drop_name = "tray", "box %s" % key_or_none
    except B.StoreError as e:
        raise RouteError(e.code, e.title, e.detail, e.hint)
    st, ang, v, hz = _start(start), c["approach_pitch_deg"], c["speed_mm_s"], c["hover_mm"]
    tz = travel_height(store, st["z"])
    home = store.home()
    rows = [
        (st["x"], st["y"], tz, st["angle"], v, "up to travel height", None, None),
        (pick["x"], pick["y"], tz, ang, v, "across to " + pick_name, None, None),
        (pick["x"], pick["y"], pick["z"] + hz, ang, v, "down to hover over " + pick_name, None, pick_name),
        (pick["x"], pick["y"], pick["z"], ang, v * SLOW, "down to the key", "grab", pick_name),
        (pick["x"], pick["y"], pick["z"] + c["lift_mm"], ang, v * SLOW, "lift the key", None, pick_name),
        (pick["x"], pick["y"], tz, ang, v, "up to travel height", None, pick_name),
        (drop["x"], drop["y"], tz, ang, v, "across to " + drop_name, None, None),
        (drop["x"], drop["y"], drop["z"] + hz, ang, v * SLOW, "down over " + drop_name, "release", drop_name),
        (drop["x"], drop["y"], tz, ang, v, "up to travel height", None, drop_name),
        (home[0], home[1], tz, home[3], v, "across to home", None, None),
        (home[0], home[1], home[2], home[3], v, "down to home", None, None),
    ]
    steps = [_step(x, y, z, a, sp, lab, act) for x, y, z, a, sp, lab, act, _ in rows]
    owners = [o for *_, o in rows]
    return _finish(store, steps, st, owners, limits)


def fetch_route(store, key, start, limits=None):
    return _route(store, key, "fetch", start, limits)


def return_route(store, key, start, limits=None):
    return _route(store, key, "return", start, limits)


def lift_route(store, start, limits=None):
    st, v = _start(start), store.constants()["speed_mm_s"]
    tz = travel_height(store, st["z"])
    if tz - st["z"] < 1.0:
        return {"steps": [], "seconds": 0.0}
    steps = [_step(st["x"], st["y"], tz, st["angle"], v * SLOW, "straight up to travel height")]
    return _finish(store, steps, st, None, limits)


def home_route(store, start, limits=None):
    st, v, home = _start(start), store.constants()["speed_mm_s"], store.home()
    tz = travel_height(store, st["z"])
    steps = [_step(st["x"], st["y"], tz, st["angle"], v, "up to travel height"),
             _step(home[0], home[1], tz, home[3], v, "across to home"),
             _step(home[0], home[1], home[2], home[3], v, "down to home")]
    return _finish(store, steps, st, None, limits)
