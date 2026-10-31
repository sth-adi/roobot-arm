"""A safer move planner for the arm: the same inverse kinematics as Sam's backend (firmware/sam/backend/motion/kinematics.py), the same
straight line, the same packet format ("time_ms/j1/j2/j3/j4", degrees), but checked before anything moves.

What it adds to Sam's CompileAnglePacket:
  * every step of the path is solved first, and the move is refused (PlanError) if any step is unreachable, outside the calibrated joint limits,
    too close to the base axis, or needs a joint to jump
  * a fixed update rate (default 50 Hz, what a hobby servo can follow) instead of 5 steps per mm
  * smooth start and stop (quintic easing) so the arm does not lurch
  * the whole move is stretched in time, never one joint clipped, so that no joint exceeds max_joint_dps: the path stays a straight line, just slower
Pure Python with no dependencies. firmware/ours/build_backend.py copies it into Sam's backend as motion/safe_planner.py.
"""
import math

# Link lengths and offsets, copied from Sam's InverseKim (mm). TOOL is the tool length (63 mm with the ball, the same as the simulation).
TOOL = 63.0
SHOULDER_Z = 74.0
UPPER = 145.95
D2 = 12444.2875          # (shoulder to wrist distance)^2 at the elbow, from his code
BISECT2 = 21301.4025

DEFAULT_LIMITS = [[15, 180], [0, 131], [154, 21], [0, 180]]    # Sam's safe startup calibration (a reversed pair is an inverted servo)
AXIS_MIN_MM = 10.0                                              # closer to the base axis than this, the base angle is meaningless
PEAK_EASE = 1.875                                               # peak speed of a quintic ease, relative to the average


class PlanError(ValueError):
    """A refused move. Carries the same fields as the hub's structured errors: code, title, detail, hint."""
    def __init__(self, code, title, detail="", hint=""):
        super().__init__(title)
        self.code, self.title, self.detail, self.hint = code, title, detail, hint

    def as_dict(self):
        return {"code": self.code, "title": self.title, "detail": self.detail, "hint": self.hint}


def inverse_kim(x, y, z, a):
    """Joint angles (degrees) for a tool position (mm) and pitch a (radians). Sam's maths; raises PlanError where his raises a bare ValueError/ZeroDivisionError."""
    reach = math.hypot(x, y)
    if reach < AXIS_MIN_MM:
        raise PlanError("AXIS_SINGULARITY", "Too close to the base axis", "x %.0f, y %.0f is within %.0f mm of the base's vertical axis, where the base angle is undefined." % (x, y, AXIS_MIN_MM),
                        "Pick a point further out from the base.")
    wr, wz = reach - TOOL * math.cos(a), z - TOOL * math.sin(a) - SHOULDER_Z
    d = math.hypot(wr, wz)
    if d < 1e-6:
        raise PlanError("UNREACHABLE", "That point is not reachable", "The wrist would be on the shoulder.", "Pick a point further from the base.")
    mid = 0.5 * (d * d - D2) / d
    b2 = BISECT2 - mid * mid
    if b2 < 0:
        raise PlanError("UNREACHABLE", "That point is out of reach", "x %.0f, y %.0f, z %.0f is further than the arm can stretch." % (x, y, z), "Pick a point nearer the base.")
    s1 = math.atan2(x, y) + 0.5 * math.pi
    s2 = math.atan2(wz, wr) + math.atan2(math.sqrt(b2), mid)
    er, ez = UPPER * math.cos(s2), UPPER * math.sin(s2) + SHOULDER_Z
    s3 = math.atan2(-ez + wz + SHOULDER_Z, -er + wr) - s2 + math.pi
    s4 = -s2 - s3 + a + 1.5 * math.pi
    return [math.degrees(v) for v in (s1, s2, s3, s4)]


def _bounds(limits):
    return [(min(p), max(p)) for p in limits]


def _check_limits(q, bounds, where):
    for j, (lo, hi) in enumerate(bounds):
        if not lo - 0.5 <= q[j] <= hi + 0.5:
            raise PlanError("JOINT_LIMIT", "A joint would leave its calibrated range",
                            "Joint %d would need %.1f degrees at %s, but its range is %d to %d." % (j + 1, q[j], where, lo, hi),
                            "Pick a different point, or re-run the calibration if the limits are wrong.")


def _linear(t):
    return t


def _quintic(t):
    return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)


def plan(start, end, limits=None, speed_mm_s=30.0, max_joint_dps=30.0, rate_hz=50.0, max_step_deg=3.0, live=False, chunk_s=0.25):
    """start, end: (x, y, z, pitch_radians) in mm. Returns {"lines": [...], "duration": s, "peak_joint_dps": float, "steps": n}.
    No movement (start == end) returns lines == []. Raises PlanError for anything unsafe. Every number it uses is checked first.

    live=True is for a target that keeps moving (the dashboard following the controller): a packet cannot be changed once the board has it, so only the
    first chunk_s seconds of the way to the target are planned, at a constant speed (an ease in and out of every short packet would make the arm pulse).
    The board asks again from where it ended, so it always chases a target at most about chunk_s old. "end" in the result is where the packet stops."""
    for name, v in (("speed_mm_s", speed_mm_s), ("max_joint_dps", max_joint_dps), ("rate_hz", rate_hz), ("max_step_deg", max_step_deg)):
        if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
            raise PlanError("BAD_SETTING", "Bad planner setting", "%s must be a positive number." % name, "")
    for p in (start, end):
        if len(p) != 4 or not all(isinstance(c, (int, float)) and math.isfinite(c) for c in p):
            raise PlanError("BAD_POINT", "Bad point", "Each point needs numeric x, y, z and pitch.", "")
    bounds = _bounds(limits or DEFAULT_LIMITS)
    dist = math.dist(start[:3], end[:3])
    dpitch = abs(end[3] - start[3])
    if dist < 1e-9 and dpitch < 1e-9:
        return {"lines": [], "duration": 0.0, "peak_joint_dps": 0.0, "steps": 0}
    if live and dist > speed_mm_s * chunk_s:           # only the first chunk of the line: the rest is planned by the next packet
        f = speed_mm_s * chunk_s / dist
        end = tuple(a + (b - a) * f for a, b in zip(start, end))
        dist, dpitch = dist * f, dpitch * f
    ease, peak_ease = (_linear, 1.0) if live else (_quintic, PEAK_EASE)

    def at(s):
        return tuple(a + (b - a) * s for a, b in zip(start, end))

    # 0. say what is wrong with the end points first, so the error names the real problem rather than whatever the line meets on the way
    for label, pt in (("start", start), ("target", end)):
        _check_limits(inverse_kim(*pt), bounds, "the " + label)

    # 1. the whole path must be solvable and inside the limits: sample it densely in distance, not in time
    n_scan = max(40, int(math.ceil(dist / 2.0)))
    path = []
    for i in range(n_scan + 1):
        s = i / n_scan
        x, y, z, a = at(s)
        q = inverse_kim(x, y, z, a)
        if path:
            for j in range(4):                                  # no 360 degree flips between neighbours
                while q[j] - path[-1][j] > 180.0:
                    q[j] -= 360.0
                while q[j] - path[-1][j] < -180.0:
                    q[j] += 360.0
        _check_limits(q, bounds, "x %.0f, y %.0f, z %.0f" % (x, y, z))
        path.append(q)

    # 2. how fast does each joint move per unit of path? the worst place sets the minimum time
    worst = max(abs(q2[j] - q1[j]) * n_scan for q1, q2 in zip(path, path[1:]) for j in range(4))     # degrees per unit s, at the steepest scan interval
    t_cart = max(dist / speed_mm_s, 0.0)
    t_joint = peak_ease * worst / max_joint_dps
    t_pitch = peak_ease * math.degrees(dpitch) / max_joint_dps
    duration = max(t_cart, t_joint, t_pitch, 2.0 / rate_hz)

    # 3. sample at a fixed rate with a quintic ease, and make sure no step is a jump
    steps = max(2, int(math.ceil(duration * rate_hz)))
    lines, prev, peak = [], None, 0.0
    for i in range(steps + 1):
        tau = i / steps
        x, y, z, a = at(ease(tau))
        q = inverse_kim(x, y, z, a)
        if prev is not None:
            for j in range(4):
                while q[j] - prev[j] > 180.0:
                    q[j] -= 360.0
                while q[j] - prev[j] < -180.0:
                    q[j] += 360.0
            biggest = max(abs(q[j] - prev[j]) for j in range(4))
            if biggest > max_step_deg:
                raise PlanError("JOINT_JUMP", "A joint would jump",
                                "Joint %d would move %.1f degrees in one %.0f ms step." % (max(range(4), key=lambda j: abs(q[j] - prev[j])) + 1, biggest, 1000.0 / rate_hz),
                                "The path passes close to a singular position: pick a different route.")
            peak = max(peak, biggest * rate_hz)
        _check_limits(q, bounds, "step %d" % i)
        c = [min(180.0, max(0.0, v)) for v in q]            # the limits allow 0.5 degrees of slack; his firmware rejects anything outside 0-180
        lines.append("%d/%.3f/%.3f/%.3f/%.3f" % (round(tau * duration * 1000.0), c[0], c[1], c[2], c[3]))
        prev = q
    return {"lines": lines, "duration": duration, "peak_joint_dps": peak, "steps": steps, "end": end}
