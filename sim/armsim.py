"""Virtual ESP32 + arm: a host-side model of the UWE-RArm firmware (Python standard library only).

It mirrors "Robot Arm Firmware/src" from Samwally1/UWE-RArm so the arm can be driven and watched with no
hardware:

  sticks -> SticksToXYZ -> inverse kinematics -> 4 joint angles -> servo commands

plus the firmware's calibration (limits and homes, inverted servos), B = go home, A = save a position,
hold A = replay the saved positions.

Two deliberate differences from the firmware, both described where they happen:
  * DAMPER. The joint speed limit (deg/s). The firmware writes every new angle straight to the servos. Here the
    joints move toward the commanded angles at no more than `rate` deg/s, with all four joints scaled together
    so the tool follows the commanded path rather than lagging behind it.
  * STATE. The pose always comes from the joint angles (forward kinematics). The firmware keeps a separate XYZ
    target that is not updated by "go home", so the first stick movement afterwards snaps the arm back to where it
    was before homing. Using the real angles as the single source of truth removes that.

Wire format (what the controller client posts, and what the firmware reads):
    {"LSX": -1..1, "LSY": -1..1, "RSX": -1..1, "RSY": -1..1, "A": bool, "B": bool, "X": bool, "Y": bool}
Sticks are raw pygame axes (stick up = negative); the firmware applies its own 0.2 deadzone.
"""
import math
import time
from collections import deque

HALF_PI = math.pi / 2

# ---- geometry (mm), from InverseKinTransform.h ----------------------------------------------------------------
SHOULDER_Z = 74.0        # shoulder axis height above the base
L_UPPER = 145.95         # shoulder to elbow
L_FORE = 183.70          # elbow to wrist
DEFAULT_TOOL_MM = 63.0   # wrist to tool tip (63 with the test ball fitted; the firmware file had 53)

# ---- control constants, from RobotCharacteristics.h / SticksToXYZ.h / main.cpp -----------------------------------
STICK_DEADZONE = 0.2
CHORD_SENSE = 20.0       # mm/s at full stick
ANGLE_SENSE = 20.0       # deg/s at full stick (tool angle)
MAX_SAVED = 20
REPLAY_HOLD_S = 0.75     # hold A this long to start replay
REPLAY_STEP_S = 1.5      # time between replayed positions
STALE_S = 1.5            # no input newer than this = treat as no input (fail-safe; the firmware has no such check)
UNLIMITED_DEG_S = 180.0  # speed used when the damper is set to 0 ("no limit")
HOME_SETTLE_S = 1.0      # after reaching home, signals stay on this long before the servos are released

DEFAULT_LIMITS = [[0, 180], [0, 180], [0, 180], [0, 180]]
DEFAULT_HOMES = [90, 90, 90, 90]


def _dz(v):
    return 0.0 if abs(v) < STICK_DEADZONE else v


# ---- kinematics ------------------------------------------------------------------------------------------------
def forward_kin(a, tool=DEFAULT_TOOL_MM):
    """Joint angles (radians s1..s4) -> (x, y, z, tool angle in degrees). Same maths as ForwardKinTransform."""
    s1, s2, s3, s4 = a
    base = s1 - HALF_PI
    fore = s2 + s3 - math.pi
    tool_a = s2 + s3 + s4 - 3.0 * HALF_PI
    elbow_r = L_UPPER * math.cos(s2)
    elbow_z = L_UPPER * math.sin(s2) + SHOULDER_Z
    wrist_r = elbow_r + L_FORE * math.cos(fore)
    wrist_z = elbow_z + L_FORE * math.sin(fore)
    tool_r = wrist_r + tool * math.cos(tool_a)
    return (tool_r * math.sin(base), tool_r * math.cos(base),
            wrist_z + tool * math.sin(tool_a), math.degrees(tool_a))


def joint_points(a, tool=DEFAULT_TOOL_MM):
    """3D points for drawing: base, shoulder, elbow, wrist, tool tip."""
    s1, s2, s3, s4 = a
    base = s1 - HALF_PI
    fore = s2 + s3 - math.pi
    tool_a = s2 + s3 + s4 - 3.0 * HALF_PI
    sb, cb = math.sin(base), math.cos(base)
    pts = [(0.0, 0.0, 0.0), (0.0, 0.0, SHOULDER_Z)]
    r, z = L_UPPER * math.cos(s2), L_UPPER * math.sin(s2) + SHOULDER_Z
    pts.append((r * sb, r * cb, z))
    r, z = r + L_FORE * math.cos(fore), z + L_FORE * math.sin(fore)
    pts.append((r * sb, r * cb, z))
    r, z = r + tool * math.cos(tool_a), z + tool * math.sin(tool_a)
    pts.append((r * sb, r * cb, z))
    return pts


def _within_limits(a, limits):
    for i in range(4):
        d = math.degrees(a[i])
        lo, hi = min(limits[i]), max(limits[i])
        if d < lo or d > hi:
            return False
    return True


def _elbow_branch(target, theta, branch, limits, tool):
    x, y, z = target
    reach = math.hypot(x, y)
    wrist_r = reach - tool * math.cos(theta)
    wrist_z = z - tool * math.sin(theta)
    d = math.hypot(wrist_r, wrist_z - SHOULDER_Z)
    if not math.isfinite(d) or d < 1e-4:
        return None
    mid = 0.5 * (d * d - (L_FORE ** 2 - L_UPPER ** 2)) / d
    bis_sq = L_UPPER ** 2 - mid * mid
    if bis_sq < 0.0:
        return None
    bis = math.sqrt(bis_sq)
    mid_axis = math.atan2(wrist_z - SHOULDER_Z, wrist_r)
    s1 = math.atan2(x, y) + HALF_PI
    s2 = mid_axis + math.atan2(bis * branch, mid)
    elbow_r = L_UPPER * math.cos(s2)
    elbow_z = L_UPPER * math.sin(s2) + SHOULDER_Z
    fore = math.atan2(wrist_z - elbow_z, wrist_r - elbow_r)
    s3 = fore - s2 + math.pi
    s4 = -s2 - s3 + theta + 3.0 * HALF_PI
    out = (s1, s2, s3, s4)
    if not all(math.isfinite(v) for v in out) or not _within_limits(out, limits):
        return None
    return out


def inverse_kin(target, prev, limits, tool=DEFAULT_TOOL_MM):
    """(x, y, z, tool angle deg) -> joint angles in radians, or None if unreachable or outside the limits.
    Tries both elbow configurations and takes the one closer to `prev`, like InverseKinTransform."""
    theta = math.radians(target[3])
    pos = _elbow_branch(target[:3], theta, 1, limits, tool)
    neg = _elbow_branch(target[:3], theta, -1, limits, tool)
    if pos is None and neg is None:
        return None
    if pos is None:
        return neg
    if neg is None:
        return pos

    def dist(c):
        s = 0.0
        for i in range(4):
            dd = c[i] - prev[i]
            s += math.atan2(math.sin(dd), math.cos(dd)) ** 2
        return s
    return pos if dist(pos) <= dist(neg) else neg


# ---- the virtual arm -------------------------------------------------------------------------------------------
def _clean_pairs(limits, homes):
    """Same rules as the server's /calibration: four [min, max] pairs and four homes, integers 0..180."""
    ok = (isinstance(limits, list) and len(limits) == 4 and isinstance(homes, list) and len(homes) == 4)
    ok = ok and all(isinstance(p, list) and len(p) == 2 and all(isinstance(v, int) and 0 <= v <= 180 for v in p)
                    for p in limits)
    ok = ok and all(isinstance(v, int) and 0 <= v <= 180 for v in homes)
    if not ok:
        raise ValueError("calibration needs four [min, max] limits and four homes, whole degrees 0-180")
    return [list(p) for p in limits], list(homes)


class ArmSim:
    def __init__(self, tool_mm=DEFAULT_TOOL_MM, rate=30.0, limits=None, homes=None):
        self.tool = float(tool_mm)
        self.rate = float(rate)                       # damper, deg/s per joint; 0 = no limit
        self.limits, self.homes = _clean_pairs(limits or DEFAULT_LIMITS, homes or DEFAULT_HOMES)
        self.angles = self._home_angles()
        self.mode = "idle"                            # idle | manual | home | replay
        self.attached = True
        self.ik_ok = True                             # False while the sticks ask for somewhere unreachable
        self.saved = []                               # saved (x, y, z, tool angle) positions
        self.replay_idx = 0
        self._goal = None                             # joint-space goal for home / replay moves
        self._prev = {"A": False, "B": False}
        self._a_down_t = 0.0
        self._replay_t = 0.0
        self._home_reached_t = None
        self._clock = 0.0
        self.status = "Starting"
        self.log = deque(maxlen=200)
        self.moved = 0                                # incremented on every servo-relevant change
        self.say("Home XYZ %.1f, %.1f, %.1f" % self.pose()[:3])

    # ---- helpers --------------------------------------------------------------------------------------------
    def say(self, msg):
        self.status = msg
        self.log.append((round(self._clock, 2), msg))

    def _home_angles(self):
        return tuple(math.radians(h) for h in self.homes)

    def pose(self):
        return forward_kin(self.angles, self.tool)

    def _limit_deg_s(self):
        return self.rate if self.rate > 0 else UNLIMITED_DEG_S

    def _slew(self, goal, dt):
        """Move all four joints toward `goal`, no joint faster than the damper. Returns True when arrived.
        Joints are scaled by one common factor so the move stays a straight line in joint space."""
        step = math.radians(self._limit_deg_s()) * dt
        d = [goal[i] - self.angles[i] for i in range(4)]
        biggest = max(abs(v) for v in d)
        if biggest <= step or biggest < 1e-9:
            self.angles = tuple(goal)
            return True
        k = step / biggest
        self.angles = tuple(self.angles[i] + d[i] * k for i in range(4))
        return False

    # ---- configuration ----------------------------------------------------------------------------------------
    def set_calibration(self, limits, homes):
        """Replace limits and homes (validated), clear saved positions and go to the new home. Raises ValueError."""
        self.limits, self.homes = _clean_pairs(limits, homes)
        self.angles = self._home_angles()
        self.saved.clear()
        self.mode, self._goal = "idle", None
        self.attached = True
        self.say("Calibration set: homes " + ", ".join(str(h) for h in self.homes))

    def set_rate(self, deg_s):
        v = float(deg_s)
        if not (0 <= v <= 1000):
            raise ValueError("speed must be 0-1000 deg/s")
        self.rate = v

    def set_tool(self, mm):
        v = float(mm)
        if not (0 <= v <= 300):
            raise ValueError("tool length must be 0-300 mm")
        self.tool = v

    def go_home(self):
        self._start_home()

    def clear_saved(self):
        self.saved.clear()
        self.say("Saved positions cleared")

    # ---- servo output (what the ESP32 would write) -----------------------------------------------------------
    def servo_commands(self):
        """Per joint: the integer 0-180 degree command written to the servo, with inverted servos mapped like
        ServoCommandAngle (a reversed calibration range marks an inverted servo)."""
        out = []
        for i in range(4):
            lo, hi = min(self.limits[i]), max(self.limits[i])
            logical = max(lo, min(hi, int(round(math.degrees(self.angles[i])))))
            logical = max(0, min(180, logical))
            out.append(180 - logical if self.limits[i][0] > self.limits[i][1] else logical)
        return out

    # ---- one control tick ---------------------------------------------------------------------------------------
    def _start_home(self):
        self._goal = self._home_angles()
        self.mode = "home"
        self._home_reached_t = None
        self.attached = True
        self.say("Moving all joints home")

    def step(self, inp, dt):
        """Advance by `dt` seconds. `inp` is a wire-format dict, or None when there is no fresh input."""
        dt = max(0.0, min(dt, 0.1))
        self._clock += dt
        now = self._clock
        a = inp if isinstance(inp, dict) else {}
        sx, sy = _dz(float(a.get("LSX", 0))), _dz(float(a.get("LSY", 0)))
        rx, ry = _dz(float(a.get("RSX", 0))), _dz(float(a.get("RSY", 0)))
        btn_a, btn_b = bool(a.get("A")), bool(a.get("B"))

        # B pressed: go home
        if btn_b and not self._prev["B"]:
            self.saved_replay_stop()
            self._start_home()
        # A pressed: save the current position
        if btn_a and not self._prev["A"]:
            self._a_down_t = now
            if len(self.saved) < MAX_SAVED:
                self.saved.append(self.pose())
                self.say("Position %d saved" % len(self.saved))
            else:
                self.say("Saved-position list is full")
        # A held: start replay; A released: stop it
        if btn_a and self.mode != "replay" and self.mode != "home" and self.saved \
                and now - self._a_down_t >= REPLAY_HOLD_S:
            self.mode, self.replay_idx = "replay", 0
            self._replay_t = now - REPLAY_STEP_S
            self.say("Replaying saved positions")
        if not btn_a and self.mode == "replay":
            self.mode, self._goal = "idle", None
            self.say("Position replay stopped")
        self._prev["A"], self._prev["B"] = btn_a, btn_b

        # replay: aim at the next saved position every REPLAY_STEP_S
        if self.mode == "replay" and now - self._replay_t >= REPLAY_STEP_S:
            goal = inverse_kin(self.saved[self.replay_idx], self.angles, self.limits, self.tool)
            if goal is not None:
                self._goal = goal
            self.replay_idx = (self.replay_idx + 1) % len(self.saved)
            self._replay_t = now

        # joint-space moves (home / replay), damped
        if self._goal is not None:
            before = self.angles
            arrived = self._slew(self._goal, dt)
            if self.angles != before:
                self.attached = True
                self.moved += 1
            if arrived:
                self._goal = None
                if self.mode == "home":
                    self._home_reached_t = now
        if self.mode == "home" and self._goal is None:
            if self._home_reached_t is not None and now - self._home_reached_t >= HOME_SETTLE_S:
                self.mode = "idle"
                self.attached = False
                self.say("Servo signals disabled")
            return

        # sticks: Cartesian jog through inverse kinematics, damped
        if self.mode in ("idle", "manual") and (sx or sy or rx or ry):
            x, y, z, th = self.pose()
            req = (x + sx * CHORD_SENSE * dt, y + sy * CHORD_SENSE * dt,
                   z + ry * CHORD_SENSE * dt, th + rx * ANGLE_SENSE * dt)
            goal = inverse_kin(req, self.angles, self.limits, self.tool)
            self.ik_ok = goal is not None
            self.mode = "manual"
            if goal is not None:
                before = self.angles
                self._slew(goal, dt)
                if self.angles != before:
                    self.attached = True
                    self.moved += 1
        elif self.mode == "manual":
            self.mode = "idle"
            self.ik_ok = True

    def saved_replay_stop(self):
        if self.mode == "replay":
            self.say("Position replay stopped")
        self.mode = "idle"

    # ---- what the dashboard shows ---------------------------------------------------------------------------------
    def snapshot(self):
        x, y, z, th = self.pose()
        r1 = lambda v: round(v, 1)
        return {
            "mode": self.mode, "attached": self.attached, "ik_ok": self.ik_ok, "status": self.status,
            "angles": [r1(math.degrees(v)) for v in self.angles],
            "servo": self.servo_commands(),
            "xyz": [r1(x), r1(y), r1(z)], "tool_angle": r1(th),
            "points": [[r1(c) for c in p] for p in joint_points(self.angles, self.tool)],
            "saved": len(self.saved), "replay_idx": self.replay_idx,
            "rate": self.rate, "tool": self.tool,
            "limits": self.limits, "homes": self.homes,
        }


def wire_from_internal(s):
    """Dashboard/GUI state (lx, ly, rx, ry, buttons{...}) -> the wire format above."""
    b = s.get("buttons") or {}
    return {"LSX": s.get("lx", 0.0), "LSY": s.get("ly", 0.0), "RSX": s.get("rx", 0.0), "RSY": s.get("ry", 0.0),
            "A": bool(b.get("a")), "B": bool(b.get("b")), "X": bool(b.get("x")), "Y": bool(b.get("y"))}


def internal_from_wire(w):
    """Wire format -> the dashboard's richer state. Triggers, bumpers and the d-pad are not on the wire, so they
    show as idle."""
    names = ("a", "b", "x", "y", "lb", "rb", "back", "start", "ls", "rs")
    wb = {"a": "A", "b": "B", "x": "X", "y": "Y"}
    return {"lx": w.get("LSX", 0.0), "ly": w.get("LSY", 0.0), "rx": w.get("RSX", 0.0), "ry": w.get("RSY", 0.0),
            "lt": 0.0, "rt": 0.0, "buttons": {k: bool(w.get(wb[k])) if k in wb else False for k in names},
            "dpad": [0, 0]}
