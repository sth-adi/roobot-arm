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

Three behaviours can be simulated (ArmSim(model=..., patch=...)):
  * "improved"        the reference above (default).
  * "stock"           the UWE-RArm firmware exactly as it is, quirks included: a separate XYZ target that "go home" does not
                      update (so the next stick move snaps the arm back), no speed limit, the whole elapsed time since the
                      last packet integrated in one go, servos released 3 s after "go home" whether or not they arrived.
  * "stock" + patch   the stock firmware plus the small patch in firmware/stock/patches: speed limit on the servo output,
                      elapsed time capped at 100 ms, stick input paused while the arm lags the target by more than
                      LAG_HOLD_DEG, "go home" waits for arrival, re-syncs the target and releases 1 s later.

Wire format (what the controller client posts, and what the firmware reads):
    {"LSX": -1..1, "LSY": -1..1, "RSX": -1..1, "RSY": -1..1, "A": bool, "B": bool, "X": bool, "Y": bool}
Sticks are raw pygame axes (stick up = negative); the firmware applies its own 0.2 deadzone.
"""
import math
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
STOCK_HOME_MS = 3000     # stock firmware: servos released this long after "go home", arrived or not
PATCH_MAX_STEP_MS = 100  # patch: longest stretch of stick input integrated in one go
LAG_HOLD_DEG = 25.0      # patch: stick input pauses while any joint is this far behind its target

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


def _lround(x):
    """C's lroundf: halves go away from zero."""
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def _within_limits(a, limits):
    for i in range(4):
        # Servo commands are whole degrees, so the check is on whole degrees too (UWE-RArm commit 98416af..314e4b4): comparing the raw
        # float rejected an exactly trained limit, e.g. 115.00001 against a limit of 115.
        d = _lround(math.degrees(a[i]))
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


def quintic(p):
    """Smootherstep, 10p^3 - 15p^4 + 6p^5: zero speed and zero acceleration at both ends (peak slope 1.875)."""
    return p * p * p * (10.0 + p * (-15.0 + 6.0 * p))


class LinearMove:
    """A straight line in x, y, z and tool angle from `start` to `end`, eased with a quintic, as UWE-RArm's CommandJointAnglesLinear does it.
    `speed_mm_s` is the PEAK tool speed: the duration is stretched by the easing's peak slope (1.875) so the peak is what was asked for.
    A move that only turns the tool takes at least 0.5 s and turns at most 30 deg/s at the peak. The firmware blocks while it runs; this
    is stepped from the arm's own clock instead, so everything else (damper, sticks, failsafes) keeps running."""

    def __init__(self, start, end, speed_mm_s, t0_ms):
        self.start, self.end, self.t0 = tuple(start), tuple(end), t0_ms
        dist = math.dist(self.start[:3], self.end[:3])
        dang = abs(self.end[3] - self.start[3])
        self.duration_ms = max(500.0, 1.875 * dist * 1000.0 / speed_mm_s, 1.875 * dang * 1000.0 / 30.0)

    def at(self, now_ms):
        """(pose, finished) at this time."""
        p = min(1.0, max(0.0, (now_ms - self.t0) / self.duration_ms))
        e = quintic(p)
        return tuple(s + (t - s) * e for s, t in zip(self.start, self.end)), p >= 1.0


class ArmSim:
    move_mm_s = 0.0      # peak speed of straight-line moves between sequence poses (mm/s); 0 = the firmware as shipped (every joint straight to the pose)

    def __init__(self, tool_mm=DEFAULT_TOOL_MM, rate=30.0, limits=None, homes=None, model="improved", patch=False):
        if model not in ("improved", "stock"):
            raise ValueError("model must be 'improved' or 'stock'")
        self.model, self.patch = model, bool(patch) and model == "stock"
        self.tool = float(tool_mm)
        self.rate = float(rate)                       # damper, deg/s per joint; 0 = no limit
        self.chord_sense = CHORD_SENSE                # stick speed: tool tip mm/s at full stick (the firmware's ChordSense, 20)
        self.angle_sense = ANGLE_SENSE                # and tool angle deg/s at full stick (AngleSense, 20)
        self.limits, self.homes = _clean_pairs(limits or DEFAULT_LIMITS, homes or DEFAULT_HOMES)
        self.angles = self._home_angles()
        self.mode = "idle"                            # idle | manual | home | replay
        self.attached = True
        self.ik_ok = True                             # False while the sticks ask for somewhere unreachable
        self.saved = []                               # saved (x, y, z, tool angle) positions
        self.saved_speed = []                         # optional per saved position: joint speed limit (deg/s) for the replay move into it
        self._seg_rate = None                         # the speed limit of the replay move in progress, if its position has one
        self.replay_idx = 0
        self._goal = None                             # joint-space goal for home / replay moves
        self._prev = {"A": False, "B": False}
        self._a_down_t = 0.0
        self.latched = False                          # replay started with start_replay(): keeps going without A held
        self._a_saved = False                         # the save made when A went down (undone if A is held to replay)
        self._replay_t = 0.0
        self._home_reached_t = None
        self._clock = 0.0
        self.status = "Starting"
        self.log = deque(maxlen=200)
        self.moved = 0                                # incremented on every servo-relevant change
        self._init_stock()
        self.say("Home XYZ %.1f, %.1f, %.1f" % self.pose()[:3])

    def _init_stock(self):
        """State of the stock firmware model: its own XYZ target and target angles, separate from what the servos hold."""
        self._ms = 0                                  # integer milliseconds, like millis()
        self._t_ang = self._home_angles()             # targetAngles
        self._t_pos = forward_kin(self._t_ang, self.tool)   # targetPosition
        self._moving_home = False
        self._home_start_ms = 0
        self._home_arrived_ms = None
        self._replaying = False
        self._lin = None                              # the straight-line move to a sequence pose that is under way, if any
        self._a_press_ms = 0
        self._replay_ms = 0
        self._last_update_ms = None
        self._sticks_active = False
        self._last_cmd = [int(round(math.degrees(v))) for v in self._home_angles()]   # patch: last whole-degree command sent

    def set_model(self, model, patch=False):
        """Switch between the reference, the stock firmware and the patched stock firmware. Resets the arm to home."""
        if model not in ("improved", "stock"):
            raise ValueError("model must be 'improved' or 'stock'")
        self.model, self.patch = model, bool(patch) and model == "stock"
        self.angles = self._home_angles()
        self.mode, self._goal, self.attached, self.ik_ok = "idle", None, True, True
        self._prev = {"A": False, "B": False}
        self._init_stock()
        self.say("Model: " + self.model_name())

    def model_name(self):
        return "improved" if self.model == "improved" else ("stock + patch" if self.patch else "stock")

    # ---- helpers --------------------------------------------------------------------------------------------
    @property
    def replaying(self):
        return self._replaying if self.model == "stock" else self.mode == "replay"

    def say(self, msg):
        self.status = msg
        self.log.append((round(self._clock, 2), msg))

    def _home_angles(self):
        return tuple(math.radians(h) for h in self.homes)

    def pose(self):
        return forward_kin(self.angles, self.tool)

    def _limit_deg_s(self):
        if self._seg_rate and self.replaying and self.mode != "home" and not getattr(self, "_moving_home", False):
            return self._seg_rate
        return self.rate if self.rate > 0 else UNLIMITED_DEG_S

    def _pick_seg_rate(self, idx):
        self._seg_rate = self.saved_speed[idx] if 0 <= idx < len(self.saved_speed) else None

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
        self._init_stock()
        self.say("Calibration set: homes " + ", ".join(str(h) for h in self.homes))

    def set_rate(self, deg_s):
        v = float(deg_s)
        if not (0 <= v <= 1000):
            raise ValueError("speed must be 0-1000 deg/s")
        self.rate = v

    def set_stick_speed(self, mm_s):
        """How fast a full stick moves the tool tip (mm/s); the tool angle turns at the same number in deg/s. The firmware's value is 20.
        This is separate from the damper, which only caps joint speed: with the stick speed at 20 the damper never gets a chance to matter."""
        v = float(mm_s)
        if not (5.0 <= v <= 200.0):
            raise ValueError("stick speed must be 5-200 mm/s")
        self.chord_sense = self.angle_sense = v

    def set_tool(self, mm):
        v = float(mm)
        if not (0 <= v <= 300):
            raise ValueError("tool length must be 0-300 mm")
        self.tool = v

    def go_home(self):
        if self.model == "stock":
            self._stock_start_home()
        else:
            self._start_home()

    def save_here(self):
        """Add the arm's current position to the saved list (what A does). False when the list is full."""
        if len(self.saved) >= MAX_SAVED:
            return False
        self.saved.append(self._t_pos if self.model == "stock" else self.pose())
        self.say("Position %d saved" % len(self.saved))
        return True

    def clear_saved(self):
        self.saved.clear()
        self.saved_speed.clear()
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

    def servo_degrees(self):
        """servo_commands() without the rounding to whole degrees: what is streamed to a board that mirrors this arm."""
        out = []
        for i in range(4):
            lo, hi = min(self.limits[i]), max(self.limits[i])
            logical = max(0.0, min(180.0, max(lo, min(hi, math.degrees(self.angles[i])))))
            out.append(round(180.0 - logical if self.limits[i][0] > self.limits[i][1] else logical, 2))
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
        if self.model == "stock":
            return self._step_stock(inp, dt)
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
            self._a_saved = len(self.saved) < MAX_SAVED
            if len(self.saved) < MAX_SAVED:
                self.saved.append(self.pose())
                self.say("Position %d saved" % len(self.saved))
            else:
                self.say("Saved-position list is full")
        # A held: start replay; A released: stop it
        if btn_a and self.mode != "replay" and self.mode != "home" and len(self.saved) - self._a_saved > 0 \
                and now - self._a_down_t >= REPLAY_HOLD_S:
            if self._a_saved:                         # holding A means replay, not save: drop the position the press added
                self.saved.pop()
                self._a_saved = False
            self.mode, self.replay_idx = "replay", 0
            self._replay_t = now - REPLAY_STEP_S
            self.say("Replaying saved positions")
        if not btn_a and self.mode == "replay" and not self.latched:
            self.mode, self._goal = "idle", None
            self.say("Position replay stopped")
        self._prev["A"], self._prev["B"] = btn_a, btn_b

        # replay: aim at the next saved position every REPLAY_STEP_S
        if self.mode == "replay" and now - self._replay_t >= REPLAY_STEP_S:
            goal = inverse_kin(self.saved[self.replay_idx], self.angles, self.limits, self.tool)
            if goal is not None:
                self._goal = goal
                self._pick_seg_rate(self.replay_idx)
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
            req = (x + sx * self.chord_sense * dt, y + sy * self.chord_sense * dt,
                   z + ry * self.chord_sense * dt, th + rx * self.angle_sense * dt)
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


    # ---- the stock firmware (main.cpp loop) -------------------------------------------------------------------------
    def _stock_write(self):
        """CommandJointAngles(targetAngles): the servos are told the target at once."""
        self.angles = self._t_ang
        self.attached = True
        self.moved += 1

    def _whole_degrees(self):
        return [int(round(math.degrees(v))) for v in self.angles]

    def _stock_start_home(self):
        self._lin = None
        self.attached = True
        self._moving_home = True
        self._home_start_ms = self._ms
        self._home_arrived_ms = None
        if not self.patch:
            self.angles = self._home_angles()     # stock writes the home angles straight to the servos
        else:
            self._last_cmd = self._whole_degrees()      # patch: attach and hold where it is
        self.moved += 1
        self.say("Moving all joints home")

    def _step_stock(self, inp, dt):
        dt = max(0.0, dt)
        self._ms += int(round(dt * 1000))
        now = self._ms
        if isinstance(inp, dict):                 # GetControllerState() returned true
            a = inp
            btn_a, btn_b = bool(a.get("A")), bool(a.get("B"))
            if btn_b and not self._prev["B"]:
                self.latched = False
                self._stock_start_home()
            self._prev["B"] = btn_b
            if btn_a and not self._prev["A"]:
                self._a_press_ms = now
                self._a_saved = len(self.saved) < MAX_SAVED
                if len(self.saved) < MAX_SAVED:
                    self.saved.append(self._t_pos)            # stock saves the target, not where the arm is
                    self.say("Position %d saved" % len(self.saved))
                else:
                    self.say("Saved-position list is full")
            n_saved = len(self.saved) - (1 if (self.patch and self._a_saved) else 0)
            if btn_a and not self._replaying and now - self._a_press_ms >= 750 and n_saved > 0:
                if self.patch and self._a_saved:      # patch: holding A means replay, not save
                    self.saved.pop()
                    self._a_saved = False
                self._replaying, self.replay_idx = True, 0
                self._replay_ms = now - 1500
                self.say("Replaying saved positions")
            if not btn_a and self._replaying and not self.latched:
                self._replaying = False
                self.say("Position replay stopped")
            self._prev["A"] = btn_a
            step_due = now - self._replay_ms >= 1500 and self._lin is None
            if self._replaying and self.wait_arrival:      # sequence runs: move on once the arm has got there, not on a fixed clock
                lag_deg = max(abs(math.degrees(self._t_ang[i] - self.angles[i])) for i in range(4))
                step_due = now - self._replay_ms >= 300 and (lag_deg < 0.6 or now - self._replay_ms >= 12000) and self._lin is None
            if self._replaying and step_due and not (self.once and self.replay_idx >= len(self.saved)):
                target = self.saved[self.replay_idx]
                goal = inverse_kin(target, self._t_ang, self.limits, self.tool)
                if goal is not None:
                    self._pick_seg_rate(self.replay_idx)
                    if self.move_mm_s > 0:
                        self._lin = LinearMove(self._t_pos, target, self.move_mm_s, now)       # a smooth straight line to the pose
                    else:
                        self._t_pos, self._t_ang = target, goal
                        if not self.patch:
                            self._stock_write()
                self.replay_idx = self.replay_idx + 1 if self.once else (self.replay_idx + 1) % len(self.saved)
                self._replay_ms = now
            self._sticks_active = False
            if self._last_update_ms is not None and not self._moving_home and not self._replaying:
                elapsed = now - self._last_update_ms
                if self.patch:
                    elapsed = min(elapsed, PATCH_MAX_STEP_MS)
                lag = max(abs(math.degrees(self._t_ang[i] - self.angles[i])) for i in range(4))
                if not (self.patch and lag > LAG_HOLD_DEG):
                    sx, sy = _dz(float(a.get("LSX", 0))), _dz(float(a.get("LSY", 0)))
                    rx, ry = _dz(float(a.get("RSX", 0))), _dz(float(a.get("RSY", 0)))
                    e = elapsed / 1000.0
                    x, y, z, th = self._t_pos
                    req = (x + sx * self.chord_sense * e, y + sy * self.chord_sense * e, z + ry * self.chord_sense * e,
                           th + rx * self.angle_sense * e)
                    self._sticks_active = bool(sx or sy or rx or ry)
                    goal = inverse_kin(req, self._t_ang, self.limits, self.tool)
                    self.ik_ok = goal is not None
                    if goal is not None:
                        self._t_pos, self._t_ang = req, goal
                        if not self.patch:
                            self._stock_write()
            self._last_update_ms = now

        if self._lin is not None:
            self._advance_linear(now)
        if self.patch:                            # damper on the servo output; every loop, whether or not a packet came
            goal = self._home_angles() if self._moving_home else self._t_ang
            arrived = self._slew(goal, max(0.0, min(dt, 0.1)))
            cmd = self._whole_degrees()
            if cmd != self._last_cmd:             # only a changed whole-degree command reaches (and re-powers) the servos
                self._last_cmd = cmd
                self.attached = True
                self.moved += 1
            if self._moving_home and arrived:
                if self._home_arrived_ms is None:
                    self._home_arrived_ms = now
                    self._t_ang = self._home_angles()                 # re-sync, so the next stick move starts here
                    self._t_pos = forward_kin(self._t_ang, self.tool)
                elif now - self._home_arrived_ms >= 1000:
                    self._moving_home, self._home_arrived_ms = False, None
                    self.attached = False
                    self.say("Servo signals disabled")
        elif self._moving_home and now - self._home_start_ms >= STOCK_HOME_MS:
            self._moving_home = False
            self.attached = False
            self.say("Servo signals disabled")
        self.mode = "home" if self._moving_home else "replay" if self._replaying else \
            "manual" if self._sticks_active else "idle"

    def _advance_linear(self, now):
        """One tick of the straight-line move: the eased point on the line goes through the inverse kinematics and becomes the new target
        angles, which the damper then follows. Where the firmware gives up on a line that leaves the workspace, the arm here goes straight
        to the pose instead, so a sequence never stalls."""
        mv = self._lin
        pos, done = mv.at(now)
        goal = inverse_kin(pos, self._t_ang, self.limits, self.tool)
        if goal is None:
            self._lin = None
            end_goal = inverse_kin(mv.end, self._t_ang, self.limits, self.tool)
            self.say("Straight line leaves the workspace: going straight to the pose")
            if end_goal is not None:
                self._t_pos, self._t_ang = mv.end, end_goal
        else:
            self._t_pos, self._t_ang = pos, goal
            if done:
                self._lin = None
        if not self.patch:
            self._stock_write()

    def set_move_speed(self, mm_s):
        """Peak tool speed of straight-line moves between sequence poses, mm/s; 0 turns it off (every joint straight to the pose)."""
        v = float(mm_s)
        if v != 0.0 and not (10.0 <= v <= 400.0):
            raise ValueError("move speed must be 0 (off) or 10-400 mm/s")
        self.move_mm_s = v

    wait_arrival = False
    once = False                       # a one-off run: after the last position is sent the arm stays there instead of starting another lap

    def start_replay(self, wait_arrival=False, once=False):
        """Play the saved positions in a loop until stop_replay(): one press instead of holding A (the simulation only)."""
        if not self.saved:
            self.say("Nothing saved to replay")
            return False
        self.latched = True
        self.wait_arrival = bool(wait_arrival) and self.model == "stock"
        self.once = bool(once) and self.model == "stock"
        if self.model == "stock":
            if not self._replaying:
                self._replaying, self.replay_idx, self._replay_ms = True, 0, self._ms - 1500
        elif self.mode != "replay":
            self.mode, self.replay_idx, self._replay_t = "replay", 0, self._clock - REPLAY_STEP_S
        self.say("Replaying saved positions")
        return True

    def stop_replay(self):
        self._lin = None
        self._seg_rate = None
        self.latched = False
        self.wait_arrival = False
        self.once = False
        if self.model == "stock":
            if self._replaying:
                self._replaying = False
                self.say("Position replay stopped")
        else:
            self.saved_replay_stop()
            self._goal = None

    def toggle_replay(self):
        return self.stop_replay() if self.replaying else self.start_replay()

    def saved_replay_stop(self):
        self.latched = False
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
            "saved": len(self.saved), "replay_idx": self.replay_idx, "replaying": self.replaying, "latched": self.latched,
            "rate": self.rate, "stick_speed": self.chord_sense, "move_speed": self.move_mm_s, "tool": self.tool, "model": self.model, "patch": self.patch,
            "target": [r1(v) for v in self._t_pos[:3]] if self.model == "stock" else None,
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
