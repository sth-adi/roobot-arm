"""What Sam's backend does for GET /UpdateMovement, with our planner behind it and a way to give the arm targets and sequences.

The firmware asks "I am at x, y, z, a: what now?". This answers with a checked movement packet (dashboard/planner.py), or 204 for "nothing to do".
It never moves anything while DISARMED, and a failed step ends the whole queue instead of carrying on with the next pose.
No HTTP in here, so it is easy to test; dashboard/sam_server.py puts Sam's endpoints on top.
"""
import math
import threading

try:
    import planner as PL
except ImportError:                                            # imported as dashboard.movement_service
    from dashboard import planner as PL

HOME_TARGET = (0.0, 250.0, 250.0, 0.0)          # Sam's constant in backend/main.py: x, y, z, pitch (radians)
ACTIONS = (None, "grab", "release")
MAX_QUEUE = 200


class MovementService:
    def __init__(self, fw_tool_mm=53.0, speed_mm_s=30.0, max_joint_dps=30.0):
        self.lock = threading.RLock()
        self.fw_tool_mm = fw_tool_mm              # the tool length the firmware uses when it reports its position: 53 in Sam's, 63 once patched
        self.speed_mm_s, self.max_joint_dps = speed_mm_s, max_joint_dps
        self.limits, self.homes = [list(p) for p in PL.DEFAULT_LIMITS], [92, 128, 28, 112]
        self.armed = False
        self.target = HOME_TARGET
        self.queue = []                            # poses still to go after the current target: dicts x, y, z, angle (radians), action
        self.pending_action = None                 # grab/release to do when the current target is reached
        self.reached = False                       # the current target has been counted as reached (so waiting there does not count again)
        self.magnet = False
        self.last_error = None
        self.sent = 0
        self.arrived = 0

    # ---- what the operator sets ---------------------------------------------------------------------------------------------------
    def set_armed(self, on):
        with self.lock:
            self.armed = bool(on)
            if not on:
                self.queue, self.pending_action = [], None

    def set_calibration(self, limits, homes):
        ok = (isinstance(limits, list) and len(limits) == 4 and all(isinstance(p, list) and len(p) == 2 and all(isinstance(v, int) and 0 <= v <= 180 for v in p) for p in limits)
              and isinstance(homes, list) and len(homes) == 4 and all(isinstance(v, int) and 0 <= v <= 180 for v in homes))
        if not ok:
            raise ValueError("Calibration must be four 0-180 limit pairs and four 0-180 homes")
        with self.lock:
            self.limits, self.homes = [list(p) for p in limits], list(homes)

    @staticmethod
    def _pose(p):
        if not isinstance(p, dict):
            raise ValueError("a pose is an object with x, y, z and angle")
        try:
            v = [float(p[k]) for k in ("x", "y", "z", "angle")]
        except (KeyError, TypeError, ValueError):
            raise ValueError("each pose needs numeric x, y, z and angle")
        if not all(math.isfinite(c) and abs(c) < 1000 for c in v):
            raise ValueError("pose out of range")
        act = p.get("action")
        if act not in ACTIONS:
            raise ValueError("action must be grab, release or null")
        return {"x": v[0], "y": v[1], "z": v[2], "angle": math.radians(v[3]), "action": act}      # angle comes in degrees, like the rest of the dashboard

    def set_target(self, pose):
        """One pose, replacing whatever was queued."""
        self.set_queue([pose])

    def set_queue(self, poses):
        if not isinstance(poses, list) or not poses or len(poses) > MAX_QUEUE:
            raise ValueError("a queue needs 1 to %d poses" % MAX_QUEUE)
        items = [self._pose(p) for p in poses]
        with self.lock:
            first = items[0]
            self.target = (first["x"], first["y"], first["z"], first["angle"])
            self.pending_action = first["action"]
            self.queue = items[1:]
            self.reached = False
            self.last_error = None

    # ---- what the firmware asks ---------------------------------------------------------------------------------------------------
    def update(self, x, y, z, a_deg):
        """(status, body). 200 with a packet; 204 nothing to do (disarmed, or at the end of the queue); 422 a refused move, with the reason in body."""
        with self.lock:
            if not self.armed:
                return 204, ""
            pos = PL.tool_tip(x, y, z, a_deg, self.fw_tool_mm)
            here = (pos[0], pos[1], pos[2], math.radians(a_deg))
            if PL.at_target(here, self.target):
                if not self.reached:
                    self._arrived()
                if PL.at_target(here, self.target):                 # nothing further queued: stay
                    return 204, ""
            try:
                out = PL.plan(here, self.target, self.limits, self.speed_mm_s, self.max_joint_dps)
            except PL.PlanError as e:
                self.queue, self.pending_action = [], None
                self.last_error = e.as_dict()
                return 422, e.as_dict()
            if not out["lines"]:
                return 204, ""
            self.sent += 1
            self.last_error = None
            return 200, "\n".join(out["lines"]) + "\n"

    def _arrived(self):
        self.arrived += 1
        self.reached = True
        if self.pending_action == "grab":
            self.magnet = True
        elif self.pending_action == "release":
            self.magnet = False
        self.pending_action = None
        if self.queue:
            nxt = self.queue.pop(0)
            self.target = (nxt["x"], nxt["y"], nxt["z"], nxt["angle"])
            self.pending_action = nxt["action"]
            self.reached = False

    def snapshot(self):
        with self.lock:
            return {"armed": self.armed, "target": {"x": self.target[0], "y": self.target[1], "z": self.target[2], "angle": math.degrees(self.target[3])},
                    "queued": len(self.queue), "magnet": self.magnet, "error": self.last_error, "packets_sent": self.sent, "poses_reached": self.arrived,
                    "calibration": {"jointLimits": self.limits, "jointHomes": self.homes}}
