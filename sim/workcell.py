"""The things the arm works with in the simulation: three boxes, each with a key, a tray, and the electromagnet at the tool tip.

Positions are millimetres in the arm's base frame (x right, y forward, z up), the same frame as ArmSim's tool position.
The electromagnet is at the tool tip. While it is on and nothing is held, a key within MAGNET_REACH_MM of the tip jumps to it and
travels with it. Switching it off lets go: the key falls onto whatever is below (the floor of a box, the tray, or the ground).
It is switched with set_magnet() or the X button of the controller packet (each press toggles it).

This is a simulation of the cell only: the stock firmware does not read X for anything, so a real magnet would need firmware support.
Nothing here collides with the arm itself.
"""
import math

BOX_CENTERS = [(-154.0, 184.0), (0.0, 240.0), (154.0, 184.0)]   # on an arc 240 mm out, reachable with the default calibration
BOX_INNER = 36.0        # half width of the inside of a box
BOX_WALL = 3.0
BOX_HEIGHT = 45.0
BOX_FLOOR = 4.0         # thickness of the floor
TRAY_CENTER = (222.0, 60.0)
TRAY_HALF = (60.0, 45.0)
TRAY_TOP = 6.0          # height of the tray surface
KEY_HALF_THICK = 1.5
MAGNET_REACH_MM = 24.0
GRAVITY = 4000.0        # mm/s^2, a little brisk so drops read well on screen


class Workcell:
    def __init__(self):
        self.reset()

    def reset(self):
        self.saved_magnet = []         # per saved position: was the magnet on when it was saved (kept in step with ArmSim.saved)
        self._pending = None           # replay: the saved position the arm is heading for, whose magnet state is not applied yet
        self.magnet = False
        self.held = None
        self.prev_x = False
        self.keys = []
        for i, (cx, cy) in enumerate(BOX_CENTERS):
            self.keys.append({"id": i + 1, "pos": [cx, cy, BOX_FLOOR + KEY_HALF_THICK], "vz": 0.0, "yaw": (-20.0, 10.0, 25.0)[i]})

    def reset_keys(self):
        """Keys back in their boxes and the magnet off; the magnet states saved with the positions are kept."""
        saved, last = list(self.saved_magnet), getattr(self, "last_applied", None)
        self.reset()
        self.saved_magnet = saved
        self.last_applied = last

    # ---- geometry ---------------------------------------------------------------------------------------------
    @staticmethod
    def surface_at(x, y):
        """(height of the surface under a point, name of where it is)."""
        for i, (cx, cy) in enumerate(BOX_CENTERS):
            if abs(x - cx) <= BOX_INNER and abs(y - cy) <= BOX_INNER:
                return BOX_FLOOR, "box %d" % (i + 1)
        if abs(x - TRAY_CENTER[0]) <= TRAY_HALF[0] and abs(y - TRAY_CENTER[1]) <= TRAY_HALF[1]:
            return TRAY_TOP, "tray"
        return 0.0, "floor"

    def where(self, k):
        if self.held == k["id"]:
            return "on the magnet"
        h, name = self.surface_at(k["pos"][0], k["pos"][1])
        return name if k["pos"][2] <= h + KEY_HALF_THICK + 0.5 else "falling"

    # ---- control ----------------------------------------------------------------------------------------------
    last_applied = 0

    def set_magnet(self, on):
        on = bool(on)
        if on == self.magnet:
            return
        self.magnet = on
        if not on and self.held is not None:
            for k in self.keys:
                if k["id"] == self.held:
                    k["vz"] = 0.0
            self.held = None

    def toggle(self):
        self.set_magnet(not self.magnet)

    def step(self, tip, x_pressed, dt):
        """tip: (x, y, z) of the tool tip. x_pressed: the X button of the latest packet (None/False when there is none)."""
        x_pressed = bool(x_pressed)
        if x_pressed and not self.prev_x:
            self.toggle()
        self.prev_x = x_pressed
        if self.magnet and self.held is None:
            best, bd = None, MAGNET_REACH_MM
            for k in self.keys:
                d = math.dist(tip, k["pos"])
                if d <= bd:
                    best, bd = k, d
            if best:
                self.held = best["id"]
        for k in self.keys:
            if k["id"] == self.held:
                k["pos"] = [tip[0], tip[1], tip[2]]
                k["vz"] = 0.0
                continue
            rest = self.surface_at(k["pos"][0], k["pos"][1])[0] + KEY_HALF_THICK
            if k["pos"][2] > rest + 1e-6 or k["vz"]:
                k["vz"] -= GRAVITY * dt
                k["pos"][2] += k["vz"] * dt
                if k["pos"][2] <= rest:
                    k["pos"][2], k["vz"] = rest, 0.0

    def track_saved(self, saved, replaying, replay_idx, tip):
        """Keep the magnet state saved with each position, and apply it during a replay when the arm gets there.
        saved: ArmSim.saved (x, y, z, angle). replay_idx: the next position the replay will aim at."""
        n = len(saved)
        while len(self.saved_magnet) < n:
            self.saved_magnet.append(self.magnet)          # a position saved just now remembers the magnet as it is
        del self.saved_magnet[n:]
        if not (replaying and n):
            self._pending, self._last_idx = None, None
            return
        idx = (replay_idx - 1) % n                           # the position the replay is moving to
        if self._pending is not None and self._pending != idx and self._pending < n:
            self.last_applied = self._pending
            self.set_magnet(self.saved_magnet[self._pending])    # it moved on before arriving: do not skip the magnet change
            self._pending = None
        if self._pending is None and idx != getattr(self, "_last_idx", None):
            self._pending = idx
            if idx == 0:                                      # a run (or the next lap) starts: keys back in the boxes, so every run can pick them up again
                self.reset_keys()
                self._pending = idx
                self.restarted = True
        self._last_idx = idx
        if self._pending is not None and math.dist(tip, saved[self._pending][:3]) < 6.0:
            self.last_applied = self._pending
            self.set_magnet(self.saved_magnet[self._pending])
            self._pending = None

    def snapshot(self):
        r1 = lambda v: round(v, 1)
        return {"magnet": self.magnet, "held": self.held, "saved_magnet": list(self.saved_magnet),
                "keys": [{"id": k["id"], "pos": [r1(c) for c in k["pos"]], "yaw": k["yaw"], "where": self.where(k)} for k in self.keys],
                "boxes": [list(c) for c in BOX_CENTERS], "box_inner": BOX_INNER, "box_wall": BOX_WALL, "box_height": BOX_HEIGHT, "box_floor": BOX_FLOOR,
                "tray": list(TRAY_CENTER), "tray_half": list(TRAY_HALF), "tray_top": TRAY_TOP}
