"""A virtual arm for trying the front door with no hardware: the same run()/stop() as lowlevel's senders, but every step is planned by the real planner
(so an unreachable step fails exactly as it would on the arm) and the arm "arrives" at once. Also a demo layout of boxes for it.
Used by server.py (--ai virtual, the default), sim/dry_run.py and sim/front_demo.py. It never talks to anything."""
import math
import os
import tempfile

try:
    import backend_client as BC
    import boxes as B
    import planner as PL
except ImportError:                                    # imported as dashboard.virtual
    from dashboard import backend_client as BC, boxes as B, planner as PL


class VirtualArm:
    def __init__(self, stall_label=None):
        self.pose, self.t, self.stall, self.stopped = {"x": 0.0, "y": 250.0, "z": 250.0, "angle": 0.0}, 0.0, stall_label, False

    def get_pose(self):
        return dict(self.pose)

    def run(self, steps, job="", on_step=None):
        self.stopped = False
        for i, s in enumerate(steps):
            if self.stopped:
                raise BC.BackendError("STOPPED", "Stopped")
            if s["label"] == self.stall:
                self.stall = None
                raise BC.BackendError("ARM_STALLED", "A joint stalled", "scripted for the demo")
            a = (self.pose["x"], self.pose["y"], self.pose["z"], math.radians(self.pose["angle"]))
            b = (s["x"], s["y"], s["z"], math.radians(s["angle"]))
            try:
                self.t += PL.plan(a, b, speed_mm_s=s["speed"])["duration"]
            except PL.PlanError as e:
                raise BC.BackendError(e.code, e.title, e.detail)
            self.pose = {k: s[k] for k in ("x", "y", "z", "angle")}
            if on_step:
                on_step(i, s)

    def stop(self):
        self.stopped = True


def demo_store():
    """Three boxes and a tray in a made-up layout, in a throw-away file: for trying things with the virtual arm only (never the real one)."""
    s = B.Store(os.path.join(tempfile.mkdtemp(prefix="demo-boxes-"), "boxes.json"))
    for k, x in ((1, -60), (2, 0), (3, 60)):
        s.set_box(k, x, 250, 120, {"w": 40, "d": 40, "h": 50})
    s.set_tray(0, 180, 60, {"w": 80, "d": 60, "h": 30})
    return s
