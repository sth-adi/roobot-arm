"""A full dry run of the coordinate design with no arm: stored coordinates -> route generator -> job runner -> a sender that plans every step with the
real planner and 'moves' a virtual arm. Prints the job statuses the page would receive.
    python sim/dry_run.py            fetch key 2, return it, then a fetch with a scripted stall to show recovery
"""
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import backend_client as BC  # noqa: E402
import boxes as B  # noqa: E402
import jobs as J  # noqa: E402
import planner as PL  # noqa: E402


class VirtualArm:
    """A sender with the same run()/stop() as lowlevel's: every step is planned by the real planner; the arm 'arrives' and its position is kept."""
    def __init__(self, stall_label=None):
        self.pose, self.t, self.stall, self.stopped = {"x": 0.0, "y": 250.0, "z": 250.0, "angle": 0.0}, 0.0, stall_label, False

    def run(self, steps, job="", on_step=None):
        for i, s in enumerate(steps):
            if self.stopped:
                raise BC.BackendError("STOPPED", "Stopped")
            if s["label"] == self.stall:
                self.stall = None
                raise BC.BackendError("ARM_STALLED", "A joint stalled", "scripted for the dry run")
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


def make_store():
    s = B.Store(os.path.join(tempfile.mkdtemp(), "boxes.json"))
    for k, x in ((1, -60), (2, 0), (3, 60)):
        s.set_box(k, x, 250, 120, {"w": 40, "d": 40, "h": 50})
    s.set_tray(0, 180, 60, {"w": 80, "d": 60, "h": 30})
    return s


def run(action, arm, store):
    log = []
    r = J.JobRunner(store, arm, lambda: dict(arm.pose), on_status=log.append)
    r.start(action)
    r.wait(60)
    return log


if __name__ == "__main__":
    store, arm = make_store(), VirtualArm()
    for action, arm_ in (({"action": "fetch_key", "key": 2}, arm), ({"action": "return_key", "key": 2}, arm),
                         ({"action": "fetch_key", "key": 3}, VirtualArm("down to the key"))):
        print("\n==", action)
        for p in run(action, arm_, store):
            print("  %-8s %2s/%-2s %-6s %s" % (p["status"], p["step"] if p["step"] is not None else "-", p["of"] if p["of"] is not None else "-", p["led"], p["message"]))
        print("  virtual arm time %.1f s, ended at (%.0f, %.0f, %.0f)" % (arm_.t, arm_.pose["x"], arm_.pose["y"], arm_.pose["z"]))
