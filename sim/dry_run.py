"""A full dry run of the coordinate design with no arm: stored coordinates -> route generator -> job runner -> a sender that plans every step with the
real planner and 'moves' a virtual arm. Prints the job statuses the page would receive.
    python sim/dry_run.py            fetch key 2, return it, then a fetch with a scripted stall to show recovery
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import jobs as J  # noqa: E402
from virtual import VirtualArm, demo_store as make_store  # noqa: E402


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
