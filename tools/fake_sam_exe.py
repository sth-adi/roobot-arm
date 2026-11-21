"""A stand-in for Sam's exe, for testing everything else while we do not have the real one.

    python tools/fake_sam_exe.py <instructions.json | json text> <baud> <port> [--speedup N] [--fail-port NAME]

It behaves the way we ASSUME his exe does (see ExeSender in dashboard/lowlevel.py): it reads his start-point/end-point JSON (or his angle-list form),
checks it, "plans" the straight line with our own planner (so an unreachable point fails, as his IK would), waits as long as the move would take, prints
what it did and exits 0. A bad instruction exits 2 and says why. It touches no hardware: the COM port is only echoed.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
import planner  # noqa: E402


def fail(msg):
    print("ERROR: " + msg)
    sys.exit(2)


def load(arg):
    text = open(arg).read() if os.path.isfile(arg) else arg
    try:
        return json.loads(text)
    except ValueError as e:
        fail("the instructions are not valid JSON (%s)" % e)


def point(d, name):
    p = d.get(name)
    try:
        return (float(p["x"]), float(p["y"]), float(p["z"]), math.radians(float(p["toolDegrees"])))
    except (TypeError, KeyError, ValueError):
        fail("%s needs numeric x, y, z and toolDegrees" % name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instructions")
    ap.add_argument("baud", type=int)
    ap.add_argument("port")
    ap.add_argument("--speedup", type=float, default=1.0, help="wait this many times shorter than the real move would take")
    ap.add_argument("--fail-port", help="act as if this port cannot be opened")
    a = ap.parse_args()
    if a.fail_port and a.port == a.fail_port:
        print("ERROR: could not open %s" % a.port)
        sys.exit(3)
    msg = load(a.instructions)
    if isinstance(msg, list):                                     # his first form: a timed list of joint angles
        try:
            rows = [(float(r["Millis"]), [float(r["S%d" % j]) for j in (1, 2, 3, 4)]) for r in msg]
        except (TypeError, KeyError, ValueError):
            fail("each angle row needs Millis, S1, S2, S3 and S4")
        if not rows or any(not 0 <= v <= 180 for _, q in rows for v in q):
            fail("the angle list is empty or has an angle outside 0-180")
        duration = max(t for t, _ in rows) / 1000.0
        what = "%d angle rows" % len(rows)
    elif isinstance(msg, dict):                                   # his second form: start point and end point
        start, end = point(msg, "StartPoint"), point(msg, "EndPoint")
        try:
            speed = float(msg["speed"])
            if not speed > 0:
                raise ValueError
        except (TypeError, KeyError, ValueError):
            fail("speed must be a positive number")
        try:
            plan = planner.plan(start, end, speed_mm_s=speed)
        except planner.PlanError as e:
            fail("%s: %s" % (e.title, e.detail))
        tool = msg.get("Tool", 0)
        if tool not in (0, 1, True, False):
            fail("Tool must be 0 or 1")
        duration = plan["duration"]
        what = "line %s -> %s at %g mm/s (%d samples), magnet %s" % (tuple(round(v, 1) for v in start[:3]), tuple(round(v, 1) for v in end[:3]), speed, plan["steps"], "ON" if tool else "off")
    else:
        fail("the instructions must be a list of angles or an object with StartPoint and EndPoint")
    print("fake exe: %s on %s at %d baud, about %.1f s" % (what, a.port, a.baud, duration))
    time.sleep(duration / max(a.speedup, 1e-6))
    print("done")


if __name__ == "__main__":
    main()
