"""Speed tuner: learns a joint speed (deg/s) for every move of a sequence, from simulated runs.

The tuner never touches the live arm. It replays a sequence headless in the virtual arm and the work cell (sim/armsim.py,
sim/workcell.py), scores each run, and searches for the per-pose speeds with the lowest cost. The search is a small evolution
strategy (black-box optimisation, pure Python, no network and no language model): propose speeds, run them, keep what scored better,
narrow the search as it converges.

What a run is scored on (the feedback):
  * time        the travel time of the whole sequence;
  * slip        while a key is held, the tool tip's speed at the start and end of a move must stay under `slip_mm_s`, or the key
                shakes off the magnet and drops;
  * knock       the tool tip's speed when it arrives at a grab or release pose must stay under `knock_mm_s`, or the key is knocked
                about (a grab misses);
  * outcome     every key must end up where it ends up in a slow reference run.
Faster is cheaper, so the search pushes free-space moves up and holds back the moves that carry a key or end at a grab or release.

The simulated arm has no inertia and the two limits are ESTIMATES, not measurements: they are the knobs to calibrate against the real
arm. Speeds tuned here are honoured by the virtual arm; firmware that reads a per-pose speed is a separate step.
"""
import json
import math
import os
import random
import time

from sim import armsim as A
from sim import sequences as SEQ
from sim.workcell import Workcell

SPEED_MIN, SPEED_MAX = 5.0, 60.0        # search bounds, deg/s (the damper default is 30; 60 is the firmware's own cap, MaxJointDegPerSecondCap)
FW_MOVE_S = 10.0                        # the firmware gives up waiting for a move after 12 s and goes on, so a tuned move must finish well inside that
REF_SPEED = 10.0                        # the slow, safe speed the reference outcome is taken at
DT = 0.02                               # the hub's 50 Hz step
MAX_SEG_S = 60.0
SETTLE_S = 0.15                         # pause at a pose before its magnet action
FALL_S = 1.0                            # time for a released key to land
DEFAULT_LIMITS = {"slip_mm_s": 220.0, "knock_mm_s": 110.0, "margin": 0.85}
FAIL_COST = 100.0                       # seconds of cost per failure: failing is never worth the time saved


def clean_limits(d):
    out = dict(DEFAULT_LIMITS)
    for k in out:
        if d and d.get(k) is not None:
            v = float(d[k])
            if not (math.isfinite(v) and (0.3 <= v <= 1.0 if k == "margin" else 10.0 <= v <= 2000.0)):
                raise ValueError("%s is out of range" % k)
            out[k] = v
    return out


def rollout(seq, speeds, limits=None, tool=A.DEFAULT_TOOL_MM, arm_limits=None, homes=None, check=True):
    """Run `seq` once in the virtual arm with these per-pose speeds. Returns
    {"time", "failures", "problems", "keys": {id: where}, "segments": [{"i", "speed", "tip_start", "tip_end", "holding", "time"}]}.
    check=False turns the slip and knock checks off (used for the reference run)."""
    lim = clean_limits(limits)
    slip, knock = lim["slip_mm_s"] * lim["margin"], lim["knock_mm_s"] * lim["margin"]
    sim = A.ArmSim(tool_mm=tool, limits=arm_limits, homes=homes, model="improved")
    cell = Workcell()
    poses = seq["poses"]
    out = {"time": 0.0, "failures": 0, "problems": [], "segments": []}

    def fail(msg):
        out["failures"] += 1
        out["problems"].append(msg)

    for i, p in enumerate(poses):
        goal = A.inverse_kin((p["x"], p["y"], p["z"], p["angle"]), sim.angles, sim.limits, sim.tool)
        if goal is None:
            fail("pose %d is out of reach" % (i + 1))
            out["segments"].append({"i": i, "speed": speeds[i], "tip_start": 0.0, "tip_end": 0.0, "holding": False, "time": 0.0, "moves": False, "lands": False})
            continue
        sim.rate = float(speeds[i])
        holding = cell.held is not None
        tip = A.forward_kin(sim.angles, sim.tool)[:3]
        t, v_start, v_end, arrived, travel = 0.0, None, 0.0, False, 0.0
        while not arrived and t < MAX_SEG_S:
            arrived = sim._slew(goal, DT)
            new = A.forward_kin(sim.angles, sim.tool)[:3]
            v_end = math.dist(new, tip) / DT
            travel += v_end * DT
            if v_start is None:
                v_start = v_end
                if check and holding and v_start > slip:
                    cell.set_magnet(False)                     # the key shakes off at the jerk of starting
                    fail("pose %d: key dropped at the start of the move (%d mm/s)" % (i + 1, v_start))
            tip = new
            cell.step(tip, False, DT)
            t += DT
        if not arrived:
            fail("pose %d: move did not finish" % (i + 1))
        elif t > FW_MOVE_S:
            fail("pose %d: move takes %.1f s, longer than the firmware waits" % (i + 1, t))
        if check and cell.held is not None and v_end > slip:
            cell.set_magnet(False)
            fail("pose %d: key dropped at the stop (%d mm/s)" % (i + 1, v_end))
        for _ in range(int(SETTLE_S / DT)):
            cell.step(tip, False, DT)
            t += DT
        act = p.get("action")
        nxt = poses[i + 1] if i + 1 < len(poses) else None
        lands = act or (nxt.get("action") if nxt and math.dist((p["x"], p["y"], p["z"]), (nxt["x"], nxt["y"], nxt["z"])) < 2.0 else None)   # a grab/release is often taught as "arrive, then act at the same spot"
        if lands and check and v_end > knock:
            fail("pose %d: arrived too fast to %s (%d mm/s)" % (i + 1, lands, v_end))
            if act == "release":
                cell.set_magnet(False)
        elif act == "grab":
            cell.set_magnet(True)
        elif act == "release":
            cell.set_magnet(False)
        for _ in range(int((FALL_S if act == "release" else DT) / DT)):
            cell.step(tip, False, DT)
            if act == "release":
                t += DT
        out["time"] += t
        out["segments"].append({"i": i, "speed": float(speeds[i]), "tip_start": round(v_start or 0.0, 1), "tip_end": round(v_end, 1),
                                "holding": holding, "time": round(t, 2), "moves": travel > 0.5, "lands": bool(lands)})
    out["time"] = round(out["time"], 2)
    out["keys"] = {k["id"]: cell.where(k) for k in cell.keys}
    return out


class Scorer:
    """Cost of a speed vector for one sequence: lower is better."""

    def __init__(self, seq, limits=None, **arm):
        self.seq, self.limits, self.arm = seq, clean_limits(limits), arm
        ref = rollout(seq, [REF_SPEED] * len(seq["poses"]), self.limits, check=False, **arm)
        if ref["failures"]:
            raise ValueError("the simulator cannot run this sequence even slowly: " + "; ".join(ref["problems"][:2]))
        self.reference = ref["keys"]
        self.evals = 0

    def run(self, speeds):
        r = rollout(self.seq, speeds, self.limits, **self.arm)
        if r["keys"] != self.reference:
            r["failures"] += 1
            r["problems"].append("keys ended up in %s instead of %s" % (r["keys"], self.reference))
        r["cost"] = r["time"] + FAIL_COST * r["failures"]
        self.evals += 1
        return r


def _clip(v):
    return max(SPEED_MIN, min(SPEED_MAX, v))


def tune(seq, start=None, limits=None, generations=40, offspring=8, restarts=3, seed=1, progress=None, cancel=None, **arm):
    """Search for per-pose speeds. `start` is the speed vector to improve on (default: the 30 deg/s damper everywhere).
    progress(gen, generations, best_cost) is called after every generation; cancel() returning True stops early.
    Returns {"speeds", "baseline", "tuned", "history", "evals", "limits", "seconds"}."""
    t0 = time.time()
    n = len(seq["poses"])
    sc = Scorer(seq, limits, **arm)
    x0 = [_clip(float(s)) if s else 30.0 for s in (start or [None] * n)]
    base = sc.run(x0)
    best, best_r, history, gen, stop = list(x0), base, [round(base["cost"], 2)], 0, False
    for rs in range(restarts):                                  # independent searches from the same start: the best of them wins
        cur, cur_r, sigma = list(x0), base, 15.0                # sigma, deg/s: how far a proposal strays from the best so far
        rng = random.Random(seed + rs)
        for g in range(1, generations + 1):
            if cancel and cancel():
                stop = True
                break
            wins = 0
            for _ in range(offspring):
                cand = [_clip(b + rng.gauss(0, sigma)) for b in cur]
                r = sc.run(cand)
                if r["cost"] < cur_r["cost"] - 1e-9:
                    cur, cur_r, wins = cand, r, wins + 1
            sigma = max(0.5, sigma * (1.25 if wins / offspring > 0.2 else 0.82))     # the 1/5 success rule: widen when it pays off, narrow when it does not
            gen += 1
            if cur_r["cost"] < best_r["cost"] - 1e-9:
                best, best_r = list(cur), cur_r
            history.append(round(best_r["cost"], 2))
            if progress:
                progress(gen, generations * restarts, best_r["cost"])
        if stop:
            break
    for i, seg in enumerate(best_r["segments"]):                # a pose the arm does not move to has no meaningful speed: keep the start value
        if not seg["moves"]:
            best[i] = x0[i]
    speeds = [round(v, 1) for v in best]
    tuned = sc.run(speeds)
    return {"speeds": speeds, "baseline": {"speeds": x0, "time": base["time"], "failures": base["failures"], "problems": base["problems"]},
            "tuned": {"time": tuned["time"], "failures": tuned["failures"], "problems": tuned["problems"], "segments": tuned["segments"]},
            "history": history, "evals": sc.evals, "generations": gen, "limits": sc.limits, "seconds": round(time.time() - t0, 1)}


# ---- feedback from real runs ------------------------------------------------------------------------------------------------
# The simulated arm has no inertia and the two limits are estimates, so they are corrected from what happens on the real arm.
# A trial runs the tuned speeds on the arm, scaled down first; the person says how it went; the verdict moves a limit.
OUTCOMES = ("ok", "dropped", "missed", "rough")
LADDER = (0.5, 0.75, 1.0)               # fractions of the tuned speeds a trial may use, in order: a rung is only open once the one below went ok
SLIP_FLOOR, KNOCK_FLOOR, MARGIN_FLOOR = 40.0, 20.0, 0.5
CAL_SHRINK = 0.9                        # a failure puts the limit at this fraction of the speed that was seen to fail


def scaled(speeds, scale):
    """The tuned speeds at a fraction of full speed, kept inside the firmware's range."""
    return [round(max(3.0, min(SPEED_MAX, v * scale)), 1) for v in speeds]


def observed(seq, speeds, limits=None, **arm):
    """What the simulator says these speeds ask of the arm: the fastest the tool tip moves while a key is carried, and when arriving
    at a grab or release. These are the numbers a failed real run is blamed on."""
    r = rollout(seq, speeds, limits, check=False, **arm)
    hold = [max(s["tip_start"], s["tip_end"]) for s in r["segments"] if s["holding"] and s["moves"]]
    land = [s["tip_end"] for s in r["segments"] if s["lands"] and s["moves"]]
    return {"time": r["time"], "hold_mm_s": round(max(hold), 1) if hold else 0.0, "land_mm_s": round(max(land), 1) if land else 0.0}


def calibrate(limits, outcome, seen):
    """New limits after a real run, and a sentence about what changed. `seen` is observed() of the speeds that were run."""
    lim, note = dict(clean_limits(limits)), "The limits are unchanged."
    if outcome not in OUTCOMES:
        raise ValueError("outcome must be one of " + ", ".join(OUTCOMES))
    if outcome == "dropped" and seen["hold_mm_s"] > 0:
        lim["slip_mm_s"] = round(max(SLIP_FLOOR, min(lim["slip_mm_s"], CAL_SHRINK * seen["hold_mm_s"])), 1)
        note = "The key slipped with the tip at up to %g mm/s while carrying it, so the slip limit is now %g mm/s." % (seen["hold_mm_s"], lim["slip_mm_s"])
    elif outcome == "missed" and seen["land_mm_s"] > 0:
        lim["knock_mm_s"] = round(max(KNOCK_FLOOR, min(lim["knock_mm_s"], CAL_SHRINK * seen["land_mm_s"])), 1)
        note = "A grab or release went wrong arriving at up to %g mm/s, so the knock limit is now %g mm/s." % (seen["land_mm_s"], lim["knock_mm_s"])
    elif outcome == "rough":
        lim["margin"] = round(max(MARGIN_FLOOR, lim["margin"] * CAL_SHRINK), 3)
        note = "The motion was too rough, so the safety margin is now %g." % lim["margin"]
    return lim, note


class Calibration:
    """Per sequence: the limits learned from real runs and the log of those runs (tuning/<name>.json, local to this PC)."""

    def __init__(self, folder):
        self.folder = folder

    def _path(self, name):
        if not SEQ.NAME_RE.match(name or ""):
            raise ValueError("name: letters, digits, - and _ only")
        return os.path.join(self.folder, name + ".json")

    def load(self, name):
        try:
            with open(self._path(name), encoding="utf-8") as f:
                d = json.load(f)
            return {"limits": clean_limits(d.get("limits")), "trials": list(d.get("trials") or [])[-50:]}
        except (OSError, ValueError):
            return {"limits": dict(DEFAULT_LIMITS), "trials": []}

    def save(self, name, data):
        os.makedirs(self.folder, exist_ok=True)
        with open(self._path(name), "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def allowed_scale(self, name, speeds):
        """The highest fraction of `speeds` a trial may use now: the first rung, or the rung above the highest one that went ok for
        exactly these speeds. A change of speeds (a new tuning) starts the ladder again."""
        key = [round(v, 1) for v in speeds]
        done = [t["scale"] for t in self.load(name)["trials"] if t.get("outcome") == "ok" and t.get("tuned") == key]
        top = max(done) if done else 0.0
        return next((r for r in LADDER if r > top + 1e-9), LADDER[-1])

    def record(self, name, tuned, scale, outcome, seen):
        data = self.load(name)
        before = data["limits"]
        data["limits"], note = calibrate(before, outcome, seen)
        data["trials"].append({"t": round(time.time()), "tuned": [round(v, 1) for v in tuned], "scale": scale, "outcome": outcome, "seen": seen,
                               "limits_before": before, "limits_after": data["limits"]})
        self.save(name, data)
        return data, note
