"""Which pulse range makes a servo go where the model says? Worked out from two angles measured on the arm.

The model's joint angles are "servo degrees" (0..180, 90 = centre); the board turns them into pulses with a straight line, lo us at 0 degrees to
hi us at 180 degrees. The model does not say what lo and hi are for YOUR servos: UWE-RArm's firmware writes 500 and 2400, an earlier guess here was
1000 and 2000 around 1500, and only the arm can tell. The check: command two angles, measure where the joint really ended up, and fit the line.

    physical angle = (pulse - pulse_at_0_physical) / (pulses per physical degree)

so two (commanded pulse, measured angle) pairs fix both the slope and the offset.
"""

import math

from sim import armsim as A

SAM = (500.0, 2400.0)          # UWE-RArm: ESP32Servo attach(pin, 500, 2400), write(angle)
EARLIER_GUESS = (1000.0, 2000.0)
WINDOW = (400.0, 2600.0)       # what the board accepts for !pulse
MIN_SPAN = 600.0


def pulse_of(deg, board):
    lo, hi = board
    return lo + deg * (hi - lo) / 180.0


def fit_pulse_range(cmd_a, meas_a, cmd_b, meas_b, board=SAM):
    """cmd_*: the servo degrees that were commanded (through the board's current mapping `board` = (lo, hi)); meas_*: the physical angles measured
    for them in the same convention (the check's instructions say how to read each joint). Returns the lo and hi that would make the commanded
    degrees come out as the model says, and which known hypothesis is closer."""
    cmd_a, meas_a, cmd_b, meas_b = (float(v) for v in (cmd_a, meas_a, cmd_b, meas_b))
    if cmd_a == cmd_b:
        raise ValueError("command two different angles")
    if meas_a == meas_b:
        raise ValueError("the two measured angles are the same: the joint did not turn between them")
    us_a, us_b = pulse_of(cmd_a, board), pulse_of(cmd_b, board)
    per_deg = (us_b - us_a) / (meas_b - meas_a)             # pulses per PHYSICAL degree
    if per_deg <= 0:
        raise ValueError("the joint turned the opposite way to the model: reverse its limits (for example 180,0) on the Control page, then measure again")
    us_90 = us_a + (90.0 - meas_a) * per_deg
    lo, hi = us_90 - 90.0 * per_deg, us_90 + 90.0 * per_deg
    if not (WINDOW[0] <= lo and hi <= WINDOW[1] and hi - lo >= MIN_SPAN):
        raise ValueError("the fit (%.0f to %.0f us) falls outside what the board accepts (%d to %d us, at least %d apart): re-measure, or the servos "
                         "do not follow a straight pulse-to-angle line" % (lo, hi, WINDOW[0], WINDOW[1], MIN_SPAN))
    d_sam = abs(lo - SAM[0]) + abs(hi - SAM[1])
    d_old = abs(lo - EARLIER_GUESS[0]) + abs(hi - EARLIER_GUESS[1])
    return {"lo": round(lo), "hi": round(hi), "us_per_deg": round(per_deg, 3), "us_at_90": round(us_90),
            "closest": "sam" if d_sam <= d_old else "earlier", "distance_sam_us": round(d_sam), "distance_earlier_us": round(d_old),
            "commanded_us": [round(us_a), round(us_b)]}


# ---- the simpler check: one pose, two candidate mappings, and the person only says which prediction is nearer ------------------------------------
# A precise angle reading is easy to get wrong, and the fit above turns a small reading error into tens of microseconds. A pose where the two candidate
# mappings predict tool-tip positions far apart does not need precision: a ruler error of a centimetre cannot change which prediction is nearer.
TEST_POSE = (0.0, 150.0, 250.0, 0.0)            # x, y, z, tool angle: 92 mm between the two predictions, and well above the table either way
CANDIDATES = (("Sam's mapping (500 to 2400 us)", SAM), ("the earlier guess (1000 to 2000 us)", EARLIER_GUESS))


def predict_tip(target=TEST_POSE, board=SAM, limits=None, tool=A.DEFAULT_TOOL_MM, candidates=CANDIDATES, inverted=None):
    """Where the tool tip ends up if the board sends pulses with the mapping `board` while the servos really follow each candidate line.
    Returns {"target", "predictions": [{"name", "lo", "hi", "tip", "reach", "height"}], "separation_mm"}. Reach is the horizontal distance from the base
    axis and height is above the table (z = 0), the two numbers a ruler gives. `inverted` marks joints whose servo runs the other way (reversed limits)."""
    limits = limits or A.DEFAULT_LIMITS
    inv = list(inverted or [False] * 4)
    home = tuple(math.radians(90) for _ in range(4))
    goal = A.inverse_kin(target, home, limits, tool)
    if goal is None:
        raise ValueError("the arm cannot reach the test pose with the current limits")
    degs = [math.degrees(v) for v in goal]
    preds = []
    for name, (lo, hi) in candidates:
        servo = [180.0 - d if inv[i] else d for i, d in enumerate(degs)]                 # what the board is told
        real = [(pulse_of(s, board) - lo) / (hi - lo) * 180.0 for s in servo]            # where the servo really goes
        phys = tuple(math.radians(180.0 - r if inv[i] else r) for i, r in enumerate(real))   # back to the model's joint angles
        x, y, z, _ = A.forward_kin(phys, tool)
        preds.append({"name": name, "lo": lo, "hi": hi, "tip": [round(x, 1), round(y, 1), round(z, 1)],
                      "reach": round(math.hypot(x, y), 1), "height": round(z, 1)})
    sep = max((math.dist((p["reach"], p["height"]), (q["reach"], q["height"])) for p in preds for q in preds), default=0.0)
    return {"target": list(target), "model_angles": [round(v, 1) for v in degs], "predictions": preds, "separation_mm": round(sep, 1),
            "model": {"reach": round(math.hypot(target[0], target[1]), 1), "height": round(target[2], 1)}}


def judge(reach, height, predicted):
    """Which prediction is the measured tip nearest to, and how sure can that be. Only the (reach, height) plane is used: it is what a ruler measures."""
    reach, height = float(reach), float(height)
    preds = predicted["predictions"]
    sep = max(float(predicted["separation_mm"]), 1.0)
    dist = sorted(((math.dist((reach, height), (p["reach"], p["height"])), p) for p in preds), key=lambda t: t[0])
    d1, best = dist[0]
    d2 = dist[1][0] if len(dist) > 1 else float("inf")
    if d1 > 0.5 * sep:
        verdict = "neither"
    elif d1 <= 0.25 * sep and d2 >= 0.6 * sep:
        verdict = "clear"
    else:
        verdict = "unclear"
    return {"verdict": verdict, "nearest": best["name"], "lo": best["lo"], "hi": best["hi"], "distance_mm": round(d1), "other_mm": round(d2),
            "separation_mm": round(sep), "distances": {p["name"]: round(math.dist((reach, height), (p["reach"], p["height"]))) for p in preds}}
