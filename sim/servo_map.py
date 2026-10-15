"""Which pulse range makes a servo go where the model says? Worked out from two angles measured on the arm.

The model's joint angles are "servo degrees" (0..180, 90 = centre); the board turns them into pulses with a straight line, lo us at 0 degrees to
hi us at 180 degrees. The model does not say what lo and hi are for YOUR servos: UWE-RArm's firmware writes 500 and 2400, an earlier guess here was
1000 and 2000 around 1500, and only the arm can tell. The check: command two angles, measure where the joint really ended up, and fit the line.

    physical angle = (pulse - pulse_at_0_physical) / (pulses per physical degree)

so two (commanded pulse, measured angle) pairs fix both the slope and the offset.
"""

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
