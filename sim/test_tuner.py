import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim import armsim as A  # noqa: E402
from sim import sequences as S  # noqa: E402
from sim import tuner as T  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQ = S.validate(json.load(open(os.path.join(ROOT, "sim", "fixtures", "pick_and_place.json"), encoding="utf-8")))
N = len(SEQ["poses"])


class SpeedFormatTest(unittest.TestCase):
    def test_speed_is_optional_and_old_files_stay_version_2(self):
        self.assertEqual(SEQ["version"], 2)
        self.assertTrue(all("speed" not in p for p in SEQ["poses"]))

    def test_speed_round_trips_and_makes_version_3(self):
        s = S.with_speeds(SEQ, [None, 40, None, 25.55, None, None, 60])
        self.assertEqual(s["version"], 3)
        self.assertEqual(S.speeds_of(s), [None, 40.0, None, 25.6, None, None, 60.0])
        self.assertEqual(S.speeds_of(S.validate(json.loads(json.dumps(s)))), S.speeds_of(s))
        self.assertEqual(S.with_speeds(s, None)["version"], 2)

    def test_bad_speed_is_refused(self):
        for bad in (0, -5, 500, "fast"):
            with self.assertRaises(ValueError):
                S.with_speeds(SEQ, [bad] + [None] * (N - 1))

    def test_reverse_moves_each_speed_to_the_other_end_of_its_move(self):
        s = S.with_speeds(SEQ, [10, 20, 30, 40, 50, 60, 70])        # speed k = the move from pose k-1 into pose k
        r = S.reverse(s)
        self.assertEqual(S.speeds_of(r), [None, 70, 60, 50, 40, 30, 20])     # going back, the move into pose j is forward move N-j
        self.assertEqual(S.speeds_of(S.reverse(r))[1:], [20, 30, 40, 50, 60, 70])


class ArmHonoursSpeedTest(unittest.TestCase):
    def travel_time(self, speed):
        """Seconds for the replay to bring the tool tip to the first saved pose."""
        sim = A.ArmSim(model="improved", rate=30)
        sim.saved[:] = [(150.0, 150.0, 100.0, 0.0), (0.0, 246.7, 150.0, 0.0)]
        sim.saved_speed[:] = [speed, speed]
        sim.start_replay()
        t = 0.0
        while t < 60:
            sim.step({}, 0.02)
            t += 0.02
            x, y, z, _ = sim.pose()
            if abs(x - 150.0) + abs(y - 150.0) + abs(z - 100.0) < 1.0:
                break
        return t

    def test_a_faster_pose_speed_finishes_sooner(self):
        self.assertLess(self.travel_time(90), self.travel_time(None) * 0.7)

    def test_stopping_the_replay_goes_back_to_the_damper(self):
        sim = A.ArmSim(model="improved", rate=30)
        sim.saved[:] = [(0.0, 246.7, 150.0, 0.0)]
        sim.saved_speed[:] = [90]
        sim._seg_rate = 90
        sim.stop_replay()
        self.assertEqual(sim._limit_deg_s(), 30)


class TunerTest(unittest.TestCase):
    def test_slow_reference_run_moves_key_2_to_the_tray(self):
        r = T.rollout(SEQ, [T.REF_SPEED] * N, check=False)
        self.assertEqual((r["failures"], r["keys"][2]), (0, "tray"))

    def test_too_fast_is_caught(self):
        r = T.rollout(SEQ, [90] * N)
        self.assertGreater(r["failures"], 0)

    def test_tuning_is_faster_and_has_no_failures(self):
        res = T.tune(SEQ, generations=25, restarts=2)
        self.assertEqual(res["tuned"]["failures"], 0)
        self.assertLess(res["tuned"]["time"], res["baseline"]["time"] * 0.85)
        self.assertTrue(all(T.SPEED_MIN <= v <= T.SPEED_MAX for v in res["speeds"]))
        self.assertEqual(res["history"], sorted(res["history"], reverse=True))      # the best cost never gets worse

    def test_cancel_stops_early(self):
        res = T.tune(SEQ, generations=50, cancel=lambda: True)
        self.assertEqual(res["generations"], 0)

    def test_unrunnable_sequence_is_refused(self):
        far = S.validate({"format": S.FORMAT, "poses": [{"x": 0, "y": 900, "z": 0, "angle": 0, "action": None}]})
        with self.assertRaises(ValueError):
            T.Scorer(far)

    def test_limits_are_validated(self):
        with self.assertRaises(ValueError):
            T.clean_limits({"slip_mm_s": 1})


class FeedbackTest(unittest.TestCase):
    SEEN = {"time": 9.0, "hold_mm_s": 150.0, "land_mm_s": 100.0}

    def test_scaled_speeds_stay_inside_the_firmware_range(self):
        self.assertEqual(T.scaled([30, 80, 4], 0.5), [15.0, 40.0, 3.0])
        self.assertTrue(all(3.0 <= v <= 60.0 for v in T.scaled([1, 200, 30], 1.0)))
        self.assertEqual(T.SPEED_MAX, 60.0)                      # the firmware's MaxJointDegPerSecondCap

    def test_a_dropped_key_lowers_only_the_slip_limit(self):
        lim, note = T.calibrate(T.DEFAULT_LIMITS, "dropped", self.SEEN)
        self.assertEqual(lim["slip_mm_s"], 135.0)               # 0.9 x the 150 mm/s that was seen to drop it
        self.assertEqual((lim["knock_mm_s"], lim["margin"]), (110.0, 0.85))
        self.assertIn("135", note)

    def test_a_missed_grab_lowers_only_the_knock_limit(self):
        lim, _ = T.calibrate(T.DEFAULT_LIMITS, "missed", self.SEEN)
        self.assertEqual((lim["knock_mm_s"], lim["slip_mm_s"]), (90.0, 220.0))

    def test_rough_tightens_the_margin_and_ok_changes_nothing(self):
        self.assertAlmostEqual(T.calibrate(T.DEFAULT_LIMITS, "rough", self.SEEN)[0]["margin"], 0.765)
        self.assertEqual(T.calibrate(T.DEFAULT_LIMITS, "ok", self.SEEN)[0], T.clean_limits(T.DEFAULT_LIMITS))
        with self.assertRaises(ValueError):
            T.calibrate(T.DEFAULT_LIMITS, "great", self.SEEN)

    def test_a_limit_never_goes_below_its_floor_or_up(self):
        lim, _ = T.calibrate({"slip_mm_s": 50.0}, "dropped", {"hold_mm_s": 10.0, "land_mm_s": 0, "time": 1})
        self.assertEqual(lim["slip_mm_s"], T.SLIP_FLOOR)
        lim, _ = T.calibrate({"slip_mm_s": 100.0}, "dropped", {"hold_mm_s": 400.0, "land_mm_s": 0, "time": 1})
        self.assertEqual(lim["slip_mm_s"], 100.0)               # a failure at a higher speed than the limit teaches nothing new

    def test_the_ladder_opens_one_rung_at_a_time_and_resets_for_new_speeds(self):
        import tempfile
        cal = T.Calibration(tempfile.mkdtemp())
        tuned = [30.0, 50.0, 40.0]
        self.assertEqual(cal.allowed_scale("seq", tuned), 0.5)
        cal.record("seq", tuned, 0.5, "ok", self.SEEN)
        self.assertEqual(cal.allowed_scale("seq", tuned), 0.75)
        cal.record("seq", tuned, 0.75, "dropped", self.SEEN)     # a failure does not open the next rung
        self.assertEqual(cal.allowed_scale("seq", tuned), 0.75)
        cal.record("seq", tuned, 0.75, "ok", self.SEEN)
        self.assertEqual(cal.allowed_scale("seq", tuned), 1.0)
        self.assertEqual(cal.allowed_scale("seq", [30.0, 45.0, 40.0]), 0.5)    # new speeds start again from the bottom
        self.assertEqual(cal.load("seq")["limits"]["slip_mm_s"], 135.0)        # and the dropped verdict was kept
        self.assertEqual(len(cal.load("seq")["trials"]), 3)

    def test_observed_reports_what_the_speeds_ask_of_the_arm(self):
        slow = T.observed(SEQ, T.scaled([40.0] * N, 0.5))
        fast = T.observed(SEQ, T.scaled([40.0] * N, 1.0))
        self.assertGreater(fast["hold_mm_s"], slow["hold_mm_s"])
        self.assertGreater(slow["time"], fast["time"])


if __name__ == "__main__":
    unittest.main()
