import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim import armsim as A  # noqa: E402
from sim import sequences as S  # noqa: E402
from sim import tuner as T  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQ = S.validate(json.load(open(os.path.join(ROOT, "sequences", "box2_to_tray.json"), encoding="utf-8")))
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


if __name__ == "__main__":
    unittest.main()
