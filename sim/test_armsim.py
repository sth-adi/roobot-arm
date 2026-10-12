"""Tests for the virtual arm. Run:  python -m unittest sim.test_armsim -v"""
import math
import random
import unittest

from sim import armsim as A

NEUTRAL = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}


def inp(**kw):
    d = dict(NEUTRAL)
    d.update(kw)
    return d


def run(sim, state, seconds, dt=0.02):
    for _ in range(int(round(seconds / dt))):
        sim.step(state, dt)


def max_joint_step(sim, state, seconds, dt=0.02):
    worst, prev = 0.0, sim.angles
    for _ in range(int(round(seconds / dt))):
        sim.step(state, dt)
        worst = max(worst, max(abs(math.degrees(sim.angles[i] - prev[i])) for i in range(4)) / dt)
        prev = sim.angles
    return worst


class Kinematics(unittest.TestCase):
    def test_home_pose(self):
        x, y, z, th = A.forward_kin(tuple(math.radians(90) for _ in range(4)), 63.0)
        self.assertAlmostEqual(x, 0, places=6)
        self.assertAlmostEqual(y, 183.70 + 63.0, places=6)
        self.assertAlmostEqual(z, 145.95 + 74.0, places=6)
        self.assertAlmostEqual(th, 0, places=6)

    def test_ik_inverts_fk(self):
        rnd = random.Random(1)
        checked = 0
        for _ in range(400):
            ang = tuple(math.radians(rnd.uniform(20, 160)) for _ in range(4))
            pose = A.forward_kin(ang)
            got = A.inverse_kin(pose, ang, A.DEFAULT_LIMITS)
            if got is None:
                continue
            checked += 1
            back = A.forward_kin(got)
            for u, v in zip(pose, back):
                self.assertAlmostEqual(u, v, places=3)
        self.assertGreater(checked, 100)

    def test_unreachable_is_rejected(self):
        self.assertIsNone(A.inverse_kin((0, 900, 100, 0), (math.pi / 2,) * 4, A.DEFAULT_LIMITS))

    def test_limits_reject(self):
        tight = [[85, 95]] * 4
        far = A.forward_kin(tuple(math.radians(40) for _ in range(4)))
        self.assertIsNone(A.inverse_kin(far, (math.pi / 2,) * 4, tight))

    def test_tool_length_changes_reach(self):
        a = tuple(math.radians(90) for _ in range(4))
        self.assertAlmostEqual(A.forward_kin(a, 63)[1] - A.forward_kin(a, 53)[1], 10.0, places=6)


class Damper(unittest.TestCase):
    def test_joint_speed_never_exceeds_limit(self):
        for rate in (5.0, 20.0, 60.0):
            sim = A.ArmSim(rate=rate)
            worst = max_joint_step(sim, inp(LSX=1.0, RSY=-1.0), 4.0)
            self.assertLessEqual(worst, rate + 1e-6, f"rate {rate}")

    def test_slow_damper_moves_less_than_fast(self):
        slow, fast = A.ArmSim(rate=2.0), A.ArmSim(rate=60.0)
        p0 = slow.pose()
        run(slow, inp(LSY=-1.0), 3.0)
        run(fast, inp(LSY=-1.0), 3.0)
        self.assertLessEqual(math.dist(slow.pose()[:3], p0[:3]), math.dist(fast.pose()[:3], p0[:3]) + 1e-9)

    def test_no_lead_after_release(self):
        sim = A.ArmSim(rate=5.0)
        run(sim, inp(LSX=1.0), 2.0)
        before = sim.angles
        run(sim, NEUTRAL, 2.0)
        self.assertEqual(before, sim.angles)          # releasing stops the arm at once: no catch-up

    def test_zero_means_capped_at_180(self):
        sim = A.ArmSim(rate=0)
        self.assertEqual(sim._limit_deg_s(), A.UNLIMITED_DEG_S)

    def test_home_is_damped_too(self):
        sim = A.ArmSim(rate=10.0)
        run(sim, inp(LSX=1.0, LSY=-1.0), 3.0)
        d0 = max(abs(math.degrees(sim.angles[i]) - 90) for i in range(4))
        self.assertGreater(d0, 1.0)
        worst = max_joint_step(sim, inp(B=True), 0.02)
        worst = max(worst, max_joint_step(sim, NEUTRAL, 5.0))
        self.assertLessEqual(worst, 10.0 + 1e-6)


class Behaviour(unittest.TestCase):
    def test_stale_input_stops_motion(self):
        sim = A.ArmSim()
        run(sim, inp(LSX=1.0), 1.0)
        before = sim.angles
        run(sim, None, 1.0)
        self.assertEqual(before, sim.angles)

    def test_deadzone(self):
        sim = A.ArmSim()
        before = sim.angles
        run(sim, inp(LSX=0.19, LSY=-0.19, RSX=0.19, RSY=0.19), 1.0)
        self.assertEqual(before, sim.angles)

    def test_unreachable_holds_and_flags(self):
        sim = A.ArmSim(rate=1000)
        run(sim, inp(LSY=1.0), 30.0)                  # push outward until the arm cannot go further
        self.assertFalse(sim.ik_ok)
        held = sim.angles
        run(sim, inp(LSY=1.0), 0.5)
        self.assertEqual(held, sim.angles)
        run(sim, inp(LSY=-1.0), 0.5)                  # backing off works again
        self.assertTrue(sim.ik_ok)

    def test_home_then_jog_does_not_snap_back(self):
        sim = A.ArmSim(rate=60.0)
        run(sim, inp(LSX=1.0, LSY=-1.0), 3.0)
        moved = sim.angles
        run(sim, inp(B=True), 0.04)
        run(sim, NEUTRAL, 6.0)
        home = tuple(math.radians(90) for _ in range(4))
        for u, v in zip(sim.angles, home):
            self.assertAlmostEqual(u, v, places=6)
        self.assertFalse(sim.attached)                # servo signals released after homing, like the firmware
        run(sim, inp(LSX=0.5), 0.04)
        self.assertLess(max(abs(sim.angles[i] - home[i]) for i in range(4)), math.radians(2.0))
        self.assertNotEqual(moved, home)

    def test_save_and_replay(self):
        sim = A.ArmSim(rate=60.0)
        run(sim, inp(A=True), 0.1)
        run(sim, NEUTRAL, 0.1)
        self.assertEqual(len(sim.saved), 1)
        run(sim, inp(LSX=1.0), 2.0)
        run(sim, inp(A=True), 0.1)
        run(sim, NEUTRAL, 0.1)
        self.assertEqual(len(sim.saved), 2)
        run(sim, inp(LSX=-1.0), 2.0)
        run(sim, inp(A=True), 2.0)                    # hold A: replay
        self.assertEqual(sim.mode, "replay")
        run(sim, NEUTRAL, 0.1)
        self.assertEqual(sim.mode, "idle")

    def test_saved_list_is_capped(self):
        sim = A.ArmSim()
        for _ in range(A.MAX_SAVED + 3):
            run(sim, inp(A=True), 0.04)
            run(sim, NEUTRAL, 0.04)
        self.assertEqual(len(sim.saved), A.MAX_SAVED)
        self.assertIn("full", sim.status)

    def test_stick_directions_match_the_firmware(self):
        # raw pygame axes are used as they are: stick up (-1) lowers y and z, like SticksToXYZ in the firmware
        sim = A.ArmSim(rate=1000)
        x0, y0, z0, _ = sim.pose()
        run(sim, inp(LSY=-1.0), 1.0)
        self.assertLess(sim.pose()[1], y0)
        sim = A.ArmSim(rate=1000)
        run(sim, inp(RSY=-1.0), 1.0)
        self.assertLess(sim.pose()[2], z0)


class Calibration(unittest.TestCase):
    def test_inverted_servo_mapping(self):
        sim = A.ArmSim(limits=[[0, 180], [180, 0], [0, 180], [0, 180]])
        cmds = sim.servo_commands()
        self.assertEqual(cmds, [90, 90, 90, 90])
        sim.angles = (math.radians(30),) * 4
        self.assertEqual(sim.servo_commands(), [30, 150, 30, 30])

    def test_set_calibration_validates(self):
        sim = A.ArmSim()
        with self.assertRaises(ValueError):
            sim.set_calibration([[0, 180]] * 3, [90] * 4)
        with self.assertRaises(ValueError):
            sim.set_calibration([[0, 181]] * 4, [90] * 4)
        with self.assertRaises(ValueError):
            sim.set_calibration([[0, 180]] * 4, [90, 90, 90, 90.5])
        sim.set_calibration([[10, 170]] * 4, [80, 90, 100, 90])
        self.assertEqual(sim.homes, [80, 90, 100, 90])
        self.assertEqual(sim.angles, tuple(math.radians(v) for v in [80, 90, 100, 90]))

    def test_wire_roundtrip(self):
        s = {"lx": 0.5, "ly": -0.25, "rx": 0.0, "ry": 1.0, "lt": 0.3, "rt": 0.0,
             "buttons": {"a": True, "b": False, "x": False, "y": True, "lb": False}, "dpad": [0, 0]}
        w = A.wire_from_internal(s)
        self.assertEqual(w, {"LSX": 0.5, "LSY": -0.25, "RSX": 0.0, "RSY": 1.0, "A": True, "B": False, "X": False, "Y": True})
        back = A.internal_from_wire(w)
        self.assertEqual((back["lx"], back["ly"], back["ry"]), (0.5, -0.25, 1.0))
        self.assertTrue(back["buttons"]["a"] and back["buttons"]["y"])


if __name__ == "__main__":
    unittest.main()


class TapAndHoldTest(unittest.TestCase):
    """A tap saves a position; holding A to replay does not leave an extra save behind."""

    def _run(self, sim, a, seconds, dt=0.02):
        pkt = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": a, "B": False, "X": False, "Y": False}
        for _ in range(int(seconds / dt)):
            sim.step(pkt, dt)

    def test_tap_saves_once(self):
        for model, patch in (("improved", False), ("stock", False), ("stock", True)):
            sim = A.ArmSim(model=model, patch=patch)
            self._run(sim, True, 0.2)
            self._run(sim, False, 0.2)
            self.assertEqual(len(sim.saved), 1, (model, patch))

    def test_hold_to_replay_does_not_add_a_save(self):
        for model, patch in (("improved", False), ("stock", True)):
            sim = A.ArmSim(model=model, patch=patch)
            self._run(sim, True, 0.1)
            self._run(sim, False, 0.1)
            self._run(sim, True, 1.2)                         # hold: replay starts after 0.75 s
            self.assertEqual(len(sim.saved), 1, (model, patch))
            self.assertEqual(sim.mode if model == "improved" else ("replay" if sim._replaying else "x"), "replay", (model, patch))

    def test_stock_hold_still_saves_the_start_position(self):
        sim = A.ArmSim(model="stock", patch=False)               # the unpatched firmware's behaviour, kept as it is
        self._run(sim, True, 0.1)
        self._run(sim, False, 0.1)
        self._run(sim, True, 1.2)
        self.assertEqual(len(sim.saved), 2)

    def test_hold_with_nothing_saved_before_does_not_replay(self):
        sim = A.ArmSim()
        self._run(sim, True, 1.2)
        self.assertEqual(len(sim.saved), 1)
        self.assertNotEqual(sim.mode, "replay")


class LatchedReplayTest(unittest.TestCase):
    """Play saved starts a replay that needs no button held, and stop ends it."""

    def test_replay_keeps_going_without_a_in_every_model(self):
        for model, patch in (("improved", False), ("stock", False), ("stock", True)):
            sim = A.ArmSim(model=model, patch=patch)
            idle = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}
            tap = dict(idle, A=True)
            for _ in range(5):
                sim.step(tap, 0.02)
            for _ in range(20):
                sim.step(idle, 0.02)
            sim._t_pos = (sim._t_pos if model == "stock" else None) or (0, 0, 0, 0)
            if model != "stock":
                sim.saved.append((0.0, 240.0, 80.0, -90.0))
            else:
                sim.saved.append((0.0, 240.0, 80.0, -90.0))
            self.assertTrue(sim.start_replay(), (model, patch))
            for _ in range(int(6 / 0.02)):                      # six seconds with no button held
                sim.step(idle, 0.02)
            self.assertTrue(sim.replaying, (model, patch))
            sim.stop_replay()
            self.assertFalse(sim.replaying, (model, patch))
            n = len(sim.saved)
            sim.step(idle, 0.02)
            self.assertEqual(len(sim.saved), n)

    def test_nothing_saved_does_not_start(self):
        sim = A.ArmSim()
        self.assertFalse(sim.start_replay())
        self.assertFalse(sim.replaying)


class OneOffRunTest(unittest.TestCase):
    def test_once_sends_each_position_once_and_never_starts_a_second_lap(self):
        sim = A.ArmSim(model="stock", patch=True)
        idle = {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}
        sim.saved[:] = [(0.0, 240.0, 80.0, 0.0), (0.0, 240.0, 40.0, 0.0), (120.0, 180.0, 80.0, 0.0)]
        sim.start_replay(wait_arrival=True, once=True)
        sent = []
        for _ in range(int(40 / 0.02)):
            before = sim.replay_idx
            sim.step(idle, 0.02)
            if sim.replay_idx != before:
                sent.append(sim.replay_idx)
        self.assertEqual(sent, [1, 2, 3])                      # three targets, then it stays at the last one
        x, y, z, _ = sim.pose()
        self.assertLess(abs(x - 120) + abs(y - 180) + abs(z - 80), 3.0)
