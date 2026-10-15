"""The servo-mapping check: working out the pulse range from two measured angles, and the hub holding one joint at a commanded angle."""
import os
import sys
import threading
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import serial_link  # noqa: E402
from sim import servo_map as M  # noqa: E402
from sim.test_mirror import linked  # noqa: E402


def servo(lo, hi):
    """A pretend servo whose true physical angle follows a straight line of pulse: lo us is 0 degrees, hi us is 180."""
    return lambda us: (us - lo) / (hi - lo) * 180.0


class FitTest(unittest.TestCase):
    def measure(self, true_servo, board, a=45.0, b=135.0):
        return true_servo(M.pulse_of(a, board)), true_servo(M.pulse_of(b, board))

    def test_servos_that_follow_sams_mapping_measure_as_commanded_and_fit_sams_range(self):
        ma, mb = self.measure(servo(500, 2400), M.SAM)
        self.assertEqual((round(ma, 6), round(mb, 6)), (45.0, 135.0))
        f = M.fit_pulse_range(45, ma, 135, mb, M.SAM)
        self.assertEqual((f["lo"], f["hi"], f["closest"]), (500, 2400, "sam"))

    def test_servos_that_follow_the_earlier_guess_are_found_even_while_the_board_uses_sams(self):
        ma, mb = self.measure(servo(1000, 2000), M.SAM)            # the arm really wants 1000..2000, the board sends Sam's 500..2400
        self.assertGreater(abs(ma - 45.0), 30)                      # and it visibly misses the model: that is what the person would see
        f = M.fit_pulse_range(45, ma, 135, mb, M.SAM)
        self.assertEqual((f["lo"], f["hi"], f["closest"]), (1000, 2000, "earlier"))

    def test_any_other_straight_servo_is_recovered_too(self):
        for lo, hi in ((600, 2300), (544, 2400), (800, 2200)):
            ma, mb = self.measure(servo(lo, hi), (1000.0, 2000.0))
            f = M.fit_pulse_range(45, ma, 135, mb, (1000.0, 2000.0))
            self.assertAlmostEqual(f["lo"], lo, delta=1)
            self.assertAlmostEqual(f["hi"], hi, delta=1)

    def test_a_joint_that_turns_the_wrong_way_is_pointed_at_the_limits(self):
        with self.assertRaises(ValueError) as cm:
            M.fit_pulse_range(45, 135.0, 135, 45.0, M.SAM)
        self.assertIn("reverse its limits", str(cm.exception))

    def test_nonsense_measurements_are_refused(self):
        with self.assertRaises(ValueError):
            M.fit_pulse_range(45, 80.0, 135, 80.0, M.SAM)            # it did not move
        with self.assertRaises(ValueError):
            M.fit_pulse_range(45, 44.0, 135, 46.0, M.SAM)            # would need a huge pulse span: outside what the board takes
        with self.assertRaises(ValueError):
            M.fit_pulse_range(45, 40.0, 45, 90.0, M.SAM)             # the same commanded angle twice


class BoardReportTest(unittest.TestCase):
    def test_the_hub_learns_the_boards_pulse_range_from_its_log(self):
        link = linked()
        lines = []
        link.on_line = lines.append
        link._rx = ""

        class Ser:
            in_waiting = 1

            def read(self, n):
                return b"mirror pulses: 0 deg = 600 us, 180 deg = 2300 us, 90 deg = 1450 us (Sam's firmware: 500 / 2400)\n"
        link.ser = Ser()
        link._drain()
        self.assertEqual(link.board_pulse, (600.0, 2300.0))


class HubHoldsAJointTest(unittest.TestCase):
    def test_one_joint_is_held_at_the_commanded_angle_while_the_rest_follow_the_virtual_arm(self):
        import hub as H
        h = H.Hub("http://127.0.0.1:1")
        h.link = linked()
        threading.Thread(target=h._pump, daemon=True).start()
        try:
            with self.assertRaises(ValueError):
                h.mirror_action({"action": "angle", "joint": 2, "deg": 45})          # mirror is off, the output is SAFE
            h.mirror_action({"action": "on"})
            h.set_armed(True)
            for bad in ({"joint": 0, "deg": 45}, {"joint": 2, "deg": 10}, {"joint": 2, "deg": 170}):
                with self.assertRaises(ValueError):
                    h.mirror_action(dict(bad, action="angle"))                     # a joint 1 to 4, 30 to 150 degrees
            h.mirror_action({"action": "angle", "joint": 3, "deg": 45})
            time.sleep(0.4)
            sent = [x for x in h.link.ser.out if x.startswith("@")]
            self.assertTrue(sent)
            last = [float(v) for v in sent[-1][1:].split(",")[:4]]
            self.assertEqual(last, [90.0, 90.0, 45.0, 90.0])                         # joint 3 at 45, the others at home
            self.assertEqual(h.mirror_status()["angle"], {"joint": 3, "deg": 45.0})
            h.mirror_action({"action": "angle_off"})
            time.sleep(0.4)
            sent = [x for x in h.link.ser.out if x.startswith("@")]
            self.assertEqual([float(v) for v in sent[-1][1:].split(",")[:4]], [90.0] * 4)
            h.mirror_action({"action": "angle", "joint": 2, "deg": 135})
            h.set_armed(False)                                                       # SAFE drops the hold
            self.assertIsNone(h.mirror_status()["angle"])
        finally:
            h.stop.set()

    def test_fit_uses_the_boards_own_range_and_says_which_it_used(self):
        import hub as H
        h = H.Hub("http://127.0.0.1:1")
        h.link = linked()
        h.link.board_pulse = (1000.0, 2000.0)
        out = h.mirror_action({"action": "fit", "cmd_a": 45, "meas_a": 45, "cmd_b": 135, "meas_b": 135})
        self.assertEqual(out["board_used"], [1000.0, 2000.0])
        self.assertEqual((out["fit"]["lo"], out["fit"]["hi"]), (1000, 2000))


if __name__ == "__main__":
    unittest.main()
