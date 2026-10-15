"""Mirror mode: the real arm follows the virtual arm's joint angles over USB.

No board is needed: the serial port is a fake that records what the hub would write. This checks the hub side (what is sent, when,
and that it stops); what the board does with the frames is in arduino/arm_receiver/arm_receiver.ino (compiled from the hub's Firmware page).
"""
import os
import re
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import serial_link  # noqa: E402
from sim import armsim as A  # noqa: E402


class FakeSerial:
    def __init__(self):
        self.out = []
        self.in_waiting = 0

    def write(self, b):
        self.out.append(b.decode("ascii"))

    def read(self, n):
        return b""

    def close(self):
        pass


def linked():
    link = serial_link.SerialLink(enabled=True)
    link.enabled = True
    link.ser = FakeSerial()
    link.device = "COMFAKE"
    return link


class FramesTest(unittest.TestCase):
    def test_frame_format(self):
        self.assertEqual(serial_link.encode_pos([90, 45.123, 10, 180], 7), b"@90.00,45.12,10.00,180.00,7\n")

    def test_unchanged_positions_are_repeated_as_a_heartbeat_not_every_call(self):
        link = linked()
        degs = [90, 90, 90, 90]
        link.update_positions(degs, now=100.00)
        link.update_positions(degs, now=100.02)      # nothing changed, too soon: nothing sent
        link.update_positions(degs, now=100.05)      # still nothing new, still inside the heartbeat
        self.assertEqual(len(link.ser.out), 1)
        link.update_positions(degs, now=100.12)      # heartbeat: the board must keep hearing from us
        self.assertEqual(len(link.ser.out), 2)

    def test_a_move_is_sent_at_once_but_not_faster_than_25_hz(self):
        link = linked()
        link.update_positions([90, 90, 90, 90], now=10.00)
        link.update_positions([91, 90, 90, 90], now=10.01)      # moved, but only 10 ms after the last frame
        self.assertEqual(len(link.ser.out), 1)
        link.update_positions([91, 90, 90, 90], now=10.05)
        self.assertEqual(len(link.ser.out), 2)
        self.assertTrue(link.ser.out[-1].startswith("@91.00,"))

    def test_nothing_is_sent_without_a_port(self):
        link = serial_link.SerialLink(enabled=False)
        link.update_positions([90, 90, 90, 90], now=1.0)       # must not raise
        self.assertFalse(link.connected)


class ArmDegreesTest(unittest.TestCase):
    def test_float_degrees_agree_with_the_rounded_servo_commands(self):
        sim = A.ArmSim()
        sim.angles = tuple(__import__("math").radians(v) for v in (12.4, 100.6, 33.3, 170.2))
        for deg, cmd in zip(sim.servo_degrees(), sim.servo_commands()):
            self.assertAlmostEqual(deg, cmd, delta=0.5)

    def test_an_inverted_servo_is_mirrored_like_the_firmware_does(self):
        sim = A.ArmSim(limits=[[180, 0], [0, 180], [0, 180], [0, 180]])      # a reversed range marks an inverted servo
        sim.angles = tuple(__import__("math").radians(v) for v in (30, 90, 90, 90))
        self.assertAlmostEqual(sim.servo_degrees()[0], 150.0, delta=0.01)

    def test_degrees_stay_inside_0_to_180_and_inside_the_calibrated_limits(self):
        sim = A.ArmSim(limits=[[20, 160]] * 4)
        sim.angles = tuple(__import__("math").radians(v) for v in (0, 90, 180, 170))
        self.assertEqual([round(v) for v in sim.servo_degrees()], [20, 90, 160, 160])

    def test_home_is_the_centre_of_every_servo(self):
        self.assertEqual(A.ArmSim().servo_degrees(), [90.0] * 4)

    def test_stick_speed_scales_the_jog_and_defaults_to_the_firmware_value(self):
        def idle():
            return {"LSX": 0, "LSY": 0, "RSX": 0, "RSY": 0, "A": False, "B": False, "X": False, "Y": False}

        def moved(v):
            sim = A.ArmSim(model="stock", patch=True, rate=60.0)
            if v:
                sim.set_stick_speed(v)
            w = idle()
            w["LSX"] = 1.0
            for _ in range(100):
                sim.step(w, 0.02)
            return sim.pose()[0]
        self.assertEqual(A.ArmSim().chord_sense, 20.0)                 # the firmware's ChordSense: unchanged unless asked
        self.assertGreater(moved(80), moved(None) * 3)                 # the damper (60 deg/s) does not hold a faster stick back here
        with self.assertRaises(ValueError):
            A.ArmSim().set_stick_speed(1000)


class SketchTest(unittest.TestCase):
    """The board's side, read from the source: the guards that make mirror mode safe must stay in the sketch."""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(ROOT, "arduino", "arm_receiver", "arm_receiver.ino"), encoding="utf-8") as f:
            cls.src = f.read()

    def test_the_board_caps_its_own_speed_and_the_pulse_range(self):
        self.assertRegex(self.src, r"POS_MAX_DEG_S\s*=\s*60")
        self.assertIn("constrain(PULSE_MID_US + (d[j] - 90.0f) * usPerDeg, (float)PULSE_MIN_US, (float)PULSE_MAX_US)", self.src)
        self.assertIn("curUs[n] += constrain(posGoalUs[n] - curUs[n], -maxStep, maxStep)", self.src)

    def test_bad_frames_are_refused_whole(self):
        self.assertIn("if (!(d[j] >= 0.0f && d[j] <= 180.0f)) return false;", self.src)     # also rejects NaN

    def test_it_holds_when_frames_stop_and_ignores_wifi_while_mirroring(self):
        self.assertIn("serial: frames stopped, holding position", self.src)
        self.assertIn("if (serialActive() || posMode) return;", self.src)

    def test_stick_frames_take_over_and_hold(self):
        self.assertRegex(self.src, r"posMode = false;\s*holdPos\(\);")

    def test_b_goes_home_in_jog_mode_and_a_stick_or_lost_input_cancels_it(self):
        self.assertIn("if (i.b && !prevB && !posMode) {", self.src)                       # rising edge of B, jog mode only
        self.assertIn("for (int n = 0; n < 4; n++) posGoalUs[n] = PULSE_MID_US;", self.src)   # home = centre, 90 degrees, like the virtual arm
        self.assertIn("} else if (homing && stickMoved) {", self.src)
        self.assertRegex(self.src, r"if \(homing\) \{\s*homing = false;")                  # fail-safe path stops homing too
        self.assertIn("min(POS_MAX_DEG_S, degPerS)", self.src)                              # homing never exceeds the cap or the jog speed setting

    def test_the_channel_map_is_validated(self):
        self.assertIn("a != b && a != c && a != d && b != c && b != d && c != d", self.src)
        self.assertIsNotNone(re.search(r'jointMap\[4\] = \{3, 2, 1, 0\}', self.src))


if __name__ == "__main__":
    unittest.main()
