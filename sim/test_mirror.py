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
        self.assertEqual(serial_link.encode_pos([90, 45.123, 10, 180], 7), b"@90.00,45.12,10.00,180.00,7*" + self.xor("90.00,45.12,10.00,180.00,7") + b"\n")

    @staticmethod
    def xor(payload):
        c = 0
        for ch in payload.encode("ascii"):
            c ^= ch
        return ("%02X" % c).encode("ascii")

    def test_a_damaged_frame_never_passes_the_checksum(self):
        """The board refuses a frame whose checksum does not match, so a dropped or changed byte cannot read as a valid but wrong pose."""
        line = serial_link.encode_pose([-40, 250, 200, 0], 17).decode("ascii").rstrip("\n")
        body, _, ck = line[1:].partition("*")
        self.assertEqual(serial_link.checksum(body), ck)
        for i in range(len(body)):
            self.assertNotEqual(serial_link.checksum(body[:i] + body[i + 1:]), ck, "dropping byte %d went unnoticed" % i)
            for ch in "0123456789.-,":
                if ch != body[i]:
                    self.assertNotEqual(serial_link.checksum(body[:i] + ch + body[i + 1:]), ck, "changing byte %d went unnoticed" % i)

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
        self.assertIn("constrain(mirrorUsOf(d[j]), mirrorUsAt0, mirrorUsAt180)", self.src)
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
        self.assertIn("for (int n = 0; n < 4; n++) posGoalUs[n] = mirrorUsOf(90.0f);", self.src)   # home = 90 degrees, like the virtual arm
        self.assertIn("} else if (homing && stickMoved) {", self.src)
        self.assertRegex(self.src, r"if \(homing\) \{\s*homing = false;")                  # fail-safe path stops homing too
        self.assertIn("min(POS_MAX_DEG_S, degPerS)", self.src)                              # homing never exceeds the cap or the jog speed setting

    def test_mirror_pulses_follow_sams_servo_mapping(self):
        self.assertIn("float mirrorUsAt0 = 500.0f, mirrorUsAt180 = 2400.0f;", self.src)       # ESP32Servo attach(pin, 500, 2400) in UWE-RArm
        self.assertIn("float mirrorUsOf(float deg) { return mirrorUsAt0 + deg * (mirrorUsAt180 - mirrorUsAt0) / 180.0f; }", self.src)
        us = lambda deg: 500.0 + deg * (2400.0 - 500.0) / 180.0                              # the same formula, as Sam writes it
        self.assertEqual((us(0), us(180), round(us(90))), (500.0, 2400.0, 1450))
        self.assertIn("!pulse needs lo,hi", self.src)                                         # and it can be changed without a reflash
        self.assertIn("lo >= 400 && hi <= 2600 && hi - lo >= 600", self.src)

    def test_mirror_speeds_use_the_mirror_scale(self):
        self.assertIn("(posMode ? (mirrorUsAt180 - mirrorUsAt0) / 180.0f : usPerDeg)", self.src)

    def test_leaving_mirror_mode_cannot_make_a_joint_jump_into_the_jog_window(self):
        self.assertIn("min((float)PULSE_MIN_US, curUs[n]), max((float)PULSE_MAX_US, curUs[n])", self.src)

    def test_the_channel_map_is_validated(self):
        self.assertIn("a != b && a != c && a != d && b != c && b != d && c != d", self.src)
        self.assertIsNotNone(re.search(r'jointMap\[4\] = \{3, 2, 1, 0\}', self.src))



class PoseFramesTest(unittest.TestCase):
    """Mirror mode with the inverse kinematics on the board: the hub sends the tool-tip pose and the real arm's calibration."""
    REAL = ([[11, 180], [0, 115], [161, 21], [0, 180]], [95, 115, 27, 127])
    CAL = "!cal 11,180,0,115,161,21,0,180,95,115,27,127\n"

    def test_frame_and_calibration_formats(self):
        self.assertEqual(serial_link.encode_pose([0, 200, 100.5, 0], 3), b"#0.00,200.00,100.50,0.00,3*" + FramesTest.xor("0.00,200.00,100.50,0.00,3") + b"\n")
        self.assertEqual(serial_link.cal_line(*self.REAL) + "\n", self.CAL)

    def test_poses_are_paced_like_angles(self):
        link = linked()
        pose = [0, 200, 100, 0]
        link.update_pose(pose, now=10.00)
        link.update_pose(pose, now=10.02)                      # nothing changed, too soon
        self.assertEqual(len(link.ser.out), 1)
        link.update_pose([0, 200, 101, 0], now=10.03)          # moved but only 30 ms after the last frame
        self.assertEqual(len(link.ser.out), 1)
        link.update_pose([0, 200, 101, 0], now=10.05)
        self.assertTrue(link.ser.out[-1].startswith("#0.00,200.00,101.00,0.00,"))
        link.update_pose([0, 200, 101, 0], now=10.16)          # heartbeat
        self.assertEqual(len(link.ser.out), 3)

    def test_the_board_must_say_it_has_the_calibration_before_poses_are_used(self):
        link = linked()
        self.assertFalse(link.ik_ready)                        # an older sketch never says
        link.ser.in_waiting = 1
        link.ser.read = lambda n: b"mirror ik: supported, calibration missing (the board runs Sam's inverse kinematics)\n"
        link.service(now=5.0)
        self.assertEqual(link.board_ik, "missing")
        self.assertFalse(link.ik_ready)
        link.ser.read = lambda n: b"mirror ik: supported, calibration set (the board runs Sam's inverse kinematics)\n"
        link.service(now=5.1)
        self.assertTrue(link.ik_ready)

    def test_a_board_that_lost_its_calibration_gets_it_again(self):
        link = linked()
        link.set_calibration(*self.REAL)
        self.assertEqual(link.ser.out[-1], self.CAL)           # given at once when the port is open
        link.ser.out.clear()
        link.ser.in_waiting = 1
        link.ser.read = lambda n: b"mirror ik: supported, calibration missing\n"                # e.g. the board reset
        link.service(now=7.0)
        self.assertEqual(link.ser.out, [self.CAL])

    def test_without_a_calibration_nothing_is_sent_to_a_board_that_lost_it(self):
        link = linked()
        link.ser.in_waiting = 1
        link.ser.read = lambda n: b"mirror ik: supported, calibration missing\n"
        link.service(now=7.0)
        self.assertEqual(link.ser.out, [])


class SketchIkTest(unittest.TestCase):
    """What the sketch does with pose frames (the maths itself is tested natively in test_ik_board.py)."""
    @classmethod
    def setUpClass(cls):
        with open(os.path.join(ROOT, "arduino", "arm_receiver", "arm_receiver.ino"), encoding="utf-8") as f:
            cls.src = f.read()

    def test_pose_frames_feed_the_same_follower_as_angle_frames(self):
        self.assertIn("serBuf[0] == '@' || serBuf[0] == '#'", self.src)
        self.assertIn("serBuf[0] == '@' ? parsePosLine(serBuf) : parsePoseLine(serBuf)", self.src)
        self.assertRegex(self.src, r"posGoalUs\[jointMap\[j\]\] = constrain\(mirrorUsOf\(d\[j\]\), mirrorUsAt0, mirrorUsAt180\);")   # pulse clamp and joint map: unchanged

    def test_a_pose_it_cannot_use_is_refused_and_the_arm_keeps_its_goal(self):
        self.assertIn("pose out of range, refused", self.src)
        self.assertIn("no calibration yet (send !cal), pose refused", self.src)
        self.assertIn("outside the arm's reach or limits, refused", self.src)
        self.assertIn("if (!mirrorIk.known) return why(", self.src)

    def test_a_new_calibration_cannot_move_a_following_arm(self):
        self.assertIn("if (posMode && serialActive()) {", self.src)
        self.assertIn("!cal refused: the arm is following the hub", self.src)

    def test_the_board_announces_that_it_can_do_this(self):
        self.assertIn("mirror ik: supported, calibration %s", self.src)
        self.assertIsNotNone(serial_link.IK_RE.search("mirror ik: supported, calibration set (the board runs Sam's inverse kinematics)"))


if __name__ == "__main__":
    unittest.main()
