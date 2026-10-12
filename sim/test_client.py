"""Tests for the controller profiles (Xbox / PS5). Run:  python -m unittest sim.test_client -v"""
import unittest

import controller_client as cc


class FakePygame:
    class event:
        @staticmethod
        def pump():
            pass


class FakeJoy:
    """A joystick reporting raw SDL-style indices."""

    def __init__(self, axes=None, buttons=(), hats=0, hat=(0, 0), n_axes=6, n_buttons=17):
        self.ax = list(axes or [0.0] * n_axes) + [0.0] * (n_axes - len(axes or []))
        self.pressed = set(buttons)
        self.hats, self.hat = hats, hat
        self.n_buttons = n_buttons

    def get_numaxes(self): return len(self.ax)
    def get_numbuttons(self): return self.n_buttons
    def get_numhats(self): return self.hats
    def get_axis(self, i): return self.ax[i]
    def get_button(self, i): return 1 if i in self.pressed else 0
    def get_hat(self, i): return self.hat


class Profiles(unittest.TestCase):
    def tearDown(self):
        cc.select_profile("xbox")

    def test_detection_by_name(self):
        for name, want in [("Xbox One Controller", "xbox"), ("Xbox Wireless Controller", "xbox"),
                           ("PS5 Controller", "ps5"), ("DualSense Wireless Controller", "ps5"),
                           ("Wireless Controller", "ps5"), ("Sony Interactive Entertainment Wireless Controller", "ps5"),
                           ("Some Generic Gamepad", "xbox"), ("", "xbox")]:
            self.assertEqual(cc.profile_name_for(name), want, name)

    def test_select_profile(self):
        self.assertEqual(cc.select_profile("auto", "DualSense Wireless Controller"), "ps5")
        self.assertEqual(cc.select_profile("xbox", "DualSense Wireless Controller"), "xbox")   # explicit wins
        with self.assertRaises(ValueError):
            cc.select_profile("n64")

    def test_ps5_face_buttons_and_sticks_reach_the_wire(self):
        cc.select_profile("ps5")
        js = FakeJoy(axes=[0.5, -1.0, 0.25, 0.0, -1.0, -1.0], buttons={0, 1, 3})        # Cross, Circle, Triangle
        wire = cc.to_wire(cc.read_state(js, FakePygame))
        self.assertEqual(wire["A"], True)      # Cross
        self.assertEqual(wire["B"], True)      # Circle: go home
        self.assertEqual(wire["X"], False)     # Square not pressed
        self.assertEqual(wire["Y"], True)      # Triangle
        self.assertGreater(wire["LSX"], 0)
        self.assertEqual(wire["LSY"], -1.0)
        self.assertGreater(wire["RSX"], 0)

    def test_ps5_shoulders_and_menu_buttons_are_not_the_xbox_ones(self):
        cc.select_profile("ps5")
        st = cc.read_state(FakeJoy(buttons={9, 10, 4, 6, 7, 8}), FakePygame)
        b = st["buttons"]
        self.assertTrue(all(b[k] for k in ("lb", "rb", "back", "start", "ls", "rs")))
        self.assertFalse(any(b[k] for k in ("a", "b", "x", "y")))
        cc.select_profile("xbox")
        st = cc.read_state(FakeJoy(buttons={4}), FakePygame)
        self.assertTrue(st["buttons"]["lb"] and not st["buttons"]["back"])                # index 4 is LB on an Xbox pad

    def test_ps5_triggers_rest_at_minus_one(self):
        cc.select_profile("ps5")
        st = cc.read_state(FakeJoy(axes=[0, 0, 0, 0, -1.0, 1.0]), FakePygame)
        self.assertEqual((st["lt"], st["rt"]), (0.0, 1.0))

    def test_dpad_from_hat_or_from_buttons(self):
        cc.select_profile("ps5")
        self.assertEqual(cc.read_state(FakeJoy(hats=1, hat=(-1, 1)), FakePygame)["dpad"], [-1, 1])
        self.assertEqual(cc.read_state(FakeJoy(buttons={11, 14}), FakePygame)["dpad"], [1, 1])     # up + right
        self.assertEqual(cc.read_state(FakeJoy(buttons={12, 13}), FakePygame)["dpad"], [-1, -1])   # down + left
        cc.select_profile("xbox")
        self.assertEqual(cc.read_state(FakeJoy(buttons={11}), FakePygame)["dpad"], [0, 0])

    def test_xbox_unchanged(self):
        cc.select_profile("xbox")
        wire = cc.to_wire(cc.read_state(FakeJoy(axes=[0, 0, 0, 0, 0, 0], buttons={0, 2}), FakePygame))
        self.assertEqual((wire["A"], wire["B"], wire["X"], wire["Y"]), (True, False, True, False))

    def test_short_controllers_do_not_crash(self):
        cc.select_profile("ps5")
        st = cc.read_state(FakeJoy(axes=[0.0] * 4, n_buttons=4), FakePygame)    # fewer axes/buttons than the profile expects
        self.assertEqual(st["lt"], 0.0)


if __name__ == "__main__":
    unittest.main()
