import unittest

from sim.workcell import Workcell, BOX_CENTERS, TRAY_CENTER, MAGNET_REACH_MM


def run(w, tip, x, seconds=1.0, dt=0.02):
    for _ in range(int(seconds / dt)):
        w.step(tip, x, dt)


class WorkcellTest(unittest.TestCase):
    def test_start(self):
        w = Workcell()
        self.assertEqual([k["where"] for k in w.snapshot()["keys"]], ["box 1", "box 2", "box 3"])

    def test_pick_from_box_and_drop_on_tray(self):
        w = Workcell()
        cx, cy = BOX_CENTERS[1]
        w.set_magnet(True)
        w.step((cx, cy, 20), False, 0.02)
        self.assertEqual(w.held, 2)
        for t in (0.0, 0.5, 1.0):                                   # carry it across to the tray
            tip = (cx + (TRAY_CENTER[0] - cx) * t, cy + (TRAY_CENTER[1] - cy) * t, 80)
            w.step(tip, False, 0.02)
        self.assertEqual(w.snapshot()["keys"][1]["where"], "on the magnet")
        w.set_magnet(False)
        run(w, tip, False)
        self.assertEqual(w.snapshot()["keys"][1]["where"], "tray")
        self.assertIsNone(w.held)

    def test_out_of_reach_not_picked(self):
        w = Workcell()
        cx, cy = BOX_CENTERS[0]
        w.set_magnet(True)
        w.step((cx, cy, 6 + MAGNET_REACH_MM + 10), False, 0.02)
        self.assertIsNone(w.held)

    def test_x_button_toggles_on_press_only(self):
        w = Workcell()
        for pressed in (True, True, True):
            w.step((0, 0, 300), pressed, 0.02)
        self.assertTrue(w.magnet)
        w.step((0, 0, 300), False, 0.02)
        w.step((0, 0, 300), True, 0.02)
        self.assertFalse(w.magnet)

    def test_dropped_off_table_lands_on_floor_and_reset(self):
        w = Workcell()
        cx, cy = BOX_CENTERS[2]
        w.set_magnet(True)
        w.step((cx, cy, 15), False, 0.02)
        far = (-100, 20, 100)
        w.step(far, False, 0.02)
        w.set_magnet(False)
        run(w, far, False)
        self.assertEqual(w.snapshot()["keys"][2]["where"], "floor")
        w.reset()
        self.assertEqual(w.snapshot()["keys"][2]["where"], "box 3")
        self.assertFalse(w.magnet)


if __name__ == "__main__":
    unittest.main()
