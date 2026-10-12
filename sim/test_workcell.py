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


class SavedMagnetTest(unittest.TestCase):
    def test_magnet_state_is_saved_with_the_position_and_applied_in_replay(self):
        w = Workcell()
        saved = []
        saved.append((0.0, 240.0, 8.0, -90.0)); w.track_saved(saved, False, 0, (0, 240, 8))        # saved with the magnet off
        w.set_magnet(True)
        saved.append((0.0, 240.0, 8.0, -90.0)); w.track_saved(saved, False, 0, (0, 240, 8))        # same spot, magnet on
        saved.append((222.0, 60.0, 40.0, -90.0)); w.track_saved(saved, False, 0, (222, 60, 40))     # over the tray, still on
        w.set_magnet(False)
        saved.append((222.0, 60.0, 40.0, -90.0)); w.track_saved(saved, False, 0, (222, 60, 40))    # and released
        self.assertEqual(w.saved_magnet, [False, True, True, False])
        w.set_magnet(False)
        # replay: the arm aims at position i (replay_idx is the next one), and the state applies on arrival
        seen = []
        for i, pos in enumerate(saved):
            idx_next = (i + 1) % len(saved)
            w.track_saved(saved, True, idx_next, (pos[0] + 50, pos[1], pos[2]))                   # still travelling
            before = w.magnet
            w.track_saved(saved, True, idx_next, pos[:3])                                         # arrived
            seen.append((before, w.magnet))
        self.assertEqual([m for _, m in seen], [False, True, True, False])

    def test_undone_and_cleared_saves_drop_their_magnet_state(self):
        w = Workcell()
        saved = [(0, 0, 0, 0)]
        w.track_saved(saved, False, 0, (0, 0, 0))
        saved.pop()
        w.track_saved(saved, False, 0, (0, 0, 0))
        self.assertEqual(w.saved_magnet, [])


