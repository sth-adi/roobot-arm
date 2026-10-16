"""The board over WiFi: the hub listens, a (fake) board connects out with the key, and the link works like a COM port.
Real sockets on this PC; no board, no network."""
import os
import socket
import sys
import threading
import time
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import netlink  # noqa: E402
import serial_link  # noqa: E402

KEY = "sekrit-key-1234"
CAL = ([[11, 180], [0, 115], [161, 21], [0, 180]], [95, 115, 27, 127])


class FakeCom:
    in_waiting = 0

    def __init__(self):
        self.out = []

    def write(self, b):
        self.out.append(b)

    def read(self, n):
        return b""

    def close(self):
        pass


def wait(cond, secs=3.0):
    end = time.time() + secs
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


class Board:
    """A stand-in for the ESP32: connects out to the hub, says hello, and reads what the hub sends."""

    def __init__(self, port, hello="HELLO " + KEY, say=""):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=2)
        self.sock.sendall((hello + "\n" + say).encode())
        self.sock.settimeout(0.05)
        self.got = b""

    def pump(self):
        try:
            while True:
                d = self.sock.recv(4096)
                if not d:
                    return False
                self.got += d
        except socket.timeout:
            return True
        except OSError:
            return False

    def lines(self):
        self.pump()
        return self.got.decode().splitlines()

    def close(self):
        self.sock.close()


class WifiLinkTest(unittest.TestCase):
    def setUp(self):
        self.link = serial_link.SerialLink(enabled=True)
        self.link.enabled = True
        self.logs = []
        self.listener = netlink.BoardListener(self.link, KEY, port=0, bind="127.0.0.1", log=self.logs.append)
        self.port = self.listener.start()
        self.boards = []

    def tearDown(self):
        self.listener.close()
        for b in self.boards:
            b.close()
        with self.link._lock:
            self.link._drop("test over")

    def board(self, **kw):
        b = Board(self.port, **kw)
        self.boards.append(b)
        return b

    def service(self):
        self.link.service(time.time())

    def attached(self):
        return wait(lambda: self.link.device is not None)

    def test_the_board_with_the_key_becomes_the_link_and_gets_the_settings_commands(self):
        self.link.set_calibration(*CAL)
        b = self.board()
        self.assertTrue(self.attached())
        self.assertEqual(self.link.device, "wifi 127.0.0.1")
        self.assertTrue(self.link.connected)
        self.assertTrue(wait(lambda: len(b.lines()) >= 2))
        self.assertEqual(b.lines()[:2], ["!cal 11,180,0,115,161,21,0,180,95,115,27,127", "!show"])     # the calibration first, then "say what you have"
        self.assertEqual(self.listener.snapshot()["connected"], True)

    def test_a_wrong_or_missing_key_is_refused(self):
        for hello in ("HELLO nope", "HELLO", "GET / HTTP/1.1", "HELLO " + KEY[:-1]):
            b = self.board(hello=hello)
            self.assertTrue(wait(lambda: not b.pump()), hello)            # the hub closes it
        self.assertIsNone(self.link.device)
        self.assertGreaterEqual(self.listener.rejected, 4)

    def test_what_the_board_prints_reaches_the_link_and_the_ik_handshake_works(self):
        lines = []
        self.link.on_line = lines.append
        b = self.board(say="mirror ik: supported, calibration missing (the board runs Sam's inverse kinematics)\n")
        self.assertTrue(self.attached())
        self.assertTrue(wait(lambda: (self.service(), self.link.board_ik)[1] == "missing"))
        self.assertFalse(self.link.ik_ready)
        b.sock.sendall(b"mirror ik: supported, calibration set (the board runs Sam's inverse kinematics)\nMIRROR goal(us) 1 2 3 4 | pos(us) 1450 1450 1450 1450\n")
        self.assertTrue(wait(lambda: (self.service(), self.link.ik_ready)[1]))
        self.assertTrue(any(l.startswith("MIRROR goal") for l in lines))

    def test_frames_go_to_the_board_exactly_as_they_would_over_usb(self):
        b = self.board()
        self.assertTrue(self.attached())
        self.link.update_pose([0, 200, 100.5, 0], now=time.time())
        want = serial_link.encode_pose([0, 200, 100.5, 0], self.link.seq).decode().strip()
        self.assertTrue(wait(lambda: want in b.lines()), b.lines())
        self.link.update_positions([90, 90, 90, 90], now=time.time() + 1)
        self.assertTrue(wait(lambda: any(l.startswith("@90.00,90.00,90.00,90.00,") and "*" in l for l in b.lines())), b.lines())

    def test_usb_wins_a_connection_while_a_com_port_is_open_is_closed(self):
        self.link.ser, self.link.device = FakeCom(), "COM5"
        b = self.board()
        self.assertTrue(wait(lambda: not b.pump()))
        self.assertEqual(self.link.device, "COM5")

    def test_no_connection_while_off_or_uploading(self):
        self.link.suspended = True
        b = self.board()
        self.assertTrue(wait(lambda: not b.pump()))
        self.assertIsNone(self.link.device)
        self.link.suspended, self.link.enabled = False, False
        b2 = self.board()
        self.assertTrue(wait(lambda: not b2.pump()))
        self.assertIsNone(self.link.device)

    def test_a_board_that_reconnects_replaces_its_old_connection(self):
        b1 = self.board()
        self.assertTrue(self.attached())
        b2 = self.board()
        self.assertTrue(wait(lambda: not b1.pump()))                          # the old one is closed
        self.assertTrue(wait(lambda: self.link.device == "wifi 127.0.0.1"))   # and the new one is the link (never a moment with neither: attach holds the lock)
        self.assertTrue(wait(lambda: "!show" in b2.lines()))

    def test_a_board_that_goes_away_is_noticed(self):
        b = self.board()
        self.assertTrue(self.attached())
        b.close()
        self.assertTrue(wait(lambda: (self.service(), self.link.device is None)[1]))
        self.assertEqual(self.link.phase, "waiting")

    def test_a_board_that_goes_quiet_is_dropped_so_a_stale_connection_cannot_hold_the_link(self):
        events = []
        self.link.on_event = events.append
        self.board()
        self.assertTrue(self.attached())
        with mock.patch.object(netlink, "SILENT_S", 0.1):
            time.sleep(0.2)
            self.service()
        self.assertIsNone(self.link.device)
        self.assertEqual(len(events), 1)                                       # and the hub says why, so a flapping link can be understood from its log
        self.assertIn("went quiet", events[0])

    def test_the_hub_pings_once_a_second_so_the_board_can_tell_it_is_there(self):
        b = self.board()
        self.assertTrue(self.attached())
        self.link.ser.last_tx = 0.0
        self.service()
        self.assertTrue(wait(lambda: "NET ping" in b.lines()))

    def test_the_key_must_be_long_enough(self):
        with self.assertRaises(ValueError):
            netlink.BoardListener(self.link, "short")


class HubOverWifiTest(unittest.TestCase):
    """The hub's own mirror stream reaches a board that is connected over WiFi, with the IK on the board once it says it has the calibration."""

    def test_mirror_frames_flow_over_the_wifi_link(self):
        import hub as H
        h = H.Hub("http://127.0.0.1:1")
        listener = netlink.BoardListener(h.link, KEY, port=0, bind="127.0.0.1", log=lambda t: None)
        port = listener.start()
        h.sim.set_tool(53.0)
        h.mirror, h.armed = {"on": True}, True                     # armed directly: arming through the API would also tell the real server to forward
        b = Board(port, say="mirror ik: supported, calibration set (the board runs Sam's inverse kinematics)\n")
        t = threading.Thread(target=h._pump, daemon=True)
        try:
            self.assertTrue(wait(lambda: h.link.device is not None))
            t.start()
            self.assertTrue(wait(lambda: any(l.startswith("#") for l in b.lines())), b.lines())
            frame = next(l for l in b.lines() if l.startswith("#"))
            body, _, tail = frame[1:].partition("*")
            self.assertEqual(tail, serial_link.checksum(body))      # the checksum survives the trip
            self.assertEqual(h.mirror_status()["ik"], "board")
        finally:
            h.stop.set()
            t.join(2)
            b.close()
            listener.close()
            with h.link._lock:
                h.link._drop("test over")


if __name__ == "__main__":
    unittest.main()
