"""The sketch's WiFi output queue (arduino/arm_receiver/net_out.h) on a PC, and what the sketch does with the WiFi link (read from its source:
the link itself needs the ESP32). Needs g++ for the queue (skipped without)."""
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from emu.emulator import EXE, compiler  # noqa: E402

SKETCH = os.path.join(ROOT, "arduino", "arm_receiver")


class QueueTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cc = compiler()
        except RuntimeError:
            raise unittest.SkipTest("needs g++")
        cls.dir = tempfile.mkdtemp()
        cls.exe = os.path.join(cls.dir, "net_out" + EXE)
        r = subprocess.run([cc, "-std=c++17", "-I" + SKETCH, os.path.join(HERE, "fixtures", "net_out_harness.cpp"), "-o", cls.exe], capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def run_q(self, script):
        r = subprocess.run([self.exe], input=script, capture_output=True, text=True)
        return r.stdout.splitlines()

    def test_bytes_come_out_in_order_and_only_once(self):
        self.assertEqual(self.run_q("push hello\npush world\ntake 4\ntake 100\ntake 5\nstat\n"),
                         ["took 4:hell", "took 6:oworld", "took 0:", "count 0 dropped 0"])

    def test_when_it_is_full_new_bytes_are_dropped_and_counted_never_waited_for(self):
        out = self.run_q("push 0123456789abcdefXYZ\nstat\ntake 100\n")                  # 16 fit, 3 do not
        self.assertEqual(out, ["count 16 dropped 3", "took 16:0123456789abcdef"])

    def test_it_wraps_around_without_mixing_bytes_up(self):
        out = self.run_q("push 0123456789\ntake 8\npush abcdefghijkl\ntake 100\nstat\n")
        self.assertEqual(out, ["took 8:01234567", "took 14:89abcdefghijkl", "count 0 dropped 0"])

    def test_clear_forgets_everything_queued(self):
        self.assertEqual(self.run_q("push abc\nclear\ntake 5\nstat\n"), ["took 0:", "count 0 dropped 0"])


class SketchWifiTest(unittest.TestCase):
    """What the sketch does with the WiFi link."""
    @classmethod
    def setUpClass(cls):
        with open(os.path.join(SKETCH, "arm_receiver.ino"), encoding="utf-8") as f:
            cls.src = f.read()
        with open(os.path.join(SKETCH, "secrets.example.h"), encoding="utf-8") as f:
            cls.example = f.read()

    def test_it_is_all_optional_and_compiled_out_without_mirror_host(self):
        self.assertIn("#ifdef MIRROR_HOST", self.src)
        self.assertIn("#define Serial Out", self.src)
        self.assertEqual(self.src.count("#define Serial Out"), 1)
        before_tee = self.src.split("#define Serial Out")[0]
        self.assertTrue(before_tee.rstrip().endswith("TeeSerial Out;"))
        self.assertIn("MIRROR_HOST", self.example)                                    # documented for whoever sets it up
        self.assertIn("MIRROR_KEY", self.example)

    def test_the_board_only_connects_to_the_one_host_and_says_the_key_first(self):
        self.assertIn("mirrorNet.connect(MIRROR_HOST, MIRROR_PORT, 300)", self.src)
        self.assertIn('mirrorNet.print("HELLO " MIRROR_KEY', self.src)
        self.assertIn('#error "MIRROR_HOST needs MIRROR_KEY', self.src)

    def test_the_link_never_waits_on_the_network(self):
        self.assertIn("mirrorNet.availableForWrite()", self.src)                     # output goes only into free socket space
        self.assertIn("netOut.take(tmp, min((size_t)room, sizeof(tmp)))", self.src)
        self.assertIn("if (usbBusy || now - lastNetTry < 1500) return;", self.src)    # short, rare connection attempts, none while USB is driving

    def test_a_silent_hub_is_noticed_and_the_arm_holds_like_it_does_for_usb(self):
        self.assertIn("now - lastNetRx > 6000", self.src)
        # frames from the WiFi link go through the same handler as USB, so the same freshness rule (400 ms) and the same checks apply
        self.assertIn("handleLine(netBuf)", self.src)
        self.assertIn("handleLine(serBuf)", self.src)
        self.assertEqual(self.src.count("void handleLine(const String &line) {"), 1)

    def test_pollserial_pumps_the_link_in_every_loop_and_wait(self):
        i = self.src.index("void pollSerial() {")
        self.assertIn("pollNet();", self.src[i:i + 120])


if __name__ == "__main__":
    unittest.main()
