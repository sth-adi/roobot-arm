"""Sam's 29 Oct firmware + our firmware-local-backend.patch: the patched tree talks plain http to HOST:PORT, nothing else changes."""
import os
import re
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "firmware", "ours"))
import build_firmware  # noqa: E402


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


class LocalFirmwareTest(unittest.TestCase):
    def setUp(self):
        self.td = tempfile.TemporaryDirectory()
        self.out = build_firmware.build(os.path.join(self.td.name, "t"), "192.168.1.50:8001", 'my "net"', "pw", "user1")
        self.src = os.path.join(self.out, "src")

    def tearDown(self):
        self.td.cleanup()

    def sources(self):
        for d, _, files in os.walk(self.src):
            for f in files:
                if f.endswith((".h", ".cpp")):
                    yield os.path.join(d, f)

    def test_every_request_goes_through_the_chosen_scheme(self):
        for p in self.sources():
            txt = read(p)
            if p.endswith("BackendConnection.h"):
                continue
            self.assertNotIn('"https://" + String(HOST)', txt, p)
            self.assertNotIn("WiFiClientSecure client", txt, p)
        urls = re.findall(r'BACKEND_SCHEME \+ String\(HOST\) \+ "(/[A-Za-z-]+)', "".join(read(p) for p in self.sources()))
        for path in ("/UpdateMovement", "/movement-complete", "/calibration", "/status", "/controller"):
            self.assertIn(path, urls)

    def test_secrets_are_generated_and_escaped(self):
        s = read(os.path.join(self.src, "config", "Secrets.h"))
        self.assertIn("#define BACKEND_PLAIN_HTTP", s)
        self.assertIn('const char* HOST = "192.168.1.50:8001";', s)
        self.assertIn('WIFI_SSID = "my \\"net\\"";', s)
        self.assertIn('WIFI_User = "user1";', s)
        self.assertFalse(os.path.exists(os.path.join(self.src, "config", "SecretsExample.h")))

    def test_his_copy_is_untouched(self):
        sam = os.path.join(ROOT, "firmware", "sam", "firmware", "src", "network", "MovementClient.h")
        self.assertIn('"https://" + String(HOST)', read(sam))

    def test_the_loop_is_faster_and_the_connection_is_kept(self):
        main = read(os.path.join(self.src, "main.cpp"))
        self.assertNotIn("delay(100)", main)                               # the original waited 100 ms after every move
        self.assertIn("BACKEND_IDLE_MS", main)
        moves = read(os.path.join(self.src, "network", "MovementClient.h"))
        self.assertEqual(moves.count("BackendSharedClient()"), 2)          # the packet request and the acknowledgement share one connection
        self.assertNotIn("HTTPClient http;", moves)
        self.assertEqual(moves.count("client.stop()"), 4)                  # a failed request drops it so the next one reconnects

    def test_servos_are_written_in_fractions_of_a_degree(self):
        mv = read(os.path.join(self.src, "motion", "RobotMovement.h"))                # his 29 Oct evening firmware does this itself: nothing of ours is needed
        self.assertIn("writeMicroseconds(", mv)
        self.assertIn("ServoMinPulseUs + commandAngle * (ServoMaxPulseUs - ServoMinPulseUs) / 180.0f", mv)
        self.assertEqual(mv, read(os.path.join(ROOT, "firmware", "sam", "firmware", "src", "motion", "RobotMovement.h")))

    def test_the_completion_is_retried_until_the_backend_has_it(self):
        main = read(os.path.join(self.src, "main.cpp"))
        self.assertIn("while (!SendMovementComplete())", main)      # otherwise one lost POST leaves his backend "moving" for good

    def test_latency_check_drops_the_port(self):
        self.assertIn("pingHost.indexOf(':')", read(os.path.join(self.src, "network", "WifiConnection.h")))

    def test_refuses_to_overwrite(self):
        with self.assertRaises(SystemExit):
            build_firmware.build(self.out, "h:1", "s")


if __name__ == "__main__":
    unittest.main()
