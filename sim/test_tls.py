import datetime
import http.client
import os
import socket
import ssl
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from http.server import BaseHTTPRequestHandler

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
import tls  # noqa: E402
from cryptography import x509  # noqa: E402

IPS = ["127.0.0.1", "192.168.1.5"]


class Hello(BaseHTTPRequestHandler):
    def do_GET(self):
        body = b"hello"
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


class CertTest(unittest.TestCase):
    def test_names_every_address_and_lasts_a_year(self):
        with tempfile.TemporaryDirectory() as d:
            cert_p, key_p = tls.ensure_cert(d, IPS)
            cert = x509.load_pem_x509_certificate(Path(cert_p).read_bytes())
            san = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
            self.assertEqual(sorted(str(i) for i in san.get_values_for_type(x509.IPAddress)), IPS)
            self.assertEqual(san.get_values_for_type(x509.DNSName), ["localhost"])
            days = (cert.not_valid_after_utc - cert.not_valid_before_utc).days
            self.assertTrue(364 <= days <= 366)
            self.assertIn("BEGIN PRIVATE KEY", Path(key_p).read_text())
            self.assertEqual(len(tls.fingerprint(cert_p).split(":")), 32)

    def test_reused_until_the_addresses_change_or_it_runs_out(self):
        with tempfile.TemporaryDirectory() as d:
            serial = lambda: x509.load_pem_x509_certificate(Path(d, "cert.pem").read_bytes()).serial_number
            tls.ensure_cert(d, IPS)
            first = serial()
            tls.ensure_cert(d, list(reversed(IPS)))
            self.assertEqual(serial(), first)                                       # same addresses, any order: kept
            tls.ensure_cert(d, IPS + ["10.0.0.7"])
            second = serial()
            self.assertNotEqual(second, first)                                      # a new WiFi gave a new address: made again
            tls.ensure_cert(d, IPS + ["10.0.0.7"], now=datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(days=345))
            self.assertNotEqual(serial(), second)                                   # within 30 days of the end: renewed

    def test_local_ips_always_has_loopback(self):
        self.assertIn("127.0.0.1", tls.local_ips())
        self.assertIn("10.1.2.3", tls.local_ips(["10.1.2.3"]))


class ServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.TemporaryDirectory()
        cls.cert_p, cls.key_p = tls.ensure_cert(cls.dir.name, IPS)
        tls.TlsServer.handshake_timeout = 2
        cls.srv = tls.TlsServer(("127.0.0.1", 0), Hello, cls.cert_p, cls.key_p)
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.ctx = ssl.create_default_context(cafile=cls.cert_p)                    # the self-signed certificate is its own trust anchor

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()
        cls.dir.cleanup()

    def get(self):
        c = http.client.HTTPSConnection("127.0.0.1", self.port, context=self.ctx, timeout=5)
        c.request("GET", "/")
        r = c.getresponse()
        out = (r.status, r.read())
        c.close()
        return out

    def test_serves_https_to_a_client_that_trusts_the_certificate(self):
        self.assertEqual(self.get(), (200, b"hello"))

    def test_a_client_that_connects_and_says_nothing_does_not_block_others(self):
        stalled = socket.create_connection(("127.0.0.1", self.port))
        try:
            self.assertEqual(self.get(), (200, b"hello"))
        finally:
            stalled.close()

    def test_plain_http_on_the_https_port_is_dropped_and_the_server_carries_on(self):
        s = socket.create_connection(("127.0.0.1", self.port))
        s.sendall(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n")
        s.settimeout(5)
        data = b""
        try:
            data = s.recv(100)                                                            # the server hangs up (or sends a TLS alert): it must not answer in plain text
        except OSError:
            pass
        self.assertNotIn(b"HTTP/1", data)
        s.close()
        self.assertEqual(self.get(), (200, b"hello"))

    def test_a_client_that_does_not_trust_the_certificate_is_refused_and_the_server_carries_on(self):
        with self.assertRaises(ssl.SSLError):
            c = http.client.HTTPSConnection("127.0.0.1", self.port, context=ssl.create_default_context(), timeout=5)
            c.request("GET", "/")
            c.getresponse()
        self.assertEqual(self.get(), (200, b"hello"))


if __name__ == "__main__":
    unittest.main()
