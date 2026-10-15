"""HTTPS for the phone link, with a certificate the hub makes for itself.

Browsers only allow the microphone on https:// pages (or localhost), so the Chat page's Mic button cannot work on the plain
http:// phone link. With --lan the hub therefore also listens on port+1 over HTTPS. The certificate is self-signed and made
on this PC (cryptography), for this PC's addresses; it lives in the git-ignored .hub_cert folder, together with its private key.
The phone shows a warning the first time: check that the fingerprint printed by the hub matches, then continue.
"""
import datetime
import hashlib
import ipaddress
import socket
import ssl
from http.server import ThreadingHTTPServer
from pathlib import Path

VALID_DAYS = 365
RENEW_DAYS = 30


def local_ips(extra=()):
    """This PC's IPv4 addresses (every adapter), plus the ones given: the certificate names each of them."""
    found = {"127.0.0.1", *extra}
    try:
        found.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    return sorted(ip for ip in found if ip)


def fingerprint(cert_path):
    der = ssl.PEM_cert_to_DER_cert(Path(cert_path).read_text())
    h = hashlib.sha256(der).hexdigest().upper()
    return ":".join(h[i:i + 2] for i in range(0, len(h), 2))


def ensure_cert(folder, ips, now=None):
    """(cert.pem, key.pem) paths. Made again when the set of addresses changes (a new WiFi gives a new address) or it is close to running out."""
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    cert_p, key_p, hosts_p = folder / "cert.pem", folder / "key.pem", folder / "hosts.txt"
    now = now or datetime.datetime.now(datetime.timezone.utc)
    names = ",".join(sorted(set(ips)))
    try:
        if hosts_p.read_text() == names and cert_p.exists() and key_p.exists():
            cert = x509.load_pem_x509_certificate(cert_p.read_bytes())
            if cert.not_valid_after_utc - now > datetime.timedelta(days=RENEW_DAYS):
                return str(cert_p), str(key_p)
    except (OSError, ValueError):
        pass
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "roobot-arm hub (self-signed)")])
    san = [x509.DNSName("localhost")] + [x509.IPAddress(ipaddress.ip_address(ip)) for ip in sorted(set(ips))]
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(now - datetime.timedelta(hours=1))
            .not_valid_after(now + datetime.timedelta(days=VALID_DAYS))
            .add_extension(x509.SubjectAlternativeName(san), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .sign(key, hashes.SHA256()))
    key_p.write_bytes(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    cert_p.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    hosts_p.write_text(names)
    return str(cert_p), str(key_p)


class TlsServer(ThreadingHTTPServer):
    """The same server over TLS. The handshake is done in the request's own thread, not in the accept loop, so a phone that connects
    and then stalls (or refuses the certificate half way) cannot hold up everyone else."""
    handshake_timeout = 10

    def __init__(self, address, handler, cert_path, key_path):
        super().__init__(address, handler)
        self.ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        self.ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        self.ctx.load_cert_chain(cert_path, key_path)
        self.daemon_threads = True

    def get_request(self):
        sock, addr = super().get_request()
        return self.ctx.wrap_socket(sock, server_side=True, do_handshake_on_connect=False), addr

    def process_request_thread(self, request, client_address):
        try:
            request.settimeout(self.handshake_timeout)
            request.do_handshake()
            request.settimeout(None)
        except (ssl.SSLError, OSError):                 # a client that did not trust the certificate, or went away
            self.shutdown_request(request)
            return
        super().process_request_thread(request, client_address)
