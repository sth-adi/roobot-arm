"""A serial port over WiFi, so the board can be driven without the USB cable.

The hub (on this PC) listens on a TCP port. The board, once it has the MIRROR_* lines in its secrets.h, connects OUT to it and says
"HELLO <key>". From then on the two exchange the same lines as over the COM port: "@", "#", "$" and "!" lines to the board, everything the
board prints back. serial_link.SerialLink uses the connection exactly like a COM port, so mirror mode, the board log, the settings commands
and the calibration all work unchanged. The board only ever connects to the one host named in its secrets.h, and the hub only accepts a
connection that starts with the key. The wire is not encrypted: this is for a home network you trust, and the arm still only moves while the
output is ARMED and holds still when frames stop for 0.4 s.
"""
import hmac
import select
import socket
import threading
import time

DEFAULT_PORT = 8767
SILENT_S = 5.0          # the board pings every second: this long without a byte from it and the connection is treated as dead
HELLO_S = 2.0           # a new connection has this long to say who it is


class SocketPort:
    """Looks like the pyserial port SerialLink expects: in_waiting, read, write, close. Never blocks for long."""

    def __init__(self, sock, peer, first=b""):
        sock.settimeout(0.05)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        self.sock, self.peer = sock, peer
        self._buf = bytearray(first)
        self._closed = False
        self.last_rx = time.time()
        self.last_tx = 0.0

    def _fill(self):
        if self._closed:
            raise OSError("the board closed the connection")
        try:
            while select.select([self.sock], [], [], 0)[0]:
                data = self.sock.recv(4096)
                if not data:
                    self._closed = True
                    raise OSError("the board closed the connection")
                self._buf += data
                self.last_rx = time.time()
        except (ConnectionError, TimeoutError, ValueError) as e:
            self._closed = True
            raise OSError(str(e))

    @property
    def dead(self):
        return self._closed or time.time() - self.last_rx > SILENT_S

    @property
    def in_waiting(self):
        self._fill()
        return len(self._buf)

    def read(self, n):
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def write(self, data):
        if self._closed:
            raise OSError("the board closed the connection")
        try:
            self.sock.sendall(data)                  # a 50 ms send timeout: a full buffer means the board is not keeping up, so the link is dropped
            self.last_tx = time.time()
        except (OSError, ValueError):
            self._closed = True
            raise

    def keepalive(self):
        """Called by the link on every pass: a line once a second, so the board can tell the hub is still there (it reconnects after 6 s of silence)."""
        if time.time() - self.last_tx >= 1.0:
            self.write(b"NET ping\n")

    def close(self):
        self._closed = True
        try:
            self.sock.close()
        except OSError:
            pass


class BoardListener:
    """Accepts the board's connection and hands it to a SerialLink. log(text) is called for anything worth showing."""

    def __init__(self, link, key, port=DEFAULT_PORT, bind="0.0.0.0", log=None):
        if not key or len(key) < 8:
            raise ValueError("the board key must be at least 8 characters")
        self.link, self.key, self.port, self.bind = link, key, port, bind
        self.log = log or (lambda text: None)
        self.srv = None
        self.stop = threading.Event()
        self.last_peer = None
        self.last_t = None
        self.rejected = 0

    def start(self):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((self.bind, self.port))
        self.port = self.srv.getsockname()[1]
        self.srv.listen(4)
        self.srv.settimeout(0.5)
        threading.Thread(target=self._accept_loop, daemon=True).start()
        return self.port

    def close(self):
        self.stop.set()
        if self.srv:
            try:
                self.srv.close()
            except OSError:
                pass

    def snapshot(self):
        dev = self.link.device or ""
        return {"listening": self.srv is not None and not self.stop.is_set(), "port": self.port,
                "connected": dev.startswith("wifi "), "peer": self.last_peer, "since": self.last_t, "rejected": self.rejected}

    def _accept_loop(self):
        while not self.stop.is_set():
            try:
                sock, addr = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._meet, args=(sock, addr[0]), daemon=True).start()

    def _meet(self, sock, ip):
        """Read "HELLO <key>" (keeping anything that came after it), then give the connection to the link."""
        sock.settimeout(0.2)
        buf, end = b"", time.time() + HELLO_S
        try:
            while b"\n" not in buf and len(buf) < 200 and time.time() < end:
                try:
                    chunk = sock.recv(200)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
            line, _, rest = buf.partition(b"\n")
            said = line.strip().decode("ascii", "replace")
            if not (said.startswith("HELLO ") and hmac.compare_digest(said[6:].strip().encode(), self.key.encode())):
                self.rejected += 1
                self.log("a connection from %s did not give the right key: refused" % ip)
                sock.close()
                return
            port = SocketPort(sock, ip, rest)
            if self.link.attach(port, "wifi " + ip):
                self.last_peer, self.last_t = ip, time.time()
                self.log("the board connected over WiFi from %s" % ip)
            else:
                self.log("the board at %s connected, but the link is busy (USB open, off or uploading): closed it, it will try again" % ip)
                port.close()
        except OSError:
            try:
                sock.close()
            except OSError:
                pass
