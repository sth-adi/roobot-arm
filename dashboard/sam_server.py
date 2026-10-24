"""A local stand-in for Sam's backend (firmware/sam/backend/main.py): the same endpoints, so his firmware talks to your PC instead of his host.

    python dashboard/sam_server.py                     # this computer only, DISARMED
    python dashboard/sam_server.py --host 0.0.0.0      # reachable by the ESP32 on your network (set ARM_TOKEN first)

His endpoints: GET /UpdateMovement?x=&y=&z=&a=, GET/POST /calibration, /status, /controller, GET /mode.
Ours (control): GET /state, POST /arm {"armed": bool}, POST /target {x,y,z,angle}, POST /queue {"poses": [...]}. With ARM_TOKEN set, these four need the
header X-Arm-Token. Nothing moves while disarmed. The firmware can only be told 'go there': it has no magnet switch until our firmware patch.
"""
import argparse
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import movement_service as MS  # noqa: E402

MAX_BODY = 65536


def make_handler(svc, token=""):
    store = {"controller": {}, "status": ""}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *a):
            if os.environ.get("ARM_VERBOSE"):
                super().log_message(fmt, *a)

        def _send(self, code, body="", ctype="text/plain; charset=utf-8"):
            data = (json.dumps(body) if isinstance(body, (dict, list)) else str(body)).encode("utf-8")
            self.send_response(code)
            if code != 204:
                self.send_header("Content-Type", "application/json" if isinstance(body, (dict, list)) else ctype)
                self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            if code != 204:
                self.wfile.write(data)

        def _json(self):
            n = int(self.headers.get("Content-Length", 0) or 0)
            if n <= 0 or n > MAX_BODY:
                raise ValueError("body must be 1 to %d bytes" % MAX_BODY)
            return json.loads(self.rfile.read(n))

        def _allowed(self):
            if token and self.headers.get("X-Arm-Token", "") != token:
                self._send(403, {"error": "missing or wrong X-Arm-Token"})
                return False
            return True

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/UpdateMovement":
                q = parse_qs(u.query)
                try:
                    x, y, z, a = (float(q[k][0]) for k in ("x", "y", "z", "a"))
                    if not all(v == v and abs(v) < 1e6 for v in (x, y, z, a)):
                        raise ValueError
                except (KeyError, IndexError, ValueError):
                    return self._send(400, "x, y, z and a query parameters are required")
                code, body = svc.update(x, y, z, a)
                return self._send(code, body)
            if u.path == "/mode":
                return self._send(200, "true")
            if u.path == "/controller":
                return self._send(200, store["controller"])
            if u.path == "/status":
                return self._send(200, store["status"])
            if u.path == "/calibration":
                return self._send(200, svc.snapshot()["calibration"])
            if u.path == "/state":
                return self._send(200, svc.snapshot()) if self._allowed() else None
            return self._send(404, "not found")

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                if path in ("/arm", "/target", "/queue") and not self._allowed():
                    return
                if path == "/controller":
                    d = self._json()
                    if not isinstance(d, dict):
                        raise ValueError("controller state must be an object")
                    store["controller"].update(d)
                elif path == "/status":
                    n = int(self.headers.get("Content-Length", 0) or 0)
                    if n <= 0 or n > 4096:
                        raise ValueError("status must be 1 to 4096 bytes")
                    store["status"] = self.rfile.read(n).decode("utf-8", "replace")
                elif path == "/calibration":
                    d = self._json()
                    svc.set_calibration(d["jointLimits"], d["jointHomes"])
                elif path == "/arm":
                    svc.set_armed(bool(self._json().get("armed")))
                    return self._send(200, svc.snapshot())
                elif path == "/target":
                    svc.set_target(self._json())
                    return self._send(200, svc.snapshot())
                elif path == "/queue":
                    svc.set_queue(self._json().get("poses"))
                    return self._send(200, svc.snapshot())
                else:
                    return self._send(404, "not found")
            except (ValueError, KeyError, TypeError, AttributeError) as e:
                return self._send(400, {"error": str(e) or "bad request"})
            self._send(204)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--fw-tool", type=float, default=53.0, help="tool length the firmware uses to report its position (53 for Sam's, 63 once patched)")
    ap.add_argument("--speed", type=float, default=30.0, help="tool speed, mm/s")
    ap.add_argument("--max-joint-dps", type=float, default=30.0, help="no joint faster than this, deg/s")
    args = ap.parse_args()
    token = os.environ.get("ARM_TOKEN", "")
    if args.host not in ("127.0.0.1", "localhost") and not token:
        sys.exit("refusing to listen on the network without ARM_TOKEN set (it protects /arm, /target and /queue)")
    svc = MS.MovementService(args.fw_tool, args.speed, args.max_joint_dps)
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(svc, token))
    print("Sam-compatible server on http://%s:%d  (DISARMED: POST /arm {\"armed\": true} to allow movement)" % (args.host, args.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
