#!/usr/bin/env python3
"""Robot arm HTTP server (standard library only, no pip installs needed).

Endpoints
  POST /input   JSON controller state from the client; stored as the latest state
  GET  /state   latest controller state (plus age in seconds)
  GET  /health  simple liveness check

Run:  python server.py [--host 0.0.0.0] [--port 8000]
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY = 64 * 1024

_lock = threading.Lock()
_state = {"data": None, "received_at": None, "count": 0}


class Handler(BaseHTTPRequestHandler):
    server_version = "RobotArm/0.1"

    def _send(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"status": "ok"})
        elif self.path == "/state":
            with _lock:
                age = None if _state["received_at"] is None else round(time.time() - _state["received_at"], 3)
                self._send(200, {"state": _state["data"], "age_s": age, "count": _state["count"]})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/input":
            return self._send(404, {"error": "not found"})
        try:
            length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            return self._send(400, {"error": "bad Content-Length"})
        if length <= 0 or length > MAX_BODY:
            return self._send(400, {"error": "body missing or too large"})
        try:
            data = json.loads(self.rfile.read(length))
        except json.JSONDecodeError:
            return self._send(400, {"error": "invalid JSON"})
        if not isinstance(data, dict):
            return self._send(400, {"error": "expected a JSON object"})
        with _lock:
            _state.update(data=data, received_at=time.time(), count=_state["count"] + 1)
        print(f"[{_state['count']}] {json.dumps(data)}", flush=True)
        self._send(200, {"ok": True})

    def log_message(self, fmt, *args):  # silence default per-request logging
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Listening on http://{args.host}:{args.port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
