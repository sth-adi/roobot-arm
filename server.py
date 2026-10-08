#!/usr/bin/env python3
"""Robot arm HTTP server (standard library only, no pip installs needed).

Endpoints
  POST /input   JSON controller state from the client; stored as the latest state
  GET  /state   latest controller state (plus age in seconds)
  GET  /stream  long-lived stream: one JSON line per new state (+ 0.5 s heartbeat), no polling
  POST /control {"forward": true|false} pause/resume forwarding to the next server (used by the dashboard SAFE switch)
  GET  /health  simple liveness check

Optionally forwards every received state to another server (--forward-url or
the FORWARD_URL env var); /state then also reports forwarding status.

Run:  python server.py [--host 0.0.0.0] [--port 8000] [--forward-url http://host:port/path]
"""
import argparse
import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY = 64 * 1024

_lock = threading.Lock()
_state = {"data": None, "received_at": None, "count": 0}
_cond = threading.Condition(_lock)  # wakes /stream clients the moment a new state arrives

# Forwarding: only the newest state matters, so a worker sends whatever is latest.
_fwd = {"url": None, "ok": None, "error": None, "sent": 0, "failed": 0, "enabled": True}
_fwd_wake = threading.Event()


def _forward_worker():
    last_count = 0
    while True:
        _fwd_wake.wait()
        _fwd_wake.clear()
        with _lock:
            data, count = _state["data"], _state["count"]
        if data is None or count == last_count:
            continue
        last_count = count
        if not _fwd["enabled"]:          # paused via POST /control: drop it, never queue for later
            continue
        req = urllib.request.Request(
            _fwd["url"], data=json.dumps(data).encode(),
            headers={"Content-Type": "application/json", "User-Agent": "roobot-arm-server/0.1"},
            method="POST")
        try:
            with urllib.request.urlopen(req, timeout=2.0) as r:
                r.read()
            with _lock:
                _fwd.update(ok=True, error=None, sent=_fwd["sent"] + 1)
        except (urllib.error.URLError, OSError) as e:
            with _lock:
                _fwd.update(ok=False, error=str(e), failed=_fwd["failed"] + 1)


class Handler(BaseHTTPRequestHandler):
    server_version = "RobotArm/0.1"

    def _send(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        """Hold the connection open and push one JSON line per new state (plus a 0.5 s heartbeat)."""
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = -1
        try:
            while True:
                with _cond:
                    if _state["count"] == last:
                        _cond.wait(timeout=0.5)
                    data, count, rec = _state["data"], _state["count"], _state["received_at"]
                age = None if rec is None else round(time.time() - rec, 3)
                line = json.dumps({"state": data, "age_s": age, "count": count}) + "\n"
                self.wfile.write(line.encode())
                self.wfile.flush()
                last = count
        except OSError:  # client went away
            pass

    def do_GET(self):
        if self.path == "/stream":
            self._stream()
        elif self.path == "/health":
            self._send(200, {"status": "ok"})
        elif self.path == "/state":
            with _lock:
                age = None if _state["received_at"] is None else round(time.time() - _state["received_at"], 3)
                out = {"state": _state["data"], "age_s": age, "count": _state["count"],
                       "forward_enabled": _fwd["enabled"]}
                if _fwd["url"]:
                    out["forward"] = dict(_fwd)
                self._send(200, out)
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path not in ("/input", "/control"):
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
        if self.path == "/control":      # {"forward": true|false} pauses/resumes forwarding to the next server
            if not isinstance(data.get("forward"), bool):
                return self._send(400, {"error": "expected {\"forward\": true|false}"})
            with _lock:
                _fwd["enabled"] = data["forward"]
            return self._send(200, {"ok": True, "forward_enabled": data["forward"]})
        with _lock:
            _state.update(data=data, received_at=time.time(), count=_state["count"] + 1)
            _cond.notify_all()
        print(f"[{_state['count']}] {json.dumps(data)}", flush=True)
        _fwd_wake.set()
        self._send(200, {"ok": True})

    def log_message(self, fmt, *args):  # silence default per-request logging
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--forward-url", default=os.environ.get("FORWARD_URL"),
                   help="also POST each received state to this URL (env: FORWARD_URL)")
    args = p.parse_args()
    if args.forward_url:
        _fwd["url"] = args.forward_url
        threading.Thread(target=_forward_worker, daemon=True).start()
        print(f"Forwarding inputs to {args.forward_url}")
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
