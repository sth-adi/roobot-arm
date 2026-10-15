#!/usr/bin/env python3
"""Robot arm HTTP server (standard library only, no pip installs needed).

Endpoints
  POST /input   JSON controller state from the client; stored as the latest state
  GET  /state   latest controller state (plus age in seconds)
  GET  /stream  long-lived stream: one JSON line per new state (+ 0.5 s heartbeat), no polling
  POST /control {"forward": true|false} pause/resume forwarding to the next server (used by the dashboard SAFE switch)
                {"maxSpeed": 1-60} joint speed limit in deg/s, added to every packet sent to the arm (and /controller)
  GET  /health  simple liveness check
  GET  /controller  the latest packet on its own (what the firmware polls on the UWE-RArm backend); neutral once stale

Dead-man: the next server (and the UWE-RArm backend in particular) keeps the last packet it received for ever, so this
server sends it a neutral packet (sticks 0, buttons released) when input has been silent for 1.5 s and when forwarding is
paused with POST /control. Nothing is queued for later.

Arm side channel (same endpoints as the UWE-RArm backend, so its firmware and the virtual arm both work):
  POST /status       arm status text (<= 4 KB);  GET /status returns it
  POST /calibration  {"jointLimits": [[min,max] x4], "jointHomes": [x4]}, whole degrees 0-180;  GET returns it (404 if unset)
  GET  /mode         "Mode" response header (env ARM_MODE, default Normal); the firmware enters training when it is Training

Sequences and arm state (our firmware; the stock firmware never asks for them):
  POST /sequence   {"poses": [{"x","y","z","angle","action": null|"grab"|"release", "speed": deg/s (optional)}, ...]} (<= 20 poses; an empty list stops a run)
                   queues ONE sequence and returns {"id": n}. The next /controller packet carries "seqId": n; the firmware then GETs it.
  GET  /sequence   {"id": n, "poses": [[x, y, z, angle, action, speed], ...]} with action 0 none, 1 grab (magnet on), 2 release (magnet off);
                   speed is the joint speed limit (deg/s) for the move into that pose, 0 = the normal limit (firmware that predates it reads five numbers and ignores it)
  POST /armstate   the firmware's own report: {"x","y","z","angle","magnet","mode","step","saved","seqId","seqDone","atHome"}
  GET  /armstate   the latest report plus its age in seconds

Optionally forwards every received state to another server (--forward-url or
the FORWARD_URL env var); /state then also reports forwarding status.

Run:  python server.py [--host 0.0.0.0] [--port 8000] [--forward-url http://host:port/path]
"""
import argparse
import hashlib
import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

def server_stamp(path=None):
    """A short id of this file (its size), so the launcher can tell a running server from the one on disk."""
    return hashlib.sha1(("server:%d" % os.stat(path or __file__).st_size).encode()).hexdigest()[:7]


MAX_BODY = 64 * 1024
STALE_S = 1.5   # no input newer than this = the sender is gone

_lock = threading.Lock()
_state = {"data": None, "received_at": None, "count": 0}
_cond = threading.Condition(_lock)  # wakes /stream clients the moment a new state arrives

# Forwarding: only the newest state matters, so a worker sends whatever is latest.
_fwd = {"url": None, "ok": None, "error": None, "sent": 0, "failed": 0, "enabled": True,
        "neutrals": 0, "flush_neutral": False}
_fwd_wake = threading.Event()

# Arm side channel: status text, calibration and mode (see the docstring).
_arm = {"status": "", "calibration": None, "mode": os.environ.get("ARM_MODE", "Normal"), "max_speed": None}
MIN_PRESS_S = 0.25          # a button press is kept this long: the arm polls slowly and would miss a quick tap
BUTTONS = ("A", "B", "X", "Y")
_pressed_at = {}            # button -> when it went down (only while it reads pressed)
_seq = {"id": 0, "poses": []}       # the one queued sequence, in the compact form the firmware reads
_armstate = {"data": None, "received_at": None}
ACTION_CODES = {None: 0, "grab": 1, "release": 2}
MAX_POSES = 20
MAX_SPEED_RANGE = (1, 60)   # deg/s per joint; the real firmware clamps to the same range


def _valid_calibration(p):
    """Return {"jointLimits", "jointHomes"} if p is a valid calibration, else None."""
    try:
        limits, homes = p["jointLimits"], p["jointHomes"]
    except (KeyError, TypeError):
        return None
    is_deg = lambda v: isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 180
    if (isinstance(limits, list) and len(limits) == 4
            and all(isinstance(x, list) and len(x) == 2 and all(is_deg(v) for v in x) for x in limits)
            and isinstance(homes, list) and len(homes) == 4 and all(is_deg(v) for v in homes)):
        return {"jointLimits": limits, "jointHomes": homes}
    return None


def _neutral_of(data):
    """The same packet with every stick at 0 and every button released. The UWE-RArm backend keeps the last packet it
    was given forever, so the arm only stops if somebody sends it one of these."""
    if isinstance(data, bool):
        return False
    if isinstance(data, (int, float)):
        return 0 if isinstance(data, int) else 0.0
    if isinstance(data, list):
        return [_neutral_of(v) for v in data]
    if isinstance(data, dict):
        return {k: _neutral_of(v) for k, v in data.items()}
    return data


def _stretch_presses(data, now):
    """Keep a pressed button reading pressed for at least MIN_PRESS_S, so a quick tap is not lost between two polls.
    Returns (data, seconds until the stretched release is due, 0 if none). Call with _lock held."""
    out, wait = data, 0.0
    for b in BUTTONS:
        if b not in data:
            continue
        if data[b] is True:
            _pressed_at.setdefault(b, now)
        elif b in _pressed_at:
            remain = _pressed_at[b] + MIN_PRESS_S - now
            if remain > 0:
                if out is data:
                    out = dict(data)
                out[b] = True
                wait = max(wait, remain)
            else:
                del _pressed_at[b]
    return out, wait


def _release_stretched(count):
    """The stretch is over: send the release, unless a newer packet has arrived (it carries its own truth)."""
    with _lock:
        if _state["count"] != count or _state["data"] is None:
            return
        data, now = dict(_state["data"]), time.time()
        for b in BUTTONS:
            if b in _pressed_at and _pressed_at[b] + MIN_PRESS_S <= now + 0.02:
                data[b] = False
                del _pressed_at[b]
        _state.update(data=data, received_at=now, count=count + 1)
        _cond.notify_all()
    _fwd_wake.set()


def _with_seq(data):
    """Add "seqId" once a sequence has been queued, so the firmware knows to fetch it. A neutral packet is made if there is no
    input yet (the firmware ignores a packet without sticks)."""
    with _lock:
        sid = _seq["id"]
    if not sid:
        return data
    if not data:
        data = {"LSX": 0.0, "LSY": 0.0, "RSX": 0.0, "RSY": 0.0, "A": False, "B": False, "X": False, "Y": False}
    return {**data, "seqId": sid}


def _valid_sequence(d):
    """Compact poses [[x, y, z, angle, action]] from the JSON the dashboard sends, or None if anything is wrong."""
    poses = d.get("poses") if isinstance(d, dict) else None
    if not isinstance(poses, list) or len(poses) > MAX_POSES:
        return None
    out = []
    for p in poses:
        try:
            v = [float(p[k]) for k in ("x", "y", "z", "angle")]
            act = ACTION_CODES[p.get("action")]
            speed = float(p.get("speed") or 0.0)         # optional joint speed limit for the move into this pose, deg/s; 0 = the normal limit
        except (KeyError, TypeError, ValueError, AttributeError):
            return None
        if not all(abs(c) < 1000 and c == c for c in v) or not (0.0 <= speed < 1000.0):
            return None
        out.append(v + [act, round(speed, 1)])
    return out


def _with_speed(data):
    """Add the joint speed limit (set with POST /control {"maxSpeed": n}) to a packet going to the arm. The firmware
    reads it as an optional "maxSpeed" key. Added after _neutral_of, so a neutral packet keeps the limit."""
    with _lock:
        speed = _arm["max_speed"]
    if speed is not None and isinstance(data, dict):
        return {**data, "maxSpeed": speed}
    return data


def _post_forward(data):
    req = urllib.request.Request(
        _fwd["url"], data=json.dumps(_with_speed(data)).encode(),
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


def _forward_worker():
    """Forward the newest state, and make sure the next server never keeps a held stick:
    a neutral packet goes out when input goes stale (client closed, network lost) and when forwarding is paused."""
    last_count, neutral_sent = 0, True
    while True:
        _fwd_wake.wait(timeout=0.5)
        _fwd_wake.clear()
        try:
            last_count, neutral_sent = _forward_once(last_count, neutral_sent)
        except Exception as e:          # never let the forwarder die: a dead one stops the arm being told to stop
            print("forwarder error:", e, flush=True)
            time.sleep(0.5)


def _forward_once(last_count, neutral_sent):
    with _lock:
        data, count, rec = _state["data"], _state["count"], _state["received_at"]
        flush = _fwd["flush_neutral"]
        _fwd["flush_neutral"] = False
    if data is None:
        return last_count, neutral_sent
    if flush:                         # forwarding was just paused (SAFE): stop whatever the arm is doing
        neutral_sent = True
        with _lock:
            _fwd["neutrals"] += 1
        _post_forward(_neutral_of(data))
    if count != last_count:
        last_count, neutral_sent = count, False
        if _fwd["enabled"]:           # paused via POST /control: drop it, never queue for later
            _post_forward(data)
    elif not neutral_sent and rec is not None and time.time() - rec >= STALE_S:
        neutral_sent = True
        if _fwd["enabled"]:
            with _lock:
                _fwd["neutrals"] += 1
            _post_forward(_neutral_of(data))
    return last_count, neutral_sent


class Handler(BaseHTTPRequestHandler):
    server_version = "RobotArm/0.1"

    def _send(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, code, text, extra=None):
        body = text.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
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
            self._send(200, {"status": "ok", "build": server_stamp()})
        elif self.path == "/status":
            with _lock:
                self._send_text(200, _arm["status"])
        elif self.path == "/mode":
            with _lock:
                self._send_text(200, "true", {"Mode": _arm["mode"]})
        elif self.path == "/calibration":
            with _lock:
                cal = _arm["calibration"]
            if cal is None:
                self._send(404, {"error": "calibration not available"})
            else:
                self._send(200, cal)
        elif self.path == "/sequence":
            with _lock:
                self._send(200, {"id": _seq["id"], "poses": _seq["poses"]})
        elif self.path == "/armstate":
            with _lock:
                age = None if _armstate["received_at"] is None else round(time.time() - _armstate["received_at"], 3)
                self._send(200, {"state": _armstate["data"], "age_s": age})
        elif self.path == "/state":
            with _lock:
                age = None if _state["received_at"] is None else round(time.time() - _state["received_at"], 3)
                out = {"state": _state["data"], "age_s": age, "count": _state["count"],
                       "forward_enabled": _fwd["enabled"], "maxSpeed": _arm["max_speed"]}
                if _fwd["url"]:
                    out["forward"] = {k: v for k, v in _fwd.items() if k != "flush_neutral"}
                self._send(200, out)
        elif self.path == "/controller":
            # The same packet the firmware polls on the UWE-RArm backend ({} until there is input). Input older than
            # STALE_S comes back neutral, so a firmware reading this server directly stops when the sender is gone.
            with _lock:
                data, rec = _state["data"], _state["received_at"]
            if data is None:
                data = {}
            elif rec is None or time.time() - rec >= STALE_S:
                data = _neutral_of(data)
            self._send(200, _with_seq(_with_speed(data)))
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path == "/shutdown":              # lets launch.py replace an old server with the current code (this computer only)
            if self.client_address[0] not in ("127.0.0.1", "::1"):
                return self._send(403, {"error": "forbidden"})
            threading.Thread(target=self.server.shutdown, daemon=True).start()
            return self._send(200, {"ok": True})
        if self.path not in ("/input", "/control", "/status", "/calibration", "/sequence", "/armstate"):
            return self._send(404, {"error": "not found"})
        try:
            length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            return self._send(400, {"error": "bad Content-Length"})
        if length <= 0 or length > MAX_BODY:
            return self._send(400, {"error": "body missing or too large"})
        if self.path in ("/sequence", "/armstate"):
            try:
                body = json.loads(self.rfile.read(length))
            except (json.JSONDecodeError, UnicodeDecodeError):
                return self._send(400, {"error": "invalid JSON"})
            if self.path == "/armstate":
                if not isinstance(body, dict):
                    return self._send(400, {"error": "expected a JSON object"})
                with _lock:
                    _armstate.update(data=body, received_at=time.time())
                self.send_response(204)
                self.end_headers()
                return
            poses = _valid_sequence(body)
            if poses is None:
                return self._send(400, {"error": "poses: at most %d, each with numeric x, y, z, angle and action null, grab or release" % MAX_POSES})
            with _lock:
                _seq["id"] += 1
                _seq["poses"] = poses
                sid = _seq["id"]
            return self._send(200, {"id": sid, "count": len(poses)})
        if self.path in ("/status", "/calibration"):          # arm side channel: small bodies only
            if length > 4096:
                return self._send(400, {"error": "body too large"})
            raw = self.rfile.read(length)
            if self.path == "/status":
                with _lock:
                    _arm["status"] = raw.decode("utf-8", "replace")
            else:
                try:
                    cal = _valid_calibration(json.loads(raw))
                except (json.JSONDecodeError, UnicodeDecodeError):
                    cal = None
                if cal is None:
                    return self._send(400, {"error": "calibration needs four [min, max] limits and four homes, whole degrees 0-180"})
                with _lock:
                    _arm["calibration"] = cal
            self.send_response(204)
            self.end_headers()
            return
        try:
            data = json.loads(self.rfile.read(length))
        except json.JSONDecodeError:
            return self._send(400, {"error": "invalid JSON"})
        if not isinstance(data, dict):
            return self._send(400, {"error": "expected a JSON object"})
        if self.path == "/control":
            # {"forward": true|false} pauses/resumes forwarding; {"maxSpeed": 1..60} sets the joint speed limit (deg/s)
            # that goes to the arm with every packet. Either or both.
            fwd, speed = data.get("forward"), data.get("maxSpeed")
            if (fwd is None and speed is None) or (fwd is not None and not isinstance(fwd, bool)):
                return self._send(400, {"error": "expected {\"forward\": true|false} and/or {\"maxSpeed\": 1-60}"})
            if speed is not None and (isinstance(speed, bool) or not isinstance(speed, (int, float))
                                      or not MAX_SPEED_RANGE[0] <= speed <= MAX_SPEED_RANGE[1]):
                return self._send(400, {"error": "maxSpeed must be a number from %d to %d deg/s" % MAX_SPEED_RANGE})
            with _lock:
                if speed is not None:
                    _arm["max_speed"] = speed
                if fwd is not None:
                    if _fwd["enabled"] and not fwd:
                        _fwd["flush_neutral"] = True      # pausing must also stop the arm: the next server holds the last packet
                    _fwd["enabled"] = fwd
                out = {"ok": True, "forward_enabled": _fwd["enabled"], "maxSpeed": _arm["max_speed"]}
            _fwd_wake.set()
            return self._send(200, out)
        with _lock:
            data, wait = _stretch_presses(data, time.time())
            _state.update(data=data, received_at=time.time(), count=_state["count"] + 1)
            count = _state["count"]
            _cond.notify_all()
        if wait:
            threading.Timer(wait + 0.01, _release_stretched, args=(count,)).start()
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
