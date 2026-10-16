#!/usr/bin/env python3
"""Robot arm hub: the local dashboard and its control API (Python standard library; pyserial optional).

    python dashboard/hub.py [--port 8765] [--server http://127.0.0.1:8000] [--no-open] [--send-speed]

Serves the dashboard at http://127.0.0.1:8765 and gives it one place to
  - watch live controller packets (from the local server's /stream),
  - start/stop the local server (Docker or Python), the pygame GUI and Docker Desktop,
  - own the ESP32's USB serial link: read the board's log, change its speed settings,
  - compile and upload the ESP32 firmware with arduino-cli,
  - show git status, and
  - switch the whole output between SAFE (nothing reaches the arm) and ARMED.

SAFE is the default. While SAFE the hub sends nothing over USB, ignores browser input, and tells the local
server to pause forwarding to the next server. ARMED reverts to SAFE by itself if no dashboard is open.
The hub only listens on 127.0.0.1 and rejects cross-origin requests.
"""
import argparse
import collections
import hmac
import json
import math
import mimetypes
import os
import queue
import re
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
STATIC = HERE / "static"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
from serial_link import SerialLink, BUTTON_ORDER  # noqa: E402
from sim.armsim import ArmSim, STALE_S, internal_from_wire, wire_from_internal  # noqa: E402
from emu_runner import EmuRunner  # noqa: E402
from stamp import code_stamp  # noqa: E402
from sim.workcell import Workcell  # noqa: E402
import sim.sequences as SEQ  # noqa: E402
import sim.tuner as TUNER  # noqa: E402
import sim.servo_map as SERVOMAP  # noqa: E402
import agent as AGENT  # noqa: E402
import preflight as PRE  # noqa: E402
import voice as VOICE  # noqa: E402
import tls as TLS  # noqa: E402
import endcal as ENDCAL  # noqa: E402
import netlink as NETLINK  # noqa: E402
LIBRARY = SEQ.Library(ROOT / "sequences")
CALIB = TUNER.Calibration(str(ROOT / "tuning"))        # limits learned from real-arm trials, per sequence (local to this PC)

IS_WIN = sys.platform == "win32"
NOWIN = subprocess.CREATE_NO_WINDOW if IS_WIN else 0
DOCKER = shutil.which("docker") or r"C:\Program Files\Docker\Docker\resources\bin\docker.exe"
DOCKER_DESKTOP = r"C:\Program Files\Docker\Docker\Docker Desktop.exe"
ARDUINO = shutil.which("arduino-cli") or r"C:\Program Files\Arduino CLI\arduino-cli.exe"
FQBN = "esp32:esp32:esp32"
SKETCH = ROOT / "arduino" / "arm_receiver"
CTYPES = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
          ".js": "text/javascript; charset=utf-8", ".svg": "image/svg+xml", ".json": "application/json"}
AXES = ("lx", "ly", "rx", "ry")
TRIGGERS = ("lt", "rt")
BOARD_CMD_RE = re.compile(r"^(!(rate|live|range) \d{1,4}(\.\d{1,2})?|!show)$")
DEADMAN_S = 15.0          # ARMED drops back to SAFE if no dashboard has been connected this long
IDLE_WIRE = {"LSX": 0.0, "LSY": 0.0, "RSX": 0.0, "RSY": 0.0, "A": False, "B": False, "X": False, "Y": False}
NEUTRAL = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lt": 0.0, "rt": 0.0,
           "buttons": {k: False for k in BUTTON_ORDER}, "dpad": [0, 0]}


def clean_state(d):
    """Validate and normalise a controller state posted by the browser. Raises ValueError."""
    if not isinstance(d, dict):
        raise ValueError("state must be an object")
    out = {}
    for k in AXES:
        out[k] = round(max(-1.0, min(1.0, float(d.get(k, 0)))), 3)
    for k in TRIGGERS:
        out[k] = round(max(0.0, min(1.0, float(d.get(k, 0)))), 3)
    b = d.get("buttons") or {}
    out["buttons"] = {k: bool(b.get(k, False)) for k in BUTTON_ORDER}
    dp = d.get("dpad") or [0, 0]
    out["dpad"] = [max(-1, min(1, int(dp[0]))), max(-1, min(1, int(dp[1])))]
    return out


def describe(prev, cur):
    if prev is None:
        return ["first packet"]
    out = []
    for k in AXES + TRIGGERS:
        if abs(cur.get(k, 0) - prev.get(k, 0)) >= 0.01:
            out.append(f"{k.upper()} {cur[k]:+.2f}")
    for k, v in (cur.get("buttons") or {}).items():
        if v != (prev.get("buttons") or {}).get(k):
            out.append(f"{k.upper()} {'down' if v else 'up'}")
    if cur.get("dpad") != prev.get("dpad"):
        out.append(f"DPAD {cur.get('dpad')}")
    return out


class Hub:
    def __init__(self, server_url, send_speed=False):
        self.server = server_url.rstrip("/")
        self.send_speed = send_speed            # off: packets reach the arm exactly as the stock UWE-RArm firmware expects
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.clients = []                       # SSE queues
        self.last_client_t = time.time()
        # safety
        self.armed = False
        self.armed_since = None
        self.forward_control = None             # did the last pause/resume reach the server?
        # controller input seen by the local server
        self.latest, self.latest_recv, self.latest_count = None, 0.0, 0
        self.latest_wire = None                 # the same packet in the UWE-RArm wire format, what the arm reads
        # virtual ESP32 + arm (sim/armsim.py): always running, so the preview works with no hardware and while SAFE
        self.sim = ArmSim(model="stock", patch=True)
        self.sim.move_mm_s = 100.0                      # sequence poses are reached on a smooth straight line (UWE-RArm's linear move, 100 mm/s peak)   # our firmware: UWE-RArm stock + firmware/stock/patches (damper, failsafes, tap-to-save)
        self.sim_lock = threading.Lock()
        self._sim_status_sent = None
        self._cal_synced = False
        self.emu = EmuRunner(self.server, self.log)   # emulated ESP32 + network (Control page)
        self._prev_y = False
        self._run_once = None
        self._esp = None                        # (state dict, time) last report of our firmware via the server's /armstate
        self._esp_run = None                    # id of the sequence the ESP is running for us
        self.phone_url = None                   # set by main() with --lan
        self.phone_url_https = None             # the same over HTTPS (port + 1), which the phone's microphone needs
        self.board_listener = None              # the board's WiFi link (port + 2): the board connects out to this PC, see netlink.py
        self.cell = Workcell()                  # boxes, keys, tray and the electromagnet (sim/workcell.py)
        self.emu_view = ArmSim()                # the emulated servos drawn as an arm
        self.packets = collections.deque(maxlen=500)
        self.packet_times = collections.deque(maxlen=400)
        self.stream_ok = False
        # board / serial link
        self.link = SerialLink()
        self.link.on_line = self._board_line
        self.link.on_event = lambda text: self.log("hub", text)      # why a WiFi connection was dropped
        self.board = {"pos": None, "wifi": None, "source": None, "last_line_t": None}
        self.board_lines = collections.deque(maxlen=400)
        # services / logs
        self.logs = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.busy = {}                          # service name -> short action text while running
        self.cache = {"server": {}, "docker_engine": "unknown", "docker_container": "", "procs": {}, "git": {}}
        self.fw = {"state": "idle", "action": None, "flash_pct": None, "ram_pct": None, "finished_t": None, "detail": ""}
        self.procs = {}                         # hub-launched Popen handles

    # ---- pub/sub -----------------------------------------------------------------
    def add_client(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            self.clients.append(q)
            self.last_client_t = time.time()
        return q

    def remove_client(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)
            self.last_client_t = time.time()

    def publish(self, kind, data):
        msg = f"event: {kind}\ndata: {json.dumps(data, separators=(',', ':'))}\n\n"
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    self.clients.remove(q)

    def log(self, src, line):
        ev = {"t": round(time.time(), 2), "src": src, "line": line}
        self.logs[src].append(ev)
        self.publish("log", ev)

    # ---- local server ----------------------------------------------------------------
    def _http(self, path, payload=None, timeout=1.0):
        data = None if payload is None else json.dumps(payload).encode()
        req = urllib.request.Request(self.server + path, data=data, method="GET" if data is None else "POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")

    def set_forward(self, enabled):
        """Pause/resume forwarding on the local server. True ok, False rejected/old server, None unreachable."""
        try:
            self._http("/control", {"forward": bool(enabled)})
            self.forward_control = True
        except urllib.error.HTTPError:
            self.forward_control = False
        except (urllib.error.URLError, OSError):
            self.forward_control = None
        return self.forward_control

    def _stream_reader(self):
        """Follow the local server's /stream and turn it into packet events."""
        prev = None
        while not self.stop.is_set():
            try:
                with urllib.request.urlopen(self.server + "/stream", timeout=5) as r:
                    self.stream_ok = True
                    for raw in r:
                        if self.stop.is_set():
                            return
                        line = raw.decode("utf-8", "replace").strip()
                        if not line:
                            continue
                        msg = json.loads(line)
                        raw, count, age = msg.get("state"), msg.get("count") or 0, msg.get("age_s")
                        if raw is None:
                            continue
                        if "LSX" in raw:                    # UWE-RArm wire format: the dashboard shows it as its richer state
                            wire, st = raw, internal_from_wire(raw)
                        else:                               # an older client still posting the previous format
                            try:
                                st = clean_state(raw)
                            except (ValueError, TypeError):
                                continue
                            wire = wire_from_internal(st)
                        now = time.time()
                        self.latest, self.latest_wire, self.latest_recv = st, wire, now - (age or 0.0)
                        if count != self.latest_count:
                            self.latest_count = count
                            self.packet_times.append(now)
                            ev = {"n": count, "t": round(self.latest_recv, 3), "s": st, "w": wire, "d": describe(prev, st)}
                            prev = st
                            self.packets.append(ev)
                            self.publish("packet", ev)
            except Exception:
                self.stream_ok = False
                time.sleep(1.5)

    def pps(self):
        now = time.time()
        n = sum(1 for t in self.packet_times if now - t <= 2.0)
        return round(n / 2.0, 1)

    def fresh_state(self):
        if self.latest is not None and time.time() - self.latest_recv < 1.0:
            return self.latest
        return None

    # ---- virtual ESP32 + arm ---------------------------------------------------------------
    def _sim_loop(self):
        """Run the arm model at 50 Hz on whatever the local server is receiving. No fresh input = no motion."""
        last, pose_t = time.monotonic(), 0.0
        while not self.stop.is_set():
            time.sleep(0.02)
            now = time.monotonic()
            dt, last = now - last, now
            wire = self.latest_wire if (self.latest_wire is not None and time.time() - self.latest_recv < STALE_S) else None
            if wire is None and self.sim.latched:
                wire = dict(IDLE_WIRE)                  # a started replay runs on without any controller input
            try:
                with self.sim_lock:
                    if wire is not None and wire.get("Y") and not self._prev_y:
                        self.sim.toggle_replay()        # Y: start or stop the replay (simulation only)
                    self._prev_y = bool(wire and wire.get("Y"))
                    self.sim.step(wire, dt)
                    snap = self._pose_snapshot() if now - pose_t >= 0.04 else None
                    if snap is not None:
                        esp = self.esp_state()
                        if esp:
                            self.cell.set_magnet(bool(esp.get("magnet")))      # the firmware switches the magnet; the cell only follows
                        self.cell.step(snap["points"][4], False if esp else (wire.get("X") if wire else False), now - pose_t if pose_t else 0.04)
                        # the virtual arm runs the same logic on the same input, so its saved list and replay schedule stand in for the
                        # emulated firmware's (which the hub cannot see); arrival is judged from the arm that is drawn
                        before = self.cell.magnet
                        if not esp:
                            self.cell.track_saved(self.sim.saved, self.sim.replaying, self.sim.replay_idx, snap["points"][4])
                        if self._run_once and not self._esp_run and self.cell.run_finished(self._run_once) and self.sim._lin is None:   # a straight-line move must finish first, or the last millimetres are cut off
                            self._run_once = None
                            self.sim.stop_replay()
                            self.log("hub", "sequence finished")
                        if getattr(self.cell, "restarted", False):
                            self.cell.restarted = False
                            self.log("hub", "replay: keys put back in the boxes for a fresh run")
                        if self.cell.magnet != before:
                            self.log("hub", "replay: magnet %s at saved position #%d" % ("ON" if self.cell.magnet else "off", self.cell.last_applied + 1))
                        snap["world"] = self.cell.snapshot()
                if snap is not None:
                    pose_t = now
                    snap["t"] = time.time()          # the page plays poses back on this clock, so network jitter does not show as stutter
                    self.publish("pose", snap)
            except Exception as e:                  # never let the simulation die
                self.log("hub", f"simulation error: {e}")

    def _pose_snapshot(self):
        """What the Control page draws: the virtual arm, or the emulated ESP32's servos while that is running.
        Call with sim_lock held."""
        got = self.emu.servo_angles_deg(self.sim.limits) if self.emu.running else None
        if got is None:
            snap = self.sim.snapshot()
            snap["world"] = self.cell.snapshot()
            return snap
        deg, attached = got
        v = self.emu_view
        v.limits, v.homes, v.tool = self.sim.limits, self.sim.homes, self.sim.tool
        v.angles = [math.radians(a) for a in deg]
        v.attached, v.mode, v.model, v.patch = attached, "emulated", "improved", False
        v.status, v.rate = "emulated ESP32", self.sim.rate
        v.saved, v.replay_idx = [], 0
        snap = v.snapshot()
        snap["emulated"] = True
        snap["world"] = self.cell.snapshot()
        return snap

    def _post_text(self, path, text, timeout=0.6):
        req = urllib.request.Request(self.server + path, data=text.encode("utf-8"), method="POST",
                                     headers={"Content-Type": "text/plain; charset=utf-8"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            r.read()

    def push_calibration(self):
        with self.sim_lock:
            cal = {"jointLimits": [list(p) for p in self.sim.limits], "jointHomes": list(self.sim.homes)}
        try:
            self._http("/calibration", cal)
            return True
        except Exception:
            return False

    BOARD_TOOL_MM = 53.0                            # the tool length Sam's inverse kinematics has built in (the board's sketch uses his code)
    cal_source = "default"                          # where the virtual arm's limits and homes came from: default | local server | end server | cache

    def _link_cal(self):
        """Give the board the virtual arm's limits and homes, so both run the inverse kinematics with the same numbers."""
        with self.sim_lock:
            limits, homes = [list(p) for p in self.sim.limits], list(self.sim.homes)
        self.link.set_calibration(limits, homes)

    def load_end_calibration(self):
        """Read the real arm's calibration from the end server (read-only), and use it in the virtual arm, the local server and the board; the tool
        length becomes the 53 mm the board's inverse kinematics has. Not while the real arm is following the virtual one: it would jump.
        Returns (ok, message)."""
        if self.armed and self.mirror["on"]:
            return False, "switch the output to SAFE first: a new calibration would move the real arm"
        src = "end server"
        try:
            limits, homes = ENDCAL.fetch(self.env_forward())
            ENDCAL.save_cache(limits, homes)
        except Exception as e:
            try:
                limits, homes = ENDCAL.load_cache()
                src = "cache"
                self.log("hub", "could not read the calibration from the end server (%s: %s): using the last one saved" % (type(e).__name__, str(e)[:80]))
            except Exception:
                return False, "could not read the calibration from the end server (%s: %s) and none is saved" % (type(e).__name__, str(e)[:80])
        with self.sim_lock:
            self.sim.set_calibration(limits, homes)
            self.sim.set_tool(self.BOARD_TOOL_MM)
        self.cal_source = src
        self.push_calibration()
        self._link_cal()
        self.log("hub", "calibration from the %s: limits %s homes %s, tool %.0f mm (reversed limits mark an inverted joint)" % (src, limits, homes, self.BOARD_TOOL_MM))
        return True, "calibration loaded from the %s" % src

    def _boot_calibration(self):
        """At start: wait for the local server's calibration sync (so it cannot overwrite this), then load the real one if FORWARD_URL is set."""
        if not self.env_forward():
            return
        end = time.time() + 20
        while time.time() < end and not self._cal_synced and not self.stop.is_set():
            time.sleep(0.3)
        ok, msg = self.load_end_calibration()
        if not ok:
            self.log("hub", "calibration: " + msg)

    def push_speed(self):
        """Send the damper to the server as "maxSpeed", which it adds to every packet for the real arm. The firmware
        accepts 1-60 deg/s, so 0 (no limit in the simulation) goes as the 60 deg/s ceiling.
        Only with --send-speed: the stock firmware has no damper, so by default the slider affects the simulation only."""
        if not self.send_speed:
            return True
        with self.sim_lock:
            rate = self.sim.rate
        try:
            self._http("/control", {"maxSpeed": min(60, max(1, rate or 60))})
            return True
        except Exception:
            return False

    def _sync_arm_channel(self, health):
        """Keep the server's /calibration and /status in step with the virtual arm, like the real ESP32 does."""
        if not health:
            self._cal_synced = False                # a restarted server has lost both: sync again when it is back
            self._speed_synced = False
            self._sim_status_sent = None
            return
        if not self._cal_synced:
            try:
                cal = self._http("/calibration", timeout=0.8)
                with self.sim_lock:
                    changed = self.sim.limits != cal["jointLimits"] or self.sim.homes != cal["jointHomes"]
                    if changed:                     # only then: loading resets the arm to home
                        self.sim.set_calibration(cal["jointLimits"], cal["jointHomes"])
                if changed:
                    self.log("hub", "virtual arm: loaded calibration from the server")
                    self._link_cal()
                self._cal_synced = True
            except urllib.error.HTTPError as e:
                if e.code == 404 and self.push_calibration():
                    self.log("hub", "virtual arm: no calibration on the server, sent the default")
                    self._cal_synced = True
            except (urllib.error.URLError, OSError, KeyError, ValueError, TypeError):
                pass
        if not getattr(self, "_speed_synced", False):
            self._speed_synced = self.push_speed()
        with self.sim_lock:
            status = self.sim.status
        if self.emu.state in ("building", "running"):
            return                                  # the emulated firmware reports its own status
        if status != self._sim_status_sent:
            try:
                self._post_text("/status", status)
                self._sim_status_sent = status
            except (urllib.error.URLError, OSError):
                pass

    # ---- serial link / board ---------------------------------------------------------------
    def _board_line(self, line):
        t = round(time.time(), 2)
        self.board_lines.append({"t": t, "line": line})
        self.board["last_line_t"] = t
        m = re.search(r"pos\(us\)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)", line)
        if m:
            self.board["pos"] = [float(x) for x in m.groups()]
        if "WiFi OK" in line:
            self.board["wifi"] = True
        elif "WiFi not connected" in line:
            self.board["wifi"] = False
        if "serial: direct USB input active" in line:
            self.board["source"] = "usb"
        elif "serial: input lost" in line:
            self.board["source"] = "wifi"
        self.publish("board", {"t": t, "line": line})

    def _pump(self):
        while not self.stop.is_set():
            time.sleep(0.02)
            now = time.time()
            try:
                st = self.fresh_state()
                if self.armed and self.mirror["on"]:
                    if self._board_ik_active():
                        self.link.update_pose(self._mirror_pose(), now)          # the virtual arm's tool-tip pose: the board runs the inverse kinematics
                    else:
                        self.link.update_positions(self._mirror_degrees(), now)  # the virtual arm's joint angles: the board follows them
                elif self.armed and st is not None:
                    self.link.update(st, now)       # send input + read the board
                else:
                    self.link.service(now)          # SAFE: keep the port open and read only
            except Exception as e:                  # never let the pump die
                self.log("hub", f"serial pump error: {e}")

    # ---- points: a few fixed places to move between, to watch the inverse kinematics and the straight-line move at work ----
    POINTS_FILE = ROOT / "tuning" / "points.json"
    # Sam's test points, in the order his demo loop (UWE-RArm main.cpp) visits them: the tool angle is 0 throughout
    DEFAULT_POINTS = [{"name": n, "x": x, "y": 200.0, "z": z, "angle": 0.0, "on": True}
                      for n, x, z in (("A", -100.0, 100.0), ("B", 0.0, 100.0), ("C", 0.0, 300.0), ("D", 0.0, 100.0), ("E", 100.0, 100.0))]

    def _points_load(self):
        if self.points is None:
            pts = [dict(p) for p in self.DEFAULT_POINTS]
            try:
                saved = json.loads(self.POINTS_FILE.read_text(encoding="utf-8"))
                for i, p in enumerate(saved[:len(pts)]):
                    pts[i].update({k: p[k] for k in ("name", "x", "y", "z", "angle", "on") if k in p})
            except (OSError, ValueError, TypeError):
                pass
            self.points = pts
        return self.points

    def _points_save(self):
        try:
            self.POINTS_FILE.parent.mkdir(parents=True, exist_ok=True)
            self.POINTS_FILE.write_text(json.dumps(self.points, indent=2), encoding="utf-8")
        except OSError:
            pass

    def points_status(self):
        pts = self._points_load()
        with self.sim_lock:
            ik = [self.sim.ik_report((p["x"], p["y"], p["z"], p["angle"])) for p in pts]
            snap = self.sim.snapshot()
        return {"points": [dict(p) for p in pts], "ik": ik, "loop": bool(self.points_loop), "arm": {"xyz": snap["xyz"], "tool_angle": snap["tool_angle"],
                "angles": snap["angles"], "linear": snap.get("linear"), "mode": snap["mode"]}}

    def _points_check_free(self):
        if self.tune.get("state") == "running" or self.trial.get("state") in ("homing", "running") or self.task.get("state") in ("working", "running"):
            raise RuntimeError("the arm is busy with something else: let it finish first")

    def points_action(self, d):
        a = d.get("action")
        pts = self._points_load()
        idx = int(d.get("index", -1)) if a in ("set", "here", "enable", "go") else -1
        if a in ("set", "here", "enable", "go") and not (0 <= idx < len(pts)):
            raise ValueError("index must be 0 to %d" % (len(pts) - 1))
        if a == "set":
            vals = {}
            for k, lim in (("x", 1000.0), ("y", 1000.0), ("z", 1000.0), ("angle", 180.0)):
                if k in d:
                    v = float(d[k])
                    if not (math.isfinite(v) and abs(v) <= lim):
                        raise ValueError("%s must be a number between -%g and %g" % (k, lim, lim))
                    vals[k] = round(v, 1)
            pts[idx].update(vals)
            self._points_save()
        elif a == "here":
            with self.sim_lock:
                x, y, z, th = self.sim.pose()
            pts[idx].update({"x": round(x, 1), "y": round(y, 1), "z": round(z, 1), "angle": round(th, 1)})
            self._points_save()
        elif a == "enable":
            pts[idx]["on"] = bool(d.get("on"))
            self._points_save()
        elif a == "go":
            self._points_check_free()
            if self.esp_state():
                raise ValueError("this moves the virtual arm (and the real one in mirror mode); it is not sent to a board running Our firmware")
            self.points_loop = False
            p = pts[idx]
            self.load_sequence(SEQ.validate({"format": SEQ.FORMAT, "poses": [{"x": p["x"], "y": p["y"], "z": p["z"], "angle": p["angle"], "action": None}]}), False, True)
            self.log("hub", "points: moving to %s (%.0f, %.0f, %.0f, tool %.0f deg)" % (p["name"], p["x"], p["y"], p["z"], p["angle"]))
        elif a == "loop":
            if d.get("on"):
                self._points_check_free()
                if self.esp_state():
                    raise ValueError("looping runs on the virtual arm (and the real one in mirror mode), not on a board running Our firmware")
                chosen = [p for p in pts if p["on"]]
                if len(chosen) < 2:
                    raise ValueError("switch on at least two points to move between")
                with self.sim_lock:
                    bad = [p["name"] for p in chosen if not self.sim.ik_report((p["x"], p["y"], p["z"], p["angle"]))["reachable"]]
                if bad:
                    raise ValueError("the arm cannot reach point %s: see why under the point" % ", ".join(bad))
                with self.sim_lock:
                    self.sim.stop_replay()
                    self.sim.saved[:] = [(p["x"], p["y"], p["z"], p["angle"]) for p in chosen]
                    self.sim.saved_speed[:] = []
                    self.cell.saved_magnet = [False] * len(chosen)
                    self.cell.reset_keys_on_lap = False
                    self.cell.last_applied = None
                    self._run_once = None
                    self.sim.start_replay(wait_arrival=True, once=False)
                self.points_loop = True
                self.log("hub", "points: moving between %s in a loop" % ", ".join(p["name"] for p in chosen))
            else:
                self._points_stop()
        elif a == "stop":
            self._points_stop()
        else:
            raise ValueError("unknown action")
        return self.points_status()

    def _points_stop(self):
        if self.points_loop:
            self.points_loop = False
        with self.sim_lock:
            self.sim.stop_replay()
        self._run_once = None
        self.log("hub", "points: stopped (the arm stays where it is)")

    # ---- mirror mode: the virtual arm does the inverse kinematics, home, sequences and speeds; the board follows its joint angles ----
    def _mirror_degrees(self):
        with self.sim_lock:
            degs = self.sim.servo_degrees()
        if self._mirror_angle:                      # the servo-mapping check: this joint straight to the commanded servo degrees, the others stay where the model has them
            joint, deg = self._mirror_angle
            degs[joint] = deg
        t = self._mirror_test
        if t:
            joint, t0 = t
            el = time.time() - t0
            if el >= 6.0:
                self._mirror_test = None
            else:
                degs[joint] = max(0.0, min(180.0, degs[joint] + 8.0 * math.sin(2 * math.pi * el / 6.0)))     # a slow +-8 degree nudge of ONE joint
        return degs

    def _board_ik_active(self):
        """True when the board's sketch runs the inverse kinematics and has the calibration, and nothing needs single joints driven
        (the nudge test and the servo-mapping check speak in joint angles) and the hub's tool length is the one the board's IK has."""
        with self.sim_lock:
            tool_ok = abs(self.sim.tool - self.BOARD_TOOL_MM) < 0.01
        return bool(self.link.ik_ready and tool_ok and not self._mirror_test and not self._mirror_angle)

    def _mirror_pose(self):
        with self.sim_lock:
            return list(self.sim.pose())

    def _mirror_prediction(self):
        """Where the tool tip should end up at the test pose under each candidate servo mapping, given the pulse range the board is using now."""
        board = tuple(self.link.board_pulse) if self.link.board_pulse else SERVOMAP.SAM
        with self.sim_lock:
            limits, tool = [list(p) for p in self.sim.limits], self.sim.tool
        out = SERVOMAP.predict_tip(board=board, limits=limits, tool=tool, inverted=[l[0] > l[1] for l in limits])
        out["board"] = list(board)
        return out

    def mirror_status(self):
        return {"on": bool(self.mirror["on"]), "active": bool(self.mirror["on"] and self.armed and self.link.connected),
                "testing": (self._mirror_test[0] + 1) if self._mirror_test else None,
                "angle": ({"joint": self._mirror_angle[0] + 1, "deg": self._mirror_angle[1]} if self._mirror_angle else None),
                "pulse": list(self.link.board_pulse) if self.link.board_pulse else None,
                "ik": "board" if self._board_ik_active() else "hub", "board_ik": self.link.board_ik, "cal_source": self.cal_source,
                "tool": self.sim.tool, "board_tool": self.BOARD_TOOL_MM}

    def mirror_action(self, d):
        a = d.get("action")
        if a == "on":
            self.mirror["on"] = True
            self.log("hub", "mirror mode ON: the real arm will follow the virtual arm whenever the output is ARMED")
        elif a == "off":
            self.mirror["on"] = False
            self._mirror_test = self._mirror_angle = None
            self.link.update(NEUTRAL)               # stick frame: the board stops mirroring and holds where it is
            self.log("hub", "mirror mode off")
        elif a == "test":
            joint = int(d.get("joint") or 0)
            if joint not in (1, 2, 3, 4):
                raise ValueError("joint must be 1 to 4")
            if not (self.mirror["on"] and self.armed and self.link.connected):
                raise ValueError("switch mirror mode on, arm the output and have the board on USB first")
            self._mirror_test = (joint - 1, time.time())
            self.log("hub", "mirror test: nudging joint %d by +-8 degrees, watch which part of the arm moves" % joint)
        elif a == "angle":
            joint, deg = int(d.get("joint") or 0), float(d.get("deg"))
            if joint not in (1, 2, 3, 4) or not (30.0 <= deg <= 150.0):
                raise ValueError("joint 1 to 4, and an angle from 30 to 150 degrees")
            if not (self.mirror["on"] and self.armed and self.link.connected):
                raise ValueError("switch mirror mode on, arm the output and have the board on USB first")
            self._mirror_test = None
            self._mirror_angle = (joint - 1, deg)
            self.log("hub", "servo check: joint %d held at %g servo degrees, measure where it really is" % (joint, deg))
        elif a == "angle_off":
            self._mirror_angle = None
            self.log("hub", "servo check: joint released back to the virtual arm")
        elif a == "predict":
            return dict(self.mirror_status(), prediction=self._mirror_prediction())
        elif a == "test_pose":
            if not (self.mirror["on"] and self.armed and self.link.connected):
                raise ValueError("switch mirror mode on, arm the output and have the board on USB first")
            self._points_check_free()
            x, y, z, ang = SERVOMAP.TEST_POSE
            self._mirror_angle = self._mirror_test = None
            self.points_loop = False
            self.load_sequence(SEQ.validate({"format": SEQ.FORMAT, "poses": [{"x": x, "y": y, "z": z, "angle": ang, "action": None}]}), False, True)
            self.log("hub", "servo check: moving to the test pose (%g out, %g up): measure the tool tip with a ruler" % (y, z))
        elif a == "judge":
            pred = self._mirror_prediction()
            return dict(self.mirror_status(), prediction=pred, judge=SERVOMAP.judge(d.get("reach"), d.get("height"), pred))
        elif a == "fit":
            board = tuple(self.link.board_pulse) if self.link.board_pulse else SERVOMAP.SAM
            fit = SERVOMAP.fit_pulse_range(d.get("cmd_a", 45), d.get("meas_a"), d.get("cmd_b", 135), d.get("meas_b"), board)
            return dict(self.mirror_status(), fit=fit, board_used=list(board))
        elif a == "pulse":
            lo, hi = float(d.get("lo")), float(d.get("hi"))
            if not (400 <= lo and hi <= 2600 and hi - lo >= 600):
                raise ValueError("pulse range: 400 <= lo, hi <= 2600 and at least 600 us apart")
            if not self.link.command("!pulse %d,%d" % (round(lo), round(hi))):
                raise ValueError("the USB link is not open")
            self.log("hub", "mirror pulse range sent to the board: %d to %d us" % (lo, hi))
        elif a == "map":
            m = [int(v) for v in (d.get("map") or [])]
            if sorted(m) != [0, 1, 2, 3]:
                raise ValueError("map is the four channel numbers 0 to 3, each once: joint 1..4 -> channel")
            if not self.link.command("!map %d,%d,%d,%d" % tuple(m)):
                raise ValueError("the USB link is not open")
            self.log("hub", "joint -> channel map sent to the board: %s" % m)
        else:
            raise ValueError("unknown action")
        return self.mirror_status()

    # ---- safety ----------------------------------------------------------------------------
    def set_armed(self, on):
        with self.lock:
            on = bool(on)
            if on == self.armed:
                return
            self.armed = on
            self.armed_since = time.time() if on else None
        if not on:
            self._mirror_angle = self._mirror_test = None
            self.link.update(NEUTRAL)               # stop any motion immediately over USB
        res = self.set_forward(on)
        self.log("hub", f"output {'ARMED' if on else 'SAFE'}"
                        + ("" if res else "  (warning: could not change forwarding on the local server)"))

    def _enforce(self):
        """Keep the server's forwarding switch matching the armed flag, and apply the dead-man rule."""
        fe = self.cache["server"].get("forward_enabled")
        if fe is not None and fe != self.armed:
            self.set_forward(self.armed)
        with self.lock:
            idle = not self.clients and time.time() - self.last_client_t > DEADMAN_S
        if self.armed and idle:
            self.log("hub", "no dashboard connected: returning to SAFE")
            self.set_armed(False)

    # ---- processes ----------------------------------------------------------------------------
    def run(self, src, cmd, cwd=None, env=None, timeout=900):
        """Run a command, streaming its output into the log. Returns (returncode, output text)."""
        self.log(src, "$ " + " ".join(str(c) for c in cmd))
        env = dict(os.environ, **(env or {}))
        env["PATH"] = str(Path(DOCKER).parent) + os.pathsep + env.get("PATH", "")
        out = []
        try:
            p = subprocess.Popen([str(c) for c in cmd], cwd=cwd or ROOT, env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, errors="replace", creationflags=NOWIN)
            t0 = time.time()
            for line in p.stdout:
                line = line.rstrip()
                out.append(line)
                if line.strip():
                    self.log(src, line[:300])
                if time.time() - t0 > timeout:
                    p.kill()
                    self.log(src, "timed out")
                    break
            p.wait(timeout=10)
            return p.returncode, "\n".join(out)
        except FileNotFoundError:
            self.log(src, f"command not found: {cmd[0]}")
            return 127, ""
        except Exception as e:
            self.log(src, f"error: {e}")
            return 1, ""

    def _quick(self, cmd, timeout=6):
        try:
            r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True, errors="replace",
                               timeout=timeout, creationflags=NOWIN, cwd=ROOT,
                               env=dict(os.environ, PATH=str(Path(DOCKER).parent) + os.pathsep + os.environ.get("PATH", "")))
            return r.returncode, (r.stdout or "").strip()
        except Exception:
            return 1, ""

    def scan_processes(self):
        procs = {}
        if not IS_WIN:
            return procs
        rc, out = self._quick(["powershell", "-NoProfile", "-Command",
                               "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | "
                               "ForEach-Object { \"$($_.ProcessId)|$($_.CommandLine)\" }"], timeout=12)
        for line in out.splitlines():
            pid, _, cmd = line.partition("|")
            if "controller_gui.py" in cmd:
                procs["gui"] = int(pid)
            elif re.search(r"server\.py", cmd) and "hub.py" not in cmd:
                procs["server_python"] = int(pid)
        return procs

    def kill_pid(self, pid):
        if IS_WIN:
            self._quick(["taskkill", "/PID", str(pid), "/T", "/F"])
        else:
            try:
                os.kill(pid, 15)
            except OSError:
                pass

    def env_forward(self):
        """FORWARD_URL from .env for a hub-launched Python server. Never sent to the browser."""
        f = ROOT / ".env"
        if f.exists():
            for line in f.read_text(encoding="utf-8", errors="replace").splitlines():
                if line.startswith("FORWARD_URL="):
                    return line.split("=", 1)[1].strip()
        return ""

    # ---- service actions --------------------------------------------------------------------------
    def service_action(self, name, action):
        """Start a service action in a thread. Returns (ok, message)."""
        if name not in ("docker_engine", "server_docker", "server_python", "gui"):
            return False, "unknown service"
        if action not in ("start", "stop", "restart"):
            return False, "unknown action"
        with self.lock:
            if name in self.busy:
                return False, f"{name} is busy ({self.busy[name]})"
            if name == "server_python" and action != "stop" and self.cache["server"].get("health") \
                    and "server_python" not in self.cache["procs"]:
                return False, "a local server is already running (Docker or external). Stop it first."
            if name == "server_docker" and action != "stop" and "server_python" in self.cache["procs"]:
                return False, "the Python server is using port 8000. Stop it first."
            self.busy[name] = f"{action}ing"
        threading.Thread(target=self._do_service, args=(name, action), daemon=True).start()
        return True, f"{name}: {action} started"

    def _do_service(self, name, action):
        try:
            if name == "docker_engine":
                if action == "stop":
                    self.log(name, "stopping Docker Desktop from here is not supported; quit it from the tray icon")
                elif os.path.exists(DOCKER_DESKTOP):
                    subprocess.Popen([DOCKER_DESKTOP], creationflags=NOWIN)
                    self.log(name, "Docker Desktop is starting (the engine takes about 10 s)")
                else:
                    self.log(name, "Docker Desktop not found")
            elif name == "server_docker":
                if action == "stop":
                    self.run(name, [DOCKER, "compose", "down"])
                elif action == "restart":
                    self.run(name, [DOCKER, "compose", "up", "-d", "--build", "--force-recreate"])
                else:
                    self.run(name, [DOCKER, "compose", "up", "-d", "--build"])
            elif name == "server_python":
                if action in ("stop", "restart"):
                    pid = self.cache["procs"].get("server_python")
                    if pid:
                        self.kill_pid(pid)
                    self.procs.pop("server_python", None)
                    self.log(name, "stopped")
                if action in ("start", "restart"):
                    env = {}
                    fwd = self.env_forward()
                    if fwd:
                        env["FORWARD_URL"] = fwd
                    p = subprocess.Popen([sys.executable, str(ROOT / "server.py"), "--port", "8000"], cwd=ROOT,
                                         env=dict(os.environ, **env), stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL, creationflags=NOWIN)
                    self.procs["server_python"] = p
                    self.log(name, f"started (pid {p.pid})" + ("" if fwd else "; no FORWARD_URL in .env, not forwarding"))
            elif name == "gui":
                if action in ("stop", "restart"):
                    pid = self.cache["procs"].get("gui")
                    if pid:
                        self.kill_pid(pid)
                    self.log(name, "stopped")
                if action in ("start", "restart"):
                    flags = (subprocess.CREATE_NEW_PROCESS_GROUP if IS_WIN else 0)
                    p = subprocess.Popen([sys.executable, str(ROOT / "controller_gui.py"), "--url", self.server + "/input", "--no-serial"],
                                         cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags)
                    self.log(name, f"started (pid {p.pid}), sending sticks to the server only: the USB port stays with this hub, which sends to the board while ARMED (jog) or in mirror mode.")
        finally:
            self.cache["procs"] = self.scan_processes() or self.cache["procs"]
            time.sleep(0.5)
            with self.lock:
                self.busy.pop(name, None)

    # ---- firmware ----------------------------------------------------------------------------------
    # firmware profiles: what can be flashed onto the board from the dashboard
    FW_PROFILES = {
        "sam": {"label": "Sam's firmware", "kind": "stock", "fqbn": "esp32:esp32:uPesy_wroom", "libs": ["ESP32Servo"], "git_libs": {"ESP32Ping": "https://github.com/marian-craciunescu/ESP32Ping.git"},
                "desc": "The UWE-RArm firmware exactly as shipped (firmware/stock/src, untouched). Polls the end server itself."},
        "ours": {"label": "Our firmware", "kind": "ours", "fqbn": "esp32:esp32:uPesy_wroom", "libs": ["ESP32Servo"], "git_libs": {"ESP32Ping": "https://github.com/marian-craciunescu/ESP32Ping.git"},
                 "desc": "Sam's firmware plus our three patches: damper and failsafes, sequences and magnet, a speed per sequence pose."},
        "usb": {"label": "USB jog sketch", "kind": None, "fqbn": FQBN, "libs": ["ArduinoJson"],
                "desc": "Our older sketch (arduino/arm_receiver): jog control over USB or WiFi."},
    }
    FWB = ROOT / ".fwbuild"

    def _fw_state(self):
        try:
            return json.loads((self.FWB / "state.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {"flashed": None, "profiles": {}}

    def _fw_save(self, st):
        self.FWB.mkdir(exist_ok=True)
        (self.FWB / "state.json").write_text(json.dumps(st), encoding="utf-8")

    def _fw_secrets(self):
        """(ssid, password, host) for the stock-based firmwares, taken from secrets.h and .env. Never sent to the browser."""
        txt = (SKETCH / "secrets.h").read_text(encoding="utf-8", errors="replace") if (SKETCH / "secrets.h").exists() else ""

        def g(k):
            m = re.search(r'#define\s+%s\s+"([^"]*)"' % k, txt)
            return m.group(1) if m else ""
        from urllib.parse import urlparse
        ssid, pw, host = g("WIFI_SSID"), g("WIFI_PASS"), urlparse(self.env_forward()).hostname or ""
        if not (ssid and host):
            raise ValueError("needs WIFI_SSID in arduino/arm_receiver/secrets.h and FORWARD_URL in .env (its host is the end server the firmware polls)")
        return ssid, pw, host

    def _fw_hash(self, pid):
        import hashlib
        h = hashlib.sha1()
        prof = self.FW_PROFILES[pid]
        roots = [SKETCH] if prof["kind"] is None else [ROOT / "firmware" / "stock" / "src", ROOT / "firmware" / "stock" / "patches"]
        for r in roots:
            for f in sorted(r.rglob("*")):
                if f.is_file():
                    h.update(f.relative_to(r).as_posix().encode() + f.read_bytes())
        if prof["kind"] is not None:
            h.update(("|".join(self._fw_secrets()) + prof["kind"]).encode())
        return h.hexdigest()[:12]

    _fw_stale_cache, _fw_stale_t = {}, 0.0         # "changed since the last build" needs every firmware file hashed: do it every 5 s, not every status update

    def _fw_is_stale(self, pid, rec):
        if time.time() - self._fw_stale_t > 5.0:
            self._fw_stale_cache, self._fw_stale_t = {}, time.time()
        if (pid, rec.get("hash")) not in self._fw_stale_cache:
            try:
                self._fw_stale_cache[(pid, rec.get("hash"))] = bool(rec) and rec.get("hash") != self._fw_hash(pid)
            except (ValueError, OSError):
                self._fw_stale_cache[(pid, rec.get("hash"))] = False
        return self._fw_stale_cache[(pid, rec.get("hash"))]

    def fw_profiles(self):
        st = self._fw_state()
        out = []
        for pid, pr in self.FW_PROFILES.items():
            rec = st["profiles"].get(pid) or {}
            stale = self._fw_is_stale(pid, rec)
            out.append({"id": pid, "label": pr["label"], "desc": pr["desc"], "built": bool(rec), "stale": stale,
                        "built_t": rec.get("built_t"), "flash_pct": rec.get("flash_pct"), "ram_pct": rec.get("ram_pct"),
                        "on_board": (st.get("flashed") or {}).get("id") == pid})
        return out

    def _fw_prepare(self, pid):
        """Write the sketch folder for a profile. Returns (sketch_dir, out_dir)."""
        prof = self.FW_PROFILES[pid]
        out_dir = self.FWB / pid / "out"
        if prof["kind"] is None:
            return SKETCH, out_dir
        import tempfile
        import emu_runner
        ssid, pw, host = self._fw_secrets()
        sk = self.FWB / pid / pid
        shutil.rmtree(sk, ignore_errors=True)
        sk.mkdir(parents=True)
        with tempfile.TemporaryDirectory() as td:                    # patched trees are built outside the repo
            src = emu_runner.emu_mod.firmware_src(prof["kind"], td)
            for f in os.listdir(src):
                if f != "SecretsExample.h":
                    shutil.copy2(os.path.join(src, f), sk / f)
        (sk / (pid + ".ino")).write_text("// Entry point only: the firmware is main.cpp in this folder.\n", encoding="utf-8")
        (sk / "Secrets.h").write_text('const char* WIFI_SSID = "%s";\nconst char* WIFI_PASSWORD = "%s";\nconst char* HOST = "%s";\n' % (ssid, pw, host), encoding="utf-8")
        return sk, out_dir

    def firmware_job(self, action, profile="usb"):
        if action not in ("compile", "upload", "swap") or profile not in self.FW_PROFILES:
            return False, "unknown action or firmware"
        with self.lock:
            if self.fw["state"] == "running":
                return False, "a firmware job is already running"
            if not os.path.exists(ARDUINO) and not shutil.which("arduino-cli"):
                return False, "arduino-cli not found"
            try:
                if self.FW_PROFILES[profile]["kind"] is None:
                    if not (SKETCH / "secrets.h").exists():
                        return False, "arduino/arm_receiver/secrets.h is missing: copy secrets.example.h and fill it in"
                else:
                    self._fw_secrets()
            except ValueError as e:
                return False, str(e)
            if action in ("upload", "swap"):
                if self.armed:
                    return False, "switch the output to SAFE before flashing (the board resets)"
                if not self.link.find_port():
                    return False, "no ESP32 found on USB (flashing needs the cable: the WiFi link cannot flash)"
            self.fw.update(state="running", action=action, profile=profile, detail="", finished_t=None)
        self.publish("status", self.snapshot())
        threading.Thread(target=self._fw_run, args=(action, profile), daemon=True).start()
        return True, f"{action} {self.FW_PROFILES[profile]['label']} started"

    def _fw_run(self, action, pid):
        ok, detail = False, ""
        prof = self.FW_PROFILES[pid]
        try:
            st = self._fw_state()
            rec = st["profiles"].get(pid) or {}
            need_build = action == "compile" or not rec or rec.get("hash") != self._fw_hash(pid)
            rc = 0
            if need_build:
                sketch, out_dir = self._fw_prepare(pid)
                if prof["libs"]:
                    self.run("firmware", [ARDUINO, "lib", "install", *prof["libs"]])
                for lib, url in prof.get("git_libs", {}).items():     # not in the Arduino index: the URL platformio.ini names
                    _, have = self.run("firmware", [ARDUINO, "lib", "list"])
                    if lib not in (have or ""):
                        self.run("firmware", [ARDUINO, "config", "set", "library.enable_unsafe_install", "true"])
                        self.run("firmware", [ARDUINO, "lib", "install", "--git-url", url])
                rc, out = self.run("firmware", [ARDUINO, "compile", "--fqbn", prof["fqbn"], "--output-dir", str(out_dir), str(sketch)])
                if rc == 0:
                    rec = {"hash": self._fw_hash(pid), "built_t": time.time()}
                    m = re.search(r"Sketch uses (\d+) bytes \((\d+)%\)", out or "")
                    rec["flash_pct"] = int(m.group(2)) if m else None
                    m = re.search(r"Global variables use (\d+) bytes \((\d+)%\)", out or "")
                    rec["ram_pct"] = int(m.group(2)) if m else None
                    st["profiles"][pid] = rec
                    self._fw_save(st)
                    self.fw.update(flash_pct=rec["flash_pct"], ram_pct=rec["ram_pct"])
            else:
                self.log("firmware", f"{prof['label']}: nothing changed since the last build, flashing the saved binary (no recompile)")
                sketch, out_dir = (SKETCH if prof["kind"] is None else self.FWB / pid / pid), self.FWB / pid / "out"
            if rc == 0 and action in ("upload", "swap"):
                port = self.link.find_port()
                if not port:
                    raise RuntimeError("the USB cable was unplugged: flashing needs it")
                self.link.suspend()                       # release the COM port for esptool
                time.sleep(0.6)
                cmd = [ARDUINO, "upload", "-p", port, "--fqbn", prof["fqbn"], "--input-dir", str(out_dir), str(sketch)]
                rc, out = self.run("firmware", cmd)
                if rc != 0 and ("No more data to read" in out or "Hash of data verified" in out):
                    self.log("firmware", "retrying once (intermittent final-step serial error)")
                    rc, out = self.run("firmware", cmd)
                self.link.resume()
                if rc == 0:
                    st = self._fw_state()
                    st["flashed"] = {"id": pid, "t": time.time()}
                    self._fw_save(st)
            ok = rc == 0
            detail = "ok" if ok else f"failed (exit {rc}); see the log"
        except Exception as e:
            detail = f"error: {e}"
            self.link.resume()
        self.fw.update(state="ok" if ok else "fail", detail=detail, finished_t=time.time())
        self.log("firmware", f"{action} {prof['label']}: {detail}")
        self.publish("status", self.snapshot())

    # ---- info ---------------------------------------------------------------------------------------
    def git_info(self):
        def g(*a):
            return self._quick(["git", "-C", str(ROOT), *a], timeout=6)
        rc, branch = g("rev-parse", "--abbrev-ref", "HEAD")
        if rc != 0:
            return {"available": False}
        _, log = g("log", "-12", "--pretty=format:%h%x1f%s%x1f%cr")
        commits = [dict(zip(("h", "s", "when"), l.split("\x1f"))) for l in log.splitlines() if l]
        _, st = g("status", "--porcelain")
        _, ahead = g("log", "origin/main..HEAD", "--oneline")
        _, url = g("remote", "get-url", "origin")
        url = re.sub(r"^(https://)[^@/]+@", r"\1", url)
        url = re.sub(r"\.git$", "", url)
        return {"available": True, "branch": branch, "dirty": len([l for l in st.splitlines() if l.strip()]),
                "unpushed": len([l for l in ahead.splitlines() if l.strip()]), "commits": commits, "url": url}

    def _poller(self):
        n = 0
        while not self.stop.is_set():
            c = self.cache
            try:
                health = False
                try:
                    self._http("/health", timeout=0.6)
                    health = True
                except Exception:
                    pass
                srv = {"health": health}
                if health:
                    try:
                        s = self._http("/state", timeout=0.8)
                        srv["forward_enabled"] = s.get("forward_enabled")
                        srv["forward"] = s.get("forward")
                        srv["count"] = s.get("count")
                    except Exception:
                        pass
                c["server"] = srv
                self._sync_arm_channel(health)
                if n % 5 == 0:
                    rc, _ = self._quick([DOCKER, "info"], timeout=8)
                    c["docker_engine"] = "running" if rc == 0 else "stopped"
                    if rc == 0:
                        _, ps = self._quick([DOCKER, "ps", "--filter", "name=arm-server", "--format", "{{.Status}}"])
                        c["docker_container"] = ps
                    else:
                        c["docker_container"] = ""
                    procs = self.scan_processes()
                    for k, p in list(self.procs.items()):          # hub-launched server counts even if scan fails
                        if p.poll() is None:
                            procs.setdefault(k, p.pid)
                    c["procs"] = procs
                if n % 15 == 0:
                    c["git"] = self.git_info()
                self._enforce()
            except Exception as e:
                self.log("hub", f"poller error: {e}")
            n += 1
            self.stop.wait(1.0)

    def service_states(self):
        c = self.cache
        health = c["server"].get("health", False)
        eng = c["docker_engine"]
        cont = c["docker_container"]
        procs = c["procs"]
        s = {
            "docker_engine": {"state": "running" if eng == "running" else ("stopped" if eng == "stopped" else "unknown"),
                              "detail": "engine answering" if eng == "running" else "Docker Desktop is not running"},
            "server_docker": {"state": "running" if cont else "stopped",
                              "detail": cont or ("no container" if eng == "running" else "needs Docker")},
            "server_python": {"state": "running" if "server_python" in procs else "stopped",
                              "detail": f"pid {procs['server_python']}" if "server_python" in procs else "not running"},
            "gui": {"state": "running" if "gui" in procs else "stopped",
                    "detail": f"pid {procs['gui']}" if "gui" in procs else "not running"},
        }
        for k, v in s.items():
            if k in self.busy:
                v["state"], v["detail"] = "busy", self.busy[k]
        mode = "none"
        if health:
            mode = "docker" if cont else ("python" if "server_python" in procs else "external")
        return s, mode

    def snapshot(self):
        services, mode = self.service_states()
        srv = self.cache["server"]
        fwd = srv.get("forward") or {}
        age = None if not self.latest_recv else round(time.time() - self.latest_recv, 2)
        link = self.link
        return {
            "t": round(time.time(), 2),
            "armed": self.armed, "armed_since": self.armed_since, "forward_control": self.forward_control,
            "mirror": self.mirror_status(),
            "pps": self.pps(), "last_packet_age": age, "packet_count": self.latest_count,
            "server": {"health": srv.get("health", False), "stream": self.stream_ok, "mode": mode,
                       "forward_enabled": srv.get("forward_enabled"),
                       "forward": {"configured": bool(fwd) or bool(self.env_forward()),
                                   "ok": fwd.get("ok"), "sent": fwd.get("sent"), "failed": fwd.get("failed"),
                                   "error": (fwd.get("error") or "")[:80] or None}},
            "services": services,
            "usb": {"enabled": link.enabled, "phase": link.phase, "port": link.device or link.find_port_cached(),
                    "status": link.status, "rate": link.board_rate, "range": link.board_range},
            "board_wifi": self.board_listener.snapshot() if self.board_listener else {"listening": False},
            "board": dict(self.board, age=None if not self.board["last_line_t"] else round(time.time() - self.board["last_line_t"], 1)),
            "firmware": dict(self.fw, profiles=self.fw_profiles(), secrets_present=(SKETCH / "secrets.h").exists(),
                             age=None if not self.fw["finished_t"] else round(time.time() - self.fw["finished_t"], 1)),
            "config": {"env_present": (ROOT / ".env").exists(), "forward_url_set": bool(self.env_forward())},
            "project": self.cache["git"],
            "sim": self._sim_snapshot(),
            "emu": self.emu.snapshot(),
            "phone_url": self.phone_url,
            "phone_url_https": self.phone_url_https,
            "build": BUILD,
            "emu_input": self.emu.running and self.forward_control is True and not self.armed,
        }

    # ---- our firmware's own reports (/armstate): when they are fresh, the ESP, not the virtual arm, is the truth ----
    def _esp_loop(self):
        while not self.stop.is_set():
            time.sleep(0.4)
            try:
                r = self._http("/armstate", timeout=0.8)
                st = r.get("state")
                if st and "seqDone" in st and r.get("age_s") is not None and r["age_s"] < 15.0:
                    self._esp = (st, time.time() - r["age_s"])
                else:
                    self._esp = None
                if self._esp_run is not None:
                    if st and st.get("seqDone") == self._esp_run:
                        self._esp_run, self._run_once = None, None
                        self.log("hub", "sequence finished on the ESP32")
                    elif self._esp is None:                      # the ESP went quiet mid-run
                        self._esp_run, self._run_once = None, None
                        self.log("hub", "lost the ESP32 during a sequence")
            except Exception:
                self._esp = None

    def esp_state(self):
        e = self._esp
        return e[0] if e and time.time() - e[1] < 15.0 else None

    def _esp_tap(self, button, hold=0.8):
        try:
            self._http("/input", dict(IDLE_WIRE, **{button: True}))
            time.sleep(hold)
            self._http("/input", dict(IDLE_WIRE))
        except Exception as e:
            self.log("hub", "could not send %s to the ESP32: %s" % (button, e))

    # ---- state packet and the task runner ------------------------------------------------------------------------
    def state_packet(self):
        """What an AI needs to see before it decides: pose, home grip, magnet, what is running, where the keys are."""
        from sim.armsim import forward_kin
        esp = self.esp_state()
        if esp:                                   # our firmware is reporting: believe it rather than the virtual arm
            with self.sim_lock:
                held = self.cell.held
                keys = {str(k["id"]): self.cell.where(k) for k in self.cell.keys}
            at_home = bool(esp.get("atHome"))
            return {"format": "roobot-arm-state", "version": 1, "time": time.time(), "build": BUILD, "source": "esp32",
                    "pose": {k: esp.get(k) for k in ("x", "y", "z", "angle")}, "at_home": at_home, "magnet": bool(esp.get("magnet")),
                    "holding": held is not None, "home_grip": at_home and not esp.get("magnet") and held is None,
                    "mode": "running" if esp.get("mode") == "sequence" else esp.get("mode"), "saved": esp.get("saved"),
                    "running_step": esp.get("step") if esp.get("mode") == "sequence" else None, "keys": keys,
                    "last_run": self.last_run, "last_move": self.last_move, "task": dict(self.task),
                    "ai": "groq" if AGENT.api_key() else "keywords", "armed": self.armed, "poses": [list(p) for p in self.sim.saved]}
        with self.sim_lock:
            x, y, z, th = self.sim.pose()
            hx, hy, hz, _ = forward_kin(self.sim._home_angles(), self.sim.tool)
            at_home = math.dist((x, y, z), (hx, hy, hz)) < 8.0 and not self.sim.replaying
            held = self.cell.held
            return {"format": "roobot-arm-state", "version": 1, "time": time.time(), "build": BUILD, "source": "virtual",
                    "pose": {"x": round(x, 1), "y": round(y, 1), "z": round(z, 1), "angle": round(th, 1)},
                    "at_home": at_home, "magnet": self.cell.magnet, "holding": held is not None, "home_grip": at_home and not self.cell.magnet and held is None,
                    "mode": "running" if self.sim.replaying else self.sim.mode, "saved": len(self.sim.saved), "running_step": self.sim.replay_idx if self.sim.replaying else None,
                    "keys": {str(k["id"]): self.cell.where(k) for k in self.cell.keys}, "last_run": self.last_run, "last_move": self.last_move, "task": dict(self.task),
                    "ai": "groq" if AGENT.api_key() else "keywords", "armed": self.armed, "poses": [list(p) for p in self.sim.saved]}

    task = {"state": "idle"}
    last_run = None
    last_move = None                                # {"dx","dy","dz"} of the nudge done most recently, while it is still the last thing done (so "move it back" can reverse it)
    tune = {"state": "idle"}                        # the speed tuner (its own mode, simulation only): idle | running | done | cancelled | failed
    _tune_cancel = False
    points = None                                   # three points of the user's choosing to move between (see points_action)
    points_loop = False
    mirror = {"on": False}                          # mirror mode: the real arm (USB jog sketch) follows the virtual arm's joint angles; off until switched on
    _mirror_test = None                             # (joint index, start time) while one joint is being wiggled to find out which is which
    _mirror_angle = None                            # (joint index, servo degrees) while one joint is held at an angle to measure it against the model
    trial = {"state": "idle"}                       # a real-arm trial of tuned speeds: idle | homing | running | verdict | done | failed | aborted
    chat = []                                       # the conversation shown on the Chat page: user / ai / robot / error messages
    _chat_seq = 0

    def chat_add(self, role, text, **extra):
        """Append one message; ids only grow, so a page can ask for everything after the last id it has."""
        Hub._chat_seq += 1
        m = {"id": Hub._chat_seq, "t": round(time.time(), 2), "role": role, "text": str(text), **extra}
        self.chat.append(m)
        del self.chat[:-300]
        return m

    def chat_since(self, since=0):
        return {"messages": [m for m in self.chat if m["id"] > since], "state": self.task.get("state", "idle"),
                "ai": "groq" if AGENT.api_key() else "keywords", "voice": VOICE.available()}

    def ask(self, request):
        """Start a task from plain language: make sure the arm is at home grip, pick a sequence, run it once."""
        request = str(request).strip()[:500]
        if not request:
            raise ValueError("say what you want done")
        if PRE.is_stop(request):                       # first, before any busy check, AI or network call
            return self._stop_now(request)
        if self.task.get("state") in ("working", "running"):
            raise RuntimeError("a task is already running")
        if self.tune.get("state") == "running":
            raise RuntimeError("the speed tuner is running: wait for it or cancel it")
        if self.trial.get("state") in ("homing", "running"):
            raise RuntimeError("a speed trial is running on the arm: wait for it or abort it")
        self.task = {"state": "working", "request": request, "log": [], "error": None, "warning": None}
        self.chat_add("user", request)
        Hub._tid += 1
        threading.Thread(target=self._do_task, args=(request, Hub._tid), daemon=True).start()

    def _tlog(self, msg, state=None, chat=True):
        self.task["log"].append(msg)
        if state:
            self.task["state"] = state
        self.log("agent", msg)
        if chat:
            self.chat_add("robot", msg, state=self.task.get("state"))

    def _wait(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(0.2)
        return False

    def _fail(self, e):
        """Stop the task with a structured error the dashboards can show: {code, title, detail, hint}."""
        self.task["error"] = e
        self._tlog("%s%s" % (e["title"], (". " + e["hint"]) if e.get("hint") else ""), "failed", chat=False)
        self.chat_add("error", e["title"], detail=e.get("detail"), hint=e.get("hint"), code=e.get("code"), state="failed")

    _tid = 0                                           # task number: a stop bumps it, so a task that was running knows it must not carry on

    def _stop_now(self, request, echo=True):
        """Stop moving right now, without asking the AI: end a sequence run (virtual or on the ESP32) and any replay. The arm holds where it is."""
        Hub._tid += 1
        if echo:
            self.chat_add("user", request)
        self._run_once = None
        self.last_move = None
        try:
            if self.esp_state():
                self._http("/sequence", {"poses": []}, timeout=2.0)         # an empty list stops a run on the ESP32
                self._esp_run = None
        except Exception as e:
            self.log("hub", "could not tell the ESP32 to stop: %s" % e)
        with self.sim_lock:
            self.sim.stop_replay()
        self.task = {"state": "stopped", "request": request, "log": ["Stopped"], "error": None, "warning": None}
        self.log("agent", "stopped on request")
        self.chat_add("robot", "Stopped. The arm holds where it is.", state="stopped")

    def _send_home(self, st, release):
        """Start the move home (B). release=True switches the magnet off first, as a sequence run needs."""
        if st.get("source") == "esp32":
            if release and st["magnet"]:
                self._esp_tap("X")                  # magnet off first
            self._esp_tap("B")                      # then go home
        else:
            with self.sim_lock:
                self.sim.stop_replay()
                if release:
                    self.cell.set_magnet(False)
                self.sim.go_home()

    def _do_action(self, act, plan, st, tid=0):
        """The quick commands the AI can ask for: status, go_home, magnet. Fixed menu: no coordinates, nothing the model wrote reaches the arm."""
        bad = PRE.check_action(plan, st, st.get("source") != "esp32" or self.armed or self.emu.running)
        if bad:
            return self._fail(bad)
        if act == "status":
            return self._tlog(PRE.describe(st), "done")
        if act == "stop":
            return self._stop_now(self.task.get("request", ""), echo=False)
        if act == "move_by":
            return self._do_move(plan, st, tid)
        if act == "go_home":
            self.last_move = None
            if st["at_home"]:
                return self._tlog("Already at home", "done")
            self._tlog("Going home", "running")
            self._send_home(st, release=False)
            if not self._wait(lambda: self.state_packet()["at_home"], 30):
                return self._fail(PRE.err("HOME_FAILED", "The arm could not get to its home position", "It did not reach home within 30 s.",
                                          "Check the arm is powered and armed, then try again."))
            return self._tlog("At home", "done")
        if act == "magnet":
            want = bool(plan.get("on"))
            if bool(st["magnet"]) == want:
                return self._tlog("The magnet is already %s" % ("on" if want else "off"), "done")
            if st.get("source") == "esp32":
                self._esp_tap("X")                  # the firmware toggles on each X press
            else:
                with self.sim_lock:
                    self.cell.set_magnet(want)
            if not self._wait(lambda: bool(self.state_packet()["magnet"]) == want, 6):
                return self._fail(PRE.err("MAGNET_FAILED", "The magnet did not switch %s" % ("on" if want else "off"), "The arm did not report the change.",
                                          "Check the relay wiring and the magnet pin setting."))
            return self._tlog("Magnet %s" % ("on" if want else "off"), "done")
        return self._fail(PRE.err("INTERNAL", "Unknown action %s" % act))

    def _do_move(self, plan, st, tid):
        """A small relative nudge, run as a one-pose sequence (so it uses the same straight-line move the ESP32 already runs). Checked first, capped, operator-style."""
        if PRE.has_topic(self.task.get("request", "")):
            return self._fail(PRE.err("FREE_MOVE_NOT_FOR_KEYS", "Free movement is not used for keys, boxes or the tray",
                                      "Fetching and returning keys only run a taught, checked sequence.", "Ask for the key by name, or teach the move as a sequence."))
        pose = st.get("pose") or {}
        with self.sim_lock:
            reach = lambda x, y, z, a: bool(self.sim.ik_report((x, y, z, a))["reachable"])
            bad = PRE.check_move(plan, pose, reach)
        if bad:
            return self._fail(bad)
        d = [float(plan.get(k) or 0.0) for k in ("dx", "dy", "dz")]
        target = (float(pose["x"]) + d[0], float(pose["y"]) + d[1], float(pose["z"]) + d[2], float(pose["angle"]))
        mm = sum(c * c for c in d) ** 0.5
        self._tlog("Moving %.0f mm (dx %+.0f, dy %+.0f, dz %+.0f)" % (mm, *d), "running")
        self.load_sequence(SEQ.make_sequence([target], [False], "nudge", "free movement"), False, True)   # no magnet action: a nudge leaves the magnet as it is
        if not self._wait(lambda: self._run_once is None, 60):
            return self._fail(PRE.err("RUN_TIMEOUT", "The move did not finish", "It was still running after 60 s.", "Say stop, then go home."))
        if self._tid != tid:
            return
        self.last_move = {"dx": d[0], "dy": d[1], "dz": d[2]}
        self._tlog("Moved", "done")

    def _ensure_home(self, st, log=None):
        """Get the arm to home grip (magnet off, at home) before a run. None when it is there, else a structured error."""
        if st["home_grip"]:
            return None
        (log or self._tlog)("Not at home grip: going home first")
        self._send_home(st, release=True)
        if not self._wait(lambda: self.state_packet()["at_home"] and not self.state_packet()["magnet"], 30):
            return PRE.err("HOME_FAILED", "The arm could not get to its home position", "It did not reach home grip within 30 s.",
                           "Check the arm is powered and armed, then try again.")
        return None

    def _do_task(self, request, tid=0):
        try:
            st = self.state_packet()
            library = LIBRARY.list()
            self.chat_add("system", "Asking the AI: %d sequences in the library, arm state %s" % (len(library), json.dumps({k: st.get(k) for k in ("at_home", "holding", "last_run", "last_move")})))
            plan = AGENT.choose(request, library, st)
            if plan.get("ai_error"):
                self.task["warning"] = "Groq was not used: " + plan["ai_error"]
                self._tlog("Groq was not used: " + plan["ai_error"])
            self.chat_add("ai", plan.get("reason") or "", by=plan.get("by"), model=plan.get("model"), action=plan.get("action"), sequence=plan.get("sequence"), reverse=plan.get("reverse"),
                          on=plan.get("on"), dx=plan.get("dx"), dy=plan.get("dy"), dz=plan.get("dz"),
                          raw=plan.get("raw"), ms=plan.get("ms"), tokens=plan.get("tokens"))
            act = plan.get("action") or "run_sequence"
            if act != "run_sequence":
                self.task["plan"] = plan
                self._tlog("Chose action %s (%s, %s)" % (act, plan.get("by"), plan.get("reason") or ""), chat=False)
                return self._do_action(act, plan, st, tid)
            if not plan["sequence"]:
                return self._fail(PRE.no_sequence(request, plan))
            seq = LIBRARY.get(plan["sequence"])
            self.task["plan"] = plan
            self._tlog("Chose %s%s (%s, %s)" % (plan["sequence"], " reversed" if plan["reverse"] else "", plan["by"], plan["reason"]), chat=False)
            if plan["reverse"]:
                seq = SEQ.reverse(seq)
            problems = PRE.check(request, plan, seq, st)
            if problems:
                return self._fail(problems[0])
            bad = self._ensure_home(st)
            if self._tid != tid:                         # stopped while going home
                return
            if bad:
                return self._fail(bad)
            self._tlog("At home grip: running", "running")
            self.load_sequence(seq, False, True)
            if not self._wait(lambda: self._run_once is None, 120):
                return self._fail(PRE.err("RUN_TIMEOUT", "The sequence did not finish", "It was still running after 120 s.", "Press B to send the arm home."))
            if self._tid != tid:                         # stopped mid-run: _stop_now already told the person
                return
            time.sleep(1.5)                                  # let a released key finish falling before it is looked at
            bad = PRE.verify(seq, st, self.state_packet(), request, plan)
            if bad:
                return self._fail(bad)
            self.last_run = {"name": plan["sequence"], "reverse": plan["reverse"]}
            self.last_move = None
            fin = self.state_packet()
            self._tlog("Done", "done", chat=False)
            self.chat_add("robot", "Done", state="done", result={"at_home": fin.get("at_home"), "holding": fin.get("holding"), "magnet": fin.get("magnet"), "keys": fin.get("keys")})
        except Exception as e:                       # a task must never kill the hub
            self._fail(PRE.err("INTERNAL", "Something went wrong inside the hub", str(e)[:200], "Check the hub console."))

    # ---- speed tuner: runs the sequence headless in a private virtual arm, so it never moves the live arm or the one on screen ----
    def tune_action(self, d):
        a = d.get("action")
        name = str(d.get("name") or "")
        if a == "start":
            if self.tune.get("state") == "running":
                raise RuntimeError("a tuning run is already going")
            if self.task.get("state") in ("working", "running"):
                raise RuntimeError("the arm is busy with a task: let it finish first")
            if self.trial.get("state") in ("homing", "running"):
                raise RuntimeError("a speed trial is running on the arm")
            seq = LIBRARY.get(name)
            lim = TUNER.clean_limits(d.get("limits"))
            gens = max(5, min(150, int(d.get("generations") or 40)))
            start = [s or (self.sim.rate or 30.0) for s in SEQ.speeds_of(seq)] if d.get("from_current") else None
            self._tune_cancel = False
            self.tune = {"state": "running", "name": name, "gen": 0, "of": gens * 3, "best": None, "limits": lim, "started": time.time()}
            threading.Thread(target=self._do_tune, args=(seq, start, lim, gens), daemon=True).start()
        elif a == "cancel":
            self._tune_cancel = True
        elif a == "trial":
            self._trial_start(name, float(d.get("scale") or 0))
        elif a == "verdict":
            self._trial_verdict(str(d.get("outcome") or ""))
        elif a == "abort":
            self._trial_abort()
        elif a in ("apply", "clear"):
            if self.tune.get("state") == "running":
                raise RuntimeError("the tuner is running")
            seq = LIBRARY.get(name)
            if a == "apply":
                res = self.tune.get("result")
                if self.tune.get("name") != name or not res:
                    raise ValueError("there is no tuning result for %s: run the tuner first" % name)
                if res["tuned"]["failures"]:
                    raise ValueError("this result still has failures in the simulation, so it will not be saved")
                LIBRARY.save(SEQ.with_speeds(seq, res["speeds"]))
                self.log("hub", "tuned speeds saved into sequence %s: %s deg/s" % (name, res["speeds"]))
            else:
                LIBRARY.save(SEQ.with_speeds(seq, None))
                self.log("hub", "per-pose speeds cleared from sequence %s" % name)
        else:
            raise ValueError("unknown action")
        return self.tune_status()

    def tune_status(self, name=None):
        d = dict(self.tune)
        d["trial"] = dict(self.trial)
        n = name or self.tune.get("name") or self.trial.get("name")
        esp = self.esp_state()
        d["board"] = {"esp": bool(esp), "pose_speeds": bool(esp and esp.get("poseSpeeds")), "armed": bool(self.armed or self.emu.running),
                      "mirror": bool(self.mirror["on"] and self.link.connected)}
        if n:
            try:
                c = CALIB.load(n)
                d["calib"] = {"name": n, "limits": c["limits"], "trials": c["trials"][-8:]}
                tuned = self._tuned_speeds(n, LIBRARY.get(n))
                if tuned:
                    d["ladder"] = {"name": n, "allowed": CALIB.allowed_scale(n, tuned), "rungs": list(TUNER.LADDER)}
            except (ValueError, KeyError):
                pass
        return d

    def _arm_args(self):
        return {"tool": self.sim.tool, "arm_limits": [list(p) for p in self.sim.limits], "homes": list(self.sim.homes)}

    def _tuned_speeds(self, name, seq):
        """The speeds a trial would use: the latest clean tuning result for this sequence, else the speeds saved in its file."""
        res = self.tune.get("result") if self.tune.get("name") == name else None
        if res and not res["tuned"]["failures"]:
            return list(res["speeds"])
        saved = SEQ.speeds_of(seq)
        return [s or (self.sim.rate or 30.0) for s in saved] if any(saved) else None

    # ---- real-arm trials: the tuned speeds run on the arm, a fraction of them at first, and what the person sees corrects the limits ----
    def _trial_log(self, msg):
        self.trial.setdefault("log", []).append(msg)
        self.log("hub", "trial: " + msg)

    def _trial_start(self, name, scale):
        if self.trial.get("state") in ("homing", "running"):
            raise RuntimeError("a trial is already running")
        if self.tune.get("state") == "running":
            raise RuntimeError("the tuner is running")
        if self.task.get("state") in ("working", "running"):
            raise RuntimeError("the arm is busy with a task: let it finish first")
        seq = LIBRARY.get(name)
        tuned = self._tuned_speeds(name, seq)
        if not tuned:
            raise ValueError("tune %s first: there are no speeds to try yet" % name)
        esp = self.esp_state()
        if esp:
            if not esp.get("poseSpeeds"):
                raise ValueError("the firmware on the board does not read a speed per pose: flash Our firmware from the Firmware page first")
        elif not (self.mirror["on"] and self.link.connected):
            raise ValueError("nothing is carrying the run out on a real arm: either switch mirror mode on (Arm and board page, board on USB) "
                             "or run Our firmware so it reports from the board")
        if not (self.armed or self.emu.running):
            raise ValueError("the output is SAFE: arm it first, because a trial moves the real arm")
        allowed = CALIB.allowed_scale(name, tuned)
        if not (0.3 <= scale <= allowed + 1e-9):
            raise ValueError("that rung is not open: the highest you can run now is %d%% (the rungs are %s, and each opens once the one below went ok)"
                             % (round(allowed * 100), ", ".join("%d%%" % round(r * 100) for r in TUNER.LADDER)))
        speeds = TUNER.scaled(tuned, scale)
        lim = CALIB.load(name)["limits"]
        self.trial = {"state": "homing", "name": name, "scale": scale, "tuned": [round(v, 1) for v in tuned], "speeds": speeds, "log": [],
                      "predicted": TUNER.observed(seq, speeds, lim, **self._arm_args()), "started": time.time()}
        self._trial_log("%d%% of the tuned speeds: %s deg/s" % (round(scale * 100), speeds))
        threading.Thread(target=self._do_trial, args=(SEQ.with_speeds(seq, speeds),), daemon=True).start()

    def _do_trial(self, seq):
        tr = self.trial
        try:
            bad = self._ensure_home(self.state_packet(), self._trial_log)
            if bad:
                tr.update(state="failed", error=bad["title"] + ". " + (bad.get("hint") or ""))
                return
            if tr.get("state") == "aborted":
                return
            tr["state"] = "running"
            self._trial_log("At home grip: running")
            self.load_sequence(seq, False, True)
            if not self._wait(lambda: self._run_once is None or tr.get("state") == "aborted", 120):
                tr.update(state="failed", error="The sequence did not finish within 120 s. Press B to send the arm home.")
                return
            if tr.get("state") == "aborted":
                return
            time.sleep(1.0)
            tr["state"] = "verdict"
            self._trial_log("Finished: watch the arm and say how it went")
        except Exception as e:                           # a trial must never kill the hub
            tr.update(state="failed", error=str(e)[:300])
            self._trial_log("failed: %s" % e)

    def _trial_abort(self):
        if self.trial.get("state") not in ("homing", "running"):
            return
        self.trial["state"] = "aborted"
        try:
            self._http("/sequence", {"poses": []}, timeout=2.0)       # an empty sequence stops a run on the ESP
        except Exception:
            pass
        self._run_once = None
        self._trial_log("aborted: the run was stopped (the SAFE button or Escape also cuts the output)")

    def _trial_verdict(self, outcome):
        tr = self.trial
        if tr.get("state") != "verdict":
            raise ValueError("there is no finished trial waiting for a verdict")
        if outcome not in TUNER.OUTCOMES:
            raise ValueError("outcome must be one of " + ", ".join(TUNER.OUTCOMES))
        name = tr["name"]
        seq = LIBRARY.get(name)
        lim = CALIB.load(name)["limits"]
        seen = TUNER.observed(seq, tr["speeds"], lim, **self._arm_args())
        data, note = CALIB.record(name, tr["tuned"], tr["scale"], outcome, seen)
        nxt = CALIB.allowed_scale(name, tr["tuned"])
        if outcome == "ok":
            note = ("These speeds are proven at full strength on the arm: save them into the sequence." if tr["scale"] >= TUNER.LADDER[-1] - 1e-9
                    else "Good. The %d%% rung is open now." % round(nxt * 100))
        else:
            note += " Tune again to get speeds that respect it."
        tr.update(state="done", outcome=outcome, note=note, seen=seen, limits=data["limits"])
        self._trial_log("verdict %s: %s" % (outcome, note))

    def _do_tune(self, seq, start, lim, gens):
        def progress(g, of, best):
            self.tune.update(gen=g, of=of, best=round(best, 2))
        try:
            res = TUNER.tune(seq, start, lim, generations=gens, progress=progress, cancel=lambda: self._tune_cancel,
                             tool=self.sim.tool, arm_limits=[list(p) for p in self.sim.limits], homes=list(self.sim.homes))
            self.tune.update(state="cancelled" if self._tune_cancel else "done", result=res)
            self.log("hub", "tuner %s on %s: %.1f s -> %.1f s, %d failures, %d runs" % (self.tune["state"], self.tune["name"], res["baseline"]["time"],
                                                                                    res["tuned"]["time"], res["tuned"]["failures"], res["evals"]))
        except Exception as e:                           # a tuning run must never kill the hub
            self.tune.update(state="failed", error=str(e)[:300])
            self.log("hub", "tuner failed: %s" % e)

    def current_sequence(self, name="", description="", requires=None, ends=None):
        """The virtual arm's saved positions (with the magnet state of each) as a sequence packet."""
        with self.sim_lock:
            return SEQ.make_sequence(list(self.sim.saved), list(self.cell.saved_magnet), name, description, requires, ends)

    def load_sequence(self, pkt, reverse=False, run=False):
        """Put a sequence into the (cleared) saved list; optionally play it backwards and/or run it once."""
        seq = SEQ.validate(pkt)
        if reverse:
            seq = SEQ.reverse(seq)
        if run and self.esp_state():                   # the ESP32 runs it itself
            if not (self.armed or self.emu.running):
                raise ValueError("the output is SAFE: arm it first, because this would move the real arm")
            r = self._http("/sequence", {"poses": seq["poses"]}, timeout=2.0)
            self._run_once, self._esp_run = len(seq["poses"]), r["id"]
            self.log("hub", "sent sequence %s%s to the ESP32 (id %d)" % (seq["name"] or "(unnamed)", " (reversed)" if reverse else "", r["id"]))
            return seq
        saved, mag = SEQ.to_saved(seq)
        with self.sim_lock:
            self.sim.stop_replay()
            self.sim.saved[:] = saved
            self.sim.saved_speed[:] = SEQ.speeds_of(seq)    # per-pose speeds from the tuner (None = the damper setting)
            self.cell.saved_magnet = mag
            self.cell.reset_keys_on_lap = not run         # a one-off run starts from the world as it is: a reversed task needs the key where it was left
            self.cell.last_applied = None
            self._run_once = len(saved) if run else None
            if run:
                self.sim.start_replay(wait_arrival=True, once=True)
        if run:
            self.log("hub", "running sequence %s%s once" % (seq["name"] or "(unnamed)", " (reversed)" if reverse else ""))
        return seq

    def store_sequence(self, name="", description="", requires=None, ends=None):
        """Write the current sequence to sequences/ on this (host) machine."""
        seq = self.current_sequence(name, description, requires, ends)
        if not name:
            name = "sequence-%s" % time.strftime("%Y%m%d-%H%M%S")
            seq["name"] = name
        path = LIBRARY.save(SEQ.validate(seq))
        self.log("hub", "sequence %s (%d poses) saved to %s" % (name, seq["count"], path))
        return path, seq

    def _sim_snapshot(self):
        with self.sim_lock:
            return self._pose_snapshot()

    def emu_action(self, d):
        a = d.get("action")
        if a == "start":
            self.push_calibration()                 # the firmware reads /calibration at boot
            self.emu.start(str(d.get("firmware", "ours")), str(d.get("preset", "home_wifi")))
        elif a == "stop":
            self.emu.stop()
        elif a == "link":
            fields = {k: d[k] for k in ("preset", "rtt_ms", "jitter_ms", "loss", "tls_cpu_ms", "keepalive") if k in d}
            self.emu.set_link(str(d.get("path", "both")), **fields)
        elif a == "outage":
            self.emu.outage(str(d.get("path", "esp")), d.get("seconds", 3))
        else:
            raise ValueError("unknown action")

    def hello(self):
        return {"status": self.snapshot(), "packets": list(self.packets)[-120:],
                "board": list(self.board_lines)[-200:],
                "logs": {k: list(v)[-80:] for k, v in self.logs.items()}}

    def start(self):
        for fn in (self._stream_reader, self._pump, self._poller, self._sim_loop, self._esp_loop):
            threading.Thread(target=fn, daemon=True).start()
        threading.Thread(target=self._ticker, daemon=True).start()
        threading.Thread(target=self._boot_calibration, daemon=True).start()
        self.set_forward(False)                     # start SAFE: the server must not forward until ARMED

    def _ticker(self):
        while not self.stop.is_set():
            self.publish("status", self.snapshot())
            self.stop.wait(1.0)

    def shutdown(self):
        self.stop.set()
        self.armed = False
        try:
            self.link.update(NEUTRAL)
        except Exception:
            pass
        self.emu.stop()
        self.set_forward(True)                      # leave the server as it behaved before the hub existed
        self.link.close()


# a cached port lookup so snapshot() (every second) does not enumerate COM ports each time
_port_cache = {"t": 0.0, "v": None}


def _find_port_cached(self):
    now = time.time()
    if now - _port_cache["t"] > 2.0:
        _port_cache.update(t=now, v=self.find_port())
    return _port_cache["v"]


SerialLink.find_port_cached = _find_port_cached
HUB = None
HTTPD = None
BUILD = code_stamp()                # which code this running hub loaded
LAN_TOKEN = None                    # set with --lan: other devices on the network need this key


class Handler(BaseHTTPRequestHandler):
    server_version = "RobotArmHub/0.1"

    # ---- helpers ----
    def log_message(self, fmt, *args):
        pass

    def _json(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    _new_cookie = False

    def end_headers(self):
        if self._new_cookie:
            self.send_header("Set-Cookie", f"hubkey={LAN_TOKEN}; Path=/; Max-Age=31536000; SameSite=Lax; HttpOnly")
        super().end_headers()

    def _lan_ok(self):
        """A request from another device (phone) is allowed only with the key from the launcher's link, in the URL or a cookie."""
        if not LAN_TOKEN:
            return False
        q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query).get("key", [""])[0]
        ck = ""
        for part in (self.headers.get("Cookie") or "").split(";"):
            k, _, v = part.strip().partition("=")
            if k == "hubkey":
                ck = v
        if hmac.compare_digest(q, LAN_TOKEN):
            self._new_cookie = True
        elif not hmac.compare_digest(ck, LAN_TOKEN):
            return False
        origin = self.headers.get("Origin")
        return not origin or urllib.parse.urlparse(origin).netloc == self.headers.get("Host")

    def _is_local(self):
        return self.client_address[0] in ("127.0.0.1", "::1", "::ffff:127.0.0.1")

    def _host_ok(self):
        if not self._is_local():
            return self._lan_ok()
        host = (self.headers.get("Host") or "").split(":")[0]
        if host not in ("127.0.0.1", "localhost"):
            return False
        origin = self.headers.get("Origin")
        if origin:
            m = re.match(r"^https?://(127\.0\.0\.1|localhost)(:\d+)?$", origin)
            if not m:
                return False
        return True

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n <= 0 or n > 65536:
            return {}
        try:
            d = json.loads(self.rfile.read(n))
            return d if isinstance(d, dict) else {}
        except json.JSONDecodeError:
            return {}

    # ---- GET ----
    def do_GET(self):
        path = self.path.split("?")[0]
        public = path in ("/apple-touch-icon.png", "/icon-192.png", "/icon-512.png")      # home-screen icons are fetched without cookies
        if not public and not self._host_ok():
            if not self._is_local() and path in ("/", "/index.html"):
                return self._json(403, {"error": "open the link the launcher printed: it carries the key for this hub"})
            return self._json(403, {"error": "forbidden host"})
        if path == "/manifest.webmanifest":
            start = "/?key=%s#/arm" % LAN_TOKEN if (LAN_TOKEN and not self._is_local()) else "/#/arm"
            return self._json(200, {"name": "Arm Hub", "short_name": "Arm Hub", "start_url": start, "scope": "/", "display": "standalone",
                                    "orientation": "any", "background_color": "#0e1013", "theme_color": "#0e1013",
                                    "icons": [{"src": "/icon-192.png", "sizes": "192x192", "type": "image/png", "purpose": "any maskable"},
                                              {"src": "/icon-512.png", "sizes": "512x512", "type": "image/png", "purpose": "any maskable"}]})
        if path == "/api/status":
            return self._json(200, HUB.snapshot())
        if path == "/api/sequence":
            return self._json(200, HUB.current_sequence())
        if path == "/api/state":
            return self._json(200, HUB.state_packet())
        if path == "/api/points":
            return self._json(200, HUB.points_status())
        if path == "/api/tune":
            q = urllib.parse.parse_qs(self.path.partition("?")[2])
            try:
                return self._json(200, HUB.tune_status((q.get("name") or [None])[0]))
            except ValueError as e:
                return self._json(400, {"error": str(e)})
        if path == "/api/boardlink":
            L = HUB.board_listener
            if not L:
                return self._json(404, {"error": "the board link is off: it needs the hub started with --lan (the launcher does that)"})
            host = lan_ip()
            snippet = '#define MIRROR_HOST "%s"\n#define MIRROR_PORT %d\n#define MIRROR_KEY "%s"\n' % (host, L.port, L.key)
            return self._json(200, {"host": host, "port": L.port, "snippet": snippet})
        if path == "/api/chat":
            q = urllib.parse.parse_qs(self.path.partition("?")[2])
            try:
                since = int((q.get("since") or ["0"])[0])
            except ValueError:
                since = 0
            return self._json(200, HUB.chat_since(since))
        if path == "/api/sequences":
            return self._json(200, {"sequences": LIBRARY.list()})
        if path.startswith("/api/sequences/"):
            try:
                return self._json(200, LIBRARY.get(urllib.parse.unquote(path.rsplit("/", 1)[1])))
            except KeyError:
                return self._json(404, {"error": "no such sequence"})
            except ValueError as e:
                return self._json(400, {"error": str(e)})
        if path == "/api/events":
            return self._events()
        if path == "/":
            path = "/studio/index.html" if (STATIC / "studio" / "index.html").is_file() and "studio" in self.path else "/index.html"
        elif path in ("/legacy", "/legacy.html"):
            path = "/index.html"
        elif path in ("/studio", "/studio/"):
            path = "/studio/index.html"
        f = (STATIC / path.lstrip("/")).resolve()
        if STATIC.resolve() not in f.parents or not f.is_file():
            return self._json(404, {"error": "not found"})
        body = f.read_bytes()
        if f.name == "index.html":     # inline the icon sprite: local <use> references never flicker or re-fetch
            sprite = (STATIC / "icons.svg").read_text(encoding="utf-8")
            inner = re.sub(r"^<svg[^>]*>|</svg>\s*$", "", sprite.strip())
            body = body.replace(b"<!--ICONS-->", ('<svg width="0" height="0" style="position:absolute" aria-hidden="true">'
                                                  + inner + "</svg>").encode("utf-8"))
        self.send_response(200)
        ctype = CTYPES.get(f.suffix) or mimetypes.guess_type(str(f))[0] or "application/octet-stream"
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        q = HUB.add_client()
        try:
            self.wfile.write(f"event: hello\ndata: {json.dumps(HUB.hello(), separators=(',', ':'))}\n\n".encode())
            self.wfile.flush()
            while not HUB.stop.is_set():
                try:
                    msg = q.get(timeout=10)
                except queue.Empty:
                    msg = ": keepalive\n\n"
                self.wfile.write(msg.encode())
                self.wfile.flush()
        except OSError:
            pass
        finally:
            HUB.remove_client(q)

    # ---- POST ----
    def _transcribe(self):
        """A recording from the Chat page's microphone button (raw 16 kHz mono 16-bit PCM) -> {"text", "stop", "ms"}. The text is only shown; asking is a separate step."""
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n <= 0 or n > VOICE.MAX_BYTES:
            return self._json(400, {"error": "no recording, or one longer than %d seconds" % VOICE.MAX_SECONDS})
        raw = self.rfile.read(n)
        if not VOICE.available():
            return self._json(501, {"error": "speech to text is not installed: pip install faster-whisper"})
        try:
            return self._json(200, VOICE.transcribe(raw))
        except VOICE.VoiceError as e:
            return self._json(400, {"error": str(e)})
        except Exception as e:                         # model download failed, out of memory: say so, never take the hub down
            HUB.log("voice", "transcription failed: %s: %s" % (type(e).__name__, str(e)[:160]))
            return self._json(500, {"error": "the speech model could not run: %s" % str(e)[:120]})

    def do_POST(self):
        if not self._host_ok() or self.headers.get("X-Hub") != "1":
            return self._json(403, {"error": "forbidden"})
        if self.path.split("?")[0] == "/api/transcribe":
            return self._transcribe()
        path, d = self.path.split("?")[0], self._body()
        try:
            if path == "/api/arm":
                HUB.set_armed(bool(d.get("armed")))
                return self._json(200, {"armed": HUB.armed, "forward_control": HUB.forward_control})
            if path == "/api/input":
                st = clean_state(d)
                ctl = HUB.emu.ctl_url()             # emulator running: sticks travel over its controller link
                virtual = ctl is not None and HUB.forward_control is True
                if not HUB.armed and not virtual:
                    return self._json(200, {"sent": False, "reason": "safe"})
                try:
                    if ctl:
                        req = urllib.request.Request(ctl + "/input", data=json.dumps(wire_from_internal(st)).encode(), method="POST",
                                                     headers={"Content-Type": "application/json"})
                        urllib.request.urlopen(req, timeout=3).read()
                    else:
                        HUB._http("/input", wire_from_internal(st), timeout=1.0)
                    return self._json(200, {"sent": True})
                except Exception:
                    return self._json(502, {"sent": False, "reason": "local server unreachable"})
            if path == "/api/sim":              # the virtual arm: damper, tool length, calibration, home, saved positions
                push = push_rate = False
                if d.get("calibration_from") == "end":
                    ok, msg = HUB.load_end_calibration()
                    return self._json(200 if ok else 409, {"ok": ok, "error": None if ok else msg, "message": msg, "sim": HUB._sim_snapshot()})
                if ("calibration" in d or "tool" in d) and HUB.armed and HUB.mirror["on"]:
                    return self._json(409, {"error": "switch the output to SAFE first: changing the calibration or the tool length would move the real arm"})
                with HUB.sim_lock:
                    if "rate" in d:
                        push_rate = True
                        HUB.sim.set_rate(d["rate"])
                    if "stick_speed" in d:
                        HUB.sim.set_stick_speed(d["stick_speed"])
                    if "move_speed" in d:
                        HUB.sim.set_move_speed(d["move_speed"])
                    if "tool" in d:
                        HUB.sim.set_tool(d["tool"])
                    if "replay" in d:
                        r = d["replay"]
                        HUB.sim.toggle_replay() if r == "toggle" else (HUB.sim.start_replay() if r else HUB.sim.stop_replay())
                    if "model" in d:
                        HUB.sim.set_model(d["model"], bool(d.get("patch")))
                    if d.get("save_here"):
                        if not HUB.sim.save_here():
                            return self._json(409, {"error": "the saved list is full (20 poses)"})
                        HUB.cell.saved_magnet = (HUB.cell.saved_magnet + [HUB.cell.magnet])[:len(HUB.sim.saved)]
                    if d.get("home"):
                        HUB.sim.go_home()
                    if d.get("clear_saved"):
                        HUB.sim.clear_saved()
                    if "calibration" in d:
                        c = d["calibration"]
                        if not isinstance(c, dict):
                            raise ValueError("calibration must be an object")
                        HUB.sim.set_calibration(c.get("jointLimits"), c.get("jointHomes"))
                        HUB.cal_source = "set by hand"
                        push = True
                if push:
                    HUB.push_calibration()
                    HUB._link_cal()
                if push_rate:
                    HUB.push_speed()
                return self._json(200, {"ok": True, "sim": HUB._sim_snapshot()})
            if path == "/api/shutdown":         # lets launch.py replace an old hub with the current code (this computer only)
                if not self._is_local():
                    return self._json(403, {"error": "forbidden"})
                threading.Thread(target=HTTPD.shutdown, daemon=True).start()
                return self._json(200, {"ok": True})
            if path == "/api/sequence":         # save the move sequence on the host as JSON, load one back, or run one from the library
                try:
                    rev, run = bool(d.get("reverse")), bool(d.get("run"))
                    if isinstance(d.get("delete"), str):
                        LIBRARY.delete(d["delete"])
                        HUB.log("hub", "sequence %s deleted" % d["delete"])
                        return self._json(200, {"ok": True})
                    if isinstance(d.get("rename"), dict):
                        LIBRARY.rename(str(d["rename"].get("from", "")), str(d["rename"].get("to", "")))
                        return self._json(200, {"ok": True})
                    if "load" in d:
                        seq = HUB.load_sequence(d["load"], rev, run)
                        return self._json(200, {"ok": True, "loaded": seq["count"], "name": seq["name"]})
                    if isinstance(d.get("name"), str) and d.get("use"):      # {"name": "x", "use": true, "reverse": false, "run": true}
                        seq = HUB.load_sequence(LIBRARY.get(d["name"]), rev, run)
                        return self._json(200, {"ok": True, "loaded": seq["count"], "name": seq["name"]})
                    pth, pkt = HUB.store_sequence(str(d.get("name") or ""), str(d.get("description") or ""), d.get("requires"), d.get("ends"))
                except KeyError:
                    return self._json(404, {"error": "no such sequence"})
                except ValueError as e:
                    return self._json(400, {"error": str(e)})
                return self._json(200, {"ok": True, "path": str(pth), "packet": pkt})
            if path == "/api/points":           # points to move between: set | here | enable | go | loop | stop
                try:
                    return self._json(200, HUB.points_action(d))
                except (ValueError, TypeError) as e:
                    return self._json(400, {"error": str(e)})
                except RuntimeError as e:
                    return self._json(409, {"error": str(e)})
            if path == "/api/mirror":           # mirror mode: on | off | test (nudge one joint) | map (joint -> channel)
                try:
                    return self._json(200, {"ok": True, "mirror": HUB.mirror_action(d)})
                except (ValueError, TypeError) as e:
                    return self._json(400, {"error": str(e)})
            if path == "/api/tune":             # the speed tuner: start | cancel | apply | clear
                try:
                    return self._json(200, HUB.tune_action(d))
                except KeyError:
                    return self._json(404, {"error": "no such sequence"})
                except ValueError as e:
                    return self._json(400, {"error": str(e)})
                except RuntimeError as e:
                    return self._json(409, {"error": str(e)})
            if path == "/api/ask":              # plain-language task: home grip first, then the best-matching sequence
                try:
                    HUB.ask(d.get("request", ""))
                except ValueError as e:
                    return self._json(400, {"error": str(e)})
                except RuntimeError as e:
                    return self._json(409, {"error": str(e)})
                return self._json(200, {"ok": True})
            if path == "/api/world":            # the electromagnet and the keys (simulation only: nothing is sent to the arm)
                with HUB.sim_lock:
                    if d.get("reset"):
                        HUB.cell.reset()
                    if "magnet" in d:
                        HUB.cell.toggle() if d["magnet"] == "toggle" else HUB.cell.set_magnet(bool(d["magnet"]))
                    snap = HUB.cell.snapshot()
                return self._json(200, {"ok": True, "world": snap})
            if path == "/api/emu":              # the emulated ESP32 and its network
                try:
                    HUB.emu_action(d)
                except RuntimeError as e:
                    return self._json(409, {"error": str(e)})
                return self._json(200, {"ok": True, "emu": HUB.emu.snapshot()})
            if path == "/api/board":
                cmd = str(d.get("cmd", "")).strip()
                if not BOARD_CMD_RE.match(cmd):
                    return self._json(400, {"error": "command not allowed"})
                ok = HUB.link.command(cmd)
                return self._json(200 if ok else 409, {"ok": ok, "reason": None if ok else "USB link is not open"})
            if path == "/api/usb":
                HUB.link.set_enabled(bool(d.get("enabled")))
                return self._json(200, {"enabled": HUB.link.enabled})
            if path == "/api/service":
                ok, msg = HUB.service_action(str(d.get("name", "")), str(d.get("action", "")))
                return self._json(200 if ok else 409, {"ok": ok, "message": msg})
            if path == "/api/firmware":
                ok, msg = HUB.firmware_job(str(d.get("action", "")), str(d.get("profile", "usb")))
                return self._json(200 if ok else 409, {"ok": ok, "message": msg})
            return self._json(404, {"error": "not found"})
        except (ValueError, TypeError) as e:
            return self._json(400, {"error": f"bad request: {e}"})


def lan_ip():
    """This computer's address on the local network (the one a phone on the same WiFi can reach)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sk:
            sk.connect(("10.255.255.255", 1))          # no packet is sent
            return sk.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def board_key():
    """The key the board gives when it connects over WiFi: MIRROR_KEY in the environment, else one made once and kept in .hub_boardkey (git-ignored)."""
    k = os.environ.get("MIRROR_KEY", "").strip()
    if len(k) >= 8:
        return k
    f = ROOT / ".hub_boardkey"
    try:
        k = f.read_text().strip()
    except OSError:
        k = ""
    if len(k) < 8:
        k = secrets.token_urlsafe(12)
        try:
            f.write_text(k)
        except OSError:
            pass
    return k


def main():
    global HUB, HTTPD
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--server", default="http://127.0.0.1:8000", help="the local input server")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser")
    ap.add_argument("--lan", action="store_true",
                    help="also serve other devices on the network (a phone), which need the key printed in the link; see launch.py")
    ap.add_argument("--no-board-link", action="store_true", help="with --lan, do not listen for the board's WiFi connection on port+2")
    ap.add_argument("--no-https", action="store_true", help="with --lan, do not also serve HTTPS on port+1 (the phone's microphone needs it)")
    ap.add_argument("--send-speed", action="store_true",
                    help="also send the damper to the arm as maxSpeed (needs the modified firmware in firmware/rarm)")
    args = ap.parse_args()
    global LAN_TOKEN
    HUB = Hub(args.server, send_speed=args.send_speed)
    bind = "127.0.0.1"
    if args.lan:
        tf = ROOT / ".hub_token"
        try:
            LAN_TOKEN = tf.read_text().strip()
        except OSError:
            LAN_TOKEN = ""
        if len(LAN_TOKEN) < 12:
            LAN_TOKEN = secrets.token_urlsafe(12)
            try:
                tf.write_text(LAN_TOKEN)
            except OSError:
                pass
        bind = "0.0.0.0"
        HUB.phone_url = f"http://{lan_ip()}:{args.port}/?key={LAN_TOKEN}"
    try:
        httpd = ThreadingHTTPServer((bind, args.port), Handler)
    except OSError as e:
        raise SystemExit(f"Cannot listen on {bind}:{args.port} ({e}). Is the hub already running?")
    httpd.daemon_threads = True
    HTTPD = httpd
    httpsd = None
    if args.lan and not args.no_https:
        try:
            cert_p, key_p = TLS.ensure_cert(ROOT / ".hub_cert", TLS.local_ips([lan_ip()]))
            httpsd = TLS.TlsServer((bind, args.port + 1), Handler, cert_p, key_p)
            HUB.phone_url_https = f"https://{lan_ip()}:{args.port + 1}/?key={LAN_TOKEN}"
            https_print = "certificate fingerprint (SHA-256): " + TLS.fingerprint(cert_p)
        except Exception as e:                      # cryptography missing, port busy: the plain link still works, only the phone's microphone does not
            httpsd = None
            https_print = "HTTPS for the phone is off (%s: %s)" % (type(e).__name__, str(e)[:100])
    board_print = ""
    if args.lan and not args.no_board_link:
        try:
            HUB.board_listener = NETLINK.BoardListener(HUB.link, board_key(), port=args.port + 2, bind=bind, log=lambda t: HUB.log("hub", t))
            board_print = "Board over WiFi (no USB cable): listening on port %d; the Arm and board page shows the lines for the board's secrets.h" % HUB.board_listener.start()
        except Exception as e:                      # port busy: USB and everything else still work
            HUB.board_listener = None
            board_print = "Board over WiFi is off (%s: %s)" % (type(e).__name__, str(e)[:100])
    HUB.start()
    if httpsd:
        threading.Thread(target=httpsd.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{args.port}"
    print(f"Robot arm hub on {url}  (output starts SAFE; Ctrl+C to stop)")
    if HUB.phone_url:
        print(f"On your phone (same WiFi): {HUB.phone_url}", flush=True)
        if HUB.phone_url_https:
            print(f"On your phone, HTTPS (needed for the microphone; accept the certificate warning once): {HUB.phone_url_https}", flush=True)
        print(https_print, flush=True)
        if board_print:
            print(board_print, flush=True)
    if not args.no_open:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        HUB.shutdown()
        if HUB.board_listener:
            HUB.board_listener.close()
        httpd.server_close()
        if httpsd:
            httpsd.shutdown()
            httpsd.server_close()


if __name__ == "__main__":
    main()
