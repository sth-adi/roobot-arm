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
import agent as AGENT  # noqa: E402
LIBRARY = SEQ.Library(ROOT / "sequences")

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
        self.sim = ArmSim(model="stock", patch=True)   # our firmware: UWE-RArm stock + firmware/stock/patches (damper, failsafes, tap-to-save)
        self.sim_lock = threading.Lock()
        self._sim_status_sent = None
        self._cal_synced = False
        self.emu = EmuRunner(self.server, self.log)   # emulated ESP32 + network (Control page)
        self._prev_y = False
        self._run_once = None
        self._esp = None                        # (state dict, time) last report of our firmware via the server's /armstate
        self._esp_run = None                    # id of the sequence the ESP is running for us
        self.phone_url = None                   # set by main() with --lan
        self.cell = Workcell()                  # boxes, keys, tray and the electromagnet (sim/workcell.py)
        self.emu_view = ArmSim()                # the emulated servos drawn as an arm
        self.packets = collections.deque(maxlen=500)
        self.packet_times = collections.deque(maxlen=400)
        self.stream_ok = False
        # board / serial link
        self.link = SerialLink()
        self.link.on_line = self._board_line
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
                        if self._run_once and not self._esp_run and self.cell.run_finished(self._run_once):
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
                if self.armed and st is not None:
                    self.link.update(st, now)       # send input + read the board
                else:
                    self.link.service(now)          # SAFE: keep the port open and read only
            except Exception as e:                  # never let the pump die
                self.log("hub", f"serial pump error: {e}")

    # ---- safety ----------------------------------------------------------------------------
    def set_armed(self, on):
        with self.lock:
            on = bool(on)
            if on == self.armed:
                return
            self.armed = on
            self.armed_since = time.time() if on else None
        if not on:
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
                    p = subprocess.Popen([sys.executable, str(ROOT / "controller_gui.py"), "--url", self.server + "/input"],
                                         cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags)
                    self.log(name, f"started (pid {p.pid}). It holds the COM port while open, so the USB link here goes busy.")
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
                 "desc": "Sam's firmware plus our two patches: damper and failsafes, sequences and magnet."},
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

    def fw_profiles(self):
        st = self._fw_state()
        out = []
        for pid, pr in self.FW_PROFILES.items():
            rec = st["profiles"].get(pid) or {}
            try:
                stale = bool(rec) and rec.get("hash") != self._fw_hash(pid)
            except (ValueError, OSError):
                stale = False
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
                if not self.link.find_port() and not self.link.device:
                    return False, "no ESP32 found on USB"
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
                port = self.link.device or self.link.find_port()
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
            "pps": self.pps(), "last_packet_age": age, "packet_count": self.latest_count,
            "server": {"health": srv.get("health", False), "stream": self.stream_ok, "mode": mode,
                       "forward_enabled": srv.get("forward_enabled"),
                       "forward": {"configured": bool(fwd) or bool(self.env_forward()),
                                   "ok": fwd.get("ok"), "sent": fwd.get("sent"), "failed": fwd.get("failed"),
                                   "error": (fwd.get("error") or "")[:80] or None}},
            "services": services,
            "usb": {"enabled": link.enabled, "phase": link.phase, "port": link.device or link.find_port_cached(),
                    "status": link.status, "rate": link.board_rate, "range": link.board_range},
            "board": dict(self.board, age=None if not self.board["last_line_t"] else round(time.time() - self.board["last_line_t"], 1)),
            "firmware": dict(self.fw, profiles=self.fw_profiles(), secrets_present=(SKETCH / "secrets.h").exists(),
                             age=None if not self.fw["finished_t"] else round(time.time() - self.fw["finished_t"], 1)),
            "config": {"env_present": (ROOT / ".env").exists(), "forward_url_set": bool(self.env_forward())},
            "project": self.cache["git"],
            "sim": self._sim_snapshot(),
            "emu": self.emu.snapshot(),
            "phone_url": self.phone_url,
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
                    "last_run": self.last_run, "task": dict(self.task),
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
                    "keys": {str(k["id"]): self.cell.where(k) for k in self.cell.keys}, "last_run": self.last_run, "task": dict(self.task),
                    "ai": "groq" if AGENT.api_key() else "keywords", "armed": self.armed, "poses": [list(p) for p in self.sim.saved]}

    task = {"state": "idle"}
    last_run = None

    def ask(self, request):
        """Start a task from plain language: make sure the arm is at home grip, pick a sequence, run it once."""
        request = str(request).strip()[:500]
        if not request:
            raise ValueError("say what you want done")
        if self.task.get("state") in ("working", "running"):
            raise RuntimeError("a task is already running")
        self.task = {"state": "working", "request": request, "log": []}
        threading.Thread(target=self._do_task, args=(request,), daemon=True).start()

    def _tlog(self, msg, state=None):
        self.task["log"].append(msg)
        if state:
            self.task["state"] = state
        self.log("agent", msg)

    def _wait(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(0.2)
        return False

    def _do_task(self, request):
        try:
            st = self.state_packet()
            library = LIBRARY.list()
            plan = AGENT.choose(request, library, st)
            if not plan["sequence"]:
                if plan.get("ai_error"):
                    self._tlog("Groq was not used: " + plan["ai_error"])
                return self._tlog("No sequence fits: " + plan["reason"], "failed")
            seq = LIBRARY.get(plan["sequence"])
            self.task["plan"] = plan
            self._tlog("Chose %s%s (%s, %s)" % (plan["sequence"], " reversed" if plan["reverse"] else "", plan["by"], plan["reason"]))
            if plan.get("ai_error"):
                self._tlog("Groq was not used: " + plan["ai_error"])
            if plan["reverse"]:
                seq = SEQ.reverse(seq)
            need = seq.get("requires", {}).get("key_in")
            if need:
                want = str(need).replace("box", "box ").replace("  ", " ").strip()
                if want not in st["keys"].values():
                    return self._tlog("Cannot start: this move needs a key in %s, and the keys are at %s" % (need, ", ".join(st["keys"].values())), "failed")
            need = seq.get("requires", {}).get("key_on")
            if need and str(need) not in st["keys"].values():
                return self._tlog("Cannot start: this move needs a key on the %s, and the keys are at %s" % (need, ", ".join(st["keys"].values())), "failed")
            if not st["home_grip"]:
                self._tlog("Not at home grip: going home first")
                if st.get("source") == "esp32":
                    if st["magnet"]:
                        self._esp_tap("X")                  # magnet off first
                    self._esp_tap("B")                      # then go home
                else:
                    with self.sim_lock:
                        self.sim.stop_replay()
                        self.cell.set_magnet(False)
                        self.sim.go_home()
                if not self._wait(lambda: self.state_packet()["at_home"] and not self.state_packet()["magnet"], 30):
                    return self._tlog("Could not get to home grip", "failed")
            self._tlog("At home grip: running", "running")
            self.load_sequence(seq, False, True)
            if not self._wait(lambda: self._run_once is None, 120):
                return self._tlog("The sequence did not finish", "failed")
            self.last_run = {"name": plan["sequence"], "reverse": plan["reverse"]}
            self._tlog("Done", "done")
        except Exception as e:                       # a task must never kill the hub
            self._tlog("Failed: %s" % e, "failed")

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
    def do_POST(self):
        if not self._host_ok() or self.headers.get("X-Hub") != "1":
            return self._json(403, {"error": "forbidden"})
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
                with HUB.sim_lock:
                    if "rate" in d:
                        push_rate = True
                        HUB.sim.set_rate(d["rate"])
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
                        push = True
                if push:
                    HUB.push_calibration()
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


def main():
    global HUB, HTTPD
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--server", default="http://127.0.0.1:8000", help="the local input server")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser")
    ap.add_argument("--lan", action="store_true",
                    help="also serve other devices on the network (a phone), which need the key printed in the link; see launch.py")
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
    HUB.start()
    url = f"http://127.0.0.1:{args.port}"
    print(f"Robot arm hub on {url}  (output starts SAFE; Ctrl+C to stop)")
    if HUB.phone_url:
        print(f"On your phone (same WiFi): {HUB.phone_url}", flush=True)
    if not args.no_open:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        HUB.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    main()
