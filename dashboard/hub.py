#!/usr/bin/env python3
"""Robot arm hub: the local dashboard and its control API (Python standard library; pyserial optional).

    python dashboard/hub.py [--port 8765] [--server http://127.0.0.1:8000] [--no-open]

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
import json
import mimetypes
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
STATIC = HERE / "static"
sys.path.insert(0, str(ROOT))
from serial_link import SerialLink, BUTTON_ORDER  # noqa: E402

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
    def __init__(self, server_url):
        self.server = server_url.rstrip("/")
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
                        st, count, age = msg.get("state"), msg.get("count") or 0, msg.get("age_s")
                        if st is None:
                            continue
                        now = time.time()
                        self.latest, self.latest_recv = st, now - (age or 0.0)
                        if count != self.latest_count:
                            self.latest_count = count
                            self.packet_times.append(now)
                            ev = {"n": count, "t": round(self.latest_recv, 3), "s": st, "d": describe(prev, st)}
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
    def firmware_job(self, action):
        if action not in ("compile", "upload"):
            return False, "unknown action"
        with self.lock:
            if self.fw["state"] == "running":
                return False, "a firmware job is already running"
            if not (SKETCH / "secrets.h").exists():
                return False, "arduino/arm_receiver/secrets.h is missing: copy secrets.example.h and fill it in"
            if not os.path.exists(ARDUINO) and not shutil.which("arduino-cli"):
                return False, "arduino-cli not found"
            if action == "upload":
                if self.armed:
                    return False, "switch the output to SAFE before uploading (the board resets)"
                if not self.link.find_port() and not self.link.device:
                    return False, "no ESP32 found on USB"
            self.fw.update(state="running", action=action, detail="", finished_t=None)
        self.publish("status", self.snapshot())
        threading.Thread(target=self._fw_run, args=(action,), daemon=True).start()
        return True, f"{action} started"

    def _fw_run(self, action):
        ok, detail = False, ""
        try:
            if action == "compile":
                rc, out = self.run("firmware", [ARDUINO, "compile", "--fqbn", FQBN, str(SKETCH)])
            else:
                port = self.link.device or self.link.find_port()
                self.link.suspend()                       # release the COM port for esptool
                time.sleep(0.6)
                rc, out = self.run("firmware", [ARDUINO, "upload", "-p", port, "--fqbn", FQBN, str(SKETCH)])
                if rc != 0:                               # the final-step serial glitch seen before: retry once
                    if "No more data to read" in out or "Hash of data verified" in out:
                        self.log("firmware", "retrying once (intermittent final-step serial error)")
                        rc, out = self.run("firmware", [ARDUINO, "upload", "-p", port, "--fqbn", FQBN, str(SKETCH)])
                self.link.resume()
            m = re.search(r"Sketch uses (\d+) bytes \((\d+)%\)", out or "")
            if m:
                self.fw["flash_pct"] = int(m.group(2))
            m = re.search(r"Global variables use (\d+) bytes \((\d+)%\)", out or "")
            if m:
                self.fw["ram_pct"] = int(m.group(2))
            ok = rc == 0
            detail = "ok" if ok else f"failed (exit {rc}); see the log"
        except Exception as e:
            detail = f"error: {e}"
            self.link.resume()
        self.fw.update(state="ok" if ok else "fail", detail=detail, finished_t=time.time())
        self.log("firmware", f"{action}: {detail}")
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
            "firmware": dict(self.fw, secrets_present=(SKETCH / "secrets.h").exists(),
                             age=None if not self.fw["finished_t"] else round(time.time() - self.fw["finished_t"], 1)),
            "config": {"env_present": (ROOT / ".env").exists(), "forward_url_set": bool(self.env_forward())},
            "project": self.cache["git"],
        }

    def hello(self):
        return {"status": self.snapshot(), "packets": list(self.packets)[-120:],
                "board": list(self.board_lines)[-200:],
                "logs": {k: list(v)[-80:] for k, v in self.logs.items()}}

    def start(self):
        for fn in (self._stream_reader, self._pump, self._poller):
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

    def _host_ok(self):
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
        if not self._host_ok():
            return self._json(403, {"error": "forbidden host"})
        path = self.path.split("?")[0]
        if path == "/api/status":
            return self._json(200, HUB.snapshot())
        if path == "/api/events":
            return self._events()
        if path == "/":
            path = "/index.html"
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
                if not HUB.armed:
                    return self._json(200, {"sent": False, "reason": "safe"})
                try:
                    HUB._http("/input", st, timeout=1.0)
                    return self._json(200, {"sent": True})
                except Exception:
                    return self._json(502, {"sent": False, "reason": "local server unreachable"})
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
                ok, msg = HUB.firmware_job(str(d.get("action", "")))
                return self._json(200 if ok else 409, {"ok": ok, "message": msg})
            return self._json(404, {"error": "not found"})
        except (ValueError, TypeError) as e:
            return self._json(400, {"error": f"bad request: {e}"})


def main():
    global HUB
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--server", default="http://127.0.0.1:8000", help="the local input server")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser")
    args = ap.parse_args()
    HUB = Hub(args.server)
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    except OSError as e:
        raise SystemExit(f"Cannot listen on 127.0.0.1:{args.port} ({e}). Is the hub already running?")
    httpd.daemon_threads = True
    HUB.start()
    url = f"http://127.0.0.1:{args.port}"
    print(f"Robot arm hub on {url}  (output starts SAFE; Ctrl+C to stop)")
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
