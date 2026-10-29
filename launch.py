#!/usr/bin/env python3
"""One tap: starts the input server, Sam's backend (our checked build) and the dashboard hub, then opens the robot viewer.

    python launch.py              # server + backend + hub, opens the browser, phone link printed
    python launch.py --no-backend # without the real-arm backend (virtual arm only)
    python launch.py --local-only # do not let other devices (a phone) connect
    python launch.py --no-open    # do not open a browser

On a phone on the same WiFi: open the "On your phone" link once, then use Share > Add to Home Screen (iPhone) or
the browser menu > Install app (Android). After that the icon opens the hub in one tap while this is running.
The first time, Windows asks to allow Python through the firewall: allow it on private networks.
Output always starts SAFE; nothing moves the real arm until you press ARM in the dashboard.
"""
import argparse
import importlib.util
import os
import subprocess
import sys
import time
import urllib.request
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
from stamp import code_stamp  # noqa: E402


def up(url, timeout=0.6):
    try:
        urllib.request.urlopen(url, timeout=timeout).read()
        return True
    except Exception:
        return False


def server_build(server_url):
    """The build id of an input server that is already running: None if none, "unknown" if it is too old to say."""
    try:
        import json
        return json.loads(urllib.request.urlopen(server_url + "/health", timeout=1.5).read()).get("build") or "unknown"
    except Exception:
        return None if not up(server_url + "/state") else "unknown"


def current_server_build():
    import hashlib
    return hashlib.sha1(("server:%d" % os.stat(os.path.join(ROOT, "server.py")).st_size).encode()).hexdigest()[:7]


def hub_build(hub_url):
    """The build id of a hub that is already running, None if there is none (or it is too old to say)."""
    try:
        import json
        return json.loads(urllib.request.urlopen(hub_url + "/api/status", timeout=1.5).read()).get("build") or "unknown"
    except Exception:
        return None


def hub_phone_url(hub_url):
    try:
        import json
        j = json.loads(urllib.request.urlopen(hub_url + "/api/status", timeout=1.5).read())
        return j.get("phone_url"), j.get("phone_url_https")
    except Exception:
        return None, None


def load_build_backend():
    spec = importlib.util.spec_from_file_location("build_backend", os.path.join(ROOT, "firmware", "ours", "build_backend.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def start_backend(py, port, lan, kids):
    """Sam's backend with our checks, built into .backend/ (again whenever its sources change) and run on its own port. Returns its URL, or None if it could not start."""
    url = "http://127.0.0.1:%d" % port
    if up(url + "/mode"):
        print("backend already running on :%d (left as it is)" % port)
        return url
    try:
        out = load_build_backend().ensure(os.path.join(ROOT, ".backend"))
    except (SystemExit, Exception) as e:
        print("could not build the backend, continuing without it: %s" % e)
        return None
    env = dict(os.environ, BACKEND_PORT=str(port), BACKEND_HOST="0.0.0.0" if lan else "127.0.0.1", PYTHONUNBUFFERED="1")
    kids.append(subprocess.Popen([py, "main.py"], cwd=str(out), env=env))
    for _ in range(30):
        if up(url + "/mode"):
            print("backend running on :%d%s" % (port, " (reachable from the network, no password: only for a trusted network)" if lan else " (this computer only)"))
            return url
        time.sleep(0.2)
    print("the backend did not start, continuing without it")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--local-only", action="store_true", help="do not accept connections from other devices")
    ap.add_argument("--no-open", action="store_true")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--server-port", type=int, default=8000)
    ap.add_argument("--no-backend", action="store_true", help="do not start Sam's backend: the virtual arm only")
    ap.add_argument("--backend-port", type=int, default=8001)
    ap.add_argument("--backend-lan", action="store_true", help="let the ESP32 reach the backend over the network (it has no password)")
    ap.add_argument("--backend-speed", type=float, default=20.0, help="tool speed for the real arm, mm/s (up to 50)")
    ap.add_argument("--backend-ignore-magnet", action="store_true", help="move the real arm through sequences that use the magnet, without switching it (its firmware cannot)")
    a = ap.parse_args()
    py = sys.executable
    kids = []
    server_url = "http://127.0.0.1:%d" % a.server_port
    hub_url = "http://127.0.0.1:%d" % a.port
    try:
        running = server_build(server_url)
        if running is not None and running != current_server_build():
            print("an older input server is running (%s, files are %s): replacing it" % (running, current_server_build()))
            try:
                urllib.request.urlopen(urllib.request.Request(server_url + "/shutdown", data=b"{}", method="POST"), timeout=3).read()
            except Exception:
                pass
            for _ in range(30):
                if not up(server_url + "/health"):
                    break
                time.sleep(0.2)
            else:
                print("could not stop the old input server: close its window (or end its python process) and run this again")
                return
        if up(server_url + "/health") or up(server_url + "/state"):
            print("input server already running on :%d" % a.server_port)
        else:
            kids.append(subprocess.Popen([py, os.path.join(ROOT, "server.py")], cwd=ROOT))
            for _ in range(30):
                if up(server_url + "/health"):
                    break
                time.sleep(0.2)
        backend_url = None if a.no_backend else start_backend(py, a.backend_port, a.backend_lan, kids)
        old = hub_build(hub_url)
        if old is not None and old != code_stamp():
            print("an older hub is running (build %s, files are %s): replacing it" % (old, code_stamp()))
            try:
                urllib.request.urlopen(urllib.request.Request(hub_url + "/api/shutdown", data=b"{}", method="POST", headers={"X-Hub": "1"}), timeout=3).read()
            except Exception:
                pass
            for _ in range(30):
                if not up(hub_url + "/api/status"):
                    break
                time.sleep(0.2)
            else:
                print("could not stop the old hub: close its window (or end its python process) and run this again")
                return
        if up(hub_url + "/api/status"):
            print("hub already running: opening it")
        else:
            cmd = [py, os.path.join(ROOT, "dashboard", "hub.py"), "--no-open", "--port", str(a.port), "--server", server_url]
            if not a.local_only:
                cmd.append("--lan")
            if backend_url:
                cmd += ["--backend", backend_url, "--backend-speed", str(a.backend_speed)]
                if a.backend_ignore_magnet:
                    cmd.append("--backend-ignore-magnet")
            kids.append(subprocess.Popen(cmd, cwd=ROOT))
            for _ in range(50):
                if up(hub_url + "/api/status"):
                    break
                time.sleep(0.2)
            else:
                raise SystemExit("the hub did not start")
        phone, phone_https = hub_phone_url(hub_url)
        if phone:
            shown = ["PHONE LINK (same WiFi):", phone]
            if phone_https:
                shown += ["", "PHONE LINK with the microphone (HTTPS: accept the certificate warning once):", phone_https]
            bar = "=" * (max(len(l) for l in shown) + 4)
            print("\n" + bar + "\n" + "\n".join("  " + l for l in shown) + "\n" + bar + "\n")
            try:
                with open(os.path.join(ROOT, "phone_link.txt"), "w") as f:
                    f.write(phone + "\n" + ((phone_https + "\n") if phone_https else ""))
            except OSError:
                pass
        if not a.no_open:
            webbrowser.open(hub_url + "/#/arm")
        if kids:
            print("Running. Close this window or press Ctrl+C to stop everything.")
            while all(k.poll() is None for k in kids):
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for k in kids:
            if k.poll() is None:
                k.terminate()
        for k in kids:
            try:
                k.wait(timeout=5)
            except subprocess.TimeoutExpired:
                k.kill()


if __name__ == "__main__":
    main()
