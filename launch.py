#!/usr/bin/env python3
"""One tap: starts the input server and the dashboard hub, then opens the robot viewer.

    python launch.py              # server + hub, opens the browser, phone link printed
    python launch.py --local-only # do not let other devices (a phone) connect
    python launch.py --no-open    # do not open a browser

On a phone on the same WiFi: open the "On your phone" link once, then use Share > Add to Home Screen (iPhone) or
the browser menu > Install app (Android). After that the icon opens the hub in one tap while this is running.
The first time, Windows asks to allow Python through the firewall: allow it on private networks.
Output always starts SAFE; nothing moves the real arm until you press ARM in the dashboard.
"""
import argparse
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


def hub_build(hub_url):
    """The build id of a hub that is already running, None if there is none (or it is too old to say)."""
    try:
        import json
        return json.loads(urllib.request.urlopen(hub_url + "/api/status", timeout=1.5).read()).get("build") or "unknown"
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--local-only", action="store_true", help="do not accept connections from other devices")
    ap.add_argument("--no-open", action="store_true")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--server-port", type=int, default=8000)
    a = ap.parse_args()
    py = sys.executable
    kids = []
    server_url = "http://127.0.0.1:%d" % a.server_port
    hub_url = "http://127.0.0.1:%d" % a.port
    try:
        if up(server_url + "/health") or up(server_url + "/state"):
            print("input server already running on :%d" % a.server_port)
        else:
            kids.append(subprocess.Popen([py, os.path.join(ROOT, "server.py")], cwd=ROOT))
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
            kids.append(subprocess.Popen(cmd, cwd=ROOT))
            for _ in range(50):
                if up(hub_url + "/api/status"):
                    break
                time.sleep(0.2)
            else:
                raise SystemExit("the hub did not start")
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
