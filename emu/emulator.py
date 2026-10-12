#!/usr/bin/env python3
"""Build and run the ESP32 firmware on this PC against a real server over real sockets.

    python emu/emulator.py --firmware stock   --host 127.0.0.1:8100 --seconds 30
    python emu/emulator.py --firmware patched --host 127.0.0.1:8000

--firmware: stock (firmware/stock/src as shipped), patched (stock + firmware/stock/patches/damper-and-failsafes.patch),
or rarm (the rewrite in firmware/rarm/src). --host is the server the firmware polls: the local server directly, or the
network proxy (emu/netproxy.py) for a link with latency, loss and handshake cost. Needs g++ (and `patch` for patched).
Prints the servo positions as they change and the network statistics every 5 s. Library use: see class Emulator.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware")
HERE = os.path.dirname(os.path.abspath(__file__))
STUBS = os.path.join(FW, "rarm", "test", "stubs")


def firmware_src(kind, workdir):
    """Return the src directory for a firmware variant (patched is built in workdir)."""
    if kind == "stock":
        return os.path.join(FW, "stock", "src")
    if kind == "rarm":
        return os.path.join(FW, "rarm", "src")
    if kind == "patched":
        tree = os.path.join(workdir, "patched")
        shutil.copytree(os.path.join(FW, "stock", "src"), os.path.join(tree, "src"))
        r = subprocess.run(["patch", "-p1", "-d", tree, "-i", os.path.join(FW, "stock", "patches", "damper-and-failsafes.patch")],
                           capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError("patch does not apply:\n" + r.stdout + r.stderr)
        return os.path.join(tree, "src")
    raise ValueError("firmware must be stock, patched or rarm")


def build(kind, out_dir):
    src = firmware_src(kind, out_dir)
    exe = os.path.join(out_dir, "esp32_" + kind)
    cmd = ["g++", "-std=c++17", "-O1", "-I" + os.path.join(HERE, "shim"), "-I" + STUBS, "-I" + src, os.path.join(HERE, "emu_main.cpp"), "-o", exe]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError("build failed:\n" + r.stderr)
    return exe


class Emulator:
    """A running emulated ESP32. .servo is the latest {"t","servo":[4],"attached":[4]}, .stats the latest STATS dict."""

    def __init__(self, exe, host, seconds=None, serial=False):
        env = dict(os.environ, EMU_HOST=host)
        if seconds:
            env["EMU_SECONDS"] = str(seconds)
        if serial:
            env["EMU_SERIAL"] = "1"
        self.proc = subprocess.Popen([exe], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.servo, self.stats, self.history, self.log = None, {}, [], []
        self._threads = [threading.Thread(target=self._read_out, daemon=True), threading.Thread(target=self._read_err, daemon=True)]
        for t in self._threads:
            t.start()

    def _read_out(self):
        for line in self.proc.stdout:
            try:
                self.servo = json.loads(line)
                self.history.append((time.monotonic(), self.servo))
            except ValueError:
                pass

    def _read_err(self):
        for line in self.proc.stderr:
            if line.startswith("STATS "):
                self.stats = json.loads(line[6:])
            else:
                self.log.append(line.rstrip())

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        for t in self._threads:
            t.join(timeout=2)
        for f in (self.proc.stdout, self.proc.stderr):
            f.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--firmware", choices=["stock", "patched", "rarm"], default="stock")
    ap.add_argument("--host", default="127.0.0.1:8000", help="server or proxy the firmware polls, host:port")
    ap.add_argument("--seconds", type=float, help="stop after this long (default: until Ctrl+C)")
    ap.add_argument("--serial", action="store_true", help="show the firmware's Serial output")
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        exe = build(a.firmware, tmp)
        print("running the %s firmware against %s (Ctrl+C to stop)" % (a.firmware, a.host), flush=True)
        emu = Emulator(exe, a.host, a.seconds, a.serial)
        last = None
        try:
            while emu.proc.poll() is None:
                time.sleep(0.25)
                if emu.servo and emu.servo["servo"] != last:
                    last = emu.servo["servo"]
                    print("servos", last, "attached" if all(emu.servo["attached"]) else "released", flush=True)
                if emu.stats and emu.stats.get("_shown") != emu.stats.get("polls"):
                    s = emu.stats
                    print("net: %d polls, mean %.0f ms, p95 %.0f ms, max %.0f ms, %d failed; longest loop gap %.0f ms" % (
                        s["polls"], s["poll_ms_mean"], s["poll_ms_p95"], s["poll_ms_max"], s["poll_fails"], s["loop_gap_ms_max"]), flush=True)
                    s["_shown"] = s["polls"]
        except KeyboardInterrupt:
            pass
        finally:
            emu.stop()


if __name__ == "__main__":
    main()
