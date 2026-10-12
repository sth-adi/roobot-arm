"""Runs the emulated ESP32 and the network between it and the server for the dashboard.

    emulated ESP32  --(esp link)-->  proxy  -->  the local server  <--(ctl link)--  proxy  <-- browser sticks / controller

Started and stopped from the Control page (hub /api/emu). The proxy (emu/netproxy.py) runs inside the hub on free ports;
the emulator is the real firmware main.cpp built for this PC (emu/emulator.py), so it needs a C++ compiler (see emu/README.md).
"""
import asyncio
import os
import shutil
import sys
import tempfile
import threading
import time
import urllib.parse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "emu"))
import emulator as emu_mod          # noqa: E402
import netproxy                     # noqa: E402

FIRMWARE = ("stock", "patched", "rarm")


class EmuRunner:
    def __init__(self, server_url, log=lambda src, line: None):
        host = urllib.parse.urlparse(server_url).netloc or "127.0.0.1:8000"
        self.upstream = host
        self.log = log
        self.lock = threading.RLock()
        self.state = "off"              # off | building | running | error
        self.error = ""
        self.firmware = None
        self.proxy = None
        self.emu = None
        self.ports = {"esp": None, "ctl": None}
        self.started = None
        self._tmp = None
        self._loop = None
        self._stop_evt = None
        self._thread = None

    # ---- lifecycle -------------------------------------------------------------------------------------------
    def start(self, firmware, preset="home_wifi"):
        if firmware not in FIRMWARE:
            raise ValueError("firmware must be one of " + ", ".join(FIRMWARE))
        if preset not in netproxy.PRESETS:
            raise ValueError("unknown preset " + str(preset))
        with self.lock:
            if self.state in ("building", "running"):
                raise RuntimeError("already " + self.state)
            self.state, self.error, self.firmware = "building", "", firmware
        threading.Thread(target=self._start, args=(firmware, preset), daemon=True).start()

    def _start(self, firmware, preset):
        tmp = tempfile.mkdtemp(prefix="roobot-emu-")
        try:
            exe = emu_mod.build(firmware, tmp)
            ready = threading.Event()
            self._thread = threading.Thread(target=self._run_proxy, args=(preset, ready), daemon=True)
            self._thread.start()
            if not ready.wait(5) or not self.ports["esp"]:
                raise RuntimeError("the network proxy did not start")
            e = emu_mod.Emulator(exe, "127.0.0.1:%d" % self.ports["esp"])
            with self.lock:
                if self.state != "building":            # stopped while it was building
                    e.stop()
                    raise RuntimeError("stopped")
                self.emu, self._tmp, self.started, self.state = e, tmp, time.time(), "running"
            self.log("emu", "emulated ESP32 (%s firmware) running; link %s" % (firmware, preset))
        except Exception as ex:
            msg = str(ex)
            self._teardown()
            shutil.rmtree(tmp, ignore_errors=True)
            with self.lock:
                if msg != "stopped":
                    self.state, self.error = "error", msg[-600:]
                    self.log("emu", "could not start: " + msg.splitlines()[0][:200] if msg else "could not start")

    def _run_proxy(self, preset, ready):
        async def amain():
            self._loop, self._stop_evt = asyncio.get_running_loop(), asyncio.Event()
            proxy = netproxy.Proxy(self.upstream)
            for l in proxy.links.values():
                l.set(preset=preset)
            esp = await asyncio.start_server(lambda r, w: proxy.handle(proxy.links["esp"], r, w), "127.0.0.1", 0)
            ctl = await asyncio.start_server(lambda r, w: proxy.handle(proxy.links["ctl"], r, w), "127.0.0.1", 0)
            self.proxy = proxy
            self.ports = {"esp": esp.sockets[0].getsockname()[1], "ctl": ctl.sockets[0].getsockname()[1]}
            ready.set()
            await self._stop_evt.wait()
            for s in (esp, ctl):
                s.close()
        try:
            asyncio.run(amain())
        except Exception as ex:                         # pragma: no cover
            self.log("emu", "proxy stopped: %s" % ex)
        finally:
            ready.set()

    def _teardown(self):
        with self.lock:
            e, self.emu = self.emu, None
            loop, evt = self._loop, self._stop_evt
            self.proxy, self._loop, self._stop_evt = None, None, None
            self.ports = {"esp": None, "ctl": None}
        if e:
            e.stop()
        if loop and evt:
            loop.call_soon_threadsafe(evt.set)
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        if self._tmp:
            shutil.rmtree(self._tmp, ignore_errors=True)
            self._tmp = None

    def stop(self):
        with self.lock:
            was = self.state
            self.state, self.error = "off", ""
        self._teardown()
        if was in ("running", "building"):
            self.log("emu", "emulated ESP32 stopped")

    # ---- the link ----------------------------------------------------------------------------------------------
    def set_link(self, path, **fields):
        links = self._links(path)
        for l in links:
            l.set(**fields)

    def outage(self, path, seconds):
        seconds = max(0.0, min(30.0, float(seconds)))
        for l in self._links(path):
            l.outage_until = time.monotonic() + seconds

    def _links(self, path):
        p = self.proxy
        if not p or self.state != "running":
            raise RuntimeError("the emulator is not running")
        if path not in ("esp", "ctl", "both"):
            raise ValueError("path must be esp, ctl or both")
        return list(p.links.values()) if path == "both" else [p.links[path]]

    # ---- what the dashboard shows --------------------------------------------------------------------------------
    @property
    def running(self):
        return self.state == "running"

    def ctl_url(self):
        return "http://127.0.0.1:%d" % self.ports["ctl"] if self.running and self.ports["ctl"] else None

    def snapshot(self):
        with self.lock:
            st, e, p = self.state, self.emu, self.proxy
            if st == "running" and e and e.proc.poll() is not None:
                self.state = st = "error"
                self.error = "the emulated firmware exited (code %s). %s" % (e.proc.returncode, " ".join(list(e.log)[-3:]))
        out = {"state": st, "error": self.error, "firmware": self.firmware, "ports": dict(self.ports),
               "uptime": round(time.time() - self.started, 1) if self.started and st == "running" else 0,
               "presets": {k: v for k, v in netproxy.PRESETS.items() if k != "none"},
               "links": None, "net": None, "servo": None}
        if st == "running" and p:
            now = time.monotonic()
            out["links"] = {k: {"preset": l.preset, "profile": dict(l.profile), "stats": dict(l.stats),
                                "outage_s": max(0.0, round(l.outage_until - now, 1))} for k, l in p.links.items()}
        if e:
            out["net"] = dict(e.stats) if e.stats else None
            out["servo"] = e.servo
        return out

    def servo_angles_deg(self, limits):
        """The joint angles (degrees) the emulated servos are at, or None before the first servo line."""
        e = self.emu
        sv = e.servo if e else None
        if not sv:
            return None
        out = []
        for i in range(4):
            c = sv["servo"][i]
            out.append(180 - c if limits[i][0] > limits[i][1] else c)
        return out, all(sv["attached"])
