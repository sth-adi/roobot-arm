#!/usr/bin/env python3
"""Network impairment proxy (standard library only): sits between the controller / the emulated ESP32 and the server and
makes the link behave like the network you choose.

    controller_client --> [ctl port]  --+
                                        +--> proxy --> server (--upstream)
    ESP32 / emulator  --> [esp port]  --+

Each path has its own link profile. For every request the proxy adds: a one-way delay before it reaches the server and
one back (rtt/2 + jitter each), retransmission stalls for lost packets, a hold during an outage, and on every NEW client
connection the TCP + TLS handshake cost (3 round trips plus the ESP32's own crypto time). With `keepalive` on the client
connection is kept open between requests, like a TLS-terminating front end (a hosted service); off, it is closed after
each response, like talking straight to a plain Python server, so every poll pays the handshake again.

Control API (--control-port): GET /state; POST /link {"path":"esp"|"ctl"|"both","preset":"home_wifi", ...fields};
POST /outage {"path":"esp","seconds":3}; GET /presets.

The preset numbers are assumptions, not measurements: calibrate them against the real board (see emu/README.md).
"""
import argparse
import asyncio
import json
import random
import time

PRESETS = {
    #            rtt   jitter loss   tls_cpu  keepalive
    "none":        dict(rtt_ms=0,   jitter_ms=0,   loss=0.0,   tls_cpu_ms=0,   keepalive=True),
    "lan":         dict(rtt_ms=3,   jitter_ms=1,   loss=0.0,   tls_cpu_ms=250, keepalive=True),
    "home_wifi":   dict(rtt_ms=25,  jitter_ms=10,  loss=0.005, tls_cpu_ms=350, keepalive=True),
    "campus_wifi": dict(rtt_ms=45,  jitter_ms=25,  loss=0.01,  tls_cpu_ms=350, keepalive=True),
    "tether_4g":   dict(rtt_ms=70,  jitter_ms=35,  loss=0.02,  tls_cpu_ms=350, keepalive=True),
    "poor":        dict(rtt_ms=180, jitter_ms=120, loss=0.06,  tls_cpu_ms=350, keepalive=True),
    "direct_http": dict(rtt_ms=25,  jitter_ms=10,  loss=0.005, tls_cpu_ms=350, keepalive=False),   # straight to a plain server
}
FIELDS = {"rtt_ms": float, "jitter_ms": float, "loss": float, "tls_cpu_ms": float, "keepalive": bool, "handshake_rtts": float}
RTO_MS = 400   # retransmission timeout added per lost packet (doubles on repeats)


class Link:
    def __init__(self, name, preset="none"):
        self.name = name
        self.profile = {"handshake_rtts": 3.0}
        self.preset = preset
        self.set(preset=preset)
        self.outage_until = 0.0
        self.stats = {"requests": 0, "connections": 0, "dropped": 0, "last_delay_ms": 0.0, "max_delay_ms": 0.0}
        self.rng = random.Random(1)

    def set(self, preset=None, **fields):
        if preset is not None:
            if preset not in PRESETS:
                raise ValueError("unknown preset %r; choose from %s" % (preset, ", ".join(PRESETS)))
            self.profile.update(PRESETS[preset])
            self.preset = preset
        for k, v in fields.items():
            if k not in FIELDS:
                raise ValueError("unknown field %r" % k)
            if FIELDS[k] is bool:
                self.profile[k] = bool(v)
            else:
                v = float(v)
                if v < 0 or (k == "loss" and v > 0.9):
                    raise ValueError("bad value for %s" % k)
                self.profile[k] = v
        if fields:
            self.preset = "custom"

    def one_way_ms(self):
        p = self.profile
        ms = p["rtt_ms"] / 2.0 + abs(self.rng.gauss(0, p["jitter_ms"] / 2.0))
        rto = RTO_MS
        while self.rng.random() < p["loss"]:        # each lost packet waits out a retransmission timeout
            ms += rto
            rto *= 2
            if rto > 3200:
                break
        return ms

    def handshake_ms(self):
        p = self.profile
        return p["handshake_rtts"] * p["rtt_ms"] + p["tls_cpu_ms"]


async def read_message(reader, is_request):
    """Read one HTTP message (head + Content-Length body). Returns (head_lines, body) or None at EOF."""
    try:
        head = await reader.readuntil(b"\r\n\r\n")
    except (asyncio.IncompleteReadError, ConnectionError):
        return None
    lines = head.decode("latin-1").split("\r\n")[:-2]
    n = 0
    for ln in lines[1:]:
        if ln.lower().startswith("content-length:"):
            n = int(ln.split(":", 1)[1])
    body = await reader.readexactly(n) if n else b""
    return lines, body


def header(lines, name):
    for ln in lines[1:]:
        if ln.lower().startswith(name.lower() + ":"):
            return ln.split(":", 1)[1].strip()
    return None


class Proxy:
    def __init__(self, upstream):
        host, _, port = upstream.rpartition(":")
        self.up = (host or "127.0.0.1", int(port))
        self.links = {"esp": Link("esp"), "ctl": Link("ctl")}

    async def forward(self, lines, body):
        """Send the request to the real server, read the whole reply (the server closes after it)."""
        r, w = await asyncio.open_connection(*self.up)
        head = [lines[0]] + [ln for ln in lines[1:] if not ln.lower().startswith(("connection:", "proxy-connection:"))]
        w.write(("\r\n".join(head) + "\r\nConnection: close\r\n\r\n").encode("latin-1") + body)
        await w.drain()
        raw = await r.read()
        w.close()
        return raw

    async def handle(self, link, reader, writer):
        link.stats["connections"] += 1
        try:
            await asyncio.sleep(link.handshake_ms() / 1000.0)           # TCP + TLS handshake before the first byte
            while True:
                msg = await read_message(reader, True)
                if msg is None:
                    break
                lines, body = msg
                t0 = time.monotonic()
                link.stats["requests"] += 1
                while time.monotonic() < link.outage_until:             # link down: the request hangs until it comes back
                    await asyncio.sleep(0.05)
                    if reader.at_eof() or writer.is_closing():
                        return
                await asyncio.sleep(link.one_way_ms() / 1000.0)         # request travels to the server
                try:
                    raw = await self.forward(lines, body)
                except OSError:
                    raw = b"HTTP/1.1 502 Bad Gateway\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
                await asyncio.sleep(link.one_way_ms() / 1000.0)         # and the reply travels back
                head_end = raw.find(b"\r\n\r\n")
                head_lines = raw[:head_end].decode("latin-1").split("\r\n") if head_end >= 0 else ["HTTP/1.1 502 Bad Gateway"]
                payload = raw[head_end + 4:] if head_end >= 0 else b""
                keep = link.profile["keepalive"] and (header(lines, "connection") or "").lower() != "close"
                out = [head_lines[0].replace("HTTP/1.0", "HTTP/1.1")]
                out += [ln for ln in head_lines[1:] if not ln.lower().startswith(("connection:", "content-length:"))]
                out += ["Content-Length: %d" % len(payload), "Connection: " + ("keep-alive" if keep else "close")]
                writer.write(("\r\n".join(out) + "\r\n\r\n").encode("latin-1") + payload)
                await writer.drain()
                d = (time.monotonic() - t0) * 1000
                link.stats["last_delay_ms"] = round(d, 1)
                link.stats["max_delay_ms"] = round(max(link.stats["max_delay_ms"], d), 1)
                if not keep:
                    break
        except (ConnectionError, asyncio.IncompleteReadError):
            link.stats["dropped"] += 1
        finally:
            writer.close()

    async def control(self, reader, writer):
        try:
            msg = await read_message(reader, True)
            if msg is None:
                return
            lines, body = msg
            method, path = lines[0].split()[:2]
            code, out = 200, {}
            try:
                if method == "GET" and path == "/state":
                    out = {k: {"preset": l.preset, "profile": l.profile, "stats": l.stats,
                               "outage_s": max(0.0, round(l.outage_until - time.monotonic(), 1))} for k, l in self.links.items()}
                elif method == "GET" and path == "/presets":
                    out = PRESETS
                elif method == "POST" and path in ("/link", "/outage"):
                    d = json.loads(body or b"{}")
                    which = ["esp", "ctl"] if d.get("path", "both") == "both" else [d.get("path")]
                    for w in which:
                        if w not in self.links:
                            raise ValueError("path must be esp, ctl or both")
                    for w in which:
                        if path == "/link":
                            self.links[w].set(**{k: v for k, v in d.items() if k != "path"})
                        else:
                            self.links[w].outage_until = time.monotonic() + float(d.get("seconds", 0))
                    out = {"ok": True}
                else:
                    code, out = 404, {"error": "not found"}
            except (ValueError, TypeError, json.JSONDecodeError) as e:
                code, out = 400, {"error": str(e)}
            data = json.dumps(out).encode()
            writer.write(b"HTTP/1.1 %d X\r\nContent-Type: application/json\r\nContent-Length: %d\r\nConnection: close\r\n\r\n" % (code, len(data)) + data)
            await writer.drain()
        finally:
            writer.close()


async def serve(args, ready=None):
    proxy = Proxy(args.upstream)
    for name, preset in (("esp", args.esp_preset or args.preset), ("ctl", args.ctl_preset or args.preset)):
        proxy.links[name].set(preset=preset)
    servers = [
        await asyncio.start_server(lambda r, w: proxy.handle(proxy.links["esp"], r, w), args.host, args.esp_port),
        await asyncio.start_server(lambda r, w: proxy.handle(proxy.links["ctl"], r, w), args.host, args.ctl_port),
        await asyncio.start_server(proxy.control, args.host, args.control_port),
    ]
    print("proxy: esp :%d  ctl :%d  control :%d  -> %s:%d  (esp=%s ctl=%s)" % (
        args.esp_port, args.ctl_port, args.control_port, proxy.up[0], proxy.up[1],
        proxy.links["esp"].preset, proxy.links["ctl"].preset), flush=True)
    if ready:
        ready.set()
    async with servers[0], servers[1], servers[2]:
        await asyncio.gather(*(s.serve_forever() for s in servers))


def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--upstream", default="127.0.0.1:8000", help="the server to forward to (host:port)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--esp-port", type=int, default=8100, help="point the ESP32 / emulator here (EMU_HOST=127.0.0.1:8100)")
    ap.add_argument("--ctl-port", type=int, default=8101, help="point the controller client here (--url http://127.0.0.1:8101/input)")
    ap.add_argument("--control-port", type=int, default=8102)
    ap.add_argument("--preset", default="home_wifi", choices=list(PRESETS))
    ap.add_argument("--esp-preset", choices=list(PRESETS))
    ap.add_argument("--ctl-preset", choices=list(PRESETS))
    return ap


if __name__ == "__main__":
    try:
        asyncio.run(serve(parser().parse_args()))
    except KeyboardInterrupt:
        pass
