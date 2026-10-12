# Emulated ESP32 on a real network

The firmware's real `main.cpp` (stock, patched, or the `firmware/rarm` rewrite), compiled for this PC and run in real time.
Its HTTP requests are real sockets to a server on your machine, so it really waits for the network and its `loop()` really blocks while it does.
Servo commands come out as JSON lines. It is not a CPU emulator: the ESP32's WiFi stack, TLS and servo timing are not reproduced, only the firmware's logic and what the network does to it.

```
controller_client.py --url http://127.0.0.1:8101/input        (or straight to the server on :8000)
        |
        v
  netproxy.py  --ctl-port 8101   --esp-port 8100   --upstream 127.0.0.1:8000   --preset home_wifi
                                       ^
                                       |
  emulator.py --firmware stock --host 127.0.0.1:8100            (the emulated ESP32 polls GET /controller)
```

## Run it

Needs g++ (and `patch` for `--firmware patched`), Linux, macOS or WSL (it uses POSIX sockets).

```
python server.py                                                     # terminal 1
python emu/netproxy.py --preset home_wifi                            # terminal 2: esp :8100, controller :8101, control :8102
python controller_client.py --url http://127.0.0.1:8101/input        # terminal 3 (or --simulate)
python emu/emulator.py --firmware stock --host 127.0.0.1:8100        # terminal 4: prints servo positions and network stats
```

Change the link while it runs: `curl -X POST localhost:8102/link -d '{"path":"esp","preset":"poor"}'`, `-d '{"path":"esp","rtt_ms":120,"loss":0.03}'`,
or drop it for 3 s with `curl -X POST localhost:8102/outage -d '{"path":"esp","seconds":3}'`. `curl localhost:8102/state` shows what each link is doing.
To use a real network instead of the proxy, point `--host` at any server (for example a deployed backend); the emulator speaks plain HTTP to whatever you give it.

## What the link models

Per request: half the round-trip time plus jitter each way, retransmission stalls for lost packets (400 ms, doubling), a hold during an outage.
Per new connection: the TCP + TLS handshake (3 round trips) plus `tls_cpu_ms` for the ESP32's own crypto. With `keepalive` on the connection is reused, like a hosted
TLS front end; with it off (`direct_http`) every poll opens a new connection, like talking straight to this repo's server or UWE-RArm's backend (both are plain Python `HTTPServer`s that close after each reply).

| preset | round trip | jitter | loss | note |
|---|---|---|---|---|
| lan | 3 ms | 1 | 0 | |
| home_wifi | 25 ms | 10 | 0.5 % | |
| campus_wifi | 45 ms | 25 | 1 % | |
| tether_4g | 70 ms | 35 | 2 % | |
| poor | 180 ms | 120 | 6 % | |
| direct_http | 25 ms | 10 | 0.5 % | new connection per poll |

**These numbers are assumptions, not measurements.** Calibrate them against the real board (log how long its polls take) before drawing conclusions about absolute delays.

## What it has shown already

* On `home_wifi` with a reused connection the stock firmware polls in about 45 ms. Straight to a plain HTTP/1.0 server with the handshake cost, a poll takes about 465 ms, so the arm gets about two updates a second.
  Whether a real deployment has that cost depends on whether something in front of the server keeps the TLS connection open.
* A 2 s outage with a stick held makes the stock firmware jump about 9 degrees at the base the moment the link returns; the patched firmware does not (`sim/test_emulator.py`).

## Tests

`python -m unittest discover -s sim` includes `sim/test_emulator.py` (about 40 s, real time). `ROOBOT_SKIP_SLOW=1` skips it.
