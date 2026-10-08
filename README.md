# roobot-arm

Xbox controller -> HTTP server, as the input path for the robot arm.

- `server.py` – HTTP server, standard library only. `POST /input`, `GET /state`, `GET /health`.
- `controller_client.py` – reads a generic Xbox One controller (pygame) and POSTs its state as JSON.
- `controller_gui.py` - live Tk window: draws the controller and logs each move as the raw HTTP packet. `python controller_gui.py [--no-send]`

## Run

```bash
# terminal 1
python server.py --port 8000

# terminal 2
pip install -r requirements.txt
python controller_client.py --url http://localhost:8000/input
```

No controller handy? `python controller_client.py --simulate` sends fake data (needs no pygame).

Check the latest state: `curl http://localhost:8000/state`

## Payload

```json
{"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lt": 0.0, "rt": 0.0,
 "buttons": {"a": false, "b": false, "x": false, "y": false, "lb": false, "rb": false,
             "back": false, "start": false, "ls": false, "rs": false},
 "dpad": [0, 0]}
```

Sticks are -1..1 with a 0.12 deadzone, triggers 0..1. The client sends on change and at least every 0.5 s.

## Notes

Axis and button indices vary by OS/driver. If something looks wrong, run the client with `--debug` and edit `AXES` / `BUTTONS` at the top of `controller_client.py`.

## Docker (server only)

The container runs `server.py` and can forward every received state to another server.

```bash
docker build -t roobot-arm-server .
docker run -p 8000:8000 -e FORWARD_URL=http://<other-server-ip>:<port>/<path> roobot-arm-server
# or: FORWARD_URL=http://... docker compose up --build
```

The controller client/GUI keep running on the PC and POST to `http://localhost:8000/input`. Forwarded requests use the same JSON payload; `GET /state` shows forwarding status (`sent`, `failed`, last `error`). Use `host.docker.internal` as the host to reach a server running on the PC itself.

Config is read from a git-ignored `.env` file: `cp .env.example .env` and fill in `FORWARD_URL`. Compose loads it automatically; for plain `docker run` use `--env-file .env`. Don't commit `.env`.

## ESP32 outputs, rate damper and input paths

`arduino/arm_receiver/` drives 4 servo-style PWM outputs (50 Hz, 1000-2000 us) from the thumbsticks.
**Jog control:** each stick axis is a *velocity*. Pushing a stick moves that output; **when you release it the output stays where it is** (it does not spring back to centre). Outputs start at 1500 us (centre) on power-up and are clamped to 1000-2000 us.

| Output | Pin | Axis |
|---|---|---|
| OUT1 | GPIO25 | Left stick X |
| OUT2 | GPIO26 | Left stick Y (up raises the pulse) |
| OUT3 | GPIO27 | Right stick X |
| OUT4 | GPIO32 | Right stick Y (up raises the pulse) |

**Rate damper.** Full stick moves an output at the max speed (default **5 deg/s**); partial deflection moves it proportionally slower.
Servo travel is assumed to be 180 deg across 1000-2000 us, so 5 deg/s = 27.8 us/s and holding full stick takes 36 s to cross the whole range.
Change it live over USB serial; the board saves the values:

```
!rate 5      max speed at full stick in deg/s (0 = no limit, capped at 180 deg/s), saved
!live 5      same, but not saved (used while dragging the slider)
!range 180   servo travel in degrees represented by 1000-2000 us (use 90/270 to match your servo)
!show        print current settings
```

or use the **speed slider in the GUI** (max speed at full stick, 0-60 deg/s, 0 = no limit): dragging changes the speed live (`!live`, RAM only, no flash writes);
releasing saves it (`!rate`). The slider syncs to the board's reported value when the cable connects and needs the USB cable.
Or let the GUI send them every time the cable connects: `python controller_gui.py --rate 5 --range 180`.
Defaults live in `arm_receiver.ino` (`DEFAULT_MAX_RATE_DEG_S`, `DEFAULT_RANGE_DEG`).
Fail-safe: with no valid input all stick commands drop to zero, so the arm **stops and holds its position** (a held stick never keeps driving without fresh input), and resumes when valid input returns. USB input is treated as lost after 400 ms, so at the maximum speed the arm can coast for up to 0.4 s after the cable is pulled (about 2 degrees at 5 deg/s).
Because position is integrated on the board, a reset or power cycle returns every output to centre.

**Input paths** (highest priority first):
1. **Direct USB serial** - `controller_gui.py` finds the board's COM port by USB ID (`serial_link.py`), opens it without resetting the board and streams compact `$` lines. Used only while the port exists; no cable = silently skipped. `--no-serial` disables it, `--serial-port COMx` forces a port. Only one program can hold the port, so close the GUI before uploading or opening the Serial Monitor.
2. **Plain-TCP stream** - define `STREAM_HOST`/`STREAM_PORT` in `secrets.h` (the PC's IP on the same network as the ESP32); the sketch holds one `GET /stream` connection open (no TLS, no polling). The server pushes a line per new state.
3. **HTTPS polling** of `STATE_URL` - used when `STREAM_HOST` is not defined.

## Dashboard (Arm Hub)

One local web page to watch, control and launch everything. Start it from the repo folder (or double-click `Start Dashboard.bat`):

```
python dashboard/hub.py        # opens http://127.0.0.1:8765
```

| Page | What it does |
|---|---|
| Overview | Live system map (input -> local server -> end server -> ESP32 -> arm), key numbers, and a "needs attention" list with one-click fixes. |
| Control | Drive from the browser (browser gamepad, virtual sticks, demo wave) or just watch the server stream. A **preview arm** mirrors the firmware's jog logic, so the whole front end can be tested with no hardware. Speed and servo-travel settings. |
| Packets | Every controller packet, exactly as sent (raw HTTP and JSON), with filter, pause and a packets-per-second graph. |
| Arm and board | The ESP32 over USB: live board log, settings, last output positions, USB link on/off. |
| Firmware | Compile and upload the ESP32 sketch with arduino-cli (flash and memory use shown, port released and retaken automatically). |
| Services | Start/stop Docker Desktop, the local server (Docker or Python) and the controller window; config checks; logs. |
| AI and jobs | Placeholder for the AI assistant and JSON instruction packets (draft packet composer, dry run only; nothing is sent). |
| Project | Git branch, uncommitted/unpushed counts, recent commits. |

**SAFE / ARMED.** The hub starts SAFE. While SAFE it sends nothing over USB, ignores browser input, and tells the local server to pause forwarding to the end server (`POST /control`). Arming needs a confirmation; **Escape** returns to SAFE instantly, and ARMED also drops back to SAFE by itself if no dashboard is open for 15 s. Input from the pygame window goes to the local server directly, but forwarding stays paused while SAFE, so it cannot reach the end server either. Firmware upload is only allowed while SAFE.

**Needs:** Python with pyserial (already in `requirements.txt`) for the USB link; Docker Desktop for the Docker server; arduino-cli for firmware (both optional, the page shows what is missing). The hub only listens on 127.0.0.1, rejects cross-origin and foreign-Host requests, and never sends `.env` values to the browser.

**Adding a page.** Each page is one function in `dashboard/static/app.js` (`mount(root)` returning optional `update/packet/board/log/frame` hooks) plus one line in `PAGES`. The hub already streams packets, board output, logs and status to every page.

Design: dense "cockpit" layout, one accent colour, square corners, dark and light themes, reduced-motion support, following the Taste skill's dashboard guidance. Icons are [Phosphor Icons](https://github.com/phosphor-icons/core) (MIT).
