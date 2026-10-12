# roobot-arm

Xbox controller -> HTTP server, as the input path for the robot arm.

- `server.py` – HTTP server, standard library only. `POST /input`, `GET /state`, `GET /stream`, `GET /health`, and the UWE-RArm backend's `/status`, `/calibration` and `/mode`.
- `controller_client.py` – reads a generic Xbox One controller (pygame) and POSTs its state as JSON.
- `controller_gui.py` - live Tk window: draws the controller and logs each move as the raw HTTP packet. `python controller_gui.py [--no-send]`

**Which firmware is "ours".** Everything defaults to the UWE-RArm firmware plus our patch (`firmware/stock/patches/damper-and-failsafes.patch`: joint speed limit, failsafes, tap-to-save): the virtual arm, the dashboard and the emulator.
Unmodified stock and the reference model are still selectable for comparison.

## Boxes, keys, tray and electromagnet (simulation)

The Robot page shows a small work cell: three boxes, each holding a key, and a tray (`sim/workcell.py` has the positions). The arm ends in a simulated electromagnet.
Bring the tool tip within 2 cm of a key and switch the magnet on to pick it up; it travels with the tip. Switch the magnet off and it falls onto whatever is below: a box floor, the tray, or the floor.
Switch it with the *Magnet* button in the side panel or the **X** button of the controller (each press toggles it; the packet field `X` is already sent). *Reset keys* puts the keys back.
**Recording the magnet in a sequence:** each saved position (tap A) also remembers whether the magnet was on. To record a pick-up, move to the key, tap A (magnet off), switch the magnet on, tap A again (same spot, magnet on);
carry the key over, tap A, switch the magnet off, tap A. Holding A then replays the moves and switches the magnet to each saved state when the arm gets there. The Robot page shows which saved positions have the magnet on.
This is simulated only (the virtual arm, not the emulated ESP32, since the emulated firmware keeps its own saved list) and lives in the hub: the firmware's saved positions have no magnet field.
It works with whichever arm is shown, the virtual one or the emulated ESP32. Nothing collides with the arm, and the stock firmware ignores `X`, so a real magnet needs a small firmware addition
(read `X`, drive a pin); say if you want that as a patch.

**Button taps and replay.** The server keeps a pressed button (A, B, X, Y) reading pressed for at least 0.25 s, because the ESP32 polls slowly and used to miss quick taps entirely.
The stock firmware saves a position the moment A goes down, so holding A to start a replay also saves where the arm was (usually home). The reference model and the firmware patch drop that one save when the hold turns into a replay; unpatched stock keeps the behaviour.

## How the ESP32 connection works

The ESP32 never talks to the controller. It is a client that polls a web server, and the controller (or this dashboard) feeds the same server.

1. **Power on.** The firmware sets all four servos to 90 degrees, then joins the WiFi network from `Secrets.h` (SSID, password, `HOST`).
2. **Checks the link.** It pings `HOST`, reports the latency to the server (`POST /status`), then asks `GET /mode` (the reply header `mode` says normal or Training).
3. **Loads calibration.** `GET /calibration` returns the joint limits and homes. With none (or Training mode, or A held at power on) it runs the calibration routine and `POST`s the result back. The dashboard pushes its calibration here before starting.
4. **Polls for input.** About every 20 ms it does `GET /controller` over HTTPS and reads `{"LSX","LSY","RSX","RSY","A","B","X","Y"}`. This is the packet the controller client or the dashboard sent to `POST /input` on the server.
5. **Moves the arm.** Each packet adds to a target position (sticks times elapsed time), inverse kinematics turns it into four joint angles, and those go straight to the servos. Reports go back with `POST /status`.
6. **If the link drops** the stock firmware has no timeout: it keeps the last target and, when packets resume, integrates the whole gap in one go. The patch, the dead-man in `server.py` and the SAFE switch exist for this.

On this PC the same steps run in the emulator, with a proxy standing in for the network (see below).

## One tap

Double-click `Start Dashboard.bat` (Windows) or run `./start.sh` / `python launch.py`: it starts the input server and the hub and opens the **Robot** page, a large 3D view of the arm
(drag to turn, scroll or pinch to zoom, Top/Side buttons) with the controls, emulator and status in a side panel. Output starts SAFE.

**Phone:** the launcher prints a link for your phone (same WiFi; it also appears in the side panel on the computer). Open it once, then *Add to Home Screen* (iPhone: Share; Android: menu, Install app).
From then on the icon opens the Robot page in one tap while the launcher is running, with virtual sticks as the default input on touch screens. The link carries a key (saved in `.hub_token`); other devices
without it are refused. `python launch.py --local-only` turns phone access off. Windows asks once to allow Python through the firewall: allow private networks.

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

The wire format is the one the UWE-RArm firmware reads:

```json
{"LSX": 0.0, "LSY": 0.0, "RSX": 0.0, "RSY": 0.0, "A": false, "B": false, "X": false, "Y": false}
```

Sticks are -1..1 (raw pygame axes, so stick up is negative; the 0.12 client deadzone is followed by the firmware's own 0.2). The client sends on change and at least every 0.5 s. Triggers, bumpers and the d-pad are not on the wire; the client, GUI and dashboard still read them locally for drawing.

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

**SAFE / ARMED.** The hub starts SAFE. While SAFE it sends nothing over USB, ignores browser input, and tells the local server to pause forwarding to the end server (`POST /control`). Arming needs a confirmation; **Escape** returns to SAFE instantly, and ARMED also drops back to SAFE by itself if no dashboard is open for 15 s. Input from the pygame window goes to the local server directly, but forwarding stays paused while SAFE, so it cannot reach the end server either. Firmware upload is only allowed while SAFE. If the hub is killed abruptly the server stays paused (the safe direction): start the hub and arm again, or resume forwarding by hand with `curl -X POST -H "Content-Type: application/json" -d '{"forward": true}' http://127.0.0.1:8000/control` (a clean Ctrl+C of the hub resumes it for you).

**Needs:** Python with pyserial (already in `requirements.txt`) for the USB link; Docker Desktop for the Docker server; arduino-cli for firmware (both optional, the page shows what is missing). The hub only listens on 127.0.0.1, rejects cross-origin and foreign-Host requests, and never sends `.env` values to the browser.

**Adding a page.** Each page is one function in `dashboard/static/app.js` (`mount(root)` returning optional `update/packet/board/log/frame` hooks) plus one line in `PAGES`. The hub already streams packets, board output, logs and status to every page.

Design: dense "cockpit" layout, one accent colour, square corners, dark and light themes, reduced-motion support, following the Taste skill's dashboard guidance. Icons are [Phosphor Icons](https://github.com/phosphor-icons/core) (MIT).

## Virtual arm (no hardware)

`sim/armsim.py` is a Python model of the UWE-RArm firmware (virtual ESP32 + arm, standard library only):
sticks -> XYZ -> inverse kinematics -> 4 joint angles, with calibration limits and homes (reversed range = inverted servo),
**B** go home, **A** save a position, hold **A** replay. The hub runs it at 50 Hz on whatever the local server receives
(always, including while SAFE), draws it on the Control page, mirrors its status to `POST /status`, and keeps
`/calibration` in step with the Calibration panel.

```
python server.py                 # terminal 1
python dashboard/hub.py          # terminal 2: Control page shows the arm
python controller_client.py --simulate    # terminal 3, or use a real pad / the browser sticks
python -m unittest discover -s sim -v       # all tests, including the C++ firmware check (needs g++)
```

**Damper.** The slider on the Control page limits how fast any joint moves, in deg/s (0 = no limit, capped at 180). It applies to the sticks,
Go home and replay; all four joints are scaled together so the tool stays on its path. By default it affects the simulation only, so the packets the arm receives are exactly what the stock UWE-RArm firmware expects. With `python dashboard/hub.py --send-speed` the limit is sent to the real arm too, which needs the modified firmware below: the hub posts it to the server (`POST /control {"maxSpeed": n}`, 1-60 deg/s; 0 is sent as 60), the server adds `"maxSpeed"` to every packet it forwards or serves on `/controller`, and the firmware in `firmware/rarm/` applies it.

**Differences from the firmware** (kept on purpose, see the module docstring): the pose always comes from the joint angles, so moving after Go home no longer
snaps the arm back to its old position; input older than 1.5 s stops the arm; the tool length is a setting (63 mm with the test ball, 53 mm without).
Calibration *training* (the A-button flow) is not simulated: enter limits and homes in the panel instead.

**Dead-man.** The server sends the arm a neutral packet when input has been silent for 1.5 s and when forwarding is paused (SAFE), because the UWE-RArm backend keeps the last packet for ever. The firmware also treats input older than 1.5 s as centred sticks.

**PS5 controller.** Yes. `controller_client.py` and `controller_gui.py` pick the DualSense layout automatically from the pad's name (`--profile ps5` forces it, `--profile xbox` the old one): Cross/Circle/Square/Triangle are A/B/X/Y, so **Circle = go home** and **Cross = save / hold to replay**. The browser sticks on the Control page use the standard Gamepad API mapping, which DualSense matches. Wire it by USB or Bluetooth; if a button or stick is off, run with `--debug` to see the raw indices and edit the `ps5` profile at the top of `controller_client.py`.

**Which firmware is simulated.** The Control page has a selector: the reference model, the **stock UWE-RArm firmware** (quirks included: it swings back to the
pre-home pose 3 s after Go home, has no speed limit, and integrates packet gaps in one go), or **stock + patch**. The stock and patched models are checked against the real
`main.cpp` compiled on a PC (`sim/test_stock.py`). See `firmware/stock/README.md`; the patch (`firmware/stock/patches/`) is the small, reviewable way to add the damper and failsafes to his firmware.

## Emulated ESP32 on a network (emu/)

`emu/` runs the firmware's real `main.cpp` on your PC against the real server over real sockets, with a proxy (`emu/netproxy.py`) that makes the link behave like the network you pick
(latency, jitter, loss, outages, TLS handshake cost). See `emu/README.md` (includes Windows setup via MSYS2 or WSL). The preset numbers are assumptions until calibrated against the board.

**From the dashboard:** the Control page has an *Emulated ESP32 and network* section. Pick the firmware and a network preset and press Start emulator (it compiles the firmware first, a few seconds;
needs g++, see `emu/README.md`). The arm drawn on that page then shows the emulated servos, and the sticks on the page travel through the emulated controller link, even while output is SAFE (nothing is
forwarded to the real arm). Change the round trip, jitter, loss or connection reuse for the ESP32 link, the controller link or both while it runs, or drop a link for 3 s, and watch the poll times and
longest stall. Run `python server.py` first, then `python dashboard/hub.py`.

## Real firmware (firmware/rarm)

`firmware/rarm/` is the UWE-RArm ESP32 firmware (copied from Samwally1/UWE-RArm at 3b9b8aa) with the damper and a stale-input stop added, plus a PC test that runs its movement code against the Python arm. This is a larger rewrite and optional; the smaller route is the patch in `firmware/stock/`. The stock firmware works with everything else here unchanged. See `firmware/rarm/README.md` before flashing it.

The "Arm and board" and "Firmware" pages still target the older USB jog sketch in `arduino/arm_receiver/`.
