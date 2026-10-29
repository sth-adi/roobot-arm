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
**One-press replay:** the *Play saved* button (or **Y** on the controller) starts the replay and keeps it looping with nothing held; press again to stop. Holding A still works but stops whenever A or the connection blips.
One-press replay is simulation only: the real firmware replays only while A is held.

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

## Arm Studio (new dashboard)

`http://127.0.0.1:8765/` opens the classic dashboard. **Arm Studio**, an alternative React + three.js app, is at `http://127.0.0.1:8765/studio`: live 3D arm and work cell, a Teach editor (record poses, mark grab/release or just "go to", reorder, save), a library with Run / Reverse / Edit / Rename / Delete (Delete asks twice), Ask the arm, sticks/keyboard control, and Ctrl+K for a command palette. The **System** tab has everything the classic dashboard had: Overview and system map, Arm & tuning (virtual arm, firmware model, damper, calibration, tool length, emulated ESP32 and network), Packets, Board log, Firmware build/upload, Services, Project and the AI packet composer. Arm Studio is optional; the classic dashboard has the same features.

The built app is committed in `dashboard/static/studio`, so you do not need Node to run it. To change it: `cd dashboard/studio && npm install && npm run build` (or `npm run dev` for live reload against a running hub).

## Driving the real arm: jog or mirror (USB jog sketch)

What the board does depends on the sketch on it (Firmware page shows which one is flashed). The **USB jog sketch** (`arduino/arm_receiver`) has two modes, and neither runs inverse kinematics on the board:

- **Jog** (default): each thumbstick axis moves one joint's pulse directly (`JOG cmd ... pos(us) ...` in the board log). **B** walks every output to 90 degrees (1450 us in Sam's mapping, the virtual arm's home) at the jog speed setting; any stick movement cancels it. No sequences.
- **Mirror** (Arm and board page, off until you switch it on): the hub's virtual arm does everything (sticks, `SticksToXYZ`, inverse kinematics, home on B, sequences and their per-pose speeds) and the hub streams its four joint angles as `@d1,d2,d3,d4,seq` over USB. The board follows them. Only while the output is ARMED. Pulses follow Sam's firmware (500 us at 0 degrees to 2400 us at 180, so 90 degrees = 1450 us; `!pulse lo,hi` changes it), because the model's joint angles and the trained limits and homes are in that convention. Safety is on the board: pulses never leave that range, it never moves faster than 60 deg/s, it holds still if frames stop for 0.4 s or a stick frame arrives, and it ignores WiFi input while mirroring.

Before trusting a sequence in mirror mode, use **Nudge joint 1..4** to see which part of the real arm each joint drives, and fix the joint to channel map (`!map 3,2,1,0` = joint 1 on GPIO32, 2 on 27, 3 on 26, 4 on 25, as wired for Sam's firmware) if they differ. **Check whether the real arm matches the model** (Arm and board page, "Does the real arm match the model?"): with mirror on and the output armed, the arm goes to one tall pose; measure the tool tip with a ruler (how far out from the base axis, how high above the table) and the hub says which servo mapping your numbers are nearer to, Sam's 500 to 2400 us or the earlier 1000 to 2000 us guess. The two predict tip positions about 9 cm apart, so a ruler error of a centimetre cannot change the answer, and a tip that matches neither points at the joint map, a reversed joint, the tool length or the limits instead. An angle-based fit (`sim/servo_map.py`) is kept under Advanced. Sam's mapping is the default and what the board starts with; the check is optional and nobody has run it on the real arm yet. Set each joint's real limits (and reverse a joint's pair, e.g. 180,0, if it turns the wrong way) on the Control page before the real arm moves: with the default 0..180 the arm can be told to go to the very end of a servo's travel. The virtual arm's tool length (`DEFAULT_TOOL_MM`, 63) differs from the stock firmware header (53): set it to the real distance from the wrist to the tip.

**The board can run the inverse kinematics itself (mirror, board IK).** With the current USB sketch the hub no longer has to turn positions into joint angles: it sends the virtual arm's **tool-tip pose** `#x,y,z,angle,seq` (mm, degrees) and the board solves it with Sam's own `InverseKinTransform.h` (unchanged copies of `firmware/stock/src` sit in the sketch folder, a test fails if they drift; glue in `arm_ik.h`), then follows the result like an `@` frame (same pulse clamp, 60 deg/s cap, hold if frames stop). It needs the real arm's calibration, which Sam's firmware keeps on the end server: the hub reads `GET <FORWARD_URL host>/calibration` (read-only; the last answer is cached in the git-ignored `.hub_calibration.json`) at start, or with the button on the Arm and board page (**not** while the output is ARMED with mirror on: it would move the arm), uses it in the virtual arm, the local server and the board (`!cal min1,max1,...,home4`, sent every time the port opens), and sets the tool length to the 53 mm Sam's code has. A reversed pair of limits (e.g. joint 3: 161 to 21) marks an inverted servo, which the board flips (`180 - angle`) like Sam's firmware. Without the calibration (or on an older sketch) the hub falls back to joint angles. The board says which it is doing: `mirror ik: supported, calibration set|missing`, shown on the Arm and board page. The five default points (Points page) are reachable with the real calibration and the board follows the whole loop to within 0.06 degrees of the virtual arm (`sim/test_ik_board.py`). Flash the **USB jog sketch** from the Firmware page (output SAFE) to get it. The servo-mapping checks on that page use a tall test pose that the real joint limits can refuse; they are optional.

**Board over WiFi, no USB cable.** The board can connect OUT to the hub on this PC and be driven exactly like over the COM port: the hub listens on its port + 2 (8767 with the default 8765; `--no-board-link` turns it off) and the board, once it has `MIRROR_HOST` (this PC's address), `MIRROR_PORT` and `MIRROR_KEY` in its git-ignored `secrets.h`, connects and says `HELLO <key>`. From then on the same lines flow both ways (`@`/`#`/`$`/`!` frames to the board, everything it prints back), so mirror mode, board IK, the calibration, the board log and the settings all work unchanged (`dashboard/netlink.py`; `serial_link.SerialLink.attach` treats the connection like a freshly opened port). The Arm and board page shows the three lines to paste (`GET /api/boardlink`), whether the board is connected, and flash it once over USB to turn the feature on. Safety is the same as USB: only while ARMED, frames older than 400 ms mean the arm holds, a dropped connection is noticed (the hub pings once a second; the board reconnects after 6 s of silence), and a COM port that is open wins (the WiFi connection is refused, the board retries). The board only ever connects to the one host in `secrets.h`; the hub only accepts a connection that starts with the key. Not encrypted: use it on a network you trust. Flashing still needs the cable. The PC's address changes with the network (give it a fixed one on the router, or change `MIRROR_HOST` and flash again). Without `MIRROR_HOST` none of it is compiled into the sketch. Tests: `sim/test_netlink.py` (real sockets), `sim/test_net_out.py`.

Our firmware (the sequence, magnet and speed patches) is the other route. It fetches sequences over HTTPS from `HOST`, so that server needs `/sequence` and `/armstate`; the server in `.env` as `FORWARD_URL` may not have them.

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
| Firmware | Swap what is on the ESP32: Sam's firmware, ours (Sam's plus both patches) or the USB jog sketch. Each is built once with arduino-cli and cached, so swapping back is just an upload; the page shows which one is on the board. WiFi name comes from `arduino/arm_receiver/secrets.h`, the host from `FORWARD_URL` in `.env`. Port released and retaken automatically. |
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

The "Arm and board" page still targets the older USB jog sketch in `arduino/arm_receiver/`; the Firmware page can flash that sketch too, next to Sam's and ours.

### Sequences (named JSON, reversible)
A sequence is a named list of poses (`x, y, z` in mm, tool `angle` in degrees) with an optional magnet `action` per pose: `grab` or `release`. Storing the magnet as an action is what lets a sequence run backwards: the order flips and grab and release swap, so a box-to-tray move becomes tray-to-box without re-teaching it. `requires` and `ends` hold the conditions the task assumes and leaves behind, for the AI to match against.

In the side panel, **Sequences**: type a name and a description, **Save to host** writes `sequences/<name>.json` on the hub's machine; pick one and **Run** or **Run reversed** plays it once (it keeps the keys where they are, so a reversed task finds the key where the forward one left it). **Copy JSON** / **Load JSON** move a packet by hand.
API: `GET /api/sequence` (the current saved positions as a packet), `GET /api/sequences` (the library), `GET /api/sequences/<name>`, `POST /api/sequence` with `{"name","description","requires","ends"}` to save, `{"load": packet}` or `{"name": n, "use": true}` to load, plus `"run": true` and `"reverse": true`.

### Ask the arm (state packet + AI)
`GET /api/state` is the packet an AI reads: `pose`, `at_home`, `magnet`, `holding`, `home_grip` (at home, magnet off, nothing held), `mode`, `running_step`, where each key is, the `last_run` sequence and the current `task`. `POST /api/ask {"request": "put the key from box 2 on the tray"}` (or the **Ask the arm** box in the side panel) does this: if the arm is not at home grip it goes home first; it then picks a sequence from the library (forwards, or reversed for the opposite task; "put it back" undoes the last run) and runs it once. A sequence moves on once the arm has reached each pose, not on the 1.5 s clock. The model only chooses a name from the library, and the answer is checked against the library. With `GROQ_API_KEY` in the environment or `.env` it asks Groq; without a key or a network it falls back to keyword matching.

**Quick commands.** Ask can also do a few things that are not sequences, from a fixed menu: *go to home position* (`go_home`), *stop* (ends any run and holds position), *magnet on / off* (also "release it") and *where is key 2?* (a status sentence built from the state packet, no motion), plus **free movement** below. The model only picks an action name and, for the magnet, on or off: it never writes coordinates, and moving the real arm still needs the output ARMED (`OUTPUT_SAFE` otherwise). **Stop** is recognised by the hub itself, before any AI or network call, so it works with no key, no network and while another task is running.

**Free movement.** *"move up 2 cm"* or *"go left 5 mm"* nudges the tool tip by a small relative distance. It needs an explicit number and unit, is capped at 100 mm in total, and both the end point and the straight line to it must be reachable (never below 10 mm). It runs as a one-pose sequence and leaves the magnet as it is. It is never used for keys: a request that mentions a key, box or the tray only runs a taught, checked sequence, and with none available it stops with an error instead of improvising.

**Voice (optional).** The Chat page has a **Mic** button when `pip install faster-whisper` is done (it is not in requirements.txt: it is only for the hub PC, not the Docker server). Click it, speak, click again (or wait 12 s). The browser makes 16 kHz mono audio and `POST /api/transcribe` (raw 16-bit PCM, header `X-Hub: 1`) turns it into text with a local Whisper model (`base.en`, about 150 MB, downloaded once; change it with `WHISPER_MODEL`), so no audio leaves the PC after that. The text is typed into the box and only a plain *stop* is sent at once; anything else waits for **Send**, so a misheard word cannot move the arm. The first use takes a while to load the model. Browsers only allow the microphone on `http://localhost:8765` (this PC) or HTTPS, so with `--lan` the hub also serves **HTTPS on port+1** (8766) with a certificate it makes for itself (`dashboard/tls.py`, kept with its private key in the git-ignored `.hub_cert/`, made again when this PC's address changes; `--no-https` turns it off). Use the second phone link (launcher window, `phone_link.txt`, or Services page): the phone warns about the certificate once, so compare the SHA-256 fingerprint the hub prints, then continue. Same key as the plain link, same checks. Code: `dashboard/voice.py`, `dashboard/tls.py`; tests: `sim/test_voice.py`, `sim/test_tls.py`.

## Sam's backend and firmware, with our upgrades on top

The plan: **his server and his firmware are the base; our upgrades are patches on top, and the dashboard drives the arm through his API.**

* `firmware/sam/` is his code, **untouched** (commit 9ad64eb, 29 Oct). The ESP32 sends its four joint angles to `GET /UpdateMovement?s1=&s2=&s3=&s4=`; the backend works out where the tool is (63 mm tool), draws a straight line to the target, and answers with timestamped joint angles and a `Movement-ID`; the firmware plays them and confirms with `POST /movement-complete`. The target comes from `POST /target` (headers `X`, `Y`, `Z` mm, `A` degrees, optional `Speed` up to 50 mm/s); `GET /movement` shows the last move. His tests: `cd firmware/sam/backend && python -m unittest discover -s tests`. The older copies (`firmware/stock`, `firmware/rarm`, his 12 Oct code) are what the simulation tests are built on.
* `firmware/ours/` is **our backend**: `python firmware/ours/build_backend.py OUT_DIR` copies his backend, applies `backend-safe-planner.patch` and adds `motion/safe_planner.py` (= `dashboard/planner.py`). Then `python main.py` in `OUT_DIR` (port 8000). It changes:
  * every move is **checked before it is sent**: each step reachable, inside the calibrated joint limits, away from the base axis, no joint jumping between steps; a refused move says why (HTTP 422 with the reason) and nothing is sent;
  * a fixed 50 Hz update rate with a smooth start and stop, and the **whole move slowed down** so no joint exceeds 30 deg/s (`MAX_JOINT_DPS` in `GlobalVariables.py`), so the path stays a straight line, just slower. (His planner: 5 steps per mm, so up to 250 updates a second at his 50 mm/s limit, and no joint limit;)
  * a move that only changes the tool angle is no longer ignored (his planner returned nothing for it);
  * `POST /target` is checked against where the arm is *now* (refused with the reason if it could not get there), `GET /position` says where the arm last reported it was (and how long ago), and `POST /hold` makes the target the current position, so the dashboard can stop wanting to go anywhere. A packet already being played still finishes;
  * one of his own tests is adapted (it assumed the number of lines does not depend on speed); the other seven pass unchanged.
* `dashboard/planner.py` is the planner (his inverse kinematics and packet format, checked first; pure Python). `dashboard/backend_client.py` is the dashboard's side: `BackendClient` (set a target, hold, read the movement and position) and `PoseRunner`, which runs a list of poses one after another through his API, waits until the arm has reported arriving, stops with a clear error if a pose is refused or not reached in time (and tells the backend to hold), and can be stopped from another thread. A pose with a grab/release action needs a way to switch the magnet, which his firmware does not have, so such a run is refused up front.
* Tests: `sim/test_planner.py`, `sim/test_ours_backend.py` (builds the backend, runs his tests and ours over HTTP), `sim/test_backend_driver.py` (the runner against the built backend and a fake ESP32 that behaves like his firmware).

Not done yet: our **firmware** patch set on his new structure (the magnet, a way to stop a packet part way, failsafes), and connecting the hub's Ask box and sequences to the runner. Nothing here has been tried on the real arm.

### Running sequences on the (emulated) ESP32 itself
Start the emulator on the Control page with **Our firmware (stock + patches, magnet and sequences)**, or flash `firmware/stock/src` with both patches applied. While that firmware is reporting its state (`GET /armstate` on the input server), the hub treats the ESP32 as the truth: **Run**, **Run reversed** and **Ask the arm** send the sequence to it (`POST /sequence`), the firmware runs it once, waiting for arrival at each pose and switching the magnet relay, and the dashboard's magnet and keys follow what it reports. Without it, the same buttons drive the virtual arm. See `firmware/stock/README.md` for the protocol and what is not covered (the real board). The launcher now also replaces an out-of-date input server, not just an out-of-date hub.
