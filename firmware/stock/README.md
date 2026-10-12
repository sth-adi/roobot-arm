# Stock firmware and the damper/failsafe patch

`src/` is the UWE-RArm firmware **untouched** (Samwally1/UWE-RArm, "Robot Arm Firmware", commit 3b9b8aa), kept here so it can be tested against the simulation.
`patches/damper-and-failsafes.patch` is the smallest change that adds a speed limit and a few failsafes without touching the control logic
(about 90 lines added, 25 removed, in `main.cpp` plus three one-line additions in the headers). Apply it to a copy:

```
cp -r firmware/stock/src /somewhere/src && patch -p1 -d /somewhere < firmware/stock/patches/damper-and-failsafes.patch
```

## What the stock firmware does that you might not expect

* After **B** (go home) the arm goes home, but the XYZ target is not updated. Three seconds later the servos are released, and the very next packet
  (sticks centred or not) writes the old target to the servos again: **the arm swings back to where it was before**.
* There is no speed limit: every new angle goes straight to the servos.
* The time since the last good packet is integrated in one go, so after a WiFi gap a held stick makes one big jump.
* "Go home" gives up after 3 s whether or not the arm has arrived.
* The backend keeps the last packet for ever; the firmware cannot tell. (This repo's server sends a neutral packet when input goes quiet.)

## What the patch changes

| | stock | patched |
|---|---|---|
| Speed | straight to the servos | servo angles follow the target at most `MaxJointDegPerSecond` (default 30, 1-60 live via `"maxSpeed"` in the packet), all joints scaled together |
| Gap in packets | whole gap integrated | at most 100 ms per packet |
| Arm behind its target | target keeps running ahead | stick input pauses while a joint lags more than 25 degrees |
| Go home | writes homes, detaches after 3 s | walks home at the limit, re-syncs the target, detaches 1 s after arriving (no swing back) |
| Servo writes | every packet | only when the whole-degree command changes, so a released servo is not powered up by rounding noise |

Replay, save, training and the IK are unchanged. The patch only adds `"maxSpeed"` parsing to `DecodeStickInput`; a packet without it works as before.

## Tests

`sim/test_stock.py` builds the real `main.cpp` (stock, and with the patch applied) on a PC with fake time and recording servos, and checks every servo command
against `ArmSim(model="stock")`. The dashboard's Control page can run the simulation as the reference, the stock firmware or the patched one.
Not covered: the ESP32 itself (WiFi, servo timing, real latency).

## Our firmware: the second patch (`patches/sequences-and-magnet.patch`)

Applied after the damper patch (`emu/emulator.py --firmware ours` builds exactly that). It is what lets an AI or the dashboard hand the arm a sequence instead of someone holding A.
Everything it adds is optional: with a server that has no `/sequence` and a packet without `"seqId"`, the arm behaves exactly like the patched one.

| Added | What it does |
|---|---|
| Magnet pin | `MagnetPin` (placeholder GPIO 23 in `RobotCharacteristics.h`, set the real one) drives a relay; `MagnetRelayActiveLow` flips the level for boards that switch on LOW. Off at power-up. **X** (press edge) toggles it. |
| `"seqId"` in the packet | When it changes, the firmware GETs `/sequence` (`{"id", "poses": [[x, y, z, angle, action]]}`, action 0 none / 1 grab / 2 release), **replaces** the saved list with it and runs it once. An empty list stops a run. A sequence queued before power-up is not run. |
| Running | Each pose is sent, then the arm waits until it has got there (at least 0.3 s, at most 12 s), does the pose's magnet action and pauses 0.4 s so the magnet takes hold. A and the sticks are ignored during a run; **B** aborts it. An unreachable pose stops the run. |
| `POST /armstate` | `{"x","y","z","angle","magnet","mode","step","saved","seqId","seqDone","atHome"}`: every 0.7 s during a run, every 2.5 s otherwise, and at once when a run ends or X is pressed. The dashboard and the AI read it from the server's `GET /armstate`. Each POST blocks the loop for the time one HTTPS request takes. |

Not covered: the real board. Pin choice, relay polarity, how long a real magnet needs to take hold, and the blocking state POST over real WiFi all need checking on hardware.
The server this firmware talks to must have `/sequence` and `/armstate` (this repo's `server.py` does; the original UWE-RArm backend does not, so point `HOST` at this server for sequences).
