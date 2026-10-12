# Arm firmware (ESP32)

The UWE-RArm firmware from <https://github.com/Samwally1/UWE-RArm> (folder "Robot Arm Firmware", commit 3b9b8aa), copied here and changed as below.
That repository has no licence file, so confirm with its authors that keeping a copy here is fine.

## What changed

| File | Change |
|---|---|
| `src/ArmMotion.h` | **New.** The movement state machine (sticks, go home, save, replay) pulled out of `main.cpp` so it can be tested on a PC. `sim/armsim.py` is the same logic. |
| `src/main.cpp` | Uses `ArmMotion`. Joint speed limit. Input older than 1.5 s counts as sticks centred, buttons released. Pose comes from the joint angles, so the arm no longer snaps back after "go home". Servos get pulse widths (`writeMicroseconds`) instead of whole degrees, so slow moves are smooth. The calibration (training) flow is unchanged. |
| `src/RobotCharacteristics.h` | `ToolLengthMm = 63` (was a hard-coded 53), `JointMaxDegPerSecond = 30`, `JointSpeedHardCap = 60`. |
| `src/InverseKinTransform.h` | Uses `ToolLengthMm` instead of 53. |
| `src/WifiFunctions.h`, `src/SticksToXYZ.h` | The packet may carry `"maxSpeed"` (deg/s); it is clamped to 1-60 and sets the limit. `StickInput` gained a `maxSpeed` field. |
| `src/SecretsExample.h` | Also shows `HOST`, which the code uses but the original example left out. |

## Before you flash

* **Tool length** is compiled in (63 mm with the ball). Change `ToolLengthMm` for a different tool; the simulation has it as a setting instead.
* **Speed limit**: 30 deg/s until the server sends `maxSpeed`; never above 60 deg/s. The dashboard slider sends it (`POST /control {"maxSpeed": n}` on the server).
* **HTTPS**: the firmware polls `https://HOST/controller` and posts `/status` (certificate not checked, `setInsecure()`). The roobot server speaks plain HTTP, so `HOST` must be a TLS endpoint in front of it (the Dockerfile deployment, a reverse proxy, a tunnel).
* Copy `src/SecretsExample.h` to `src/Secrets.h` and fill it in. Build with PlatformIO (`pio run -e upesy_wroom`).
* **Not compiled for the ESP32 here.** What was checked on a PC: `main.cpp` compiles against stub Arduino/WiFi/servo headers, and the C++ movement code matches the Python arm to 0.05 degrees over scripted runs (jog, save, replay, home, unreachable targets, input going stale) at three speed settings. Wiring, servo power and the ESP32 libraries are not covered: start with the arm unloaded and your hand on the power.

## Tests

```
python -m unittest discover -s sim -v        # includes test_firmware.py; needs g++
```

`test/stubs/` holds the stand-in headers; `test/motion_harness.cpp` is the program the Python test drives.
