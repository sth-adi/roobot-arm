# Firmware file map

Application headers are grouped under `src/` by responsibility. `main.cpp` remains at the root of `src/`.

| File | Responsibility |
| --- | --- |
| `main.cpp` | Arduino setup and movement request/playback loop |
| `core/StartupProcess.h` | Startup, training workflow and homing |
| `core/RobotTypes.h` | Shared controller input, position and joint angle structures |
| `config/RobotCharacteristics.h` | Pins, calibration values and controller sensitivity |
| `network/WifiConnection.h` | Wi-Fi connection and latency checks |
| `network/BackendConnection.h` | Shared HTTP dependencies and backend configuration include |
| `control/ControllerProtocol.h` | Controller response parsing |
| `network/ControllerClient.h` | Controller input requests |
| `network/StatusClient.h` | Arm status reporting |
| `network/MovementClient.h` | Movement packet requests |
| `calibration/CalibrationFuncs.h` | Calibration parsing, loading, saving and status formatting |
| `control/SticksToXYZ.h` | Stick deadzone and Cartesian input conversion |
| `motion/RobotMovement.h` | Servo state, home angles, inversion and packet playback |

`config/Secrets.h` supplies local credentials; use `config/SecretsExample.h` as the template.
The regrouping preserves function names and implementations.
