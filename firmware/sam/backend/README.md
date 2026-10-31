# Backend file map

| File | Responsibility |
| --- | --- |
| `GlobalVariables.py` | Shared state, speed limit, path resolution and server settings |
| `main.py` | HTTP server entry point on port 8000 |
| `api/handlers.py` | GET/POST handlers using shared state from GlobalVariables.py |
| `models/robot.py` | `Angles` and `Coordinates` data classes |
| `motion/kinematics.py` | `ForwardKim` and `InverseKim` calculations |
| `motion/packets.py` | Path interpolation and timestamped packet formatting |
| `Dockerfile` | Packages all backend Python modules |

Run `python main.py` from this directory, or use the existing Docker Compose configuration.
Function names, calculations, routes and response formats are unchanged by the regrouping.

## Movement API

- `POST /target`: supply `X`, `Y`, `Z` in millimetres and `A` in degrees as headers or query parameters (for example `/target?X=200&Y=200&Z=200&A=0`). Headers take precedence when both are supplied. Optional `Speed` (header or query parameter) sets Cartesian speed in mm/s (finite, greater than zero and no greater than `MAX_SPEED` in `GlobalVariables.py` (initially 50 mm/s)). If omitted, the previous speed is kept; startup default is 10 mm/s. Returns 204; missing/non-finite values return 400 and unreachable targets return 422. Targets can be replaced until a movement packet is issued. While the arm is moving, new targets return 409 without changing the target or speed; retry after completion.
- `GET /UpdateMovement?s1=...&s2=...&s3=...&s4=...`: logical joint degrees. A 200 response contains timestamped commands. Issuing a packet sets `arm_moving` to true. A 204 response means already at target or a packet is still awaiting completion.
- `POST /movement-complete`: clears `arm_moving` and returns 204. It may safely be repeated.
- `GET /movement`: JSON boolean (`true` while a packet is being played; otherwise `false`).

Example (PowerShell):

```powershell
Invoke-WebRequest -Method Post -Uri 'http://localhost:8000/target' -Headers @{X='0'; Y='250'; Z='250'; A='0'; Speed='20'}
```

The firmware acknowledges only successful packet playback and retries failed acknowledgements before requesting another packet. Completion means all servo commands were sent, without physical position feedback. State is held in memory and resets when the backend restarts. This protocol assumes one arm.

Run API tests from this directory: `python -m unittest discover -s tests -v`.

Edit `GlobalVariables.py` to configure `MAX_SPEED`, `STEPS_PER_MM`, server host/port and initial state. Speeds above the limit return HTTP 400 without changing the target or current speed.
