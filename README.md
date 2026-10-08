# roobot-arm

Xbox controller -> HTTP server, as the input path for the robot arm.

- `server.py` – HTTP server, standard library only. `POST /input`, `GET /state`, `GET /health`.
- `controller_client.py` – reads a generic Xbox One controller (pygame) and POSTs its state as JSON.

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
