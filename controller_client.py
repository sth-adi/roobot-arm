#!/usr/bin/env python3
"""Read a generic Xbox One controller and POST its state to the robot arm server.

Run:  python controller_client.py --url http://localhost:8000/input
      python controller_client.py --simulate     (no controller needed, for testing)

Axis/button indices differ between OS and drivers. If sticks or triggers look
wrong, run with --debug to print raw values and adjust the maps below.
"""
import argparse
import json
import math
import os
import time
import urllib.error
import urllib.request

# --- Mapping (pygame 2 / SDL2 defaults for Xbox One pads) -------------------
AXES = {"lx": 0, "ly": 1, "rx": 2, "ry": 3, "lt": 4, "rt": 5}
BUTTONS = {
    "a": 0, "b": 1, "x": 2, "y": 3,
    "lb": 4, "rb": 5, "back": 6, "start": 7, "ls": 8, "rs": 9,
}
DEADZONE = 0.12
SEND_HZ = 30
HEARTBEAT_S = 0.5  # resend unchanged state at least this often


def deadzone(v, dz=DEADZONE):
    """Apply a radial-free deadzone and rescale so output still reaches +-1."""
    if abs(v) < dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1 - dz), v)


def trigger(v):
    """Triggers rest at -1 and go to +1 on some drivers, 0..1 on others."""
    return round(max(0.0, min(1.0, (v + 1) / 2 if v < -0.001 else v)), 3)


def read_state(js, pygame):
    pygame.event.pump()
    n_axes, n_btn = js.get_numaxes(), js.get_numbuttons()

    def axis(name):
        i = AXES[name]
        return js.get_axis(i) if i < n_axes else 0.0

    state = {
        "lx": round(deadzone(axis("lx")), 3),
        "ly": round(deadzone(axis("ly")), 3),
        "rx": round(deadzone(axis("rx")), 3),
        "ry": round(deadzone(axis("ry")), 3),
        "lt": trigger(axis("lt")),
        "rt": trigger(axis("rt")),
        "buttons": {k: bool(js.get_button(i)) if i < n_btn else False for k, i in BUTTONS.items()},
        "dpad": list(js.get_hat(0)) if js.get_numhats() else [0, 0],
    }
    return state


def simulated_state(t):
    return {
        "lx": round(math.sin(t), 3), "ly": round(math.cos(t), 3), "rx": 0.0, "ry": 0.0,
        "lt": 0.0, "rt": round((math.sin(t / 2) + 1) / 2, 3),
        "buttons": {k: False for k in BUTTONS}, "dpad": [0, 0],
    }


def post(url, state):
    req = urllib.request.Request(
        url, data=json.dumps(state).encode(), headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=1.0) as r:
        r.read()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="http://localhost:8000/input")
    p.add_argument("--index", type=int, default=0, help="controller index if several are plugged in")
    p.add_argument("--simulate", action="store_true", help="send fake data, no controller or pygame needed")
    p.add_argument("--debug", action="store_true", help="print raw axis/button values")
    args = p.parse_args()

    js = pygame = None
    if not args.simulate:
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        import pygame  # noqa: E402  (pip install pygame)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= args.index:
            raise SystemExit("No controller found. Plug one in, or use --simulate.")
        js = pygame.joystick.Joystick(args.index)
        js.init()
        print(f"Using controller: {js.get_name()}")

    print(f"Sending to {args.url} at up to {SEND_HZ} Hz (Ctrl+C to stop)")
    last, last_sent, warned = None, 0.0, False
    t0 = time.time()
    try:
        while True:
            now = time.time()
            state = simulated_state(now - t0) if args.simulate else read_state(js, pygame)
            if args.debug and js:
                print([round(js.get_axis(i), 2) for i in range(js.get_numaxes())],
                      [js.get_button(i) for i in range(js.get_numbuttons())])
            if state != last or now - last_sent >= HEARTBEAT_S:
                try:
                    post(args.url, state)
                    last, last_sent, warned = state, now, False
                except (urllib.error.URLError, OSError) as e:
                    if not warned:
                        print(f"Server unreachable ({e}); retrying...")
                        warned = True
                    last_sent = now
            time.sleep(1 / SEND_HZ)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
