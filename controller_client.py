#!/usr/bin/env python3
"""Read an Xbox One or PS5 (DualSense) controller and POST its state to the robot arm server.

Run:  python controller_client.py --url http://localhost:8000/input
      python controller_client.py --profile ps5   (default: picked from the controller's name)
      python controller_client.py --simulate      (no controller needed, for testing)

Axis/button indices differ between OS and drivers. If sticks or buttons look
wrong, run with --debug to print raw values and adjust the matching profile below.
On a PS5 pad the face buttons map by position: Cross = A, Circle = B (go home), Square = X, Triangle = Y.
"""
import argparse
import json
import math
import os
import time
import urllib.error
import urllib.request

# --- Mappings (pygame 2 / SDL2) ---------------------------------------------
# Only the four stick axes and A/B/X/Y go to the arm; the rest is for drawing and the dashboard.
PROFILES = {
    "xbox": {   # Xbox One pad
        "axes": {"lx": 0, "ly": 1, "rx": 2, "ry": 3, "lt": 4, "rt": 5},
        "buttons": {"a": 0, "b": 1, "x": 2, "y": 3, "lb": 4, "rb": 5, "back": 6, "start": 7, "ls": 8, "rs": 9},
        "dpad_buttons": None,
    },
    "ps5": {    # DualSense as SDL2 reports it: Cross, Circle, Square, Triangle, Share, PS, Options, L3, R3, L1, R1
        "axes": {"lx": 0, "ly": 1, "rx": 2, "ry": 3, "lt": 4, "rt": 5},
        "buttons": {"a": 0, "b": 1, "x": 2, "y": 3, "back": 4, "start": 6, "ls": 7, "rs": 8, "lb": 9, "rb": 10},
        "dpad_buttons": {"up": 11, "down": 12, "left": 13, "right": 14},   # used only when SDL reports no hat
    },
}
PS_NAMES = ("dualsense", "ps5", "wireless controller", "playstation")
PROFILE = PROFILES["xbox"]
BUTTON_NAMES = tuple(PROFILES["xbox"]["buttons"])
DEADZONE = 0.12
SEND_HZ = 30
HEARTBEAT_S = 0.5  # resend unchanged state at least this often


def profile_name_for(device_name):
    """Pick a profile from the controller's name. Xbox is checked first because the Series pad is called
    "Xbox Wireless Controller", which also contains the PlayStation-style "wireless controller"."""
    n = (device_name or "").lower()
    if "xbox" in n or "x-box" in n:
        return "xbox"
    return "ps5" if any(k in n for k in PS_NAMES) else "xbox"


def select_profile(choice, device_name=""):
    """Activate a profile: "auto" (from the device name), "xbox" or "ps5". Returns the name used."""
    global PROFILE
    name = profile_name_for(device_name) if choice in (None, "", "auto") else choice
    if name not in PROFILES:
        raise ValueError(f"unknown profile {name!r}; choose from auto, {', '.join(PROFILES)}")
    PROFILE = PROFILES[name]
    return name


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
    axes, buttons = PROFILE["axes"], PROFILE["buttons"]

    def axis(name):
        i = axes[name]
        return js.get_axis(i) if i < n_axes else 0.0

    def down(i):
        return bool(js.get_button(i)) if i < n_btn else False

    if js.get_numhats():
        dpad = list(js.get_hat(0))
    elif PROFILE["dpad_buttons"]:
        d = PROFILE["dpad_buttons"]
        dpad = [int(down(d["right"])) - int(down(d["left"])), int(down(d["up"])) - int(down(d["down"]))]
    else:
        dpad = [0, 0]

    state = {
        "lx": round(deadzone(axis("lx")), 3),
        "ly": round(deadzone(axis("ly")), 3),
        "rx": round(deadzone(axis("rx")), 3),
        "ry": round(deadzone(axis("ry")), 3),
        "lt": trigger(axis("lt")),
        "rt": trigger(axis("rt")),
        "buttons": {k: down(i) for k, i in buttons.items()},
        "dpad": dpad,
    }
    return state


def simulated_state(t):
    return {
        "lx": round(math.sin(t), 3), "ly": round(math.cos(t), 3), "rx": 0.0, "ry": 0.0,
        "lt": 0.0, "rt": round((math.sin(t / 2) + 1) / 2, 3),
        "buttons": {k: False for k in BUTTON_NAMES}, "dpad": [0, 0],
    }


def to_wire(state):
    """The packet the server and the arm firmware use (UWE-RArm format): sticks and A/B/X/Y only.
    The richer state above (triggers, bumpers, d-pad) stays local for drawing and the dashboard."""
    b = state["buttons"]
    return {"LSX": state["lx"], "LSY": state["ly"], "RSX": state["rx"], "RSY": state["ry"],
            "A": b["a"], "B": b["b"], "X": b["x"], "Y": b["y"]}


def post(url, state):
    req = urllib.request.Request(
        url, data=json.dumps(to_wire(state)).encode(), headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=1.0) as r:
        r.read()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="http://localhost:8000/input")
    p.add_argument("--index", type=int, default=0, help="controller index if several are plugged in")
    p.add_argument("--profile", choices=["auto", *PROFILES], default="auto",
                   help="button/axis layout: auto picks from the controller's name (Xbox or PS5)")
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
        print(f"Using controller: {js.get_name()} (profile {select_profile(args.profile, js.get_name())})")

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
