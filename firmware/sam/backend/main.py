from http.server import HTTPServer
import threading
import time

from api.handlers import Handler
import GlobalVariables as state


def print_movement_state_forever():
    """Log the active movement target once per second for diagnostics."""
    while True:
        print(
            f"TargetCords = {state.TargetCords}\n"
            f"TargetSpeed = {state.TargetSpeed}\n"
            f"arm_moving = {state.arm_moving}",
            flush=True,
        )
        time.sleep(1)


if __name__ == "__main__":
    threading.Thread(target=print_movement_state_forever, daemon=True).start()
    server = HTTPServer((state.SERVER_HOST, state.SERVER_PORT), Handler)
    server.serve_forever()
