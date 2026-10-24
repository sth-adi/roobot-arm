import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse
# main.py is run directly by the Docker entrypoint, so this must be an
# absolute (same-directory) import rather than a package-relative import.
from AnglePacketCompilation import Angles, Coordinates, CompileAnglePacket


controller_state = {}
arm_status = ""
# Safe startup calibration.  A later successful POST to /calibration replaces
# these values for the lifetime of the backend process.
#
# Joint 3's reversed limits intentionally preserve its inverted-servo setup.
calibration_data = {
    "jointLimits": [[15, 180], [0, 131], [154, 21], [0, 180]],
    "jointHomes": [92, 128, 28, 112],
}
arm_mode = os.getenv("ARM_MODE", "Normal")

TargetCords = Coordinates(0, 250, 250 , 0)


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):

        if self.path == "/mode":
            self.send_response(200)
            self.send_header("Mode", arm_mode)
            self.end_headers()

            self.wfile.write(b"true")

        elif self.path == "/controller":
            response = json.dumps(controller_state).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        elif self.path == "/status":
            response = arm_status.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        elif self.path == "/calibration":
            if calibration_data is None:
                self.send_error(404, "Calibration not available")
                return

            response = json.dumps(calibration_data, separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)


        elif urlparse(self.path).path == "/UpdateMovement":
            # The ESP32 supplies its measured Cartesian position as query
            # parameters.  GET requests do not have a reliable request body.
            query = parse_qs(urlparse(self.path).query)
            try:
                current_cords = Coordinates(
                    float(query["x"][0]),
                    float(query["y"][0]),
                    float(query["z"][0]),
                    float(query["a"][0]),
                )
            except (KeyError, IndexError, TypeError, ValueError):
                self.send_error(400, "x, y, z and a query parameters are required")
                return

            # Do not tell the arm to move when it has reached the target.
            if (
                current_cords.X == TargetCords.X
                and current_cords.Y == TargetCords.Y
                and current_cords.Z == TargetCords.Z
                and current_cords.A == TargetCords.A
            ):
                self.send_response(204)
                self.end_headers()
                return

            try:
                angle_packet = CompileAnglePacket(current_cords, TargetCords, 10, 5)#
                print(angle_packet)
            except (ValueError, ZeroDivisionError, OverflowError):
                self.send_error(422, "Unable to compile movement packet")
                return

            # One timestamped servo command per line.  Keeping this in the
            # response body avoids HTTP header-size limits for long moves.
            response = ("\n".join(angle_packet) + "\n").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/controller":
            target = controller_state
        elif self.path == "/status":
            content_length = int(self.headers.get("Content-Length", 0))
            if content_length <= 0 or content_length > 4096:
                self.send_error(400, "Invalid arm status size")
                return

            global arm_status
            arm_status = self.rfile.read(content_length).decode("utf-8")
            self.send_response(204)
            self.end_headers()
            return
        elif self.path == "/calibration":
            content_length = int(self.headers.get("Content-Length", 0))
            if content_length <= 0 or content_length > 4096:
                self.send_error(400, "Invalid calibration size")
                return

            try:
                payload = json.loads(self.rfile.read(content_length))
                joint_limits = payload["jointLimits"]
                joint_homes = payload["jointHomes"]
            except (json.JSONDecodeError, UnicodeDecodeError, KeyError, TypeError):
                self.send_error(400, "Calibration must contain jointLimits and jointHomes")
                return

            if (
                not isinstance(joint_limits, list)
                or len(joint_limits) != 4
                or any(
                    not isinstance(limits, list)
                    or len(limits) != 2
                    or not all(isinstance(value, int) and 0 <= value <= 180 for value in limits)
                    for limits in joint_limits
                )
                or not isinstance(joint_homes, list)
                or len(joint_homes) != 4
                or not all(isinstance(value, int) and 0 <= value <= 180 for value in joint_homes)
            ):
                self.send_error(400, "Calibration values must be four 0-180 limits and home positions")
                return

            global calibration_data
            calibration_data = {
                "jointLimits": joint_limits,
                "jointHomes": joint_homes,
            }
            self.send_response(204)
            self.end_headers()
            return
        else:
            self.send_error(404)
            return

        content_length = int(self.headers.get("Content-Length", 0))
        if content_length <= 0 or content_length > 4096:
            self.send_error(400, "Invalid controller state size")
            return

        try:
            payload = json.loads(self.rfile.read(content_length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.send_error(400, "Controller state must be JSON")
            return

        if not isinstance(payload, dict):
            self.send_error(400, "Controller state must be an object")
            return

        target.update(payload)
        self.send_response(204)
        self.end_headers()


server = HTTPServer(("0.0.0.0", 8000), Handler)

server.serve_forever()
