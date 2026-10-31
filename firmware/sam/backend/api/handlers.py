import json
import math
import GlobalVariables as state
from http.server import BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse
from models.robot import Angles, Coordinates
from motion.kinematics import ForwardKim, InverseKim
from motion.packets import CompileAnglePacket


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):

        if self.path == "/mode":
            self.send_response(200)
            self.send_header("Mode", state.arm_mode)
            self.end_headers()

            self.wfile.write(b"true")

        elif self.path == "/controller":
            response = json.dumps(state.controller_state).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        elif self.path == "/movement":
            response = json.dumps(state.arm_moving).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        elif self.path == "/status":
            response = state.arm_status.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        elif self.path == "/calibration":
            if state.calibration_data is None:
                self.send_error(404, "Calibration not available")
                return

            response = json.dumps(state.calibration_data, separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        elif urlparse(self.path).path == "/UpdateMovement":
            if state.arm_moving:
                self.send_response(204)
                self.end_headers()
                return

            # Firmware reports logical joint degrees, before servo inversion.
            query = parse_qs(urlparse(self.path).query)
            try:
                degrees = [float(query[name][0]) for name in ("s1", "s2", "s3", "s4")]
                if not all(math.isfinite(value) and 0 <= value <= 180 for value in degrees):
                    raise ValueError("Invalid joint angle")
                current_cords = ForwardKim(Angles(*(math.radians(value) for value in degrees)))
            except (KeyError, IndexError, TypeError, ValueError):
                self.send_error(400, "s1, s2, s3 and s4 must be finite joint degrees in 0-180")
                return

            # Do not tell the arm to move when it has reached the target.
            if (
                math.dist((current_cords.X, current_cords.Y, current_cords.Z),
                          (state.TargetCords.X, state.TargetCords.Y, state.TargetCords.Z)) <= 0.1
                and abs(math.remainder(current_cords.A - state.TargetCords.A, 2 * math.pi)) <= math.radians(0.1)
            ):
                self.send_response(204)
                self.end_headers()
                return

            try:
                angle_packet = CompileAnglePacket(current_cords, state.TargetCords, state.TargetSpeed, state.STEPS_PER_MM)
            except (ValueError, ZeroDivisionError, OverflowError):
                self.send_error(422, "Unable to compile movement packet")
                return

            # One timestamped servo command per line.  Keeping this in the
            # response body avoids HTTP header-size limits for long moves.
            response = ("\n".join(angle_packet) + "\n").encode("utf-8")
            state.arm_moving = True
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        else:
            self.send_error(404)

    def do_POST(self):
        target_url = urlparse(self.path)
        if target_url.path == "/target":
            
            if state.arm_moving:
                self.send_error(409, "Arm is moving; retry after movement completes")
                return

            query = parse_qs(target_url.query, keep_blank_values=True)

            def target_value(name, default=None):
                # Headers take precedence for existing clients.
                if name in self.headers:
                    return self.headers[name]
                values = query.get(name)
                if values is None:
                    return default
                if len(values) != 1:
                    raise ValueError("Duplicate target parameter")
                return values[0]

            try:
                x, y, z, a = (float(target_value(name)) for name in ("X", "Y", "Z", "A"))
                if not all(math.isfinite(value) for value in (x, y, z, a)):
                    raise ValueError("Non-finite coordinate")
                speed = float(target_value("Speed", state.TargetSpeed))
                if not math.isfinite(speed) or not 0 < speed <= state.MAX_SPEED:
                    raise ValueError("Invalid speed")
                candidate = Coordinates(x, y, z, math.radians(a))
            except (KeyError, TypeError, ValueError, OverflowError):
                self.send_error(400, f"X, Y, Z (mm) and A (degrees) must be finite; Speed must be greater than 0 and at most {state.MAX_SPEED:g} mm/s")
                return
            try:
                if math.hypot(x, y) - 63.0 * math.cos(candidate.A) < 0:
                    raise ValueError("Wrist behind base")
                InverseKim(candidate)
            except (ValueError, ZeroDivisionError, OverflowError):
                self.send_error(422, "Target is unreachable")
                return
            state.TargetCords = candidate
            state.TargetSpeed = speed
            self.send_response(204)
            self.end_headers()
            return

        elif self.path == "/movement-complete":
            state.arm_moving = False
            print("Movement complete")
            self.send_response(204)
            self.end_headers()
            return

        elif self.path == "/controller":
            target = state.controller_state
        elif self.path == "/status":
            content_length = int(self.headers.get("Content-Length", 0))
            if content_length <= 0 or content_length > 4096:
                self.send_error(400, "Invalid arm status size")
                return

            state.arm_status = self.rfile.read(content_length).decode("utf-8")
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

            state.calibration_data = {
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
