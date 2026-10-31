"""Backend configuration and process-local shared state.

Speeds are millimetres/second; coordinates are millimetres and internal
orientation angles are radians. State resets when the process restarts.
"""
import os

from models.robot import Coordinates

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000
MAX_SPEED = 50.0  # Configured command limit; tune for the physical arm.
STEPS_PER_MM = 5

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
TargetSpeed = 10.0  # Cartesian millimetres per second
# True between issuing a packet and receiving the firmware's completion POST.
arm_moving = False
