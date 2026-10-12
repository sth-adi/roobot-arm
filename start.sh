#!/bin/sh
# One tap: starts the input server and the Arm Hub, opens the robot viewer.
cd "$(dirname "$0")" && exec python3 launch.py "$@"
