# Sam's latest code, untouched (reference)

A verbatim copy of the `Robot Arm Firmware` and `Robot Arm Backend` folders of <https://github.com/Samwally1/UWE-RArm> at commit
`8e19b37fbc782f7cdc061e2037964d85225066be` (16 Oct 2026), folders renamed to `firmware/` and `backend/`. **Do not edit these files.**
Our changes live elsewhere (`dashboard/planner.py`, and later a firmware patch set). That repository has no licence file, so confirm with its authors that keeping a copy is fine.

How it works (read from the code): the ESP32 asks the backend's `GET /UpdateMovement?x=&y=&z=&a=` for a move. The backend solves the inverse kinematics (`AnglePacketCompilation.py`),
draws a straight line to the target and answers with lines `time_ms/j1/j2/j3/j4` (degrees). The firmware plays them back on one timer and moves the servos.
The target is a constant in `backend/main.py`; nothing sets it at run time.

The older copies (`firmware/stock`, `firmware/rarm`) are Sam's code from 12 Oct (3b9b8aa); the simulation tests are built around them.
