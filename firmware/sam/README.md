# Sam's latest code, untouched (reference)

A verbatim copy of the `Robot Arm Firmware` and `Robot Arm Backend` folders of <https://github.com/Samwally1/UWE-RArm> at commit
`64216fd` (29 Oct 2026, evening), folders renamed to `firmware/` and `backend/`. His `Documentation` folder (large images and a .docx) is left out. **Do not edit these files.**
Our changes live elsewhere and are applied on top (see `firmware/ours/`). That repository has no licence file, so confirm with its authors that keeping a copy is fine.

How it works (read from the code): the ESP32 asks the backend's `GET /UpdateMovement?s1=&s2=&s3=&s4=` (its joint angles in degrees) for each move. The backend works out the tool position
(63 mm tool), draws a straight line to the target and answers with a packet: the step count, the interval in milliseconds, then `steps + 1` lines `time_ms/j1/j2/j3/j4` (degrees; the firmware
rejects any angle outside 0-180). While a packet is being played the backend is "moving" (`arm_moving`): `/UpdateMovement` answers 204 and `POST /target` answers 409. The firmware ends the move with
`POST /movement-complete`. The target is set with `POST /target` (headers or query parameters `X`, `Y`, `Z` in mm, `A` in degrees, optional `Speed` up to 50 mm/s). `GET /movement` answers `true` or `false`.
(Before this commit there were movement ids and a status; he removed them.) His own tests: `cd backend && python -m unittest discover -s tests`. His native firmware tests were deleted in this commit.

The older copies (`firmware/stock`, `firmware/rarm`) are his code from 12 Oct (3b9b8aa); the simulation tests are built on them.
