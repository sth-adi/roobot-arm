# Sam's latest code, untouched (reference)

A verbatim copy of the `Robot Arm Firmware` and `Robot Arm Backend` folders of <https://github.com/Samwally1/UWE-RArm> at commit
`9ad64eb` (29 Oct 2026), folders renamed to `firmware/` and `backend/`. His `Documentation` folder (large images and a .docx) is left out. **Do not edit these files.**
Our changes live elsewhere and are applied on top (see `firmware/ours/`). That repository has no licence file, so confirm with its authors that keeping a copy is fine.

How it works (read from the code): the ESP32 asks the backend's `GET /UpdateMovement?s1=&s2=&s3=&s4=` (its joint angles in degrees) for each move. The backend works out the tool position
(63 mm tool), draws a straight line to the target and answers with lines `time_ms/j1/j2/j3/j4` (degrees) and a `Movement-ID` header. The firmware plays them back on one timer and then
confirms with `POST /movement-complete`. The target is set with `POST /target` (headers `X`, `Y`, `Z` in mm, `A` in degrees, optional `Speed` up to 50 mm/s).
`GET /movement` reports the last move's id and status. His own tests: `cd backend && python -m unittest discover -s tests`.

The older copies (`firmware/stock`, `firmware/rarm`) are his code from 12 Oct (3b9b8aa); the simulation tests are built on them.
