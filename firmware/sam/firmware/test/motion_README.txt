Native motion checks (Windows PowerShell, run from Robot Arm Firmware):

    New-Item -ItemType Directory -Force .pio/native-tests | Out-Null
    g++ -std=c++11 -Wall -Wextra -Itest/stubs -Isrc test/motion_planner_test.cpp -o .pio/native-tests/motion_planner_test.exe
    ./.pio/native-tests/motion_planner_test.exe
    g++ -std=c++11 -Wall -Wextra -Itest/stubs -Isrc test/motion_playback_test.cpp -o .pio/native-tests/motion_playback_test.exe
    ./.pio/native-tests/motion_playback_test.exe

The planner tests cover the full five-position cycle and initial transition,
Cartesian path accuracy, calibration/inversion, joint motion bounds, orientation,
near-axis refinement, cache limits, and rejection of invalid or unreachable paths.
The playback tests use a simulated clock to check spacing, late-frame handling,
and timer rollover. No hardware is moved by these tests.

Waypoints and the Cartesian speed ceiling are in src/main.cpp. Joint speed and
acceleration ceilings and cache limits are in src/MotionPlanner.h. The sequence
is fully planned after startup/calibration and reused on each cycle. If any path
cannot be planned, playback stays stopped and Serial reports the reason.

Native tests cannot measure servo power, backlash, or physical tracking error.
