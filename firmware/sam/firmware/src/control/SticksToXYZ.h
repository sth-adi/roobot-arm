#pragma once

#include <Arduino.h>

#include "../config/RobotCharacteristics.h"
#include "../core/RobotTypes.h"

inline float ApplyStickDeadzone(float value) {
    return abs(value) < 0.2f ? 0.0f : value;
}

inline RobotPosition SticksToXYZ(
    const StickInput& input,
    float elapsedMilliseconds,
    RobotPosition position
) {
    const float elapsedSeconds = elapsedMilliseconds / 1000.0f;

    position.x += ApplyStickDeadzone(input.leftX) * ChordSense * elapsedSeconds;
    position.y += ApplyStickDeadzone(input.leftY) * ChordSense * elapsedSeconds;
    position.z += ApplyStickDeadzone(input.rightY) * ChordSense * elapsedSeconds;
    // Tool orientation is a geometric angle and can be negative.  Physical
    // limits are checked later by inverse kinematics against joint calibration.
    position.angle += ApplyStickDeadzone(input.rightX)
        * AngleSense * elapsedSeconds;

    return position;
}
