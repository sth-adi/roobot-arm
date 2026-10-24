#pragma once

#include <Arduino.h>
#include <math.h>

#include "SticksToXYZ.h"

// Joint angles are radians; RobotPosition::angle stays in controller degrees.
struct RobotAngles {
    float s1;
    float s2;
    float s3;
    float s4;
};

inline RobotPosition ForwardKinTransform(const RobotAngles& angles)
{
    const float baseAngle = angles.s1 - HALF_PI;
    const float forearmAngle = angles.s2 + angles.s3 - PI;
    const float toolAngle = angles.s2 + angles.s3 + angles.s4
        - (3.0f * HALF_PI);
    const float elbowR = 145.95f * cosf(angles.s2);
    const float elbowZ = 145.95f * sinf(angles.s2) + 74.0f;
    const float wristR = elbowR + 183.70f * cosf(forearmAngle);
    const float wristZ = elbowZ + 183.70f * sinf(forearmAngle);
    const float toolR = wristR + 53.0f * cosf(toolAngle);

    RobotPosition position;
    position.x = toolR * sinf(baseAngle);
    position.y = toolR * cosf(baseAngle);
    position.z = wristZ + 53.0f * sinf(toolAngle);
    position.angle = degrees(toolAngle);
    return position;
}
