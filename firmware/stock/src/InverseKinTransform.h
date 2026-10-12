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

inline bool IsFiniteAngleSet(const RobotAngles& angles)
{
    return isfinite(angles.s1) && isfinite(angles.s2)
        && isfinite(angles.s3) && isfinite(angles.s4);
}

inline bool AreAnglesWithinCalibrationLimits(const RobotAngles& angles)
{
    const float jointAngles[4] = {angles.s1, angles.s2, angles.s3, angles.s4};
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        const float servoDegrees = degrees(jointAngles[jointIndex]);
        const int minimum = min(JointLims[jointIndex][0], JointLims[jointIndex][1]);
        const int maximum = max(JointLims[jointIndex][0], JointLims[jointIndex][1]);
        if (servoDegrees < minimum || servoDegrees > maximum) {
            return false;
        }
    }
    return true;
}

inline float WrappedAngleDifference(float first, float second)
{
    return atan2f(sinf(first - second), cosf(first - second));
}

inline float AngleDistanceSquared(const RobotAngles& candidate,
                                  const RobotAngles& previous)
{
    const float d1 = WrappedAngleDifference(candidate.s1, previous.s1);
    const float d2 = WrappedAngleDifference(candidate.s2, previous.s2);
    const float d3 = WrappedAngleDifference(candidate.s3, previous.s3);
    const float d4 = WrappedAngleDifference(candidate.s4, previous.s4);
    return d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4;
}

inline bool CalculateElbowBranch(const RobotPosition& target, float theta,
                                 int elbowBranch, RobotAngles& angles)
{
    const float reach = sqrtf(target.x * target.x + target.y * target.y);
    const float wristR = reach - 53.0f * cosf(theta);
    const float wristZ = target.z - 53.0f * sinf(theta);
    const float shoulderWristDistance = sqrtf(
        wristR * wristR + (wristZ - 74.0f) * (wristZ - 74.0f)
    );
    if (!isfinite(shoulderWristDistance) || shoulderWristDistance < 0.0001f) {
        return false;
    }

    const float midpointDistance = 0.5f
        * (shoulderWristDistance * shoulderWristDistance - 12444.2875f)
        / shoulderWristDistance;
    const float bisectorSquared = 21301.4025f
        - midpointDistance * midpointDistance;
    if (!isfinite(midpointDistance) || bisectorSquared < 0.0f) {
        return false;
    }

    const float bisectorLength = sqrtf(bisectorSquared);
    const float midAxisAngle = atan2f(wristZ - 74.0f, wristR);
    angles.s1 = atan2f(target.x, target.y) + HALF_PI;
    angles.s2 = midAxisAngle + atan2f(bisectorLength * elbowBranch,
                                      midpointDistance);

    const float elbowR = 145.95f * cosf(angles.s2);
    const float elbowZ = 145.95f * sinf(angles.s2) + 74.0f;
    const float forearmAngle = atan2f(wristZ - elbowZ, wristR - elbowR);
    angles.s3 = forearmAngle - angles.s2 + PI;
    angles.s4 = -angles.s2 - angles.s3 + theta + (3.0f * HALF_PI);
    return IsFiniteAngleSet(angles) && AreAnglesWithinCalibrationLimits(angles);
}

// Tests both elbow configurations.  On failure outputAngles is unchanged.
inline bool InverseKinTransform(const RobotPosition& target,
                                const RobotAngles& previousAngles,
                                RobotAngles& outputAngles)
{
    const float theta = radians(target.angle);
    RobotAngles positiveElbow;
    RobotAngles negativeElbow;
    const bool positiveValid = CalculateElbowBranch(target, theta, 1,
                                                    positiveElbow);
    const bool negativeValid = CalculateElbowBranch(target, theta, -1,
                                                    negativeElbow);
    if (!positiveValid && !negativeValid) {
        return false;
    }

    if (!negativeValid || (positiveValid
            && AngleDistanceSquared(positiveElbow, previousAngles)
                <= AngleDistanceSquared(negativeElbow, previousAngles))) {
        outputAngles = positiveElbow;
    } else {
        outputAngles = negativeElbow;
    }
    return true;
}
