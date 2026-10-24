#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "MotionPlanner.h"

namespace {
int failures = 0;

void Check(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

float Joint(const RobotAngles& angles, int index)
{
    const float values[] = {angles.s1, angles.s2, angles.s3, angles.s4};
    return values[index];
}

void SetLimits(bool inverted = false)
{
    for (int joint = 0; joint < 4; ++joint) {
        JointLims[joint][0] = inverted ? 180 : 0;
        JointLims[joint][1] = inverted ? 0 : 180;
    }
}

uint16_t ExpectedPulse(const RobotAngles& angles, int joint)
{
    float value = degrees(Joint(angles, joint));
    if (value < 0.0f) value = 0.0f;
    if (value > 180.0f) value = 180.0f;
    if (JointLims[joint][0] > JointLims[joint][1]) value = 180.0f - value;
    return static_cast<uint16_t>(std::lround(500.0f + value * 1900.0f / 180.0f));
}

RobotAngles Decode(const ServoFrame& frame)
{
    float angles[4];
    for (int joint = 0; joint < 4; ++joint) {
        float value = (static_cast<float>(frame.pulseUs[joint]) - 500.0f) * 180.0f / 1900.0f;
        if (JointLims[joint][0] > JointLims[joint][1]) value = 180.0f - value;
        angles[joint] = radians(value);
    }
    return {angles[0], angles[1], angles[2], angles[3]};
}

bool Solve(const RobotPosition& position, const RobotAngles& previous, RobotAngles& output)
{
    const bool result = InverseKinTransform(position, previous, output);
    Check(result, "test fixture must be reachable");
    return result;
}

void CheckTrajectory(const PlannedMovement& movement, const RobotPosition& start,
                     const RobotPosition& end)
{
    Check(movement.frames != nullptr && movement.frameCount >= 2, "nonzero move has buffered frames");
    if (!movement.frames || movement.frameCount < 2) return;

    const size_t last = movement.frameCount - 1;
    for (int joint = 0; joint < 4; ++joint) {
        Check(movement.frames[0].pulseUs[joint] == ExpectedPulse(movement.startAngles, joint),
              "first frame matches starting angles");
        Check(movement.frames[last].pulseUs[joint] == ExpectedPulse(movement.endAngles, joint),
              "last frame matches final angles");
    }

    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float dz = end.z - start.z;
    const float distanceSquared = dx * dx + dy * dy + dz * dz;
    float previousProgress = -1.0f;
    const float dt = MotionControlIntervalMs / 1000.0f;
    const float pulseDegrees = 180.0f / 1900.0f;
    // Integer microseconds contribute at most one pulse to a velocity difference,
    // and two pulses to a second difference. A 100ms window gives a useful
    // acceleration check without mistaking pulse quantization for vibration.
    const size_t accelerationStride = 5;
    const float accelerationDt = dt * accelerationStride;
    const float accelerationAllowance = 2.0f * pulseDegrees / (accelerationDt * accelerationDt);

    for (size_t index = 0; index <= last; ++index) {
        const RobotAngles current = Decode(movement.frames[index]);
        const RobotPosition pose = ForwardKinTransform(current);
        float progress = static_cast<float>(index) / last;
        if (distanceSquared > 0.001f) {
            progress = ((pose.x - start.x) * dx + (pose.y - start.y) * dy
                        + (pose.z - start.z) * dz) / distanceSquared;
            const float errorX = pose.x - (start.x + progress * dx);
            const float errorY = pose.y - (start.y + progress * dy);
            const float errorZ = pose.z - (start.z + progress * dz);
            Check(std::sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ) < 1.25f,
                  "quantized trajectory remains on its Cartesian line");
            Check(progress >= previousProgress - 0.01f, "Cartesian progress is monotonic");
            previousProgress = progress;
        }
        Check(std::fabs(pose.angle - (start.angle + progress * (end.angle - start.angle))) < 0.3f,
              "tool orientation follows the Cartesian interpolation");
        for (int joint = 0; joint < 4; ++joint) {
            Check(movement.frames[index].pulseUs[joint] >= 500
                  && movement.frames[index].pulseUs[joint] <= 2400, "servo pulses stay in range");
            if (index != 0) {
                const float previous = degrees(Joint(Decode(movement.frames[index - 1]), joint));
                const float speed = std::fabs(degrees(Joint(current, joint)) - previous) / dt;
                Check(speed <= MaxJointSpeedDegreesPerSecond + pulseDegrees / dt + 0.1f,
                      "joint velocity respects damper within one-pulse quantization");
            }
            const size_t before = index > accelerationStride ? index - accelerationStride : 0;
            const size_t after = min(last, index + accelerationStride);
            const float a = degrees(Joint(Decode(movement.frames[before]), joint));
            const float b = degrees(Joint(current, joint));
            const float c = degrees(Joint(Decode(movement.frames[after]), joint));
            const float acceleration = std::fabs(c - 2.0f * b + a)
                / (accelerationDt * accelerationDt);
            Check(acceleration <= MaxJointAccelerationDegreesPerSecondSquared
                  + accelerationAllowance + 0.5f, "joint acceleration respects configured bound");
        }
    }

    const RobotPosition finish = ForwardKinTransform(Decode(movement.frames[last]));
    Check(std::fabs(finish.x - end.x) < 1.0f && std::fabs(finish.y - end.y) < 1.0f
          && std::fabs(finish.z - end.z) < 1.0f, "final pulses reach requested position");
}

void TestRouteAndReuse()
{
    SetLimits();
    const RobotPosition route[] = {
        {-100.0f, 200.0f, 100.0f, 0.0f},
        {0.0f, 200.0f, 100.0f, 0.0f},
        {0.0f, 200.0f, 300.0f, 0.0f},
        {0.0f, 200.0f, 100.0f, 0.0f},
        {100.0f, 200.0f, 100.0f, 0.0f},
        {-100.0f, 200.0f, 100.0f, 0.0f}
    };
    RobotAngles angles = {HALF_PI, HALF_PI, HALF_PI, HALF_PI};
    RobotPosition start = ForwardKinTransform(angles);
    PlannedMovement movement;
    for (const RobotPosition& end : route) {
        Check(PlanLinearMovement(end, start, angles, 100.0f, movement) == MotionPlanError::None,
              "initial move and every cyclic route leg plan successfully");
        CheckTrajectory(movement, start, end);
        angles = movement.endAngles;
        start = end;
    }
    movement.Clear();
    Check(movement.frames == nullptr && movement.frameCount == 0, "Clear releases buffered trajectory");
}

void TestInvertedAndStationary()
{
    SetLimits(true);
    const RobotPosition start = {-100.0f, 200.0f, 100.0f, 0.0f};
    const RobotPosition end = {0.0f, 200.0f, 100.0f, 0.0f};
    RobotAngles angles;
    if (!Solve(start, {HALF_PI, HALF_PI, HALF_PI, HALF_PI}, angles)) return;
    PlannedMovement movement;
    Check(PlanLinearMovement(end, start, angles, 100.0f, movement) == MotionPlanError::None,
          "inverted servo calibration supports normal planning");
    CheckTrajectory(movement, start, end);
    Check(PlanLinearMovement(start, start, angles, 100.0f, movement) == MotionPlanError::None,
          "zero-distance move is valid");
    Check(movement.frames != nullptr && movement.frameCount > 0, "stationary move has a hold frame");
    SetLimits();
}

void TestOrientationOnly()
{
    SetLimits();
    const RobotPosition start = {-100.0f, 200.0f, 100.0f, 0.0f};
    const RobotPosition end = {-100.0f, 200.0f, 100.0f, 10.0f};
    RobotAngles angles;
    if (!Solve(start, {HALF_PI, HALF_PI, HALF_PI, HALF_PI}, angles)) return;
    PlannedMovement movement;
    Check(PlanLinearMovement(end, start, angles, 100.0f, movement) == MotionPlanError::None,
          "orientation-only move plans successfully");
    if (!movement.frames || movement.frameCount < 2) return;
    float previousAngle = -1.0f;
    for (size_t index = 0; index < movement.frameCount; ++index) {
        const RobotPosition pose = ForwardKinTransform(Decode(movement.frames[index]));
        Check(std::fabs(pose.x - start.x) < 1.0f && std::fabs(pose.y - start.y) < 1.0f
              && std::fabs(pose.z - start.z) < 1.0f, "orientation-only move holds tool position");
        Check(pose.angle >= previousAngle - 0.2f, "orientation-only move advances without reversals");
        previousAngle = pose.angle;
    }
    Check(std::fabs(previousAngle - end.angle) < 0.2f, "orientation-only move reaches requested angle");
}

void TestNearAxisRefinement()
{
    SetLimits();
    const RobotPosition start = {-1.0f, 0.5f, 300.0f, 90.0f};
    const RobotPosition end = {1.0f, 0.5f, 300.0f, 90.0f};
    RobotAngles angles;
    if (!Solve(start, {HALF_PI, HALF_PI, HALF_PI, HALF_PI}, angles)) return;
    PlannedMovement movement;
    const MotionPlanError error = PlanLinearMovement(end, start, angles, 100.0f, movement);
    Check(error == MotionPlanError::None,
          "continuous near-axis motion is refined rather than rejected as a branch flip");
    if (error != MotionPlanError::None) {
        std::fprintf(stderr, "Near-axis planning error: %s\n", MotionPlanErrorText(error));
        return;
    }
    Check(movement.frameCount > 26 && movement.frameCount <= MaxFramesPerMovement,
          "near-axis motion is retimed within the configured frame budget");
    const float dt = MotionControlIntervalMs / 1000.0f;
    const float quantizationAllowance = (180.0f / 1900.0f) / dt;
    for (size_t index = 1; index < movement.frameCount; ++index) {
        for (int joint = 0; joint < 4; ++joint) {
            const float change = std::fabs(degrees(Joint(Decode(movement.frames[index]), joint)
                                         - Joint(Decode(movement.frames[index - 1]), joint)));
            Check(change / dt <= MaxJointSpeedDegreesPerSecond + quantizationAllowance + 0.1f,
                  "refined near-axis motion respects the joint speed damper");
        }
        Check(movement.frames[index].pulseUs[0] >= movement.frames[index - 1].pulseUs[0],
              "near-axis base rotation remains continuous and monotonic");
    }
    const size_t last = movement.frameCount - 1;
    for (int joint = 0; joint < 4; ++joint) {
        Check(movement.frames[0].pulseUs[joint] == ExpectedPulse(angles, joint),
              "near-axis move preserves its starting pulse");
        Check(movement.frames[last].pulseUs[joint] == ExpectedPulse(movement.endAngles, joint),
              "near-axis move reaches its final pulse");
    }
}

void TestFractionalCalibrationLimits()
{
    SetLimits();
    JointLims[0][1] = 90;
    RobotAngles angles = {radians(90.00001f), HALF_PI, HALF_PI, HALF_PI};
    Check(AreAnglesWithinCalibrationLimits(angles),
          "numerical roundoff at a calibrated boundary remains accepted");
    angles.s1 = radians(90.1f);
    Check(!AreAnglesWithinCalibrationLimits(angles),
          "fractional angle above the calibrated maximum is rejected");
    const RobotPosition start = ForwardKinTransform(angles);
    PlannedMovement movement;
    Check(PlanLinearMovement(start, start, angles, 100.0f, movement) == MotionPlanError::InvalidInput,
          "planner rejects an out-of-limit fractional starting angle");
    Check(movement.frames == nullptr && movement.frameCount == 0,
          "out-of-limit start exposes no playable frames");
    SetLimits();
}

void TestValidationAndAllOrNone()
{
    SetLimits();
    const RobotPosition start = {-100.0f, 200.0f, 100.0f, 0.0f};
    const RobotPosition end = {0.0f, 200.0f, 100.0f, 0.0f};
    RobotAngles angles;
    if (!Solve(start, {HALF_PI, HALF_PI, HALF_PI, HALF_PI}, angles)) return;
    PlannedMovement movement;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Check(PlanLinearMovement(end, start, angles, 0.0f, movement) == MotionPlanError::InvalidInput,
          "zero speed is rejected");
    Check(PlanLinearMovement(end, start, angles, -1.0f, movement) == MotionPlanError::InvalidInput,
          "negative speed is rejected");
    Check(PlanLinearMovement(end, start, angles, nan, movement) == MotionPlanError::InvalidInput,
          "non-finite speed is rejected");
    RobotPosition invalidEnd = end;
    invalidEnd.x = nan;
    Check(PlanLinearMovement(invalidEnd, start, angles, 100.0f, movement) == MotionPlanError::InvalidInput,
          "non-finite Cartesian target is rejected");
    RobotPosition staleStart = start;
    staleStart.x += 10.0f;
    Check(PlanLinearMovement(end, staleStart, angles, 100.0f, movement) == MotionPlanError::InvalidInput,
          "stale Cartesian starting pose is rejected");
    RobotAngles invalidAngles = angles;
    invalidAngles.s1 = nan;
    Check(PlanLinearMovement(end, start, invalidAngles, 100.0f, movement) == MotionPlanError::InvalidInput,
          "non-finite starting joint is rejected");
    Check(PlanLinearMovement(end, start, angles, 100.0f, movement, 0) == MotionPlanError::MemoryLimit,
          "empty frame budget fails cleanly");
    Check(PlanLinearMovement(end, start, angles, 100.0f, movement, 2) == MotionPlanError::MemoryLimit,
          "small frame budget fails cleanly");
    Check(movement.frames == nullptr && movement.frameCount == 0, "failed budget exposes no partial frames");

    Check(PlanLinearMovement(end, start, angles, 100.0f, movement) == MotionPlanError::None,
          "planner remains reusable after invalid inputs");
    const RobotPosition unreachable = {1000.0f, 200.0f, 100.0f, 0.0f};
    Check(PlanLinearMovement(unreachable, start, angles, 100.0f, movement) == MotionPlanError::Unreachable,
          "unreachable endpoint is rejected");
    Check(movement.frames == nullptr && movement.frameCount == 0, "failure clears any previously planned frames");

    // Both endpoints are reachable, but this line passes through the forbidden
    // region near the shoulder before it would cross the base axis.
    const RobotPosition left = {-100.0f, 0.0f, 100.0f, 0.0f};
    const RobotPosition right = {100.0f, 0.0f, 100.0f, 0.0f};
    RobotAngles leftAngles;
    RobotAngles rightAngles;
    if (!Solve(left, angles, leftAngles) || !Solve(right, leftAngles, rightAngles)) return;
    Check(PlanLinearMovement(right, left, leftAngles, 100.0f, movement) == MotionPlanError::Unreachable,
          "unreachable interior rejects a path even with reachable endpoints");
    Check(movement.frames == nullptr && movement.frameCount == 0,
          "unreachable interior exposes no partially playable movement");
}
}

int main()
{
    TestRouteAndReuse();
    TestInvertedAndStationary();
    TestOrientationOnly();
    TestNearAxisRefinement();
    TestFractionalCalibrationLimits();
    TestValidationAndAllOrNone();
    if (failures) {
        std::fprintf(stderr, "%d motion planner checks failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("Motion planner checks passed.");
    return EXIT_SUCCESS;
}
