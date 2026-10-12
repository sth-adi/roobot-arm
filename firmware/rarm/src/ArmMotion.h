#pragma once

// The arm's movement state machine, kept free of servos and WiFi so it can be tested on a PC.
// sim/armsim.py (the virtual arm) is the same logic line for line, and firmware/rarm/test checks the two agree.
//
//   sticks -> SticksToXYZ -> InverseKinTransform -> joint angles -> speed limit (damper) -> servo pulses
//
// B = go home, A = save the position, hold A = replay the saved positions (as in the original main.cpp).
// Changes from the original firmware:
//   * Joint speed limit: joints move toward the commanded angles at no more than maxDegPerSecond, all four scaled
//     together so the tool keeps to the path.  Home and replay are limited the same way.
//   * The joint angles are the single source of truth for the pose (forward kinematics), so after "go home" the
//     next stick movement starts from where the arm really is instead of snapping back to the old target.

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "SticksToXYZ.h"
#include "InverseKinTransform.h"

constexpr int MaxSavedPositions = 20;
constexpr float ReplayStartHoldSeconds = 0.75f;
constexpr float ReplayStepSeconds = 1.5f;
constexpr float HomeSettleSeconds = 1.0f;     // after arriving home the signals stay on this long, then servos release
constexpr float MaxStepSeconds = 0.1f;        // a long gap between ticks never becomes one big jump
constexpr float UnlimitedDegPerSecond = 180.0f;

// Bit flags returned by ArmMotion::Step; main.cpp turns them into status messages.
enum ArmEvent : uint8_t {
    EventHomeStarted    = 1 << 0,
    EventPositionSaved  = 1 << 1,
    EventSaveListFull   = 1 << 2,
    EventReplayStarted  = 1 << 3,
    EventReplayStopped  = 1 << 4,
    EventServosReleased = 1 << 5,
};

enum class ArmMode : uint8_t { Idle, Manual, Home, Replay };

inline float JointAngleAt(const RobotAngles& angles, int jointIndex)
{
    switch (jointIndex) {
        case 0: return angles.s1;
        case 1: return angles.s2;
        case 2: return angles.s3;
        default: return angles.s4;
    }
}

inline RobotAngles HomeJointAngles()
{
    return {
        static_cast<float>(radians(JointHomes[0])),
        static_cast<float>(radians(JointHomes[1])),
        static_cast<float>(radians(JointHomes[2])),
        static_cast<float>(radians(JointHomes[3])),
    };
}

// Servo pulse width in microseconds for a joint angle in degrees (fractions allowed, so slow moves are smooth
// instead of stepping a whole degree at a time).  Same mapping as ESP32Servo's write() after attach(pin, 500, 2400):
// 0 deg = 500 us, 180 deg = 2400 us.  A reversed calibration range marks an inverted servo (angle becomes 180 - angle).
inline int ServoPulseMicros(int jointIndex, float angleDegrees)
{
    const float minimum = static_cast<float>(min(JointLims[jointIndex][0], JointLims[jointIndex][1]));
    const float maximum = static_cast<float>(max(JointLims[jointIndex][0], JointLims[jointIndex][1]));
    float logical = angleDegrees;
    if (logical < minimum) logical = minimum;
    if (logical > maximum) logical = maximum;
    if (logical < 0.0f) logical = 0.0f;
    if (logical > 180.0f) logical = 180.0f;
    const bool inverted = JointLims[jointIndex][0] > JointLims[jointIndex][1];
    const float command = inverted ? 180.0f - logical : logical;
    return static_cast<int>(lroundf(500.0f + command * (1900.0f / 180.0f)));
}

struct ArmMotion {
    RobotAngles angles = {HALF_PI, HALF_PI, HALF_PI, HALF_PI};
    ArmMode mode = ArmMode::Idle;
    bool attached = true;          // false once the servos have been released after going home
    bool ikOk = true;              // false while the sticks ask for somewhere unreachable
    uint32_t moved = 0;            // incremented whenever the angles change

    RobotPosition saved[MaxSavedPositions];
    int savedCount = 0;
    int replayIndex = 0;

    RobotAngles goal = {0, 0, 0, 0};
    bool hasGoal = false;
    bool prevA = false;
    bool prevB = false;
    double clock = 0.0;
    double aDownTime = 0.0;
    double replayTime = 0.0;
    double homeReachedTime = 0.0;
    bool homeReached = false;

    void Reset(const RobotAngles& startAngles)
    {
        angles = startAngles;
        mode = ArmMode::Idle;
        attached = true;
        ikOk = true;
        hasGoal = false;
        homeReached = false;
        savedCount = 0;
        replayIndex = 0;
    }

    RobotPosition Pose() const { return ForwardKinTransform(angles); }

    // Move all four joints toward `target`, no joint faster than the limit.  Returns true on arrival.
    bool Slew(const RobotAngles& target, float maxDegPerSecond, float dt)
    {
        const float limit = maxDegPerSecond > 0.0f ? maxDegPerSecond : UnlimitedDegPerSecond;
        const float step = radians(limit) * dt;
        const float d[4] = {target.s1 - angles.s1, target.s2 - angles.s2,
                            target.s3 - angles.s3, target.s4 - angles.s4};
        float biggest = 0.0f;
        for (int i = 0; i < 4; ++i) {
            if (fabsf(d[i]) > biggest) biggest = fabsf(d[i]);
        }
        if (biggest <= step || biggest < 1e-9f) {
            angles = target;
            return true;
        }
        const float k = step / biggest;
        angles = {angles.s1 + d[0] * k, angles.s2 + d[1] * k, angles.s3 + d[2] * k, angles.s4 + d[3] * k};
        return false;
    }

    void StartHome()
    {
        goal = HomeJointAngles();
        hasGoal = true;
        mode = ArmMode::Home;
        homeReached = false;
        attached = true;
    }

    // Advance by dt seconds.  `input` is null when there is no fresh input (the sender is gone), which counts as
    // sticks centred and buttons released.  Returns ArmEvent flags.
    uint8_t Step(const StickInput* input, float maxDegPerSecond, float dt)
    {
        uint8_t events = 0;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > MaxStepSeconds) dt = MaxStepSeconds;
        clock += dt;
        const double now = clock;

        const StickInput none = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        const StickInput& in = input ? *input : none;
        const bool btnA = in.aButton == 1;
        const bool btnB = in.bButton == 1;

        // B pressed: go home.
        if (btnB && !prevB) {
            if (mode == ArmMode::Replay) events |= EventReplayStopped;
            mode = ArmMode::Idle;
            StartHome();
            events |= EventHomeStarted;
        }
        // A pressed: save the current position.
        if (btnA && !prevA) {
            aDownTime = now;
            if (savedCount < MaxSavedPositions) {
                saved[savedCount++] = Pose();
                events |= EventPositionSaved;
            } else {
                events |= EventSaveListFull;
            }
        }
        // A held: start the replay.  A released: stop it.
        if (btnA && mode != ArmMode::Replay && mode != ArmMode::Home && savedCount > 0
            && now - aDownTime >= ReplayStartHoldSeconds) {
            mode = ArmMode::Replay;
            replayIndex = 0;
            replayTime = now - ReplayStepSeconds;
            events |= EventReplayStarted;
        }
        if (!btnA && mode == ArmMode::Replay) {
            mode = ArmMode::Idle;
            hasGoal = false;
            events |= EventReplayStopped;
        }
        prevA = btnA;
        prevB = btnB;

        // Replay: aim at the next saved position every ReplayStepSeconds.
        if (mode == ArmMode::Replay && now - replayTime >= ReplayStepSeconds) {
            RobotAngles replayAngles;
            if (InverseKinTransform(saved[replayIndex], angles, replayAngles)) {
                goal = replayAngles;
                hasGoal = true;
            }
            replayIndex = (replayIndex + 1) % savedCount;
            replayTime = now;
        }

        // Joint-space moves (home and replay), limited in speed.
        if (hasGoal) {
            const RobotAngles before = angles;
            const bool arrived = Slew(goal, maxDegPerSecond, dt);
            if (angles.s1 != before.s1 || angles.s2 != before.s2 || angles.s3 != before.s3
                || angles.s4 != before.s4) {
                attached = true;
                ++moved;
            }
            if (arrived) {
                hasGoal = false;
                if (mode == ArmMode::Home) {
                    homeReached = true;
                    homeReachedTime = now;
                }
            }
        }
        if (mode == ArmMode::Home && !hasGoal) {
            if (homeReached && now - homeReachedTime >= HomeSettleSeconds) {
                mode = ArmMode::Idle;
                attached = false;
                events |= EventServosReleased;
            }
            return events;
        }

        // Sticks: Cartesian jog through inverse kinematics, speed limited.
        const bool sticksActive = ApplyStickDeadzone(in.leftX) != 0.0f || ApplyStickDeadzone(in.leftY) != 0.0f
            || ApplyStickDeadzone(in.rightX) != 0.0f || ApplyStickDeadzone(in.rightY) != 0.0f;
        if ((mode == ArmMode::Idle || mode == ArmMode::Manual) && sticksActive) {
            const RobotPosition requested = SticksToXYZ(in, dt * 1000.0f, Pose());
            RobotAngles requestedAngles;
            ikOk = InverseKinTransform(requested, angles, requestedAngles);
            mode = ArmMode::Manual;
            if (ikOk) {
                const RobotAngles before = angles;
                Slew(requestedAngles, maxDegPerSecond, dt);
                if (angles.s1 != before.s1 || angles.s2 != before.s2 || angles.s3 != before.s3
                    || angles.s4 != before.s4) {
                    attached = true;
                    ++moved;
                }
            }
        } else if (mode == ArmMode::Manual) {
            mode = ArmMode::Idle;
            ikOk = true;
        }
        return events;
    }
};
