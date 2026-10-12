// UWE-RArm firmware (Samwally1/UWE-RArm @3b9b8aa) with a joint speed limit and a stale-input stop.
// The movement logic now lives in ArmMotion.h; see firmware/rarm/README.md for what changed.
#include <Arduino.h>
#include <ESP32Servo.h>
#include "WifiFunctions.h"
#include "SticksToXYZ.h"
#include "InverseKinTransform.h"
#include "CalibrationFuncs.h"
#include "ArmMotion.h"

String ArmMode = "";
String ArmStatus = "Starting";

static unsigned long lastPositionUpdateTime = 0;
static unsigned long lastPositionPrintTime = 0;
static unsigned long lastTrainingAnglePrintTime = 0;

// No new packet for this long = the sender is gone: behave as if the sticks were centred and no button pressed.
constexpr unsigned long StaleInputMs = 1500;

StickInput controllerInput = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Servo jointServos[4];
ArmMotion arm;

// Used only while calibrating (training): the joint under test is driven from the tool-angle stick axis.
RobotPosition trainingPosition = {
    HomeChordsX,
    HomeChordsY,
    HomeChordsZ,
    HeadAngle,
};

int ServoCommandAngle(int jointIndex, int logicalAngle)
{
    const int clampedAngle = constrain(logicalAngle, 0, 180);
    // A reversed calibration range marks an inverted servo: logical 0 degrees
    // maps to the physical 180-degree command, and vice versa.
    const bool inverted = JointLims[jointIndex][0] > JointLims[jointIndex][1];
    return inverted ? 180 - clampedAngle : clampedAngle;
}

void AttachServo(int jointIndex)
{
    if (!jointServos[jointIndex].attached()) {
        jointServos[jointIndex].setPeriodHertz(50);
        jointServos[jointIndex].attach(JointPins[jointIndex], 500, 2400);
    }
}

// Write the arm's joint angles to the servos as pulse widths, so slow moves are smooth (see ServoPulseMicros).
void CommandJointAngles(const RobotAngles& angles)
{
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        AttachServo(jointIndex);
        jointServos[jointIndex].writeMicroseconds(
            ServoPulseMicros(jointIndex, degrees(JointAngleAt(angles, jointIndex)))
        );
    }
}


void setup(){

    Serial.begin(9600);

    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        AttachServo(jointIndex);
        jointServos[jointIndex].write(ServoCommandAngle(jointIndex, 90));
    }

    if (ConnectToWiFi()) {
        SendArmStatus("Power on: connected to WiFi");
        const float latency = TestLatency();
        delay(1000);
        if (latency >= 0.0f) {
            SendArmStatus("Latency = " + String(latency, 2) + " ms");
        } else {
            SendArmStatus("Latency test failed");
        }
    }
    ArmMode = CheckControllerMode();

    const bool calibrationLoaded = LoadCalibration();
    const bool trainingRequestedAtStartup = GetControllerState(controllerInput)
        && controllerInput.aButton == 1;

    if (ArmMode == "Training" || !calibrationLoaded || trainingRequestedAtStartup) {
        if (trainingRequestedAtStartup) {
            SendArmStatus("Training requested: release A to begin");
            while (controllerInput.aButton == 1) {
                GetControllerState(controllerInput);
                delay(10);
            }
        }

        if (!calibrationLoaded){
            SendArmStatus("No Trianing Config Found ...");
        };

        SendArmStatus("Training Angles");

        const char* operationNames[3] = {
            "Min angle",
            "Max angle",
            "Home angle",
        };

        // Begin with all joints limp.  The active joint is attached below;
        // each completed joint remains at its newly recorded home position.
        for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
            jointServos[jointIndex].detach();
        }

        for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
            int angleOutputs[4] = {0, 0, 0, 0};

            if (!jointServos[jointIndex].attached()) {
                jointServos[jointIndex].setPeriodHertz(50);
                jointServos[jointIndex].attach(JointPins[jointIndex], 500, 2400);
            }

            for (int operationIndex = 0; operationIndex < 3; ++operationIndex) {

                SendArmStatus(
                    "Joint " + String(jointIndex + 1) + ": " + operationNames[operationIndex]
                );

                while (controllerInput.aButton == 0) {
                    if (!GetControllerState(controllerInput)) {
                        continue;
                    }

                    const unsigned long currentTime = millis();
                    if (lastPositionUpdateTime != 0) {
                        trainingPosition = SticksToXYZ(
                            controllerInput,
                            currentTime - lastPositionUpdateTime,
                            trainingPosition
                        );
                    }
                    lastPositionUpdateTime = currentTime;

                    if (currentTime - lastPositionPrintTime >= 100) {
                        const int servoAngle = constrain(
                            static_cast<int>(trainingPosition.angle),
                            0,
                            180
                        );
                        jointServos[jointIndex].write(
                            ServoCommandAngle(jointIndex, servoAngle)
                        );

                        lastPositionPrintTime = currentTime;
                    }

                    if (currentTime - lastTrainingAnglePrintTime >= 1000) {
                        Serial.printf(
                            "Training joint %d angle: %.2f deg\n",
                            jointIndex + 1,
                            trainingPosition.angle
                        );
                        lastTrainingAnglePrintTime = currentTime;
                    }
                }

                angleOutputs[operationIndex] = static_cast<int>(trainingPosition.angle);

                // Once both end limits are known, apply the joint's direction
                // convention while the operator selects its home position.
                if (operationIndex == 1) {
                    JointLims[jointIndex][0] = angleOutputs[0];
                    JointLims[jointIndex][1] = angleOutputs[1];
                }

                while (controllerInput.aButton == 1) {
                    GetControllerState(controllerInput);
                }
            }

            JointLims[jointIndex][0] = angleOutputs[0];
            JointLims[jointIndex][1] = angleOutputs[1];
            JointHomes[jointIndex] = angleOutputs[2];
            trainingPosition.angle = JointHomes[jointIndex];
            jointServos[jointIndex].write(
                ServoCommandAngle(jointIndex, JointHomes[jointIndex])
            );
        }

        Serial.println("Joint calibration:");

        for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
            const String jointCalibration = "Joint " + String(jointIndex + 1)
                + " | Min: " + String(JointLims[jointIndex][0])
                + " | Max: " + String(JointLims[jointIndex][1])
                + " | Home: " + String(JointHomes[jointIndex]) + " deg";
            SendArmStatus(jointCalibration);
        }

        if (SaveCalibration()) {
            SendArmStatus("Calibration saved");
        } else {
            SendArmStatus("Calibration save failed");
        }
    } else {
        SendArmStatus(CalibrationStatus());

    }

    // Start Cartesian control at the trained home pose, rather than at an
    // arbitrary coordinate which may not be reachable.
    arm.Reset(HomeJointAngles());
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        AttachServo(jointIndex);
        jointServos[jointIndex].write(
            ServoCommandAngle(jointIndex, JointHomes[jointIndex])
        );
    }
    SendArmStatus("Moving all joints to loaded home positions");
    Serial.println("Loaded joint calibration:");
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        Serial.printf(
            "Joint %d | Min: %d | Max: %d | Home: %d deg\n",
            jointIndex + 1,
            JointLims[jointIndex][0],
            JointLims[jointIndex][1],
            JointHomes[jointIndex]
        );
    }
    const RobotPosition homePosition = arm.Pose();
    Serial.printf(
        "Home XYZ: %.2f, %.2f, %.2f | Wrist: %.2f deg | Tool length %.1f mm | Speed limit %.0f deg/s\n",
        homePosition.x,
        homePosition.y,
        homePosition.z,
        homePosition.angle,
        ToolLengthMm,
        JointMaxDegPerSecond
    );
    lastPositionUpdateTime = 0;
}


void loop()
{
    static unsigned long lastTickTime = 0;
    static unsigned long lastInputTime = 0;
    static bool haveInput = false;
    static uint32_t writtenMoves = 0;
    static bool servosWereAttached = true;

    const unsigned long currentTime = millis();

    if (GetControllerState(controllerInput)) {
        haveInput = true;
        lastInputTime = currentTime;
        if (controllerInput.maxSpeed >= 1.0f) {
            JointMaxDegPerSecond = controllerInput.maxSpeed;   // already clamped to 1..JointSpeedHardCap
        }
    }
    const bool inputFresh = haveInput && currentTime - lastInputTime < StaleInputMs;

    const float dt = lastTickTime == 0 ? 0.0f : (currentTime - lastTickTime) / 1000.0f;
    lastTickTime = currentTime;

    const uint8_t events = arm.Step(inputFresh ? &controllerInput : nullptr, JointMaxDegPerSecond, dt);

    if (events & EventHomeStarted) SendArmStatus("Moving all joints home");
    if (events & EventPositionSaved) SendArmStatus("Position " + String(arm.savedCount) + " saved");
    if (events & EventSaveListFull) SendArmStatus("Saved-position list is full");
    if (events & EventReplayStarted) SendArmStatus("Replaying saved positions");
    if (events & EventReplayStopped) SendArmStatus("Position replay stopped");

    if (arm.attached) {
        if (!servosWereAttached || arm.moved != writtenMoves) {
            CommandJointAngles(arm.angles);
            writtenMoves = arm.moved;
            servosWereAttached = true;
        }
    } else if (servosWereAttached) {
        for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
            jointServos[jointIndex].detach();
        }
        servosWereAttached = false;
    }
    if (events & EventServosReleased) SendArmStatus("Servo signals disabled");

    if (currentTime - lastPositionPrintTime >= 100) {
        const RobotPosition pose = arm.Pose();
        Serial.printf(
            "Target XYZ: %.2f, %.2f, %.2f | Wrist: %.2f deg"
            " | Output angles: S1=%.2f, S2=%.2f, S3=%.2f, S4=%.2f deg%s\n",
            pose.x, pose.y, pose.z, pose.angle,
            degrees(arm.angles.s1), degrees(arm.angles.s2),
            degrees(arm.angles.s3), degrees(arm.angles.s4),
            arm.ikOk ? "" : " | unreachable"
        );
        lastPositionPrintTime = currentTime;
    }

    delay(20);
}
