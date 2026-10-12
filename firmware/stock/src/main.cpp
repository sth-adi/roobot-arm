#include <Arduino.h>
#include <ESP32Servo.h>
#include "WifiFunctions.h"
#include "SticksToXYZ.h"
#include "InverseKinTransform.h"
#include "CalibrationFuncs.h"

String ArmMode = "";
String ArmStatus = "Starting";

static unsigned long lastPositionUpdateTime = 0;
static unsigned long lastPositionPrintTime = 0;
static unsigned long lastTrainingAnglePrintTime = 0;

StickInput controllerInput = {0, 0, 0, 0, 0, 0, 0, 0};
Servo jointServos[4];

constexpr int MaxSavedPositions = 20;
RobotPosition savedPositions[MaxSavedPositions];
int savedPositionCount = 0;

RobotPosition targetPosition = {
    HomeChordsX,
    HomeChordsY,
    HomeChordsZ,
    HeadAngle,
};

RobotAngles targetAngles = {HALF_PI, HALF_PI, 0.0f, 0.0f};

float JointAngleAt(const RobotAngles& angles, int jointIndex)
{
    switch (jointIndex) {
        case 0: return angles.s1;
        case 1: return angles.s2;
        case 2: return angles.s3;
        default: return angles.s4;
    }
}

RobotAngles HomeJointAngles()
{
    return {
        static_cast<float>(radians(JointHomes[0])),
        static_cast<float>(radians(JointHomes[1])),
        static_cast<float>(radians(JointHomes[2])),
        static_cast<float>(radians(JointHomes[3])),
    };
}

int ServoCommandAngle(int jointIndex, int logicalAngle)
{
    const int clampedAngle = constrain(logicalAngle, 0, 180);
    // A reversed calibration range marks an inverted servo: logical 0 degrees
    // maps to the physical 180-degree command, and vice versa.
    const bool inverted = JointLims[jointIndex][0] > JointLims[jointIndex][1];
    return inverted ? 180 - clampedAngle : clampedAngle;
}

void CommandJointAngles(const RobotAngles& angles)
{
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (!jointServos[jointIndex].attached()) {
            jointServos[jointIndex].setPeriodHertz(50);
            jointServos[jointIndex].attach(JointPins[jointIndex], 500, 2400);
        }
        const float angleDegrees = degrees(JointAngleAt(angles, jointIndex));
        const int minimum = min(JointLims[jointIndex][0],
                                JointLims[jointIndex][1]);
        const int maximum = max(JointLims[jointIndex][0],
                                JointLims[jointIndex][1]);
        const int logicalAngle = constrain(
            static_cast<int>(lroundf(angleDegrees)), minimum, maximum
        );
        jointServos[jointIndex].write(
            ServoCommandAngle(jointIndex, logicalAngle)
        );
    }
}


void setup(){   

    Serial.begin(9600);

    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        jointServos[jointIndex].setPeriodHertz(50);
        jointServos[jointIndex].attach(JointPins[jointIndex], 500, 2400);
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
                        targetPosition = SticksToXYZ(
                            controllerInput,
                            currentTime - lastPositionUpdateTime,
                            targetPosition
                        );
                    }
                    lastPositionUpdateTime = currentTime;

                    if (currentTime - lastPositionPrintTime >= 100) {
                        const int servoAngle = constrain(
                            static_cast<int>(targetPosition.angle),
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
                            targetPosition.angle
                        );
                        lastTrainingAnglePrintTime = currentTime;
                    }
                }

                angleOutputs[operationIndex] = static_cast<int>(targetPosition.angle);

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
            targetPosition.angle = JointHomes[jointIndex];
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
    targetAngles = HomeJointAngles();
    targetPosition = ForwardKinTransform(targetAngles);
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (!jointServos[jointIndex].attached()) {
            jointServos[jointIndex].setPeriodHertz(50);
            jointServos[jointIndex].attach(JointPins[jointIndex], 500, 2400);
        }
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
    Serial.printf(
        "Home XYZ: %.2f, %.2f, %.2f | Wrist: %.2f deg"
        " | Output angles: S1=%.2f, S2=%.2f, S3=%.2f, S4=%.2f deg\n",
        targetPosition.x,
        targetPosition.y,
        targetPosition.z,
        targetPosition.angle,
        degrees(targetAngles.s1),
        degrees(targetAngles.s2),
        degrees(targetAngles.s3),
        degrees(targetAngles.s4)
    );
}


void loop()
{
    static bool previousB = false;
    static bool previousA = false;
    static bool movingHome = false;
    static bool replayingPositions = false;
    static unsigned long homeStartTime = 0;
    static unsigned long aPressStartTime = 0;
    static unsigned long lastReplayPositionTime = 0;
    static int replayPositionIndex = 0;
    constexpr unsigned long replayStartHoldMs = 750;
    constexpr unsigned long replayPositionIntervalMs = 1500;

    if (GetControllerState(controllerInput)) {
        bool currentB = controllerInput.bButton == 1;
        bool currentA = controllerInput.aButton == 1;
        const unsigned long currentTime = millis();

        if (currentB && !previousB) {
            for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
                if (!jointServos[jointIndex].attached()) {
                    jointServos[jointIndex].setPeriodHertz(50);
                    jointServos[jointIndex].attach(
                        JointPins[jointIndex], 500, 2400
                    );
                }

                jointServos[jointIndex].write(
                    ServoCommandAngle(jointIndex, JointHomes[jointIndex])
                );
            }

            homeStartTime = millis();
            movingHome = true;

            SendArmStatus("Moving all joints home");
        }

        previousB = currentB;

        if (currentA && !previousA) {
            aPressStartTime = currentTime;
            if (savedPositionCount < MaxSavedPositions) {
                savedPositions[savedPositionCount++] = targetPosition;
                SendArmStatus("Position " + String(savedPositionCount) + " saved");
            } else {
                SendArmStatus("Saved-position list is full");
            }
        }

        if (currentA && !replayingPositions
            && currentTime - aPressStartTime >= replayStartHoldMs
            && savedPositionCount > 0) {
            replayingPositions = true;
            replayPositionIndex = 0;
            lastReplayPositionTime = currentTime - replayPositionIntervalMs;
            SendArmStatus("Replaying saved positions");
        }
        if (!currentA && replayingPositions) {
            replayingPositions = false;
            SendArmStatus("Position replay stopped");
        }
        previousA = currentA;

        if (replayingPositions
            && currentTime - lastReplayPositionTime >= replayPositionIntervalMs) {
            const RobotPosition& replayTarget = savedPositions[replayPositionIndex];
            RobotAngles replayAngles;
            if (InverseKinTransform(replayTarget, targetAngles, replayAngles)) {
                targetPosition = replayTarget;
                targetAngles = replayAngles;
                CommandJointAngles(targetAngles);
            }
            replayPositionIndex = (replayPositionIndex + 1) % savedPositionCount;
            lastReplayPositionTime = currentTime;
        }

        if (lastPositionUpdateTime != 0 && !movingHome && !replayingPositions) {
            const RobotPosition requestedPosition = SticksToXYZ(
                controllerInput,
                currentTime - lastPositionUpdateTime,
                targetPosition
            );
            RobotAngles requestedAngles;
            if (InverseKinTransform(requestedPosition, targetAngles,
                                    requestedAngles)) {
                targetPosition = requestedPosition;
                targetAngles = requestedAngles;
                CommandJointAngles(targetAngles);
            }
        }
        lastPositionUpdateTime = currentTime;

        if (currentTime - lastPositionPrintTime >= 100) {
            Serial.printf(
                "Target XYZ: %.2f, %.2f, %.2f | Wrist: %.2f deg"
                " | Output angles: S1=%.2f, S2=%.2f, S3=%.2f, S4=%.2f deg\n",
                targetPosition.x,
                targetPosition.y,
                targetPosition.z,
                targetPosition.angle,
                degrees(targetAngles.s1),
                degrees(targetAngles.s2),
                degrees(targetAngles.s3),
                degrees(targetAngles.s4)
            );
            lastPositionPrintTime = currentTime;
        }
    }

    if (movingHome && millis() - homeStartTime >= 3000) {
        for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
            jointServos[jointIndex].detach();
        }

        movingHome = false;
        SendArmStatus("Servo signals disabled");
    }

    delay(20);
}
