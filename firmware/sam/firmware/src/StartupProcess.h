#include <Arduino.h>
#include "WifiFunctions.h"
#include "SticksToXYZ.h"
#include "CalibrationFuncs.h"
#include "RobotMovement.h"

static unsigned long lastPositionUpdateTime = 0;
static unsigned long lastPositionPrintTime = 0;
StickInput controllerInput = {0, 0, 0, 0, 0, 0, 0, 0};

// State used only during the startup/calibration workflow.
String ArmMode = "";
static unsigned long lastTrainingAnglePrintTime = 0;

void StartupProcess(){   

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

                // Store exactly the same safe angle that is sent to the servo.
                // This prevents an out-of-range controller position from being
                // saved as calibration data and rejected by the backend.
                angleOutputs[operationIndex] = constrain(
                    static_cast<int>(targetPosition.angle),
                    0,
                    180
                );

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
    Serial.println("Startup instructions complete");
}
