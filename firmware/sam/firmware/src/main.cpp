#include "core/StartupProcess.h"
#include "network/MovementClient.h"
#include "network/TestConnection.h"

void setup()
{
    StartupProcess();
    Serial.println("[Movement] Continuously checking backend for target movements");
}

void loop()
{
    TestConnection();

    String movementPacket;
    if (GetMovementPacket(targetAngles, movementPacket)) {
        Serial.printf("[Movement] Received packet: %u bytes\n", movementPacket.length());
        const RobotAngles savedTargetAngles = targetAngles;

        if (!ExecuteMovementPacket(movementPacket)) {
            Serial.println("[ERROR][Movement] Playback aborted; returning to target angles");
            targetAngles = savedTargetAngles;

            const float angles[] = {
                targetAngles.s1,
                targetAngles.s2,
                targetAngles.s3,
                targetAngles.s4,
            };
            for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
                WriteJointAngle(jointIndex, degrees(angles[jointIndex]));
            }
        }
        SendMovementComplete();
    }

    delay(100);
}
