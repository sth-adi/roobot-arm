#include "core/StartupProcess.h"
#include "network/MovementClient.h"

void setup()
{
    StartupProcess();
}

void loop()
{
    static String pendingCompletion;
    if (pendingCompletion.length() > 0) {
        if (SendMovementComplete(pendingCompletion)) {
            pendingCompletion = "";
        } else {
            delay(1000);
        }
        return;
    }

    String movementPacket;
    String movementId;
    if (GetMovementPacket(targetAngles, movementPacket, movementId)) {
        if (ExecuteMovementPacket(movementPacket, targetAngles)) {
            pendingCompletion = movementId;
        } else {
            Serial.println("[ERROR][Movement] Playback aborted; retaining last commanded angles");
        }
    }
    // Avoid immediately re-requesting a packet after a 204 or failed request.
    delay(100);
}
