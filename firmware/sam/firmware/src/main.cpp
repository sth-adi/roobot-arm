#include "StartupProcess.h"

void setup()
{
    StartupProcess();
    targetPosition = ForwardKinTransform(targetAngles);
}

void loop()
{
    String movementPacket;
    if (GetMovementPacket(targetPosition, movementPacket)
        && ExecuteMovementPacket(movementPacket, targetAngles)) {
        targetPosition = ForwardKinTransform(targetAngles);
    }
    // Avoid immediately re-requesting a packet after a 204 or failed request.
    delay(100);
}
