import math

from models.robot import Angles, Coordinates
from motion.kinematics import InverseKim


def FormatPacketEntry(Time, ServoAngles: Angles):
    """Create a packet line using whole milliseconds and degree angles."""
    return "/".join(
        (
            f"{Time * 1000:.0f}",
            f"{math.degrees(ServoAngles.S1):.3f}",
            f"{math.degrees(ServoAngles.S2):.3f}",
            f"{math.degrees(ServoAngles.S3):.3f}",
            f"{math.degrees(ServoAngles.S4):.3f}",
        )
    )


def CompileAnglePacket(StartingCords: Coordinates, EndingCords: Coordinates, Speed, StepsPmm):

    XDiff = EndingCords.X - StartingCords.X
    YDiff = EndingCords.Y - StartingCords.Y
    ZDiff = EndingCords.Z - StartingCords.Z
    ADiff = EndingCords.A - StartingCords.A

    Distance = math.sqrt(XDiff**2 + YDiff**2 + ZDiff**2)
    if Speed <= 0 or StepsPmm <= 0:
        raise ValueError("Speed and StepsPmm must be greater than zero")
    if Distance == 0:
        return []

    TimeTaken = Distance / Speed
    Steps = max(1, math.ceil(Distance * StepsPmm))
    StepTiming = TimeTaken / Steps

    MovementSet = []

    InitialAngles = InverseKim(StartingCords)

    MovementSet.append(str(Steps))

    MovementSet.append(f"{StepTiming * 1000:.0f}")  # milliseconds per step
    
    MovementSet.append(FormatPacketEntry(0, InitialAngles))



    for step in range(1, Steps + 1):
        fraction = step / Steps
        MidTargetCords = Coordinates(
            StartingCords.X + XDiff * fraction,
            StartingCords.Y + YDiff * fraction,
            StartingCords.Z + ZDiff * fraction,
            StartingCords.A + ADiff * fraction,
        )
        target_angles = InverseKim(MidTargetCords)
        MovementSet.append(FormatPacketEntry(step * StepTiming, target_angles))

    return MovementSet
