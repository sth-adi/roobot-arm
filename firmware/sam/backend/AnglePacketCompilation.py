import math

class Angles:
    def __init__(self, S1=0.0, S2=0.0, S3=0.0, S4=0.0):
        self.Change(S1, S2, S3, S4)

    def Change(self, S1, S2, S3, S4):
        self.S1 = S1
        self.S2 = S2
        self.S3 = S3
        self.S4 = S4


class Coordinates:
    def __init__(self, X=0.0, Y=0.0, Z=0.0, A=0.0):
        self.Change(X, Y, Z, A)

    def Change(self, X, Y, Z, A):
        self.X = X
        self.Y = Y
        self.Z = Z
        self.A = A


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




def InverseKim(InputCords: Coordinates):

    X,Y,Z,A = InputCords.X, InputCords.Y, InputCords.Z, InputCords.A

    Elbow_Branch = 1

    Reach = math.sqrt(X**2 + Y**2)
    Wrist_r = Reach - 63.0*math.cos(A)
    Wrist_z = Z - 63.0*math.sin(A)
    Shoulder_Wrist_Distance = math.sqrt(Wrist_r**2 + (Wrist_z - 74.0)**2)
    Midpoint_Distance = (1/2)*(Shoulder_Wrist_Distance**2 - 12444.2875)/Shoulder_Wrist_Distance
    Midpoint_s1_wrist_r = Midpoint_Distance*Wrist_r/Shoulder_Wrist_Distance
    Midpoint_s1_wrist_z = Midpoint_Distance*(Wrist_z - 74.0)/Shoulder_Wrist_Distance + 74.0
    BisectorLengh = math.sqrt(21301.4025 - Midpoint_Distance**2)
    MidAxis_Angle = math.atan2(Wrist_z - 74.0, Wrist_r)
    S1 = math.atan2(X, Y) + (1/2)*math.pi
    S2 = MidAxis_Angle + math.atan2(BisectorLengh*Elbow_Branch, Midpoint_Distance)
    Elbow_r = 145.95*math.cos(S2)
    Elbow_z = 145.95*math.sin(S2) + 74.0
    Forearm_Angle = math.atan2(-Elbow_z + Wrist_z, -Elbow_r + Wrist_r)
    S3 = Forearm_Angle - S2 + math.pi
    S4 = -S2 - S3 + A + (3/2)*math.pi

    OutputAngles = Angles(S1, S2, S3, S4)

    return OutputAngles