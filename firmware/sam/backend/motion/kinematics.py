import math

from models.robot import Angles, Coordinates


def ForwardKim(servo_angles: Angles):
    """Convert logical joint angles in radians to millimetres and tool radians."""
    s1, s2, s3, s4 = (servo_angles.S1, servo_angles.S2,
                       servo_angles.S3, servo_angles.S4)
    if not all(math.isfinite(value) for value in (s1, s2, s3, s4)):
        raise ValueError("Joint angles must be finite")
    base = s1 - math.pi / 2
    forearm = s2 + s3 - math.pi
    tool = s2 + s3 + s4 - 3 * math.pi / 2
    radius = (145.95 * math.cos(s2) + 183.70 * math.cos(forearm)
              + 63.0 * math.cos(tool))
    height = (74.0 + 145.95 * math.sin(s2) + 183.70 * math.sin(forearm)
              + 63.0 * math.sin(tool))
    return Coordinates(radius * math.sin(base), radius * math.cos(base), height, tool)


def InverseKim(InputCords: Coordinates):

    X,Y,Z,A = InputCords.X, InputCords.Y, InputCords.Z, InputCords.A

    Elbow_Branch = 1

    Reach = math.sqrt(X**2 + Y**2)
    Wrist_r = Reach - 63.0*math.cos(A)
    Wrist_z = Z - 63.0*math.sin(A)
    Shoulder_Wrist_Distance = math.sqrt(Wrist_r**2 + (Wrist_z - 74.0)**2)
    Midpoint_Distance = (1/2)*(Shoulder_Wrist_Distance**2 - 12444.2875)/Shoulder_Wrist_Distance
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
