#pragma once
// Mirror mode with the inverse kinematics ON THE BOARD.
//
// The hub sends the tool-tip pose "#x,y,z,angle,seq" (mm and degrees) instead of four joint angles. This turns it into servo degrees with
// Sam's own inverse kinematics (InverseKinTransform.h, SticksToXYZ.h and RobotCharacteristics.h are unchanged copies of firmware/stock/src;
// sim/test_ik_board.py fails if they drift), using the real arm's calibration, which the hub sends as "!cal". Same maths, same limits, same
// elbow choice and same reversed-joint rule (ServoCommandAngle) as Sam's firmware, so the arm ends up where his firmware would put it.
//
// Pure maths and no hardware, so a PC can compile and test it (sim/test_ik_board.py).
#include "InverseKinTransform.h"

struct MirrorIk {
  bool known;          // a calibration has been received
  RobotAngles prev;    // last solution (radians): picks the nearer of the two elbow branches
};
static MirrorIk mirrorIk = {false, {0, 0, 0, 0}};

// Joint limits {min,max} per joint, and the homes, in whole servo degrees. A reversed pair (min > max) marks an inverted servo, as in Sam's firmware.
inline bool MirrorSetCalibration(const int lims[4][2], const int homes[4]) {
  for (int j = 0; j < 4; j++) {
    if (lims[j][0] < 0 || lims[j][0] > 180 || lims[j][1] < 0 || lims[j][1] > 180 || lims[j][0] == lims[j][1]) return false;
    if (homes[j] < 0 || homes[j] > 180) return false;
  }
  for (int j = 0; j < 4; j++) { JointLims[j][0] = lims[j][0]; JointLims[j][1] = lims[j][1]; JointHomes[j] = homes[j]; }
  mirrorIk.prev = {radians((float)homes[0]), radians((float)homes[1]), radians((float)homes[2]), radians((float)homes[3])};
  mirrorIk.known = true;
  return true;
}

// Tool-tip pose -> the four SERVO degrees (0..180, already flipped for inverted joints). False when there is no calibration yet or the pose
// cannot be reached inside the joint limits; the outputs are then left alone.
inline bool MirrorPoseToServoDegrees(float x, float y, float z, float angle, float servoDeg[4]) {
  if (!mirrorIk.known) return false;
  RobotPosition target;
  target.x = x; target.y = y; target.z = z; target.angle = angle;
  RobotAngles out;
  if (!InverseKinTransform(target, mirrorIk.prev, out)) return false;
  const float logical[4] = {degrees(out.s1), degrees(out.s2), degrees(out.s3), degrees(out.s4)};
  for (int j = 0; j < 4; j++) {
    const float clamped = constrain(logical[j], 0.0f, 180.0f);
    const bool inverted = JointLims[j][0] > JointLims[j][1];
    servoDeg[j] = inverted ? 180.0f - clamped : clamped;
  }
  mirrorIk.prev = out;
  return true;
}
