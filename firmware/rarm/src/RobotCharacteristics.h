float HomeChordsX = 0;
float HomeChordsY = 0;
float HomeChordsZ = 0;
float HeadAngle = 90;

// Wrist to tool tip, mm.  63 with the ball fitted (the original file had 53).
float ToolLengthMm = 63;

// Joint speed limit (damper), degrees per second per joint.  The dashboard can change it live with "maxSpeed" in
// the controller packet; whatever arrives is clamped to 1..JointSpeedHardCap.
float JointMaxDegPerSecond = 30;
const float JointSpeedHardCap = 60;

float AngleSense = 20;
float ChordSense = 20;

int JointPins[4] = {
    32,
    27,
    26,
    25
};

int JointLims[4][2] = {
    {0, 0},  // Joint 1: min, max
    {0, 0},  // Joint 2: min, max
    {0, 0},  // Joint 3: min, max
    {0, 0}   // Joint 4: min, max
};



int JointHomes[4] = {
    90,
    90,
    90,
    90
};


