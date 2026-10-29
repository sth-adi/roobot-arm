#pragma once

float HomeChordsX = 0;
float HomeChordsY = 0;
float HomeChordsZ = 0;
float HeadAngle = 90;

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


