#pragma once

#include <Arduino.h>

struct StickInput {
    float leftX;
    float leftY;
    float rightX;
    float rightY;
    int aButton;
    int bButton;
    int xButton;
    int yButton;
};

struct RobotPosition {
    float x;
    float y;
    float z;
    float angle;
};

// Joint angles are radians; RobotPosition::angle stays in controller degrees.
struct RobotAngles {
    float s1;
    float s2;
    float s3;
    float s4;
};
