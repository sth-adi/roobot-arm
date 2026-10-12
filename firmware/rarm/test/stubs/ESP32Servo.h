#pragma once
#include <Arduino.h>
// Records what the real library would send: pulse width in microseconds, like ESP32Servo's write()/writeMicroseconds().
class Servo {
public:
    bool isAttached = false;
    int minUs = 500, maxUs = 2400, lastUs = 0;
    void setPeriodHertz(int) {}
    int attach(int, int lo, int hi) { isAttached = true; minUs = lo; maxUs = hi; return 1; }
    void detach() { isAttached = false; }
    bool attached() { return isAttached; }
    void write(int v) { if (v < minUs) { v = v < 0 ? 0 : (v > 180 ? 180 : v); v = (v * (maxUs - minUs)) / 180 + minUs; /* Arduino map(): integer maths */ } lastUs = v; }
    void writeMicroseconds(int us) { lastUs = us; }
};
