#pragma once
#include <Arduino.h>
// A servo that records what the firmware tells it. emu_main prints the four servos whenever any changed.
struct EmuServoState { int us[4]; bool attached[4]; bool dirty; };
inline EmuServoState& emuServos() { static EmuServoState s = {{0, 0, 0, 0}, {false, false, false, false}, true}; return s; }
inline int& emuServoCounter() { static int n = 0; return n; }   // which Servo object is which: order of construction

class Servo {
public:
    int slot;
    int minUs = 500, maxUs = 2400;
    Servo() : slot(emuServoCounter()++ % 4) {}
    void setPeriodHertz(int) {}
    int attach(int, int lo, int hi) { minUs = lo; maxUs = hi; emuServos().attached[slot] = true; emuServos().dirty = true; return 1; }
    void detach() { emuServos().attached[slot] = false; emuServos().dirty = true; }
    bool attached() { return emuServos().attached[slot]; }
    void write(int v)
    {
        if (v < minUs) { v = v < 0 ? 0 : (v > 180 ? 180 : v); v = (v * (maxUs - minUs)) / 180 + minUs; }
        writeMicroseconds(v);
    }
    void writeMicroseconds(int us) { emuServos().us[slot] = us; emuServos().dirty = true; }
};
