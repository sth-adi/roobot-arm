#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

using std::abs;
using std::isfinite;
using std::min;
using std::max;

constexpr float PI = 3.14159265358979323846f;
constexpr float HALF_PI = PI * 0.5f;
inline float radians(float value) { return value * PI / 180.0f; }
inline float degrees(float value) { return value * 180.0f / PI; }
template <typename T> inline T constrain(T value, T lower, T upper) { return min(max(value, lower), upper); }

namespace ArduinoTest {
static uint64_t clockUs = 0;
static uint64_t nextDelayExtraUs = 0;
inline void ResetClock(uint64_t startUs = 0)
{
    clockUs = startUs;
    nextDelayExtraUs = 0;
}
inline void Advance(uint64_t intervalUs)
{
    clockUs += intervalUs + nextDelayExtraUs;
    nextDelayExtraUs = 0;
}
}
inline uint32_t micros() { return static_cast<uint32_t>(ArduinoTest::clockUs); }
inline void delay(unsigned long intervalMs) { ArduinoTest::Advance(static_cast<uint64_t>(intervalMs) * 1000); }
inline void delayMicroseconds(unsigned int intervalUs) { ArduinoTest::Advance(intervalUs); }
