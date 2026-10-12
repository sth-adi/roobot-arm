#pragma once
#include <Arduino.h>
struct PingStub { bool ping(const char*, int) { return true; } float averageTime() { return 10.0f; } };
inline PingStub Ping;
