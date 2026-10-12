// Minimal Arduino core stand-in so the firmware headers compile and run on a PC (tests only).
#pragma once
#if defined(_WIN32) && defined(EMU_REALTIME)
// Winsock first: its headers must be read before the Arduino-style abs() macro below.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <cstdarg>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <cctype>

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define abs(x) ((x) > 0 ? (x) : -(x))
using std::min;
using std::max;

#ifdef EMU_REALTIME
// Real clock for the networked emulator (firmware/emu): millis() counts from "boot", delay() really sleeps.
inline unsigned long millis()
{
    static const auto t0 = std::chrono::steady_clock::now();
    return 1000 + (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
}
inline void delay(unsigned long ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
#else
// Fake clock for the unit-style tests: time only moves when the test says so or the firmware calls delay().
inline unsigned long& fakeMillis() { static unsigned long t = 0; return t; }
inline unsigned long millis() { return fakeMillis(); }
inline void delay(unsigned long ms) { fakeMillis() += ms; }
#endif

class String {
public:
    std::string s;
    String() {}
    String(const char* c) : s(c ? c : "") {}
    String(const std::string& c) : s(c) {}
    String(int v) : s(std::to_string(v)) {}
    String(float v, int decimals = 2) { char b[64]; snprintf(b, sizeof b, "%.*f", decimals, v); s = b; }
    const char* c_str() const { return s.c_str(); }
    unsigned length() const { return (unsigned)s.size(); }
    int indexOf(char c, unsigned from = 0) const { size_t p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
    int indexOf(const String& t, unsigned from = 0) const { size_t p = s.find(t.s, from); return p == std::string::npos ? -1 : (int)p; }
    String substring(unsigned from) const { return String(s.substr(from)); }
    void trim() { size_t a = s.find_first_not_of(" \t\r\n"); size_t b = s.find_last_not_of(" \t\r\n"); s = a == std::string::npos ? "" : s.substr(a, b - a + 1); }
    bool startsWith(const char* p) const { return s.rfind(p, 0) == 0; }
    void replace(const char* a, const char* b) { size_t p; std::string A(a); while ((p = s.find(A)) != std::string::npos) s.replace(p, A.size(), b); }
    void remove(unsigned from) { s.erase(from); }
    String operator+(const String& o) const { return String(s + o.s); }
    String operator+(const char* o) const { return String(s + o); }
    String& operator+=(const String& o) { s += o.s; return *this; }
    String& operator+=(char c) { s += c; return *this; }
    bool operator==(const String& o) const { return s == o.s; }
    bool operator==(const char* o) const { return s == o; }
    bool operator!=(const String& o) const { return s != o.s; }
    bool operator!=(const char* o) const { return s != o; }
};
inline String operator+(const char* a, const String& b) { return String(std::string(a) + b.s); }

#ifdef EMU_REALTIME
inline bool serialOn() { static const bool on = getenv("EMU_SERIAL") != nullptr; return on; }   // EMU_SERIAL=1 shows the firmware's Serial output
struct SerialStub {
    void begin(int) {}
    void print(const char* c) { if (serialOn()) fputs(c, stderr); }
    void print(const String& c) { if (serialOn()) fputs(c.c_str(), stderr); }
    void println(const char* c = "") { if (serialOn()) fprintf(stderr, "%s\n", c); }
    void println(const String& c) { if (serialOn()) fprintf(stderr, "%s\n", c.c_str()); }
    template <class T> void println(const T&) {}
    void printf(const char* f, ...) { if (!serialOn()) return; va_list a; va_start(a, f); vfprintf(stderr, f, a); va_end(a); }
};
#else
struct SerialStub {
    void begin(int) {}
    void print(const char*) {}
    void print(const String&) {}
    void println(const char* = "") {}
    void println(const String&) {}
    template <class T> void println(const T&) {}
    void printf(const char*, ...) {}
};
#endif
inline SerialStub Serial;
