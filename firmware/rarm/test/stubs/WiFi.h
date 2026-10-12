#pragma once
#include <Arduino.h>
#define WL_CONNECTED 3
#define WIFI_STA 1
struct IPAddressStub {};
struct WiFiStub {
    void mode(int) {}
    void begin(const char*, const char*) {}
    int status() { return 3; }
    IPAddressStub localIP() { return {}; }
};
inline WiFiStub WiFi;
