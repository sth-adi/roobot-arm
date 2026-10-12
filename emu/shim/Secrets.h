#pragma once
// Emulator build: no WiFi credentials, and HOST comes from the environment (EMU_HOST=host:port, default the local server).
// The firmware asks for https://HOST/...; the emulator talks plain HTTP to that address, so point it at the local
// server or at the network proxy (emu/netproxy.py), which adds the latency, loss and handshake cost of a real link.
#include <stdlib.h>
#include <string>
const char* WIFI_SSID = "emulated";
const char* WIFI_PASSWORD = "emulated";
inline const char* emuHost()
{
    static const std::string h = getenv("EMU_HOST") ? getenv("EMU_HOST") : "127.0.0.1:8000";
    return h.c_str();
}
#define HOST emuHost()
