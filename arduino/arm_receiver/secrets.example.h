// Copy to secrets.h (git-ignored) and fill in. Never commit secrets.h.
#pragma once

#define WIFI_SSID "your-wifi-name"
#define WIFI_PASS "your-wifi-password"
#define STATE_URL "https://<your-end-server>/state"   // must be https://

// Optional: plain-TCP low-latency stream from the local server (no TLS, same network only).
// STREAM_HOST = IP of the PC running the Docker server, reachable from the ESP32's WiFi.
// Leave both lines out to use HTTPS polling of STATE_URL instead.
// #define STREAM_HOST "192.168.x.x"
// #define STREAM_PORT 8000
