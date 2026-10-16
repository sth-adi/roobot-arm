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

// Optional: mirror mode over WiFi, so the board needs no USB cable. The board connects OUT to the hub on your PC (the hub listens on its port + 2,
// 8767 by default) and sends the key; then the hub drives it exactly like the COM port. The Arm and board page of the hub shows these three lines
// ready to paste. MIRROR_HOST is the PC's address (give the PC a fixed address on your router, or the board needs this line changed when it moves).
// Leave all three out to compile none of it. The link is not encrypted: use it on a network you trust.
// #define MIRROR_HOST "192.168.x.x"
// #define MIRROR_PORT 8767
// #define MIRROR_KEY "the key from the hub"
