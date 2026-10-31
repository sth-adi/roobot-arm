#pragma once

#include <Arduino.h>
#include <WiFi.h>

inline void TestConnection()
{
    static unsigned long lastReconnectTime = 0;
    static bool reconnectRequested = false;

    while (WiFi.status() != WL_CONNECTED) {
        const unsigned long now = millis();

        if (!reconnectRequested || now - lastReconnectTime >= 5000) {
            lastReconnectTime = now;
            reconnectRequested = true;
            Serial.println("[Movement] Reconnecting to WiFi");
            WiFi.reconnect();
        }

        delay(10); // Yield to the Wi-Fi stack and watchdog.
    }

    if (reconnectRequested) {
        reconnectRequested = false;
        Serial.println("[Movement] WiFi connected");
    }
}
