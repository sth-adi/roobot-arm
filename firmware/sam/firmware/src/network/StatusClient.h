#pragma once

#include "BackendConnection.h"

void SendArmStatus(const String& armStatus)
{

    Serial.println(armStatus);

    static unsigned long lastStatusTime = 0;
    constexpr unsigned long statusIntervalMs = 1000;

    if (WiFi.status() != WL_CONNECTED || millis() - lastStatusTime < statusIntervalMs) {
        return;
    }
    lastStatusTime = millis();

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    const String statusUrl = "https://" + String(HOST) + "/status";
    if (!http.begin(client, statusUrl)) {
        Serial.println("Failed to initialise status request");
        return;
    }

    http.addHeader("Content-Type", "text/plain; charset=utf-8");
    const int statusCode = http.POST(armStatus);
    if (statusCode != HTTP_CODE_NO_CONTENT) {
        Serial.printf("Status POST failed: %d\n", statusCode);
    }

    http.end();
}
