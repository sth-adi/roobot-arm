#pragma once

#include "BackendConnection.h"
#include "../core/RobotTypes.h"

// Returns false for both "already at target" (HTTP 204) and request errors.
// On success packet contains one line per command:
// milliseconds/S1-deg/S2-deg/S3-deg/S4-deg
bool GetMovementPacket(const RobotAngles& currentAngles, String& packet, String& movementId)
{
    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long lastErrorTime = 0;
        static bool reported = false;
        if (!reported || millis() - lastErrorTime >= 5000) {
            Serial.println("[ERROR][Movement] WiFi disconnected; request skipped");
            lastErrorTime = millis();
            reported = true;
        }
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    // Send logical joint degrees; the backend calculates Cartesian position.
    const String movementUrl = "https://" + String(HOST) + "/UpdateMovement?s1="
        + String(degrees(currentAngles.s1), 3) + "&s2=" + String(degrees(currentAngles.s2), 3)
        + "&s3=" + String(degrees(currentAngles.s3), 3) + "&s4="
        + String(degrees(currentAngles.s4), 3);
    if (!http.begin(client, movementUrl)) {
        Serial.println("[ERROR][Movement] Cannot initialise HTTP request");
        return false;
    }

    const char* headerKeys[] = {"Movement-ID"};
    http.collectHeaders(headerKeys, 1);
    const int statusCode = http.GET();
    if (statusCode == HTTP_CODE_OK) {
        movementId = http.header("Movement-ID");
        packet = http.getString();
        http.end();
        if (movementId.length() == 0) {
            Serial.println("[ERROR][Movement] Backend returned no Movement-ID");
            return false;
        }
        if (packet.length() == 0) {
            Serial.println("[ERROR][Movement] Backend returned an empty packet");
            return false;
        }
        return true;
    }
    if (statusCode != HTTP_CODE_NO_CONTENT) {
        Serial.printf("[ERROR][Movement] GET failed: %d (%s)\n", statusCode,
            statusCode < 0 ? HTTPClient::errorToString(statusCode).c_str() : "HTTP response");
    }
    http.end();
    return false;
}

// Retry this acknowledgement before asking for another movement packet.
bool SendMovementComplete(const String& movementId)
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ERROR][Movement] WiFi disconnected; completion pending");
        return false;
    }
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    const String url = "https://" + String(HOST) + "/movement-complete";
    if (!http.begin(client, url)) {
        Serial.println("[ERROR][Movement] Cannot initialise completion request");
        return false;
    }
    http.addHeader("Movement-ID", movementId);
    const int statusCode = http.POST("");
    http.end();
    if (statusCode != HTTP_CODE_NO_CONTENT) {
        Serial.printf("[ERROR][Movement] Completion POST failed: %d; will retry\n", statusCode);
        return false;
    }
    Serial.printf("[Movement] Completion acknowledged: %s\n", movementId.c_str());
    return true;
}
