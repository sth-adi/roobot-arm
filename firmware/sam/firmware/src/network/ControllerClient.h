#pragma once

#include "BackendConnection.h"
#include "../control/ControllerProtocol.h"

bool GetControllerState(StickInput& input)
{
    static WiFiClientSecure client;
    static HTTPClient http;
    static bool requestInitialised = false;
    static unsigned long lastRequestTime = 0;
    constexpr unsigned long pollIntervalMs = 20;

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        if (requestInitialised) {
            http.end();
            requestInitialised = false;
        }
        return false;
    }

    if (millis() - lastRequestTime < pollIntervalMs) {
        return false;
    }
    lastRequestTime = millis();

    if (!requestInitialised) {
        client.setInsecure();
        http.setReuse(true);

        const String controllerUrl = "https://" + String(HOST) + "/controller";
        if (!http.begin(client, controllerUrl)) {
            Serial.println("Failed to initialise controller request");
            return false;
        }

        requestInitialised = true;
    }

    const int statusCode = http.GET();
    if (statusCode == HTTP_CODE_OK) {
        const String state = http.getString();

        if (!DecodeStickInput(state, input)) {
            Serial.println("Invalid controller state");
            return false;
        }

        Serial.println("Controller packet received");
        return true;
    } else if (statusCode > 0) {
        Serial.printf("Controller HTTP error: %d\n", statusCode);
    } else {
        Serial.printf(
            "Controller GET failed: %s\n",
            HTTPClient::errorToString(statusCode).c_str()
        );
    }

    if (statusCode != HTTP_CODE_OK) {
        http.end();
        requestInitialised = false;
    }

    return false;
}
