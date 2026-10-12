#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include "Secrets.h"
#include "SticksToXYZ.h"


bool ConnectToWiFi() {

    // Print a message to the Serial Monitor.
    Serial.println("Connecting to WiFi...");

    // Set WiFi to station mode so the board connects to a router.
    WiFi.mode(WIFI_STA);

    // Start connecting using the WiFi name and password.
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Record the start time in milliseconds to check for a timeout.
    unsigned long StartTime = millis();

    // Wait until the WiFi connection is established.
    while (WiFi.status() != WL_CONNECTED) {

        // Stop trying after 15 seconds and report failure.
        if (millis() - StartTime >= 15000) {
            Serial.println("\nWiFi connection failed!");
            return false;
        }

        // Wait half a second between connection checks.
        delay(500);

        // Print a dot to show that connection is still in progress.
        Serial.print(".");
    }

    // Report a successful connection.
    Serial.println("\nWiFi connected!");

    // Print the local IP address assigned to the board.
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Return true to indicate a successful connection.
    return true;
}

float TestLatency()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return -1.0f;
    }

    String pingHost = String(HOST);
    pingHost.replace("https://", "");
    pingHost.replace("http://", "");
    const int pathStart = pingHost.indexOf('/');
    if (pathStart >= 0) {
        pingHost.remove(pathStart);
    }

    // Ping only the host name; Ping cannot resolve a URL with a protocol or path.
    if (Ping.ping(pingHost.c_str(), 3)) {
        Serial.printf("Average latency: %.2f ms\n", Ping.averageTime());
        return (Ping.averageTime());
    } else {
        Serial.println("Ping failed");
        return -1.0f;
    }
}

String CheckControllerMode()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return "";
    }

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;

    // HOST should contain only the hostname, without https://.
    String controllerUrl = "https://" + String(HOST) + "/mode";

    if (!http.begin(client, controllerUrl)) {
        Serial.println("Failed to initialise request");
        return "";
    }

    const char* headerKeys[] = {"mode"};
    http.collectHeaders(headerKeys, 1);

    int statusCode = http.GET();
    String mode = "";

    if (statusCode == HTTP_CODE_OK) {
        mode = http.header("mode");

        Serial.printf("Controller mode: %s\n", mode.c_str());
    }
    else if (statusCode > 0) {
        Serial.printf("HTTP error: %d\n", statusCode);
    }
    else {
        Serial.printf(
            "GET failed: %s\n",
            HTTPClient::errorToString(statusCode).c_str()
        );
    }

    http.end();

    return mode;
}

bool ReadControllerValue(const String& state, const char* name, float& value)
{
    const String key = "\"" + String(name) + "\"";
    const int keyStart = state.indexOf(key);
    if (keyStart < 0) {
        return false;
    }

    const int valueStart = state.indexOf(':', keyStart + key.length());
    if (valueStart < 0) {
        return false;
    }

    char* valueEnd = nullptr;
    value = strtof(state.c_str() + valueStart + 1, &valueEnd);
    return valueEnd != state.c_str() + valueStart + 1;
}

bool ReadControllerButton(const String& state, const char* name, int& value)
{
    const String key = "\"" + String(name) + "\"";
    const int keyStart = state.indexOf(key);
    if (keyStart < 0) {
        return false;
    }

    const int valueStart = state.indexOf(':', keyStart + key.length());
    if (valueStart < 0) {
        return false;
    }

    String buttonValue = state.substring(valueStart + 1);
    buttonValue.trim();
    if (buttonValue.startsWith("true")) {
        value = 1;
        return true;
    }

    if (buttonValue.startsWith("false")) {
        value = 0;
        return true;
    }

    return false;
}

bool DecodeStickInput(const String& state, StickInput& input)
{
    return ReadControllerValue(state, "LSX", input.leftX)
        && ReadControllerValue(state, "LSY", input.leftY)
        && ReadControllerValue(state, "RSX", input.rightX)
        && ReadControllerValue(state, "RSY", input.rightY)
        && ReadControllerButton(state, "A", input.aButton)
        && ReadControllerButton(state, "B", input.bButton)
        && ReadControllerButton(state, "X", input.xButton)
        && ReadControllerButton(state, "Y", input.yButton);
}

bool GetControllerState(StickInput& input)
{
    static WiFiClientSecure client;
    static HTTPClient http;
    static bool requestInitialised = false;
    static String lastState;
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
        if (state != lastState) {
            lastState = state;
        }

        if (!DecodeStickInput(state, input)) {
            Serial.println("Invalid controller state");
            return false;
        }

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
