#pragma once

#include "../network/BackendConnection.h"
#include "../config/RobotCharacteristics.h"

inline bool ReadNextCalibrationValue(const char*& cursor, int& value)
{
    while (*cursor != '\0' && *cursor != '-' && (*cursor < '0' || *cursor > '9')) {
        ++cursor;
    }
    if (*cursor == '\0') {
        return false;
    }

    char* valueEnd = nullptr;
    const long parsedValue = strtol(cursor, &valueEnd, 10);
    if (valueEnd == cursor || parsedValue < 0 || parsedValue > 180) {
        return false;
    }

    value = static_cast<int>(parsedValue);
    cursor = valueEnd;
    return true;
}

inline bool LoadCalibration()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ERROR][Calibration] WiFi disconnected");
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    const String calibrationUrl = "https://" + String(HOST) + "/calibration";
    if (!http.begin(client, calibrationUrl)) {
        Serial.println("[ERROR][Calibration] Cannot initialise HTTP request");
        return false;
    }

    const int statusCode = http.GET();
    if (statusCode != HTTP_CODE_OK) {
        Serial.printf("[ERROR][Calibration] GET failed: %d (%s)\n", statusCode,
            statusCode < 0 ? HTTPClient::errorToString(statusCode).c_str() : "HTTP response");
        http.end();
        return false;
    }

    const String calibration = http.getString();
    http.end();
    const int limitsKey = calibration.indexOf("\"jointLimits\"");
    int homesKey = calibration.indexOf("\"jointHomes\"");
    if (homesKey < 0) {
        homesKey = calibration.indexOf("\"joint0s\"");
    }
    if (limitsKey < 0 || homesKey < 0) {
        Serial.println("[ERROR][Calibration] Missing jointLimits or jointHomes");
        return false;
    }

    const int limitsArrayStart = calibration.indexOf('[', limitsKey);
    if (limitsArrayStart < 0) {
        Serial.println("[ERROR][Calibration] Missing jointLimits array");
        return false;
    }

    const char* cursor = calibration.c_str() + limitsArrayStart;
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (!ReadNextCalibrationValue(cursor, JointLims[jointIndex][0])
            || !ReadNextCalibrationValue(cursor, JointLims[jointIndex][1])) {
            Serial.printf("[ERROR][Calibration] Joint %d limits missing or outside 0-180\n", jointIndex + 1);
            return false;
        }
    }

    const int homesArrayStart = calibration.indexOf('[', homesKey);
    if (homesArrayStart < 0) {
        Serial.println("[ERROR][Calibration] Missing jointHomes array");
        return false;
    }

    cursor = calibration.c_str() + homesArrayStart;
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (!ReadNextCalibrationValue(cursor, JointHomes[jointIndex])) {
            Serial.printf("[ERROR][Calibration] Joint %d home missing or outside 0-180\n", jointIndex + 1);
            return false;
        }
    }
    return true;
}

inline bool SaveCalibration()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ERROR][Calibration] WiFi disconnected");
        return false;
    }

    String payload = "{\"jointLimits\":[";
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (jointIndex > 0) {
            payload += ',';
        }
        payload += "[" + String(JointLims[jointIndex][0]) + "," + String(JointLims[jointIndex][1]) + "]";
    }
    payload += "],\"jointHomes\":[";
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (jointIndex > 0) {
            payload += ',';
        }
        payload += String(JointHomes[jointIndex]);
    }
    payload += "],\"joint0s\":[";
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (jointIndex > 0) {
            payload += ',';
        }
        payload += String(JointHomes[jointIndex]);
    }
    payload += "]}";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    const String calibrationUrl = "https://" + String(HOST) + "/calibration";
    if (!http.begin(client, calibrationUrl)) {
        Serial.println("[ERROR][Calibration] Cannot initialise HTTP request");
        return false;
    }

    http.addHeader("Content-Type", "application/json");
    const int statusCode = http.POST(payload);
    if (statusCode != HTTP_CODE_NO_CONTENT) {
        Serial.printf("[ERROR][Calibration] Save failed: %d (%s)\n", statusCode,
            statusCode < 0 ? HTTPClient::errorToString(statusCode).c_str() : "HTTP response");
    }
    http.end();
    return statusCode == HTTP_CODE_NO_CONTENT;
}

inline String CalibrationStatus()
{
    String status = "Loaded calibration | Joint homes: ";
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (jointIndex > 0) {
            status += ", ";
        }
        status += String(JointHomes[jointIndex]);
    }
    return status;
}
