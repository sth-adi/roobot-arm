#pragma once

#include "WifiFunctions.h"

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
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    const String calibrationUrl = "https://" + String(HOST) + "/calibration";
    if (!http.begin(client, calibrationUrl)) {
        return false;
    }

    const int statusCode = http.GET();
    if (statusCode != HTTP_CODE_OK) {
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
        return false;
    }

    const int limitsArrayStart = calibration.indexOf('[', limitsKey);
    if (limitsArrayStart < 0) {
        return false;
    }

    const char* cursor = calibration.c_str() + limitsArrayStart;
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (!ReadNextCalibrationValue(cursor, JointLims[jointIndex][0])
            || !ReadNextCalibrationValue(cursor, JointLims[jointIndex][1])) {
            return false;
        }
    }

    const int homesArrayStart = calibration.indexOf('[', homesKey);
    if (homesArrayStart < 0) {
        return false;
    }

    cursor = calibration.c_str() + homesArrayStart;
    for (int jointIndex = 0; jointIndex < 4; ++jointIndex) {
        if (!ReadNextCalibrationValue(cursor, JointHomes[jointIndex])) {
            return false;
        }
    }
    return true;
}

inline bool SaveCalibration()
{
    if (WiFi.status() != WL_CONNECTED) {
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
        return false;
    }

    http.addHeader("Content-Type", "application/json");
    const int statusCode = http.POST(payload);
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
