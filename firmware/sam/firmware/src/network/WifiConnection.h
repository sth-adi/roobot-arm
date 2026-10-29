#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include "../config/Secrets.h"

bool ConnectToWiFi() {

    // Print a message to the Serial Monitor.
    Serial.println("Connecting to WiFi...");

    // Set WiFi to station mode so the board connects to a router.
    WiFi.mode(WIFI_STA);

    // Use WPA2 Enterprise (PEAP) only when a username is configured. An empty
    // username indicates a standard WPA/WPA2 personal network.
    if (WIFI_User != nullptr && WIFI_User[0] != '\0') {
        WiFi.begin(WIFI_SSID, WPA2_AUTH_PEAP, WIFI_User, WIFI_User, WIFI_Password);
    } else {
        WiFi.begin(WIFI_SSID, WIFI_Password);
    }

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
