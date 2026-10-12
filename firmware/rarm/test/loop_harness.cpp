// Runs the REAL main.cpp (setup() once, then loop() per tick) on a PC with fake time, a fake controller packet and
// recording servos. Which main.cpp is picked by the -I path: firmware/stock/src (as shipped) or a patched copy.
//   stdin : lines "dt LSX LSY RSX RSY A B rate fresh"     (dt must be 0.02: loop() ends with delay(20))
//   stdout: per tick  "servo1..servo4 attached1..attached4"  (servo = degrees written, -1 = never written)
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "main.cpp"

int main()
{
    fakeControllerBody() = "{\"jointLimits\":[[0,180],[0,180],[0,180],[0,180]],\"jointHomes\":[90,90,90,90]}";
    setup();

    double dt, v[6], rate; int fresh;
    while (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %d", &dt, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rate, &fresh) == 9) {
        if (fresh) {
            char b[256];
            snprintf(b, sizeof b, "{\"LSX\":%g,\"LSY\":%g,\"RSX\":%g,\"RSY\":%g,\"A\":%s,\"B\":%s,\"X\":false,\"Y\":false%s%s}",
                     v[0], v[1], v[2], v[3], v[4] > 0 ? "true" : "false", v[5] > 0 ? "true" : "false",
                     rate > 0 ? ",\"maxSpeed\":" : "", rate > 0 ? std::to_string(rate).c_str() : "");
            fakeControllerBody() = b;
        } else {
            fakeControllerBody() = "{}";            // no valid packet: GetControllerState() returns false
        }
        loop();
        for (int i = 0; i < 4; ++i) {
            const int us = jointServos[i].lastUs;
            printf("%d ", us == 0 ? -1 : (int)lround((us - 500) * 180.0 / 1900.0));
        }
        for (int i = 0; i < 4; ++i) printf("%d ", jointServos[i].attached() ? 1 : 0);
        printf("\n");
    }
    return 0;
}
