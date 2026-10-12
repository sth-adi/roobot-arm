// Native check of ArmMotion.h / ServoPulseMicros / DecodeStickInput. Build and run with sim/test_firmware.py.
//   stdin : lines "dt LSX LSY RSX RSY A B rate fresh"    (rate: deg/s, 0 = no limit; fresh: 0 = no input)
//   stdout: per tick "a1 a2 a3 a4 mode attached ikok events saved"  (angles in degrees)
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "ArmMotion.h"
#include "WifiFunctions.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

static void selfTests()
{
    // ServoPulseMicros: same mapping as ESP32Servo write() with attach(pin, 500, 2400)
    for (int i = 0; i < 4; ++i) { JointLims[i][0] = 0; JointLims[i][1] = 180; }
    CHECK(ServoPulseMicros(0, 90.0f) == 1450);
    CHECK(ServoPulseMicros(0, 0.0f) == 500);
    CHECK(ServoPulseMicros(0, 180.0f) == 2400);
    CHECK(ServoPulseMicros(0, 200.0f) == 2400);
    CHECK(ServoPulseMicros(0, -5.0f) == 500);
    for (int a = 0; a <= 180; ++a) {         // whole degrees match the library's own write() to within 1 us
        Servo s; s.attach(1, 500, 2400); s.write(a);
        CHECK(abs(ServoPulseMicros(0, (float)a) - s.lastUs) <= 1);   // the library's map() truncates; we round
    }
    JointLims[1][0] = 150; JointLims[1][1] = 30;      // reversed range = inverted servo
    CHECK(abs(ServoPulseMicros(1, 30.0f) - 2083) <= 1);
    CHECK(abs(ServoPulseMicros(1, 150.0f) - 817) <= 1);
    CHECK(ServoPulseMicros(1, 10.0f) == ServoPulseMicros(1, 30.0f));   // clamped to the calibrated range
    JointLims[1][0] = 0; JointLims[1][1] = 180;

    // DecodeStickInput: optional maxSpeed, clamped to 1..hard cap
    StickInput in = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(DecodeStickInput(String("{\"LSX\":0.5,\"LSY\":-1,\"RSX\":0,\"RSY\":0,\"A\":true,\"B\":false,\"X\":false,\"Y\":false}"), in));
    CHECK(in.leftX == 0.5f && in.leftY == -1.0f && in.aButton == 1 && in.bButton == 0 && in.maxSpeed == 0.0f);
    CHECK(DecodeStickInput(String("{\"LSX\":0,\"LSY\":0,\"RSX\":0,\"RSY\":0,\"A\":false,\"B\":false,\"X\":false,\"Y\":false,\"maxSpeed\":25}"), in));
    CHECK(in.maxSpeed == 25.0f);
    CHECK(DecodeStickInput(String("{\"LSX\":0,\"LSY\":0,\"RSX\":0,\"RSY\":0,\"A\":false,\"B\":false,\"X\":false,\"Y\":false,\"maxSpeed\":500}"), in));
    CHECK(in.maxSpeed == JointSpeedHardCap);
    CHECK(DecodeStickInput(String("{\"LSX\":0,\"LSY\":0,\"RSX\":0,\"RSY\":0,\"A\":false,\"B\":false,\"X\":false,\"Y\":false,\"maxSpeed\":0.2}"), in));
    CHECK(in.maxSpeed == 1.0f);
    CHECK(DecodeStickInput(String("{\"LSX\":0,\"LSY\":0,\"RSX\":0,\"RSY\":0,\"A\":false,\"B\":false,\"X\":false,\"Y\":false,\"maxSpeed\":-3}"), in));
    CHECK(in.maxSpeed == 0.0f);
    CHECK(!DecodeStickInput(String("{}"), in));

    // Tool length: home pose is x 0, y 246.7, z 219.95 with the 63 mm tool
    for (int i = 0; i < 4; ++i) JointHomes[i] = 90;
    ArmMotion arm; arm.Reset(HomeJointAngles());
    RobotPosition p = arm.Pose();
    CHECK(fabsf(p.x) < 0.01f && fabsf(p.y - 246.7f) < 0.05f && fabsf(p.z - 219.95f) < 0.05f && fabsf(p.angle) < 0.01f);
}

int main(int argc, char** argv)
{
    selfTests();
    if (argc > 1 && std::string(argv[1]) == "--selftest") { printf(failures ? "FAILED\n" : "OK\n"); return failures != 0; }

    for (int i = 0; i < 4; ++i) { JointLims[i][0] = 0; JointLims[i][1] = 180; JointHomes[i] = 90; }
    ArmMotion arm; arm.Reset(HomeJointAngles());
    double dt, v[6]; double rate; int fresh;
    while (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %d", &dt, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rate, &fresh) == 9) {
        StickInput in = {(float)v[0], (float)v[1], (float)v[2], (float)v[3], (int)v[4], (int)v[5], 0, 0, 0};
        uint8_t ev = arm.Step(fresh ? &in : nullptr, (float)rate, (float)dt);
        printf("%.4f %.4f %.4f %.4f %d %d %d %d %d\n", degrees(arm.angles.s1), degrees(arm.angles.s2),
               degrees(arm.angles.s3), degrees(arm.angles.s4), (int)arm.mode, arm.attached ? 1 : 0,
               arm.ikOk ? 1 : 0, ev, arm.savedCount);
    }
    return failures != 0;
}
