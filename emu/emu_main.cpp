// The ESP32 firmware (its real main.cpp, picked by the -I path) running on this PC in real time:
//   * real sockets to EMU_HOST (the local server, or the network proxy in front of it)
//   * servo commands printed as JSON lines on stdout whenever a servo changes
//   * network/loop timing statistics printed as "STATS {json}" lines on stderr
// Environment: EMU_HOST=host:port  EMU_SECONDS=run time (default: until killed)  EMU_SERIAL=1 shows the firmware's Serial output.
#define EMU_REALTIME 1
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <signal.h>
#include "main.cpp"

static volatile sig_atomic_t running = 1;
static void onSignal(int) { running = 0; }

static double percentile(std::vector<float> v, double p)
{
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, (size_t)(p * v.size()))];
}

static void printStats(double maxLoopGapMs)
{
    const EmuNetStats& s = emuNet();
    fprintf(stderr, "STATS {\"polls\":%u,\"poll_fails\":%u,\"connects\":%u,\"poll_ms_mean\":%.1f,\"poll_ms_p95\":%.1f,\"poll_ms_max\":%.1f,"
                    "\"other_requests\":%u,\"other_fails\":%u,\"other_ms_max\":%.1f,\"connect_ms_max\":%.1f,\"loop_gap_ms_max\":%.1f}\n",
            s.polls, s.pollFails, s.connects, s.polls ? s.pollMsSum / s.polls : 0.0, percentile(s.pollMs, 0.95), s.pollMsMax,
            s.others, s.otherFails, s.otherMsMax, s.connectMsMax, maxLoopGapMs);
    fflush(stderr);
}

static void printServos()
{
    EmuServoState& s = emuServos();
    int deg[4];
    for (int i = 0; i < 4; ++i) deg[i] = s.us[i] ? (int)lround((s.us[i] - 500) * 180.0 / 1900.0) : -1;
    printf("{\"t\":%lu,\"servo\":[%d,%d,%d,%d],\"attached\":[%d,%d,%d,%d]}\n", millis(), deg[0], deg[1], deg[2], deg[3],
           s.attached[0], s.attached[1], s.attached[2], s.attached[3]);
    fflush(stdout);
    s.dirty = false;
}

int main()
{
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);
#ifdef SIGPIPE
    signal(SIGPIPE, SIG_IGN);
#endif
    const char* secs = getenv("EMU_SECONDS");
    const unsigned long endAt = secs ? millis() + (unsigned long)(atof(secs) * 1000) : 0;

    setup();
    unsigned long lastStart = 0, lastStats = millis();
    double maxGap = 0;   // longest time between two loop() starts: the 20 ms delay at the end of loop() plus anything that blocked
    while (running && (endAt == 0 || millis() < endAt)) {
        const unsigned long start = millis();
        if (lastStart) maxGap = std::max(maxGap, (double)(start - lastStart));
        lastStart = start;
        loop();
        if (emuServos().dirty) printServos();
        if (millis() - lastStats >= 5000) { printStats(maxGap); lastStats = millis(); }
    }
    printStats(maxGap);
    return 0;
}
