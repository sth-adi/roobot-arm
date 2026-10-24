#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "MotionPlayback.h"

namespace {
int failures = 0;
constexpr uint64_t IntervalUs = MotionControlIntervalMs * 1000ULL;
struct Write {
    size_t sample;
    uint64_t time;
};

void Check(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void MakeMovement(PlannedMovement& movement, size_t count = 6)
{
    movement.Clear();
    movement.frames = static_cast<ServoFrame*>(std::malloc(count * sizeof(ServoFrame)));
    if (!movement.frames) std::abort();
    movement.frameCount = count;
    for (size_t index = 0; index < count; ++index) {
        for (int joint = 0; joint < 4; ++joint) {
            movement.frames[index].pulseUs[joint] = static_cast<uint16_t>(600 + index + joint * 100);
        }
    }
}

void CheckSequential(const std::vector<Write>& writes)
{
    for (size_t index = 0; index < writes.size(); ++index) {
        Check(writes[index].sample == index, "playback writes every frame once in order");
        if (index != 0) {
            Check(writes[index].time - writes[index - 1].time >= IntervalUs,
                  "adjacent frame commands remain at least 20ms apart");
        }
    }
}

void TestNormalAndOutputTime(uint64_t outputTimeUs)
{
    ArduinoTest::ResetClock();
    PlannedMovement movement;
    MakeMovement(movement);
    std::vector<Write> writes;
    size_t lastFrame = 99;
    const bool completed = PlayMotionFrames(movement, [&](const ServoFrame& frame) {
        writes.push_back({static_cast<size_t>(frame.pulseUs[0] - 600), ArduinoTest::clockUs});
        ArduinoTest::clockUs += outputTimeUs;
    }, lastFrame);
    Check(completed, "normal playback completes");
    Check(writes.size() == movement.frameCount && lastFrame == movement.frameCount - 1,
          "completed playback reports the exact final frame");
    CheckSequential(writes);
    for (size_t index = 0; index < writes.size(); ++index) {
        Check(writes[index].time == index * IntervalUs,
              "short output execution does not accumulate scheduling drift");
    }
}

void TestNineteenMillisecondLateWake()
{
    ArduinoTest::ResetClock();
    // The first wait sleeps 19ms. A further 20ms pause wakes at 39ms,
    // exactly 19ms after the original 20ms deadline.
    ArduinoTest::nextDelayExtraUs = 20000;
    PlannedMovement movement;
    MakeMovement(movement);
    std::vector<Write> writes;
    size_t lastFrame = 99;
    const bool completed = PlayMotionFrames(movement, [&](const ServoFrame& frame) {
        writes.push_back({static_cast<size_t>(frame.pulseUs[0] - 600), ArduinoTest::clockUs});
    }, lastFrame);
    Check(completed, "wake less than one full period late continues playback");
    Check(writes.size() == movement.frameCount && lastFrame == movement.frameCount - 1,
          "late wake still outputs every sequential sample");
    CheckSequential(writes);
    if (writes.size() >= 3) {
        Check(writes[1].time == 39000, "simulated wake is exactly 19ms late");
        Check(writes[2].time == 59000, "late wake rebases the following deadline without catch-up");
    }
}

void TestFullPeriodLateStop(uint64_t extraPauseUs)
{
    ArduinoTest::ResetClock();
    PlannedMovement movement;
    MakeMovement(movement);
    std::vector<Write> writes;
    size_t lastFrame = 99;
    const bool completed = PlayMotionFrames(movement, [&](const ServoFrame& frame) {
        const size_t index = static_cast<size_t>(frame.pulseUs[0] - 600);
        writes.push_back({index, ArduinoTest::clockUs});
        if (index == 1) ArduinoTest::nextDelayExtraUs = extraPauseUs;
    }, lastFrame);
    Check(!completed, "wake at least one complete period late stops playback");
    Check(writes.size() == 2 && lastFrame == 1,
          "late stop retains the last sample and never writes the overdue sample");
    CheckSequential(writes);
}

void TestMicrosRollover()
{
    const uint64_t begin = static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) - 10000;
    ArduinoTest::ResetClock(begin);
    PlannedMovement movement;
    MakeMovement(movement);
    std::vector<Write> writes;
    size_t lastFrame = 99;
    const bool completed = PlayMotionFrames(movement, [&](const ServoFrame& frame) {
        writes.push_back({static_cast<size_t>(frame.pulseUs[0] - 600), ArduinoTest::clockUs});
        ArduinoTest::clockUs += 100;
    }, lastFrame);
    Check(completed, "playback completes across micros() 32-bit rollover");
    Check(writes.size() == movement.frameCount && lastFrame == movement.frameCount - 1,
          "rollover preserves all sequential frames");
    CheckSequential(writes);
    for (size_t index = 0; index < writes.size(); ++index) {
        Check(writes[index].time == begin + index * IntervalUs,
              "rollover preserves exact scheduling intervals");
    }
}

void TestInvalidMovement()
{
    ArduinoTest::ResetClock();
    PlannedMovement movement;
    size_t lastFrame = 99;
    size_t writes = 0;
    Check(!PlayMotionFrames(movement, [&](const ServoFrame&) { ++writes; }, lastFrame),
          "empty trajectory is rejected");
    Check(writes == 0 && lastFrame == 99, "empty trajectory writes no servo commands");
    MakeMovement(movement, 1);
    Check(!PlayMotionFrames(movement, [&](const ServoFrame&) { ++writes; }, lastFrame),
          "single-frame trajectory is rejected");
    Check(writes == 0 && lastFrame == 99, "incomplete trajectory writes no servo commands");
}
}

int main()
{
    TestNormalAndOutputTime(0);
    TestNormalAndOutputTime(100);
    TestNineteenMillisecondLateWake();
    TestFullPeriodLateStop(21000); // 19ms normal delay + 21ms pause = 20ms late.
    TestFullPeriodLateStop(61000); // Larger scheduling stall must also hold.
    TestMicrosRollover();
    TestInvalidMovement();
    if (failures) {
        std::fprintf(stderr, "%d motion playback checks failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("Motion playback checks passed.");
    return EXIT_SUCCESS;
}
