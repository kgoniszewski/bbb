// Warm-up at prepare (engine.h, setWarmUpOnPrepare): prepare() renders a
// short scene on the preparing thread and resets. The engine must then be
// exactly a freshly prepared one: bit-identical output and identical
// statistics, also after a second prepare of the same instance.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
constexpr int kBlock = 64;
const int kN = static_cast<int>(1.5 * kFs);

std::vector<pm::Event> scene() {
    auto at = [](double s) { return static_cast<std::uint64_t>(s * kFs); };
    return {noteOn(at(0.0), 21, 70),  noteOn(at(0.01), 60, 100), noteOn(at(0.02), 108, 60),
            sustain(at(0.2), true),   noteOn(at(0.25), 45, 90),  noteOff(at(0.4), 60),
            noteOff(at(0.45), 45),    sustain(at(0.8), false),   noteOff(at(0.9), 21),
            noteOn(at(1.0) + 7, 72, 110), noteOff(at(1.3), 72),  noteOff(at(1.3), 108)};
}

struct Run {
    std::vector<float> l, r;
    pm::EngineStats stats;
    pm::RenderInfo last;
    int strikes = 0;
};

void play(pm::Engine& eng, Run& out) {
    const std::vector<pm::Event> ev = scene();
    out.l.assign(kN, 0.0f);
    out.r.assign(kN, 0.0f);
    std::size_t next = 0;
    for (int pos = 0; pos < kN; pos += kBlock) {
        const int n = std::min(kBlock, kN - pos);
        while (next < ev.size() && ev[next].time < static_cast<std::uint64_t>(pos + n)) eng.pushEvent(ev[next++]);
        eng.render(out.l.data() + pos, out.r.data() + pos, n, nullptr);
    }
    out.stats = eng.stats();
    out.last = eng.lastRender();
    pm::StrikeInfo si;
    while (eng.popStrikeInfo(si)) ++out.strikes;
}

bool same(const Run& a, const Run& b) {
    const pm::EngineStats &x = a.stats, &y = b.stats;
    return std::memcmp(a.l.data(), b.l.data(), a.l.size() * sizeof(float)) == 0 &&
           std::memcmp(a.r.data(), b.r.data(), a.r.size() * sizeof(float)) == 0 && x.totalModes == y.totalModes &&
           x.groups == y.groups && x.awakeGroups == y.awakeGroups && x.awakeModes == y.awakeModes &&
           x.activeHammers == y.activeHammers && x.peakActiveHammers == y.peakActiveHammers &&
           x.droppedEvents == y.droppedEvents && x.deferredStrikes == y.deferredStrikes &&
           x.lateEvents == y.lateEvents && x.limitedSamples == y.limitedSamples &&
           x.nonFiniteSamples == y.nonFiniteSamples && x.peakPreLimiter == y.peakPreLimiter &&
           std::memcmp(&a.last, &b.last, sizeof(pm::RenderInfo)) == 0 && a.strikes == b.strikes;
}

void checkRoom(bool room) {
    auto cold = std::make_unique<pm::Engine>();
    auto warm = std::make_unique<pm::Engine>();
    cold->setWarmUpOnPrepare(false);
    cold->setRoomEnabled(room);
    warm->setRoomEnabled(room);
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();
    CHECK(cold->prepare(kFs, kBlock));
    auto t1 = Clock::now();
    CHECK(warm->prepare(kFs, kBlock));
    auto t2 = Clock::now();
    std::printf("  room %d: prepare %.1f ms without warm-up, %.1f ms with\n", int(room),
                std::chrono::duration<double, std::milli>(t1 - t0).count(),
                std::chrono::duration<double, std::milli>(t2 - t1).count());
    // Fresh statistics before anything is played.
    const pm::EngineStats s = warm->stats();
    CHECK(s.activeHammers == 0 && s.peakActiveHammers == 0 && s.awakeGroups == 0 && s.peakPreLimiter == 0.0f);
    CHECK(warm->sampleTime() == 0 && warm->sustain() == 0.0f && warm->pedalPosition() == 0.0f);
    pm::StrikeInfo si;
    CHECK(!warm->popStrikeInfo(si));

    Run a, b, c;
    play(*cold, a);
    play(*warm, b);
    CHECK(a.strikes > 0);
    CHECK(same(a, b));
    // Re-prepare after playing, with warm-up: still a fresh engine.
    CHECK(warm->prepare(kFs, kBlock));
    play(*warm, c);
    CHECK(same(a, c));
}

}  // namespace

TEST(warm_up_is_invisible_room_off) { checkRoom(false); }
TEST(warm_up_is_invisible_room_on) { checkRoom(true); }
