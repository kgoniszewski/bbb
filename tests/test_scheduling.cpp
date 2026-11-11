// Sample-accurate event scheduling (plan §3.1, §4.2).
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;

std::vector<pm::Event> scene(std::uint64_t t) {
    // Note-ons, a pedal, note-offs: every event kind that changes the sound.
    return {noteOn(t, 60, 90),          noteOn(t + 5, 64, 70),
            sustain(t + 2000, true),    noteOff(t + 3000, 60),
            noteOn(t + 3001, 67, 110),  noteOff(t + 7000, 64),
            sustain(t + 12345, false),  noteOn(t + 12400, 60, 40),
            noteOff(t + 20000, 60),     simple(t + 22222, pm::EventType::AllNotesOff)};
}

}  // namespace

// The whole scene shifted by k samples inside a buffer gives the same
// output shifted by k. Exact without culling; with culling the sleep grid
// is absolute, so only culling-level differences (< -100 dB) remain.
TEST(event_offset_shifts_output) {
    const int n = 36000;
    for (bool cull : {false, true}) {
        pm::Engine a, b;
        a.setCulling(cull, -100.0f);
        b.setCulling(cull, -100.0f);
        // Damper-lift noise is a random signal whose realisation depends on
        // where the pedal event falls on the control grid.
        a.setNoiseLevels(a.knockLevel(), 0.0f);
        b.setNoiseLevels(b.knockLevel(), 0.0f);
        a.prepare(kFs, 64);
        const Stereo ref = renderEngine(a, scene(0), n);
        const double peak = peakAbs(ref.l);
        for (int k : {1, 17, 31, 32, 45, 63}) {
            b.prepare(kFs, 64);
            const Stereo s = renderEngine(b, scene(64 * 10 + k), n + 64 * 10 + k);
            double d = 0.0;
            for (int i = 0; i < n; ++i) {
                d = std::max(d, double(std::fabs(s.l[i + 640 + k] - ref.l[i])));
                d = std::max(d, double(std::fabs(s.r[i + 640 + k] - ref.r[i])));
            }
            bool silentBefore = true;
            for (int i = 0; i < 640 + k; ++i) silentBefore = silentBefore && s.l[i] == 0.0f;
            std::printf("  culling %s, offset %2d: max difference %.3g (%.1f dB re peak)\n",
                        cull ? "on " : "off", k, d, 20 * std::log10(d / peak + 1e-30));
            CHECK(silentBefore);
            // Damper envelopes update on an absolute 32-sample grid, so a
            // note-off moved by k changes the damping curve by under one
            // control period: a tiny, inaudible difference.
            CHECK_LE(20 * std::log10(d / peak + 1e-30), -60.0);
        }
    }
}

// Note-ons alone (no control-rate state involved) are shift-invariant to
// rounding. (Bit-exact before the block-wise soundboard loop: its forced
// response is summed per sub-block, so where an event cuts a sub-block
// changes the rounding, -100 to -110 dB re peak; asserted at -90 dB.)
TEST(note_on_offset_is_exact) {
    pm::Engine a, b;
    a.setCulling(false, -100.0f);
    b.setCulling(false, -100.0f);
    a.prepare(kFs, 64);
    const Stereo ref = renderEngine(a, {noteOn(0, 40, 90), noteOn(0, 75, 60)}, 24000);
    for (int k : {1, 13, 63}) {
        b.prepare(kFs, 64);
        const Stereo s = renderEngine(b, {noteOn(k, 40, 90), noteOn(k, 75, 60)}, 24000 + k);
        double d = 0.0;
        for (int i = 0; i < 24000; ++i) d = std::max(d, double(std::fabs(s.l[i + k] - ref.l[i])));
        const double db = 20.0 * std::log10(d / peakAbs(ref.l) + 1e-30);
        std::printf("  offset %d: max difference %.3g (%.1f dB re peak)\n", k, d, db);
        CHECK_LE(db, -90.0);
    }
}

// Rendering with different buffer sizes gives the same output to rounding
// (bit-identical before the block-wise soundboard loop, see above; a host
// buffer boundary cuts a sub-block like an event does).
TEST(block_size_invariance) {
    const int n = 30000;
    pm::Engine eng;
    eng.prepare(kFs, 512);
    const Stereo ref = renderEngine(eng, scene(100), n, 64);
    for (int block : {1, 7, 37, 128, 500}) {
        eng.prepare(kFs, 512);
        const Stereo s = renderEngine(eng, scene(100), n, block);
        double d = 0.0;
        for (int i = 0; i < n; ++i)
            d = std::max(d, std::max(double(std::fabs(s.l[i] - ref.l[i])), double(std::fabs(s.r[i] - ref.r[i]))));
        const double db = 20.0 * std::log10(d / peakAbs(ref.l) + 1e-30);
        std::printf("  block %3d: max difference %.1f dB re peak\n", block, db);
        CHECK_LE(db, -90.0);
    }
}

// Host-time events: lowest-latency mode puts due events at the start of the
// buffer and future ones at their sample; constant-latency mode adds the
// fixed delay and keeps the exact position.
TEST(host_time_mapping) {
    const double ticksPerSample = 1e9 / kFs;  // nanosecond clock
    const std::uint64_t start = 5'000'000'000ull;
    auto firstSound = [](const Stereo& s) {
        for (std::size_t i = 0; i < s.l.size(); ++i)
            if (s.l[i] != 0.0f) return static_cast<int>(i);
        return -1;
    };
    auto run = [&](pm::SchedulingMode mode, std::int64_t eventSampleOffset) {
        pm::Engine eng;
        eng.setSchedulingMode(mode);
        eng.setConstantLatencyFrames(64);
        eng.prepare(kFs, 64);
        // Render one buffer first so the engine clock is at 64.
        Stereo s;
        s.l.assign(64 * 4, 0.0f);
        s.r.assign(64 * 4, 0.0f);
        pm::ClockMapping clk{start, ticksPerSample};
        eng.render(s.l.data(), s.r.data(), 64, &clk);
        pm::Event e = noteOn(0, 60, 100);
        e.timeBase = pm::TimeBase::HostTime;
        const double hostT = double(start) + (64 + eventSampleOffset) * ticksPerSample;
        e.time = static_cast<std::uint64_t>(hostT + 0.5);
        eng.pushEvent(e);
        for (int b = 1; b < 4; ++b) {
            clk.hostTimeAtBufferStart = static_cast<std::uint64_t>(double(start) + 64.0 * b * ticksPerSample + 0.5);
            eng.render(s.l.data() + 64 * b, s.r.data() + 64 * b, 64, &clk);
        }
        return firstSound(s);
    };
    // A hammer launched at sample t sounds from sample t (its first output
    // sample is the string's response during that sample).
    const int lowPast = run(pm::SchedulingMode::LowestLatency, -20);
    const int lowNow = run(pm::SchedulingMode::LowestLatency, 10);
    const int constPast = run(pm::SchedulingMode::ConstantLatency, -20);
    const int constNow = run(pm::SchedulingMode::ConstantLatency, 10);
    std::printf("  lowest latency: past event -> %d, event at +10 -> %d; constant latency (64): "
                "past -> %d, +10 -> %d\n",
                lowPast, lowNow, constPast, constNow);
    CHECK(lowPast == 64);
    CHECK(lowNow == 74);
    CHECK(constPast == 64 + 64 - 20);
    CHECK(constNow == 64 + 64 + 10);
}

TEST(full_queue_drops_and_counts) {
    pm::Engine eng;
    eng.prepare(kFs, 64);
    int accepted = 0;
    for (int i = 0; i < pm::Engine::kQueueSize + 10; ++i) accepted += eng.pushEvent(noteOn(100000, 60, 1));
    CHECK(accepted == pm::Engine::kQueueSize);
    CHECK(eng.stats().droppedEvents == 10);
}
