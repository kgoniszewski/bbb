// Dense random performance: the engine stays bounded and hammers return to
// the pool.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

TEST(dense_random_events_stay_bounded) {
    constexpr double kFs = 48000.0;
    const int seconds = 30;
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> note(21, 108), vel(1, 127), gap(0, 1500), kind(0, 99);
    std::vector<pm::Event> ev;
    std::uint64_t t = 0;
    bool ped = false;
    while (t < static_cast<std::uint64_t>((seconds - 2) * kFs)) {
        t += static_cast<std::uint64_t>(gap(rng));
        const int k = kind(rng);
        if (k < 55) {
            // Sometimes a chord: several note-ons at the same sample.
            const int count = k < 10 ? 6 : 1;
            for (int c = 0; c < count; ++c) ev.push_back(noteOn(t, note(rng), vel(rng)));
        } else if (k < 92) {
            ev.push_back(noteOff(t, note(rng)));
        } else if (k < 98) {
            ped = !ped;
            ev.push_back(sustain(t, ped));
        } else if (k == 98) {
            ev.push_back(simple(t, pm::EventType::AllNotesOff));
        } else {
            ev.push_back(simple(t, pm::EventType::Panic));
        }
    }
    // A glissando with pedal, the worst case for the hammer pool.
    for (int n = 21; n <= 108; ++n) ev.push_back(noteOn(t + 200 + n * 20, n, 127));
    ev.push_back(simple(t + 4000, pm::EventType::AllNotesOff));
    ev.push_back(sustain(t + 4000, false));

    pm::Engine eng;
    eng.prepare(kFs, 128);
    const Stereo s = renderEngine(eng, ev, static_cast<int>(seconds * kFs), 128);
    bool finite = true;
    double peak = 0.0;
    for (std::size_t i = 0; i < s.l.size(); ++i) {
        finite = finite && std::isfinite(s.l[i]) && std::isfinite(s.r[i]);
        peak = std::max({peak, double(std::fabs(s.l[i])), double(std::fabs(s.r[i]))});
    }
    const pm::EngineStats st = eng.stats();
    std::printf("  %zu events, %d s: peak %.3f, peak before limiter %.3f, limited samples %llu "
                "(%.3f %%), peak hammers %d, deferred strikes %llu, non-finite %llu, free hammers %d\n",
                ev.size(), seconds, peak, st.peakPreLimiter,
                static_cast<unsigned long long>(st.limitedSamples),
                100.0 * double(st.limitedSamples) / (2.0 * s.l.size()), st.peakActiveHammers,
                static_cast<unsigned long long>(st.deferredStrikes),
                static_cast<unsigned long long>(st.nonFiniteSamples), eng.freeHammers());
    CHECK(finite);
    CHECK(peak < 1.0);
    CHECK(st.nonFiniteSamples == 0);
    CHECK(st.droppedEvents == 0);
    CHECK(eng.freeHammers() == pm::Engine::kHammerPool);
    CHECK(st.activeHammers == 0);
}
