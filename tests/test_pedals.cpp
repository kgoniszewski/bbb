// Sostenuto (CC66) and una corda (CC67) in the engine (plan §1.9).
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;

pm::Event pedal(std::uint64_t t, pm::EventType type, double amount) {
    pm::Event e = simple(t, type);
    e.value = static_cast<std::uint16_t>(std::lround(amount * 65535.0));
    return e;
}

// Renders and returns the engine (for state inspection) and the output.
Stereo run(pm::Engine& eng, const std::vector<pm::Event>& ev, int n) {
    eng.prepare(kFs, 64);
    return renderEngine(eng, ev, n);
}

double levelDb(const Stereo& s, int from, int len) { return rmsDb(s.l, from, from + len); }

}  // namespace

// Keys held when the sostenuto goes down stay undamped after release; keys
// played afterwards are damped normally; releasing the pedal damps the
// latched keys. The sustain pedal is independent of it.
TEST(sostenuto_latches_held_keys) {
    pm::Engine eng;
    const int n = 3 * 48000;
    const Stereo s = run(eng,
                         {noteOn(0, 48, 90),                                   // C3 held
                          pedal(4800, pm::EventType::Sostenuto, 1.0),          // latch C3
                          noteOn(9600, 64, 90),                                // E4 after the pedal
                          noteOff(24000, 48), noteOff(24000, 64),              // release both
                          pedal(96000, pm::EventType::Sostenuto, 0.0)},        // C3 damped
                         n);
    // Isolated renders of each note with the same key timing, no pedal.
    pm::Engine a, b;
    const Stereo c3 = run(a, {noteOn(0, 48, 90), noteOff(24000, 48)}, n);
    const Stereo e4 = run(b, {noteOn(9600, 64, 90), noteOff(24000, 64)}, n);
    pm::Engine c;
    const Stereo c3held = run(c, {noteOn(0, 48, 90)}, n);
    const double latched = levelDb(s, 60000, 9600), heldRef = levelDb(c3held, 60000, 9600);
    const double dampedRef = levelDb(c3, 60000, 9600);
    std::printf("  1.25 s (keys up, sostenuto down): %.1f dB; C3 held alone %.1f dB; C3 released alone %.1f dB\n",
                latched, heldRef, dampedRef);
    CHECK_NEAR(latched, heldRef, 0.5);  // C3 rings as if held, E4 is gone
    CHECK(dampedRef < heldRef - 40.0);
    CHECK(levelDb(e4, 60000, 9600) < heldRef - 40.0);
    const double after = levelDb(s, 96000 + 24000, 9600);
    std::printf("  0.5 s after the sostenuto is released: %.1f dB\n", after);
    CHECK(after < heldRef - 40.0);

    // State: only the key held at the moment of pressing is latched; a key
    // whose damper is up only because of the sustain pedal is not.
    pm::Engine d;
    d.prepare(kFs, 64);
    renderEngine(d, {sustain(0, true), noteOn(10, 50, 80), noteOff(100, 50), noteOn(200, 60, 80),
                     pedal(1000, pm::EventType::Sostenuto, 1.0)},
                 2000);
    CHECK(d.sostenutoDown());
    CHECK(d.sostenutoLatched(60 - 21));
    CHECK(!d.sostenutoLatched(50 - 21));
    // Keys without dampers are never latched.
    pm::Engine e;
    e.prepare(kFs, 64);
    renderEngine(e, {noteOn(0, 100, 80), pedal(100, pm::EventType::Sostenuto, 1.0)}, 1000);
    CHECK(!e.sostenutoLatched(100 - 21));
    // Pressing the sostenuto again while it is down does not re-latch.
    pm::Engine f;
    f.prepare(kFs, 64);
    renderEngine(f, {pedal(0, pm::EventType::Sostenuto, 1.0), noteOn(100, 60, 80),
                     pedal(200, pm::EventType::Sostenuto, 0.9)},
                 1000);
    CHECK(!f.sostenutoLatched(60 - 21));
}

// Moving the una corda pedal between notes never changes a sounding note
// (it is latched per strike): the output is the same until the next strike
// (to rounding: the pedal event cuts a sub-block of the block-wise
// soundboard loop, about -110 dB re peak, asserted at -90 dB; bit-identical
// before that loop),
// which then differs.
TEST(una_corda_change_is_click_free) {
    const int n = 2 * 48000;
    pm::Engine a, b;
    const Stereo ref = run(a, {noteOn(0, 60, 90), noteOn(48000, 64, 90)}, n);
    const Stereo s = run(b, {noteOn(0, 60, 90), pedal(24000 + 7, pm::EventType::UnaCorda, 1.0),
                             pedal(30000, pm::EventType::UnaCorda, 0.4), noteOn(48000, 64, 90)},
                         n);
    double before = 0.0;
    for (int i = 0; i < 48000; ++i)
        before = std::max(before, std::max(double(std::fabs(s.l[i] - ref.l[i])), double(std::fabs(s.r[i] - ref.r[i]))));
    double diff = 0.0;
    for (int i = 48000; i < n; ++i) diff = std::max(diff, double(std::fabs(s.l[i] - ref.l[i])));
    const double beforeDb = 20.0 * std::log10(before / peakAbs(ref.l) + 1e-30);
    std::printf("  before the next strike: max difference %.1f dB re peak; after it max difference %.3g\n", beforeDb, diff);
    CHECK_LE(beforeDb, -90.0);
    CHECK(diff > 1e-4);
    CHECK(std::fabs(b.unaCorda() - 0.4f) < 1e-4f);
}

// Panic resets both pedals.
TEST(panic_resets_pedals) {
    pm::Engine e;
    e.prepare(kFs, 64);
    renderEngine(e, {noteOn(0, 60, 80), pedal(10, pm::EventType::Sostenuto, 1.0),
                     pedal(20, pm::EventType::UnaCorda, 1.0), simple(100, pm::EventType::Panic)},
                 1000);
    CHECK(!e.sostenutoDown());
    CHECK(!e.sostenutoLatched(60 - 21));
    CHECK(e.unaCorda() == 0.0f);
}
