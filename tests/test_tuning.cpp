// Stretch tuning by the virtual tuner (plan §2.3).
#include <cmath>
#include <cstdio>

#include "pm_voicing/key_loader.h"
#include "pm_voicing/piano_table.h"
#include "pm_voicing/soundboard_model.h"
#include "pm_voicing/tuning.h"
#include "test_harness.h"

namespace {

double cents(const pm::PianoTable& t, int key) {
    return pm::centsFromEqualTemperament(t.keys[key - 1].f1, key, 440.0);
}

// Frequency of partial n of key k as tuned.
double partial(const pm::PianoTable& t, int key, int n) {
    const pm::KeyParams& k = t.keys[key - 1];
    return k.f1 * pm::partialRatio(k.B, n);
}

}  // namespace

// The deviation from equal temperament follows the reference Railsback-like
// curve (tuning.h) within 5 cents on every key, A4 is exact, and the curve
// rises monotonically (to 0.3 cent) from A0 to C8.
TEST(stretch_curve_matches_reference) {
    pm::PianoTable t;
    pm::buildPianoTable(t);
    double worst = 0.0, worstDip = 0.0, last = -1e9;
    int worstKey = 0;
    for (int k = 1; k <= pm::kNumKeys; ++k) {
        const double c = cents(t, k), r = pm::railsbackReferenceCents(k);
        if (std::fabs(c - r) > std::fabs(worst)) {
            worst = c - r;
            worstKey = k;
        }
        worstDip = std::max(worstDip, last - c);
        last = c;
    }
    std::printf("  deviation from 12-TET (cents):");
    for (int k : {1, 4, 16, 28, 40, 49, 52, 64, 76, 88}) std::printf(" key %d %+.1f (ref %+.1f);", k, cents(t, k), pm::railsbackReferenceCents(k));
    std::printf("\n  worst difference from the reference %+.2f cents (key %d), largest dip %.3f cents\n", worst,
                worstKey, worstDip);
    CHECK_LE(std::fabs(worst), 5.0);
    CHECK_LE(worstDip, 0.3);
    CHECK(t.keys[48].f1 == 440.0);
}

// Where one octave type is used alone, its partial pair coincides: 4:2 for
// the temperament octave and the mid range, 6:3 in the bass, 2:1 in the
// treble below the 4:1 blend.
TEST(octave_partials_coincide) {
    pm::PianoTable t;
    pm::buildPianoTable(t);
    double worst = 0.0;
    auto check = [&](int lo, int hi, int nLo, int nHi) {
        const double d = 1200.0 * std::log2(partial(t, hi, nHi) / partial(t, lo, nLo));
        worst = std::max(worst, std::fabs(d));
        return d;
    };
    for (int k = 29; k <= 37; ++k) check(k, k + 12, 4, 2);
    for (int k = 50; k <= 56; ++k) check(k - 12, k, 4, 2);
    for (int k = 2; k <= 18; ++k) {
        const pm::OctaveWeights w = pm::octaveWeights(k);
        if (w.w63 == 1.0) check(k, k + 12, 6, 3);
    }
    for (int k = 68; k <= 76; ++k) check(k - 12, k, 2, 1);
    std::printf("  A3-A4 4:2 beat %.4f cents; worst mismatch of pure-type octaves %.2e cents\n",
                1200.0 * std::log2(partial(t, 49, 2) / partial(t, 37, 4)), worst);
    CHECK_LE(worst, 0.1);
}

// Stretch amount 0 is equal temperament; the deviation scales linearly with
// the amount; A4 follows the reference pitch; equal temperament on request.
TEST(stretch_amount_and_reference_pitch) {
    pm::TableOptions o;
    pm::PianoTable full, none, more, et, a442;
    pm::buildPianoTable(full, o);
    o.stretch = 0.0;
    pm::buildPianoTable(none, o);
    o.stretch = 1.5;
    pm::buildPianoTable(more, o);
    o.stretch = 1.0;
    o.equalTemperament = true;
    pm::buildPianoTable(et, o);
    o.equalTemperament = false;
    o.a4Hz = 442.0;
    pm::buildPianoTable(a442, o);
    double worst0 = 0.0, worstScale = 0.0, worstEt = 0.0;
    for (int k = 1; k <= pm::kNumKeys; ++k) {
        worst0 = std::max(worst0, std::fabs(cents(none, k)));
        worstEt = std::max(worstEt, std::fabs(cents(et, k)));
        worstScale = std::max(worstScale, std::fabs(cents(more, k) - 1.5 * cents(full, k)));
    }
    std::printf("  stretch 0: worst %.2e cents from 12-TET; 150 %%: C8 %+.1f cents, worst deviation from "
                "1.5x %.2e; A4 at 442 Hz: %.6f Hz\n",
                worst0, cents(more, 88), worstScale, a442.keys[48].f1);
    CHECK_LE(worst0, 1e-9);
    CHECK_LE(worstEt, 1e-9);
    CHECK_LE(worstScale, 1e-9);
    CHECK_NEAR(a442.keys[48].f1, 442.0, 1e-9);
}

// With the soundboard, the engine re-runs the tuner on the sounding partials
// (key_loader.h tuneToSoundingPartials): bridge shifts and string-board
// hybrids included, the pure-type octave pairs coincide as they sound
// (each key at its compensated pitch, as the engine bakes it). The retune
// moves the targets by the board's pull on the partials (a first partial
// pulled by a nearby board mode relative to its upper partials propagates
// along the octave chain): reported.
TEST(octaves_coincide_on_sounding_partials) {
    pm::PianoTable t;
    pm::buildPianoTable(t);
    const pm::BoardModel& board = pm::defaultBoard();
    const pm::SoundboardBridge pre(board, 48000.0, 1.0, &t);
    pm::PianoTable tuned = t;
    pm::tuneToSoundingPartials(tuned, pre);
    const pm::SoundboardBridge bridge(board, 48000.0, 1.0, &tuned);
    double f0c[pm::kNumKeys];
    for (int k = 0; k < pm::kNumKeys; ++k)
        f0c[k] = pm::compensatedF0(tuned.keys[k], bridge);
    auto sounding = [&](int key, int n) { return pm::soundingPartial(tuned.keys[key - 1], f0c[key - 1], n, bridge).freqHz; };
    double worst = 0.0, maxShift = 0.0;
    int worstKey = 0, shiftKey = 0;
    for (int k = 1; k <= pm::kNumKeys; ++k)
        if (std::fabs(cents(tuned, k) - cents(t, k)) > std::fabs(maxShift)) {
            maxShift = cents(tuned, k) - cents(t, k);
            shiftKey = k;
        }
    auto check = [&](int lo, int hi, int nLo, int nHi) {
        const double d = 1200.0 * std::log2(sounding(hi, nHi) / sounding(lo, nLo));
        if (std::fabs(d) > std::fabs(worst)) {
            worst = d;
            worstKey = hi;
        }
    };
    for (int k = 29; k <= 37; ++k) check(k, k + 12, 4, 2);
    for (int k = 50; k <= 56; ++k) check(k - 12, k, 4, 2);
    for (int k = 2; k <= 18; ++k)
        if (pm::octaveWeights(k).w63 == 1.0) check(k, k + 12, 6, 3);
    for (int k = 68; k <= 76; ++k) check(k - 12, k, 2, 1);
    std::printf("  pure-type octaves on the sounding partials: worst mismatch %+.3f cents (key %d); largest change of "
                "a target by the retune %+.2f cents (key %d)\n",
                worst, worstKey, maxShift, shiftKey);
    CHECK_LE(std::fabs(worst), 0.2);
}
