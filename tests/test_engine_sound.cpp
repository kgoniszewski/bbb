// Engine-level sound checks: brightness and level against velocity, tuning
// of every key, re-striking a sounding string.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "signal_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;

}  // namespace

// Level over the first 200 ms rises monotonically with velocity on every
// key, and the spectral centroid over the first 50 ms from the bass to C5.
// Above that only 3-12 partials are left below 0.45 fs; the centroid is then
// pinned near the first partials and saturates at high velocity (printed,
// not checked).
TEST(centroid_and_level_rise_with_velocity) {
    const int vels[] = {16, 32, 48, 64, 80, 96, 112, 127};
    pm::Engine eng;
    for (int note : {24, 36, 48, 60, 72, 84, 96, 108}) {
        double lastC = 0.0, lastL = -1e9;
        bool monoC = true, monoL = true;
        std::printf("  note %3d:", note);
        for (int v : vels) {
            eng.prepare(kFs, 64);
            const Stereo s = renderEngine(eng, {noteOn(0, note, v)}, 9600);
            std::vector<float> mono(s.l.size());
            for (std::size_t i = 0; i < mono.size(); ++i) mono[i] = s.l[i] + s.r[i];
            const double c = spectralCentroid(mono, 0, 2400, kFs);
            const double l = rmsDb(mono, 0, 9600);
            std::printf(" %d:%.0fHz/%.1fdB", v, c, l);
            monoC = monoC && c > lastC;
            monoL = monoL && l > lastL;
            lastC = c;
            lastL = l;
        }
        std::printf("\n");
        CHECK(monoL);
        if (note <= 72) CHECK(monoC);
    }
}

// The sounding first partial of every key is at the tuner's target
// (stretch tuning, A4 = 440 Hz) despite the bridge reactance, which the key
// loader compensates: within 0.5 cent with unison width 0 (strings in
// unison). With the default detuned unisons the partial is a beating
// cluster of normal modes 0.05-1 Hz apart; the phase slope of the sum then
// wanders by up to about 1.5 cent over the 1 s window, so 2 cents are allowed.
TEST(every_key_in_tune) {
    for (double width : {0.0, 1.0}) {
        pm::TableOptions o;
        o.unisonWidth = width;
        pm::PianoTable table;
        pm::buildPianoTable(table, o);
        const double tol = width == 0.0 ? 0.5 : 2.0;
        pm::Engine eng;
        double worst = 0.0;
        int worstKey = 0;
        for (int key = 1; key <= 88; ++key) {
            eng.prepare(kFs, 64, table);
            const double f1 = table.keys[key - 1].f1;
            const int n = static_cast<int>(kFs * (key < 30 ? 2.0 : 1.0));
            const Stereo s = renderEngine(eng, {noteOn(0, key + 20, 110)}, n);
            std::vector<float> tail(s.l.begin() + 2400, s.l.end());
            const pm_test::PartialFit fit = pm_test::measurePartial(tail, kFs, f1, f1, 50.0);
            const double c = fit.frames >= 3 ? pm_test::cents(fit.freqHz, f1) : 1e9;
            if (std::fabs(c) > std::fabs(worst)) {
                worst = c;
                worstKey = key;
            }
            if (!(std::fabs(c) <= tol)) std::printf("  key %d: %.3f cents (%d frames)\n", key, c, fit.frames);
            CHECK_LE(std::fabs(c), tol);
        }
        std::printf("  unison width %.0f: worst first-partial error %.4f cents (key %d)\n", width, worst,
                    worstKey);
    }
}

// Re-striking a sounding string: no step at the event beyond the signal's
// own motion, no state reset (the result differs from a strike on a silent
// string and depends on the string's phase).
TEST(restrike_depends_on_phase_without_click) {
    const int note = 60;
    const double f1 = 261.6256;
    const int t0 = 24000 + 7;  // 0.5 s, off the block and control grids
    pm::Engine eng;
    eng.prepare(kFs, 64);
    const Stereo fresh = renderEngine(eng, {noteOn(0, note, 80)}, 24000);
    std::vector<double> levels;
    for (int q = 0; q < 8; ++q) {
        const int t = t0 + static_cast<int>(std::lround(q * kFs / f1 / 8.0));
        eng.prepare(kFs, 64);
        const Stereo s = renderEngine(eng, {noteOn(0, note, 100), noteOn(t, note, 80)}, t + 24000);
        // The string's own slope just before the event.
        double ownStep = 0.0;
        for (int k = t - 960; k < t; ++k) ownStep = std::max(ownStep, double(std::fabs(s.l[k + 1] - s.l[k])));
        double stepAt = 0.0;
        for (int k = t - 1; k < t + 2; ++k) stepAt = std::max(stepAt, double(std::fabs(s.l[k + 1] - s.l[k])));
        // Level of the restruck note, and its difference from a fresh strike.
        std::vector<float> after(s.l.begin() + t, s.l.end());
        const double lev = rmsDb(after, 2400, 24000);
        double diff = 0.0;
        for (int k = 0; k < 24000; ++k) diff = std::max(diff, double(std::fabs(after[k] - fresh.l[k])));
        std::printf("  restrike at phase %d/8: step at event %.4f (own max step %.4f), level %.2f dB, "
                    "max diff from fresh strike %.3f\n",
                    q, stepAt, ownStep, lev, diff);
        CHECK_LE(stepAt, 1.0 * ownStep);
        CHECK(diff > 0.05 * peakAbs(fresh.l));
        levels.push_back(lev);
    }
    const auto [lo, hi] = std::minmax_element(levels.begin(), levels.end());
    std::printf("  level spread over phase: %.2f dB\n", *hi - *lo);
    CHECK(*hi - *lo > 0.5);
}
