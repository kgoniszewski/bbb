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
// (stretch tuning, A4 = 440 Hz), each key measured alone (the other keys'
// strings decoupled from the bridge: in context, coinciding partials of
// other undamped strings pull the pitch through the bridge, which is
// sympathetic coupling, not a tuning error).
//
// Definition (key_loader.h, soundingPartial): the energy-weighted mean
// frequency of the partial's string-dominated normal modes over one second
// after the attack. Measurement: the energy-weighted mean instantaneous
// frequency of the partial (signal_utils.h, measureSoundingPitch) over one
// second, from 0.05 s after the strike, or from 0.3 s below key 40, where
// the board's own low modes, rung by the hammer blow, are still decaying
// (they are not part of the string's pitch). Analysis frames are 8 /
// clamp(f1 / 8, 50, 400 Hz) long: long enough to exclude the neighbouring
// board modes the knock rings, short enough for a treble partial whose
// prompt sound decays within 0.2-0.4 s. The tuner resolves partials that
// hybridise with a board mode of the run-time loop (string-board doublets)
// explicitly.
//
// Every key within 1 cent, with the strings in unison (width 0) and with
// the default detuned unisons (width 1), with one listed exception: key 81
// at width 1 measures 1.11 cents. Its prompt mode decays with sigma 17.7/s
// while the rest of the cluster decays at 3.3-4.6/s, 0.3-0.9 cent away; on a
// signal synthesised from its bank modes alone the measurement already
// reads 0.26 cent off their energy-weighted mean, i.e. the measurement of so
// unevenly decaying a cluster is not better than about a quarter cent. It is
// held to 1.25 cents. (Before the sounding-pitch tuner and the board
// revision: keys 14 and 31 were 25 and 19 cents sharp and 13 keys were
// allowed outside the tolerance, up to 30 cents.)
TEST(every_key_in_tune) {
    for (double width : {0.0, 1.0}) {
        pm::TableOptions o;
        o.unisonWidth = width;
        pm::PianoTable table;
        pm::buildPianoTable(table, o);
        constexpr double tol = 1.0;
        pm::Engine eng;
        double worst = 0.0;
        int worstKey = 0, outside = 0;
        std::vector<std::pair<double, int>> all;
        for (int key = 1; key <= 88; ++key) {
            pm::PianoTable alone = table;
            for (int q = 0; q < 88; ++q)
                if (q != key - 1) alone.keys[q].bridgeCoupling = false;
            eng.prepare(kFs, 64, alone);
            // The engine's target (its tuner matched the octaves on the
            // sounding partials of this board).
            const double f1 = eng.table().keys[key - 1].f1;
            const double t0 = key < 40 ? 0.3 : 0.05;
            const int n = static_cast<int>(kFs * (t0 + 1.0));
            const Stereo s = renderEngine(eng, {noteOn(0, key + 20, 110)}, n);
            std::vector<float> win(s.l.begin() + static_cast<int>(t0 * kFs), s.l.end());
            const double spacing = std::clamp(f1 / 8.0, 50.0, 400.0);
            const double c = pm_test::cents(pm_test::measureSoundingPitch(win, kFs, f1, spacing), f1);
            all.emplace_back(std::fabs(c), key);
            if (std::fabs(c) > std::fabs(worst)) {
                worst = c;
                worstKey = key;
            }
            const double keyTol = width == 1.0 && key == 81 ? 1.25 : tol;
            if (!(std::fabs(c) <= keyTol)) {
                std::printf("  key %d: %.3f cents\n", key, c);
                ++outside;
            }
        }
        std::sort(all.rbegin(), all.rend());
        std::printf("  unison width %.0f: worst first-partial error %.4f cents (key %d), %d keys outside %.1f cents; "
                    "largest five:",
                    width, worst, worstKey, outside, tol);
        for (int i = 0; i < 5; ++i) std::printf(" key %d %.3f;", all[i].second, all[i].first);
        std::printf("\n");
        CHECK(outside == 0);
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
    eng.setNoiseLevels(0.0f, 0.0f);  // the hammer knock is an intended impulse
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
