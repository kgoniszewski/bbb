// Continuous sustain pedal (plan §1.9): the decay of a held-then-released
// note varies monotonically and smoothly with the pedal position across the
// transition zone; moving the pedal does not click; re-pedalling keeps the
// energy left in the strings.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "decay_utils.h"
#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;

pm::Event pedal(std::uint64_t t, double pos) {
    pm::Event e = sustain(t, true);
    e.value = static_cast<std::uint16_t>(std::lround(std::clamp(pos, 0.0, 1.0) * 65535.0));
    return e;
}

std::vector<float> mono(const Stereo& s) {
    std::vector<float> m(s.l.size());
    for (std::size_t i = 0; i < m.size(); ++i) m[i] = 0.5f * (s.l[i] + s.r[i]);
    return m;
}

Stereo render(const std::vector<pm::Event>& ev, double seconds, double sympathetic = 1.0) {
    pm::Engine eng;
    pm::SoundboardConfig c;
    c.sympathetic = sympathetic;
    eng.setSoundboardConfig(c);
    eng.setNoiseLevels(0.0f, 0.0f);
    eng.prepare(kFs, 64);
    return renderEngine(eng, ev, static_cast<int>(seconds * kFs));
}

// Effective T60 of the fundamental after t0 (s): three times the time its
// envelope takes to fall 20 dB (the decay is not exponential in the
// transition zone: the felt damps large amplitudes more).
double t60After(const std::vector<float>& x, double f, double t0) {
    const double win = std::max(8.0 / f, 0.03), hop = 0.005;
    const auto bb = baseband(x, kFs, f, win, hop);
    double ref = 0.0, start = -1.0;
    for (std::size_t k = 0; k < bb.size(); ++k) {
        const double tk = k * hop + 0.5 * win;
        if (tk < t0) continue;
        const double db = 20.0 * std::log10(std::abs(bb[k]) + 1e-30);
        if (start < 0.0) {
            start = tk;
            ref = db;
        }
        if (db < ref - 20.0) return 3.0 * (tk - start);
    }
    return 3.0 * (bb.size() * hop - start) * 1e3;  // did not fall 20 dB: effectively undamped
}

}  // namespace

// C4 held 0.5 s with the pedal at position P (set before the note), then
// released: effective T60 of the fundamental after the release, against P. The felt
// moves from pressed (full damping) through light, amplitude-dependent
// contact to clear: T60 must rise monotonically (within 3 % measurement
// noise) and without jumps (adjacent positions 0.03 apart differ by less
// than x2.5 in T60). Measured with the board listening only: with the loop
// the other strings, whose dampers the same pedal lifts, exchange energy
// with C4's partials (sympathetic resonance, reported but not asserted).
TEST(t60_graded_with_pedal_position) {
    const double f1 = tableKey(40).f1;
    const std::vector<double> pos = {0.0, 0.2, 0.28, 0.31, 0.34, 0.37, 0.40, 0.43, 0.46, 0.49, 0.52, 0.55, 0.58, 0.61, 0.64, 0.7, 0.8, 1.0};
    std::vector<double> t60, t60Loop;
    for (double p : pos) {
        const std::vector<pm::Event> ev = {pedal(0, p), noteOn(4800, 60, 80), noteOff(4800 + 24000, 60)};
        t60.push_back(t60After(mono(render(ev, 3.5, 0.0)), f1, 0.6 + 0.03));
        t60Loop.push_back(t60After(mono(render(ev, 3.5, 1.0)), f1, 0.6 + 0.03));
    }
    std::printf("  pedal position -> effective T60 of C4 after release (board listening only | with the loop):\n");
    for (std::size_t i = 0; i < pos.size(); ++i) {
        if (t60[i] > 100.0) std::printf("    %.2f: undamped (no 20 dB fall in 2.9 s) | ", pos[i]);
        else std::printf("    %.2f: %6.2f s | ", pos[i], t60[i]);
        if (t60Loop[i] > 100.0) std::printf("undamped\n");
        else std::printf("%6.2f s\n", t60Loop[i]);
    }
    bool mono = true, smooth = true;
    for (std::size_t i = 1; i < pos.size(); ++i) {
        mono = mono && t60[i] >= 0.97 * t60[i - 1];
        if (pos[i] - pos[i - 1] <= 0.031) smooth = smooth && t60[i] <= 2.5 * t60[i - 1];
    }
    CHECK(mono);
    CHECK(smooth);
    CHECK(t60.front() < 0.5);                // pedal up: damped
    CHECK(t60.back() > 10.0 * t60.front());  // pedal down: rings on
    int graded = 0;  // positions with partial damping
    for (double v : t60) graded += v > 1.3 * t60.front() && v < 0.5 * t60.back();
    CHECK(graded >= 3);
}

// A note held with the pedal half down, the pedal then moved in steps
// through the transition zone and back: no step in the output at any pedal
// event beyond the signal's own sample-to-sample motion (damping changes
// only decay rates; the pedal is smoothed).
TEST(pedal_moves_without_click) {
    std::vector<pm::Event> ev = {noteOn(0, 48, 90), noteOff(12000, 48), pedal(6000, 1.0)};
    const double steps[] = {0.6, 0.45, 0.4, 0.35, 0.5, 0.0, 1.0};
    std::uint64_t t = 24000;
    for (double p : steps) {
        ev.push_back(pedal(t + 7, p));
        t += 4800;
    }
    const Stereo s = render(ev, 1.4);
    double worst = 0.0;
    t = 24000;
    for (std::size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i, t += 4800) {
        double before = 0.0, around = 0.0;
        for (std::uint64_t k = t - 960; k < t; ++k) before = std::max(before, double(std::fabs(s.l[k + 1] - s.l[k])));
        for (std::uint64_t k = t; k < t + 960; ++k) around = std::max(around, double(std::fabs(s.l[k + 1] - s.l[k])));
        worst = std::max(worst, around / before);
    }
    std::printf("  largest sample step after a pedal move / before it: %.3f\n", worst);
    CHECK_LE(worst, 1.05);
}

// Re-pedalling: C3 struck and released with the pedal down; at 1.0 s the
// pedal comes up for 60 ms and goes down again. The strings keep what was
// left after 60 ms of damping and ring on (nothing is reset), far above the
// level with the pedal left up.
TEST(repedal_keeps_energy) {
    const double f1 = tableKey(28).f1;
    auto scene = [](bool repedal) {
        std::vector<pm::Event> ev = {pedal(0, 1.0), noteOn(480, 48, 90), noteOff(9600, 48), pedal(48000, 0.0)};
        if (repedal) ev.push_back(pedal(48000 + 2880, 1.0));
        return ev;
    };
    const std::vector<float> re = mono(render(scene(true), 2.5)), off = mono(render(scene(false), 2.5));
    const double win = 0.05, hop = 0.01;
    const auto bre = baseband(re, kFs, f1, win, hop), boff = baseband(off, kFs, f1, win, hop);
    auto at = [&](const std::vector<std::complex<double>>& b, double tt) {
        return 20.0 * std::log10(std::abs(b[static_cast<std::size_t>((tt - 0.5 * win) / hop)]) + 1e-30);
    };
    const double before = at(bre, 0.98), after = at(bre, 1.2), later = at(bre, 2.2), damped = at(boff, 1.2);
    const double t60 = 60.0 / ((after - later) / 1.0);
    std::printf("  C3 fundamental: %.1f dB before the pedal lift, %.1f dB 0.14 s after re-pedalling, %.1f dB 1 s "
                "later (T60 %.1f s); pedal left up: %.1f dB\n",
                before, after, later, t60, damped);
    CHECK(after > before - 20.0);   // 60 ms of damping removed part of the energy, not all
    CHECK(after < before - 0.5);    // but some
    CHECK(after > damped + 20.0);   // far more than with the pedal left up
    CHECK(t60 > 3.0);               // and it rings on undamped
}
