// Continuous sustain pedal (plan §1.9): the decay of a held-then-released
// note varies monotonically and smoothly with the pedal position across the
// transition zone; moving the pedal does not click; re-pedalling keeps the
// energy left in the strings.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <utility>
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

// Damper-response budget (engine.h, "Cost of events"; ModeBank::
// coupledResponse). Many strings sounding, the pedal swept continuously
// through its whole travel for 4 s (CC64 every 5 ms), so that every tick
// changes far more coupled-group responses than the budget: the output stays
// finite and bounded, close to the exact (unlimited) responses, the same
// for every buffer size; once the pedal rests every response and the zone
// kernel are exact again; with the pedal up the sound decays; a panic in
// the middle of a stroke silences it.
TEST(pedal_sweep_bounded_and_exact_at_rest) {
    auto at = [](double s) { return static_cast<std::uint64_t>(s * kFs); };
    std::vector<pm::Event> ev = {pedal(0, 1.0)};
    for (int i = 0; i < 30; ++i) {
        const int n = 24 + 2 * i + i / 3;
        ev.push_back(noteOn(at(0.05 + 0.004 * i), n, 60 + (i * 13) % 60));
        if (i % 4) ev.push_back(noteOff(at(0.35), n));
    }
    for (int i = 0; i < 6; ++i) ev.push_back(noteOn(at(0.9 + 0.6 * i), 60 + 3 * i, 100));  // held, re-struck
    for (double t = 0.5; t < 4.5; t += 0.005)
        ev.push_back(pedal(at(t), 0.5 + 0.5 * std::cos(2.0 * std::numbers::pi * (t - 0.5) / 1.3)));
    ev.push_back(pedal(at(4.5), 1.0));  // rest, pedal down (4.5 .. 5.5 s)
    for (int i = 0; i < 30; i += 4) ev.push_back(noteOff(at(5.5), 24 + 2 * i + i / 3));
    for (int i = 0; i < 6; ++i) ev.push_back(noteOff(at(5.5), 60 + 3 * i));
    ev.push_back(pedal(at(5.5), 0.0));  // all dampers down: decay
    std::stable_sort(ev.begin(), ev.end(), [](const pm::Event& a, const pm::Event& b) { return a.time < b.time; });
    const int n = static_cast<int>(7.0 * kFs);

    struct Out {
        Stereo s;
        long approx = 0, exactRecomputes = 0;
        int maxRecomputesPerTick = 0;
        int approxAtRest = -1;
        bool kernelAtRest = false;
    };
    auto run = [&](bool exact, int block) {
        Out o;
        pm::Engine eng;
        eng.setExactDamperResponses(exact);
        eng.prepare(kFs, 512);
        o.s.l.assign(static_cast<std::size_t>(n), 0.0f);
        o.s.r.assign(static_cast<std::size_t>(n), 0.0f);
        std::size_t next = 0;
        for (int pos = 0; pos < n; pos += block) {
            const int m = std::min(block, n - pos);
            while (next < ev.size() && ev[next].time < eng.sampleTime() + static_cast<std::uint64_t>(m))
                eng.pushEvent(ev[next++]);
            eng.render(o.s.l.data() + pos, o.s.r.data() + pos, m, nullptr);
            const pm::RenderInfo& info = eng.lastRender();
            o.approx += info.approxResponses;
            o.exactRecomputes += info.responseGroups;
            // Per tick: a 32-frame buffer is at most one tick (and the
            // refinement and the loop's re-reads share the tick's budget).
            if (block == 32) o.maxRecomputesPerTick = std::max(o.maxRecomputesPerTick, info.responseGroups);
            if (pos + m == at(5.4)) {  // pedal at rest (down) since 4.5 s
                o.approxAtRest = eng.approximateResponses();
                o.kernelAtRest = eng.loopKernelExact();
            }
        }
        return o;
    };
    const Out ex = run(true, 32), bud = run(false, 32);
    const double peak = std::max(peakAbs(ex.s.l), peakAbs(ex.s.r));
    double d = 0.0, d2 = 0.0, peakBud = 0.0;
    bool finite = true;
    for (int i = 0; i < n; ++i)
        for (const auto* p : {&bud.s.l, &bud.s.r}) {
            const float* x = p->data();
            finite = finite && std::isfinite(x[i]);
            peakBud = std::max(peakBud, double(std::fabs(x[i])));
        }
    for (int i = 0; i < n; ++i) {
        const double e = std::max(std::fabs(ex.s.l[i] - bud.s.l[i]), std::fabs(ex.s.r[i] - bud.s.r[i]));
        d = std::max(d, e);
        d2 += e * e;
    }
    const double maxDb = 20 * std::log10(d / peak + 1e-30), rmsDiffDb = 10 * std::log10(d2 / n / (peak * peak) + 1e-30);
    const double tail = rmsDb(bud.s.l, static_cast<int>(6.7 * kFs), n), sweep = rmsDb(bud.s.l, static_cast<int>(1.0 * kFs), static_cast<int>(4.5 * kFs));
    std::printf("  exact: %ld group responses recomputed; budget %d/tick: %ld recomputed (at most %d per tick), %ld "
                "approximated\n",
                ex.exactRecomputes, pm::Engine::kResponseBudget, bud.exactRecomputes, bud.maxRecomputesPerTick, bud.approx);
    std::printf("  budget vs exact: max %.1f dB, rms %.1f dB re peak; peak %.4f vs %.4f; at rest: %d approximate, "
                "kernel exact %d; level 6.7-7 s %.1f dB vs sweep %.1f dB\n",
                maxDb, rmsDiffDb, peakBud, peak, bud.approxAtRest, bud.kernelAtRest ? 1 : 0, tail, sweep);
    CHECK(finite);
    CHECK(bud.approx > 10000);  // the sweep runs far over the budget
    CHECK(bud.maxRecomputesPerTick <= pm::Engine::kResponseBudget);
    CHECK(peakBud <= peak * 1.01);
    CHECK_LE(maxDb, -80.0);  // measured -107 dB
    CHECK(bud.approxAtRest == 0 && bud.kernelAtRest);
    CHECK(ex.approxAtRest == 0 && ex.kernelAtRest);
    CHECK(tail < sweep - 40.0);  // pedal up: the strings are damped

    // Buffer-size invariance with the budget: its schedule is the tick grid's.
    for (int block : {37, 512}) {
        const Out o = run(false, block), oe = run(true, block);
        double db = 0.0, dbe = 0.0;
        for (int i = 0; i < n; ++i) {
            db = std::max(db, double(std::max(std::fabs(o.s.l[i] - bud.s.l[i]), std::fabs(o.s.r[i] - bud.s.r[i]))));
            dbe = std::max(dbe, double(std::max(std::fabs(oe.s.l[i] - ex.s.l[i]), std::fabs(oe.s.r[i] - ex.s.r[i]))));
        }
        std::printf("  block %d vs 32: %.1f dB re peak (exact responses: %.1f dB)\n", block,
                    20 * std::log10(db / peak + 1e-30), 20 * std::log10(dbe / peak + 1e-30));
        // Block 37 differs from 32 by -86.7 dB with exact responses too
        // (measured; the hammers and the loop's sub-blocks cut differently,
        // engine.h "Determinism"), so the budget is held to adding nothing
        // to that, and to the usual -90 dB where exact responses meet it.
        CHECK_LE(20 * std::log10(db / peak + 1e-30), std::max(-90.0, 20 * std::log10(dbe / peak + 1e-30) + 0.5));
    }

    // Panic in the middle of a stroke (damper amounts above 1, approximated
    // at amount 1): silent within its 0.3 s.
    std::vector<pm::Event> pev;
    for (const pm::Event& e : ev)
        if (e.time < at(2.0)) pev.push_back(e);
    pev.push_back(simple(at(2.0), pm::EventType::Panic));
    pm::Engine eng;
    eng.prepare(kFs, 64);
    const Stereo p = renderEngine(eng, pev, static_cast<int>(2.6 * kFs));
    const double before = rmsDb(p.l, static_cast<int>(1.8 * kFs), static_cast<int>(2.0 * kFs));
    const double after = rmsDb(p.l, static_cast<int>(2.4 * kFs), static_cast<int>(2.6 * kFs));
    std::printf("  panic during the sweep: %.1f dB before, %.1f dB 0.4-0.6 s after\n", before, after);
    CHECK(after < before - 60.0);
}

// With the dampers at rest, or moving fewer responses per tick than the
// budget (a few keys released with the pedal up, the pedal pressed in
// silence and notes played and released under it), nothing is approximated
// and the output is bit-identical to exact responses. (Pressing the pedal
// while strings sound, or with its lift noise, wakes every key as the
// dampers lift, which does run over the budget for a few ticks:
// pedal_sweep_bounded_and_exact_at_rest.)
TEST(damper_budget_is_exact_at_rest) {
    auto at = [](double s) { return static_cast<std::uint64_t>(s * kFs); };
    const std::vector<pm::Event> ev = {
        noteOn(at(0.1), 48, 90),  noteOn(at(0.1), 60, 80),     noteOn(at(0.1) + 3, 64, 70), noteOff(at(0.6), 64),
        noteOff(at(0.9), 48),     noteOff(at(1.0), 60),        pedal(at(3.5), 1.0),         noteOn(at(4.0), 43, 100),
        noteOn(at(4.0) + 9, 67, 90), noteOff(at(4.2), 43),     noteOff(at(4.3), 67),        noteOn(at(4.8), 72, 60),
        noteOff(at(5.0), 72),     noteOn(at(5.2), 36, 110),    noteOff(at(5.3), 36)};
    auto run = [&](bool exact) {
        pm::Engine eng;
        eng.setExactDamperResponses(exact);
        // The damper-lift noise of a pedal press drives the board, which
        // wakes every key as its damper lifts, as sounding strings would.
        eng.setNoiseLevels(eng.knockLevel(), 0.0f);
        eng.prepare(kFs, 64);
        long approx = 0;
        Stereo s;
        s.l.assign(static_cast<std::size_t>(6.0 * kFs), 0.0f);
        s.r = s.l;
        std::size_t next = 0;
        for (std::size_t pos = 0; pos < s.l.size(); pos += 64) {
            while (next < ev.size() && ev[next].time < eng.sampleTime() + 64) eng.pushEvent(ev[next++]);
            eng.render(s.l.data() + pos, s.r.data() + pos, 64, nullptr);
            approx += eng.lastRender().approxResponses;
        }
        return std::make_pair(s, approx);
    };
    const auto a = run(true), b = run(false);
    int differ = 0;
    for (std::size_t i = 0; i < a.first.l.size(); ++i)
        differ += (a.first.l[i] != b.first.l[i] || a.first.r[i] != b.first.r[i]) ? 1 : 0;
    std::printf("  %ld approximated responses, %d of %zu samples differ from exact\n", b.second, differ, a.first.l.size());
    CHECK(b.second == 0);
    CHECK(differ == 0);
    CHECK(peakAbs(a.first.l) > 1e-3 && peakAbs(a.first.l, static_cast<int>(4.0 * kFs)) > 1e-3);
}
