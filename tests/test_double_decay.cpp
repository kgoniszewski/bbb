// Engine renders of coupled unisons (plan §1.3): two-stage decay, beating
// at the predicted rate, limiting cases, una corda.
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
constexpr double kPi = 3.14159265358979323846;
using PianoTable = pm::PianoTable;

std::vector<float> mono(const Stereo& s) {
    std::vector<float> m(s.l.size());
    for (std::size_t i = 0; i < m.size(); ++i) m[i] = s.l[i] + s.r[i];
    return m;
}

struct NoteDecay {
    TwoStage fit;
    double beat = 0.0;
    double levelDb = 0.0;  // partial 1 at 0.2 s
    double rmsDb = 0.0;    // broadband, first 0.3 s
};

NoteDecay analyse(const PianoTable& table, int note, int vel, double seconds, double unaCorda = 0.0) {
    pm::Engine eng;
    eng.prepare(kFs, 64, table);
    std::vector<pm::Event> ev;
    if (unaCorda > 0.0) {
        pm::Event u = simple(0, pm::EventType::UnaCorda);
        u.value = static_cast<std::uint16_t>(std::lround(unaCorda * 65535.0));
        ev.push_back(u);
    }
    ev.push_back(noteOn(10, note, vel));
    const Stereo s = renderEngine(eng, ev, static_cast<int>(seconds * kFs));
    const double f1 = table.keys[note - 21].f1;
    const double win = std::max(8.0 / f1, 0.05), hop = 0.02;
    const auto bb = baseband(mono(s), kFs, f1, win, hop);
    NoteDecay d;
    d.fit = twoStageFit(bb, hop, win, 0.2, 55.0);
    d.beat = beatRate(bb, hop, win, 0.5, std::min(8.0, seconds - 1.0));
    double e = 0.0;
    const std::vector<float> x = mono(s);
    for (int i = 0; i < static_cast<int>(0.3 * kFs); ++i) e += double(x[i]) * x[i];
    d.rmsDb = 10.0 * std::log10(e / (0.3 * kFs));
    d.levelDb = 20.0 * std::log10(std::abs(bb[static_cast<std::size_t>((0.2 - 0.5 * win) / hop)]));
    return d;
}



PianoTable table(double width = 1.0, double mating = 15e-6) {
    pm::TableOptions o;
    o.unisonWidth = width;
    o.matingSpreadM = mating;
    PianoTable t;
    pm::buildPianoTable(t, o);
    return t;
}

}  // namespace

// Mid-range keys of the default table: the fundamental decays in two stages,
// the aftersound clearly slower than the prompt sound.
TEST(double_decay_mid_range) {
    const PianoTable t = table();
    struct Case {
        int note;
        double seconds;
    };
    for (const Case c : {Case{36, 40.0}, Case{48, 30.0}, Case{60, 20.0}, Case{72, 12.0}, Case{84, 8.0}}) {
        const NoteDecay d = analyse(t, c.note, 80, c.seconds);
        std::printf("  note %d (key %d, %d strings), partial 1: prompt T60 %.2f s, aftersound T60 %.2f s "
                    "(x%.1f), break at %.2f s, crossover %.1f dB, beat %.3f Hz\n",
                    c.note, c.note - 20, t.keys[c.note - 21].strings, d.fit.promptT60, d.fit.afterT60,
                    d.fit.afterT60 / d.fit.promptT60, d.fit.breakTime, d.fit.crossoverDb, d.beat);
        CHECK(d.fit.promptT60 > 0.0);
        CHECK(d.fit.afterT60 > 2.0 * d.fit.promptT60);
    }
}

// A detuned bichord beats at the rate the eigen-solve predicts (the
// frequency difference of its two dominant normal modes, pulled slightly by
// the coupling) to within 0.1 Hz.
TEST(beating_at_predicted_rate) {
    for (double cents : {2.0, 4.0, 6.0}) {
        PianoTable t = table();
        pm::KeyParams& k = t.keys[39];  // C4 as a bichord
        k.strings = 2;
        k.detuneCents[0] = -0.5 * cents;
        k.detuneCents[1] = 0.5 * cents;
        k.unaCordaString = -1;
        const pm::KeyModeSet m = pm::computeKeyModes(k, kFs);
        const pm::UnisonPartial& u = m.partials[0].unison;
        int a = 0, b = 1;
        for (int j = 0; j < u.oscillators; ++j) {
            if (u.mode[j].score > u.mode[a].score) {
                b = a;
                a = j;
            } else if (j != a && u.mode[j].score > u.mode[b].score) {
                b = j;
            }
        }
        const double expected = std::fabs(u.mode[a].lambda.imag() - u.mode[b].lambda.imag()) / (2.0 * kPi);
        const double nominal = k.f1 * (std::pow(2.0, cents / 1200.0) - 1.0);
        const NoteDecay d = analyse(t, 60, 80, 10.0);
        std::printf("  C4 bichord detuned %.0f cents: nominal %.3f Hz, normal modes %.3f Hz, measured beat %.3f Hz\n",
                    cents, nominal, expected, d.beat);
        CHECK_NEAR(d.beat, expected, 0.1);
    }
}

// Limits with identical strings (width 0) and perfect hammer mating: the
// in-phase mode alone is excited, so the note decays at its rate (sigma +
// 3 kappa Re Yvv); the antisymmetric vertical modes do not move the bridge
// at all, so the remaining aftersound comes from the horizontal modes only,
// and una corda (which excites the antisymmetric modes strongly) adds none
// (Weinreich: without mistuning the antisymmetric motion is inaudible). With
// the default detuning, una corda raises the aftersound (next test).
TEST(identical_strings_limits) {
    const PianoTable t0 = table(0.0, 0.0);
    const pm::KeyModeSet m = pm::computeKeyModes(t0.keys[39], kFs);
    double symRate = 0.0, best = -1.0;
    for (int j = 0; j < m.partials[0].unison.oscillators; ++j)
        if (m.partials[0].unison.mode[j].score > best) {
            best = m.partials[0].unison.mode[j].score;
            symRate = -m.partials[0].unison.mode[j].lambda.real();
        }
    const NoteDecay same = analyse(t0, 60, 80, 20.0);
    const NoteDecay sameUna = analyse(t0, 60, 80, 20.0, 1.0);
    std::printf("  C4 identical strings: prompt T60 %.2f s (in-phase mode %.2f s), aftersound %.2f s, "
                "crossover %.1f dB, beat %.3f Hz; with una corda: crossover %.1f dB\n",
                same.fit.promptT60, 6.9078 / symRate, same.fit.afterT60, same.fit.crossoverDb, same.beat,
                sameUna.fit.crossoverDb);
    CHECK_NEAR(same.fit.promptT60, 6.9078 / symRate, 0.1 * 6.9078 / symRate);
    CHECK_NEAR(sameUna.fit.crossoverDb, same.fit.crossoverDb, 2.0);
    CHECK(same.beat == 0.0);
}

// Una corda (continuous): on a trichord the level falls and the share of
// aftersound rises monotonically with the pedal.
TEST(una_corda_level_and_aftersound) {
    const PianoTable t = table();
    double lastLevel = 1e9, lastCross = -1e9;
    bool monoLevel = true, monoCross = true;
    double first = 0.0, firstCross = 0.0;
    for (double u : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        const NoteDecay d = analyse(t, 60, 80, 15.0, u);
        std::printf("  C4 una corda %.2f: first 0.3 s %.2f dB, partial 1 at 0.2 s %.2f dB, crossover %.1f dB, prompt %.2f s, after %.2f s\n",
                    u, d.rmsDb, d.levelDb, d.fit.crossoverDb, d.fit.promptT60, d.fit.afterT60);
        if (u == 0.0) {
            first = d.rmsDb;
            firstCross = d.fit.crossoverDb;
        }
        monoLevel = monoLevel && d.rmsDb < lastLevel;
        monoCross = monoCross && d.fit.crossoverDb > lastCross - 1.0;
        lastLevel = d.rmsDb;
        lastCross = d.fit.crossoverDb;
    }
    CHECK(monoLevel);
    CHECK(monoCross);
    CHECK(lastLevel < first - 1.0);
    CHECK(lastCross > firstCross + 3.0);
}
