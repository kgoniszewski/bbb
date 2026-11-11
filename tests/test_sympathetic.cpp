// Sympathetic resonance through the soundboard loop (plan §1.6): a silently
// held key rings at the partials it shares with a struck note; an unrelated
// key rings far less; with the sustain pedal down a staccato chord leaves a
// broadband bloom that grows with the sympathetic gain. No reference
// recording exists yet, so presence, a sane level range and monotonicity
// are asserted and the per-band numbers are reported.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "decay_utils.h"
#include "pm_string/string_modes.h"
#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
constexpr double kPi = std::numbers::pi;

pm::Event keyDownSilent(std::uint64_t t, int note) {
    pm::Event e = noteOn(t, note, 1);
    e.type = pm::EventType::KeyDownSilent;
    return e;
}

std::vector<float> mono(const Stereo& s) {
    std::vector<float> m(s.l.size());
    for (std::size_t i = 0; i < m.size(); ++i) m[i] = 0.5f * (s.l[i] + s.r[i]);
    return m;
}

Stereo render(double g, const std::vector<pm::Event>& ev, double seconds) {
    pm::Engine eng;
    pm::SoundboardConfig c;
    c.sympathetic = g;
    eng.setSoundboardConfig(c);
    eng.setNoiseLevels(0.0f, 0.0f);
    eng.prepare(kFs, 64);
    return renderEngine(eng, ev, static_cast<int>(seconds * kFs));
}

double levelDb(const std::vector<float>& x, double t0, double t1) {
    return rmsDb(x, static_cast<int>(t0 * kFs), static_cast<int>(t1 * kFs));
}

// Level of the component at f (Hz) over [t0, t1): Hann-windowed DFT bin.
double toneDb(const std::vector<float>& x, double f, double t0, double t1) {
    const int a = static_cast<int>(t0 * kFs), n = static_cast<int>((t1 - t0) * kFs);
    std::complex<double> acc = 0.0;
    double ws = 0.0;
    for (int i = 0; i < n; ++i) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * (i + 0.5) / n);
        ws += w;
        acc += w * double(x[a + i]) * std::polar(1.0, -2.0 * kPi * f * i / kFs);
    }
    return 20.0 * std::log10(2.0 * std::abs(acc) / ws + 1e-30);
}

void fft(std::vector<std::complex<double>>& a) {
    const std::size_t n = a.size();
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const std::complex<double> wl = std::polar(1.0, -2.0 * kPi / double(len));
        for (std::size_t i = 0; i < n; i += len) {
            std::complex<double> w = 1.0;
            for (std::size_t k = 0; k < len / 2; ++k) {
                const std::complex<double> u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

// Energy per third-octave band (centres 63 Hz .. 4 kHz) of x[t0, t0 + 2^15 / fs).
std::vector<double> thirdOctaveDb(const std::vector<float>& x, double t0, const std::vector<double>& centres) {
    const std::size_t n = 1 << 15;
    std::vector<std::complex<double>> a(n);
    const std::size_t s0 = static_cast<std::size_t>(t0 * kFs);
    for (std::size_t i = 0; i < n; ++i)
        a[i] = (0.5 - 0.5 * std::cos(2.0 * kPi * (i + 0.5) / n)) * double(s0 + i < x.size() ? x[s0 + i] : 0.0f);
    fft(a);
    std::vector<double> out;
    for (double fc : centres) {
        const double lo = fc / std::pow(2.0, 1.0 / 6.0), hi = fc * std::pow(2.0, 1.0 / 6.0);
        double e = 0.0;
        for (std::size_t k = static_cast<std::size_t>(lo * n / kFs); k <= static_cast<std::size_t>(hi * n / kFs); ++k)
            e += std::norm(a[k]);
        out.push_back(10.0 * std::log10(e / n + 1e-30));
    }
    return out;
}

}  // namespace

// C3 held down silently, C4 struck staccato: after C4's damper falls, C3's
// strings ring at the partials the two share (C3's 2nd and 4th = C4's 1st and
// 2nd). With C#3 held instead (no shared low partials) far less remains.
TEST(silently_held_key_resonates) {
    const pm::KeyParams& c3 = tableKey(28);
    const double f2 = pm::partialFrequency(c3.f0, c3.B, 2), f4 = pm::partialFrequency(c3.f0, c3.B, 4);
    auto scene = [](int held) {
        std::vector<pm::Event> ev;
        if (held > 0) ev.push_back(keyDownSilent(0, held));
        ev.push_back(noteOn(4800, 60, 90));
        ev.push_back(noteOff(4800 + 9600, 60));  // 0.2 s staccato
        return ev;
    };
    const std::vector<float> related = mono(render(1.0, scene(48), 2.0));
    const std::vector<float> unrelated = mono(render(1.0, scene(49), 2.0));
    const std::vector<float> none = mono(render(1.0, scene(0), 2.0));
    const std::vector<float> noLoop = mono(render(0.0, scene(48), 2.0));
    const double t0 = 0.8, t1 = 1.8;  // C4 damped (landing <= 80 ms) and decayed
    const double direct = levelDb(related, 0.1, 0.3);
    const double lR = levelDb(related, t0, t1), lU = levelDb(unrelated, t0, t1), lN = levelDb(none, t0, t1),
                 lL = levelDb(noLoop, t0, t1);
    std::printf("  C4 staccato: direct %.1f dB; 0.8-1.8 s later: C3 held %.1f dB, C#3 held %.1f dB, none held "
                "%.1f dB, C3 held without the loop %.1f dB\n",
                direct, lR, lU, lN, lL);
    const double p2R = toneDb(related, f2, t0, t1), p2N = toneDb(none, f2, t0, t1), p2U = toneDb(unrelated, f2, t0, t1);
    const double p4R = toneDb(related, f4, t0, t1), p4N = toneDb(none, f4, t0, t1);
    std::printf("  at C3's 2nd partial (%.1f Hz): C3 held %.1f dB, C#3 held %.1f dB, none %.1f dB; 4th (%.1f Hz): "
                "%.1f dB vs %.1f dB\n",
                f2, p2R, p2U, p2N, f4, p4R, p4N);
    CHECK(lR > lN + 15.0);           // the held key rings
    CHECK(lR > lU + 10.0);           // far more than an unrelated one
    CHECK(lR > lL + 15.0);           // and only through the loop
    CHECK(p2R > p2N + 20.0);         // at the shared partials
    CHECK(p4R > p4N + 10.0);
    CHECK(lR < direct - 15.0);       // but well below the struck note
}

// Staccato C major chord (C4 E4 G4, 0.2 s) with the sustain pedal down:
// third-octave levels 0.3-1.0 s after the release, pedal down with the loop
// (gain 1), pedal down without it (gain 0: only the chord's own strings ring
// on) and pedal up. The bloom (the other strings, rung through the bridge)
// is the difference between the first two; its decay per band is reported
// from a second window 1.0-1.7 s after the release.
TEST(pedal_bloom_per_third_octave) {
    const std::vector<double> centres = {63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800,
                                         1000, 1250, 1600, 2000, 2500, 3150, 4000};
    auto scene = [](bool pedal) {
        std::vector<pm::Event> ev;
        if (pedal) ev.push_back(sustain(0, true));
        for (int n : {60, 64, 67}) ev.push_back(noteOn(4800, n, 90));
        for (int n : {60, 64, 67}) ev.push_back(noteOff(4800 + 9600, n));
        return ev;
    };
    const double rel = 0.3;  // chord release time, s
    const std::vector<double> gains = {0.0, 0.25, 0.5, 1.0};
    std::vector<std::vector<double>> early, late;
    std::vector<double> broad;
    for (double g : gains) {
        const std::vector<float> x = mono(render(g, scene(true), 2.4));
        early.push_back(thirdOctaveDb(x, rel + 0.3, centres));
        late.push_back(thirdOctaveDb(x, rel + 1.0, centres));
        broad.push_back(levelDb(x, rel + 0.3, rel + 1.0));
    }
    const std::vector<float> up = mono(render(1.0, scene(false), 2.4));
    const std::vector<double> upB = thirdOctaveDb(up, rel + 0.3, centres);
    const double direct = levelDb(up, 0.1, rel);
    std::printf("  chord direct level %.1f dB; 0.3-1.0 s after release: pedal down %.1f dB (gain 1), %.1f dB "
                "(gain 0.5), %.1f dB (gain 0.25), %.1f dB (gain 0, no loop); pedal up %.1f dB\n",
                direct, broad[3], broad[2], broad[1], broad[0], levelDb(up, rel + 0.3, rel + 1.0));
    std::printf("  band    down g=1  g=0.5  g=0.25  g=0   up    bloom (g=1 - g=0)   bloom decay T60\n");
    int bloomBands = 0;
    for (std::size_t b = 0; b < centres.size(); ++b) {
        // Energy of the bloom alone in each window (the chord's own strings subtracted).
        const double e1 = std::pow(10.0, early[3][b] / 10.0) - std::pow(10.0, early[0][b] / 10.0);
        const double e2 = std::pow(10.0, late[3][b] / 10.0) - std::pow(10.0, late[0][b] / 10.0);
        const double bloomDb = e1 > 0.0 ? 10.0 * std::log10(e1) : -300.0;
        const double t60 = e1 > 0.0 && e2 > 0.0 && e2 < e1 ? 60.0 * 0.7 / (10.0 * std::log10(e1 / e2)) : -1.0;
        std::printf("  %5.0f Hz %7.1f %6.1f %7.1f %6.1f %6.1f   %6.1f dB (%+5.1f dB re g=0)   %s%.2f s\n",
                    centres[b], early[3][b], early[2][b], early[1][b], early[0][b], upB[b], bloomDb,
                    early[3][b] - early[0][b], t60 < 0 ? "n/a " : "", t60 < 0 ? 0.0 : t60);
        bloomBands += early[3][b] > early[0][b] + 3.0;
    }
    // Below the chord's fundamental (bands 63-200 Hz) the chord strings have
    // no partials: the bloom (lower strings rung through the bridge at the
    // partials they share with the chord) stands alone there. Above, each
    // band is dominated by the chord's own partials, which the sympathetic
    // strings join or beat against (reported, not asserted). Presence, sane
    // range (well below the chord itself) and monotonic growth with the
    // sympathetic gain are asserted on the low bands.
    std::vector<double> low(gains.size(), 0.0);
    for (std::size_t gi = 0; gi < gains.size(); ++gi) {
        double e = 0.0;
        for (std::size_t b = 0; b < centres.size(); ++b)
            if (centres[b] <= 200.0) e += std::pow(10.0, early[gi][b] / 10.0);
        low[gi] = 10.0 * std::log10(e);
    }
    double all = 0.0;
    for (std::size_t b = 0; b < centres.size(); ++b) all += std::pow(10.0, early[3][b] / 10.0);
    const double share = low[3] - 10.0 * std::log10(all);
    std::printf("  %d of %zu bands at least 3 dB above the no-loop level; 63-200 Hz bloom: %.1f / %.1f / %.1f / %.1f dB "
                "at gain 0 / 0.25 / 0.5 / 1, at gain 1 %.1f dB re all bands\n",
                bloomBands, centres.size(), low[0], low[1], low[2], low[3], share);
    CHECK(bloomBands >= 6);
    CHECK(low[3] > low[0] + 10.0);
    CHECK(low[1] > low[0] && low[2] > low[1] && low[3] > low[2]);
    CHECK(share < -3.0 && share > -60.0);
    CHECK(broad[3] < direct);
}
