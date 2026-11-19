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
// Pedal-down residual of a held mf C4 re its direct (partial) energy over
// 1.1-5.1 s, 100-5300 Hz, as measured with the low-regime board, 2026-11-15 (pedal_residual_per_bark_band);
// -29.4 dB since the A/B voicing of 2026-11-19 (board loss x2, felt x0.7; was -22.5).
constexpr double kC4ResidualReDirectDb = -29.4;

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

namespace {

// Power spectrum of x[t0, t0 + len) (Hann window, zero-padded to 2^18
// points), bin spacing kFs / 2^18.
std::vector<double> powerSpectrum(const std::vector<float>& x, double t0, double len) {
    const std::size_t n = 1 << 18, m = static_cast<std::size_t>(len * kFs);
    std::vector<std::complex<double>> a(n);
    const std::size_t s0 = static_cast<std::size_t>(t0 * kFs);
    for (std::size_t i = 0; i < m && i < n; ++i)
        a[i] = (0.5 - 0.5 * std::cos(2.0 * kPi * (i + 0.5) / m)) * double(s0 + i < x.size() ? x[s0 + i] : 0.0f);
    fft(a);
    std::vector<double> p(n / 2);
    for (std::size_t k = 0; k < n / 2; ++k) p[k] = std::norm(a[k]);
    return p;
}

// The tone's partials in a spectrum (tools/analysis/analyze_pedal_residual.py):
// partial n at the largest bin near the prediction of a least-squares fit
// of f_n^2 = n^2 f0^2 (1 + B n^2) to the partials found so far.
std::vector<double> findPartials(const std::vector<double>& p, double f1, double fmax) {
    const double df = kFs / double(2 * p.size());
    std::vector<double> found;
    for (int n = 1;; ++n) {
        double pred, tol;
        if (found.size() >= 3) {
            double s22 = 0, s24 = 0, s44 = 0, r2 = 0, r4 = 0;
            for (std::size_t i = 0; i < found.size(); ++i) {
                const double k = double(i + 1), k2 = k * k, k4 = k2 * k2, y = found[i] * found[i];
                s22 += k2 * k2;
                s24 += k2 * k4;
                s44 += k4 * k4;
                r2 += k2 * y;
                r4 += k4 * y;
            }
            const double det = s22 * s44 - s24 * s24;
            const double c0 = (r2 * s44 - s24 * r4) / det, c1 = (s22 * r4 - s24 * r2) / det;
            pred = std::sqrt(std::max(c0 * n * n + c1 * double(n) * n * n * n, 1.0));
            tol = 0.01;
        } else {
            pred = found.empty() ? f1 : n * found.back() / double(found.size());
            tol = 0.03;
        }
        if (pred > fmax) break;
        std::size_t best = 0;
        for (std::size_t k = static_cast<std::size_t>(pred * (1 - tol) / df);
             k <= static_cast<std::size_t>(pred * (1 + tol) / df) && k < p.size(); ++k)
            if (best == 0 || p[k] > p[best]) best = k;
        if (best == 0) break;
        found.push_back(best * df);
    }
    return found;
}

}  // namespace

// Sustain-pedal residual (plan §1.6 note (7)), the method of Lehtonen et al.
// (JASA 122, 2007): a tone held with the pedal down and the same tone with
// it up; the tone's own partials (and +-(3 Hz + 0.2 %) around each) removed
// from both spectra over 1-5 s after the strike; the remaining energy per
// Bark band compared. Published for recorded grand-piano tones: the
// residual rises 5-30 dB per Bark band with the pedal, about 10 dB over
// 1-5 s [quoted from the paper's abstract and text; VERIFY the bands and
// notes against the full paper]. The model's pedal-up residual has no
// floor (no room, no microphone or room noise, no longitudinal or phantom
// components yet; noises off here), so its rise is far larger (30-100 dB
// per band) and is not comparable as a level: only the published lower
// bound, 5 dB in every band from 100 Hz to 5.3 kHz whose pedal-down
// residual is within 100 dB of the tone, is asserted. The
// residual relative to the tone's direct (partial) energy, the metric of
// plan §2.4, is reported per band and guarded (broadband, C4) within 4 dB
// of the value this model gave with the low-regime board (2026-11-15): a regression guard, not a
// comparison with a reference (none exists before M6).
TEST(pedal_residual_per_bark_band) {
    static const double edges[] = {100, 200, 300, 400, 510, 630, 770, 920, 1080, 1270, 1480,
                                   1720, 2000, 2320, 2700, 3150, 3700, 4400, 5300};
    constexpr int nb = sizeof(edges) / sizeof(edges[0]) - 1;
    const double df = kFs / double(1 << 18);
    double c4ResidualReDirect = 0.0;
    for (int note : {48, 60, 72}) {
        std::vector<double> spec[2];
        for (int ped = 0; ped < 2; ++ped) {
            std::vector<pm::Event> ev;
            if (ped) ev.push_back(sustain(0, true));
            ev.push_back(noteOn(4800, note, 80));
            spec[ped] = powerSpectrum(mono(render(1.0, ev, 5.2)), 1.1, 4.0);
        }
        const double f1 = 440.0 * std::pow(2.0, (note - 69) / 12.0);
        const std::vector<double> partials = findPartials(spec[0], f1, 16000.0);
        std::vector<char> mask(spec[0].size(), 1);
        for (double f : partials) {
            const double w = 3.0 + 0.002 * f;
            for (std::size_t k = static_cast<std::size_t>(std::max(0.0, f - w) / df);
                 k <= static_cast<std::size_t>((f + w) / df) && k < mask.size(); ++k)
                mask[k] = 0;
        }
        double direct = 0.0, resUp = 0.0, resDown = 0.0;
        for (std::size_t k = 0; k < mask.size(); ++k) {
            if (!mask[k]) direct += spec[0][k];
            else if (k * df >= edges[0] && k * df < edges[nb]) {
                resUp += spec[0][k];
                resDown += spec[1][k];
            }
        }
        std::printf("  note %d: %zu partials removed; residual 1.1-5.1 s, dB re the tone's partials (pedal up):\n"
                    "    band (Hz)   pedal up  pedal down  rise\n",
                    note, partials.size());
        double minRise = 1e9;
        for (int b = 0; b < nb; ++b) {
            double u = 0.0, d = 0.0;
            for (std::size_t k = static_cast<std::size_t>(edges[b] / df); k < static_cast<std::size_t>(edges[b + 1] / df);
                 ++k)
                if (mask[k]) {
                    u += spec[0][k];
                    d += spec[1][k];
                }
            const double ud = 10.0 * std::log10(u / direct + 1e-300), dd = 10.0 * std::log10(d / direct + 1e-300);
            std::printf("    %4.0f-%4.0f %8.1f %9.1f %+7.1f\n", edges[b], edges[b + 1], ud, dd, dd - ud);
            // A band whose residual is more than 100 dB below the tone even
            // with the pedal down is below any recording's noise floor, so the
            // published bound cannot refer to it (C3 above 4.4 kHz: -129 dB).
            if (dd >= -100.0) minRise = std::min(minRise, dd - ud);
        }
        const double reDirect = 10.0 * std::log10(resDown / direct);
        std::printf("    100-5300 Hz: pedal down %.1f dB re direct, rise %+.1f dB; smallest rise (bands above -100 dB) %+.1f dB\n",
                    reDirect, 10.0 * std::log10(resDown / resUp), minRise);
        CHECK(minRise >= 5.0);
        if (note == 60) c4ResidualReDirect = reDirect;
    }
    CHECK_NEAR(c4ResidualReDirect, kC4ResidualReDirectDb, 4.0);
}
