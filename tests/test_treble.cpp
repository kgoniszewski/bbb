// Tone EQ on the dry microphone pair (Engine::setTrebleDb, setBassDb,
// setMidDb): 0 dB is the unfiltered engine, the measured responses are the
// 3 kHz high shelf, the 200 Hz low shelf and the 1 kHz peak, a change keeps
// the buffer-size invariance and does not click, idle sleep still ends in
// exact zeros, and the range ends are stable.
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
constexpr double kPiD = 3.14159265358979323846;

std::vector<pm::Event> chord() {
    return {noteOn(0, 48, 100), noteOn(3, 60, 100), noteOn(7, 96, 110), noteOff(20000, 48), noteOff(20000, 60),
            noteOff(20000, 96)};
}

double maxDiff(const Stereo& a, const Stereo& b) {
    double d = 0.0;
    for (std::size_t i = 0; i < a.l.size(); ++i)
        d = std::max(d, std::max(double(std::fabs(a.l[i] - b.l[i])), double(std::fabs(a.r[i] - b.r[i]))));
    return d;
}

// Energy of x after an RBJ band-pass (constant 0 dB peak, Q 4) run twice.
double bandEnergy(const std::vector<float>& x, double fc) {
    const double w = 2.0 * kPiD * fc / kFs, al = std::sin(w) / (2.0 * 4.0);
    const double a0 = 1.0 + al, b0 = al / a0, b2 = -al / a0, a1 = -2.0 * std::cos(w) / a0, a2 = (1.0 - al) / a0;
    std::vector<double> y(x.begin(), x.end());
    for (int pass = 0; pass < 2; ++pass) {
        double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        for (double& v : y) {
            const double o = b0 * v + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1, x1 = v, y2 = y1, y1 = o, v = o;
        }
    }
    double e = 0.0;
    for (double v : y) e += v * v;
    return e;
}

// Renders with the treble set to `db0`, switched to `db1` before the buffer
// that contains sample `at` (the change is latched at the first control tick
// of that buffer).
// The shelf tests run on the voicing before A/B listening rounds 1-3
// (felt x1/0.7, board loss x0.5 against the current table; width 0.35):
// with the softer felt and the more damped board the chord has so little
// energy near 8 kHz that the band filter's skirts (2-4 kHz, where the shelf
// is still on its slope) dominate, and the measured 8 kHz gain reads -2.3
// instead of about -2.6 dB for a -3 dB shelf. The shelf itself is unchanged.
void prepareBrightVoicing(pm::Engine& eng, int maxFrames) {
    pm::TableOptions o;
    o.feltHardness = 1.0 / 0.7;
    pm::PianoTable t;
    pm::buildPianoTable(t, o);
    pm::SoundboardConfig c;
    c.dampingScale = 0.5;
    // Radiation tilt off (default -4 dB/oct above 700 Hz since A/B round 4,
    // 2026-11-20): it removes the 8 kHz energy the same way, and the measured
    // 8 kHz gain read -2.2 dB. This test measures the shelf, not the voicing.
    c.radiationTiltDbPerOct = 0.0;
    eng.setSoundboardConfig(c);
    eng.setStereoWidth(0.35f);
    eng.prepare(kFs, maxFrames, t);
}

enum Band { kTreble, kBass, kMid };

void setBand(pm::Engine& eng, Band band, float db) {
    if (band == kBass) eng.setBassDb(db);
    else if (band == kMid) eng.setMidDb(db);
    else eng.setTrebleDb(db);
}

Stereo renderChange(float db0, float db1, int at, int n, int block, const std::vector<pm::Event>& ev,
                    Band band = kTreble) {
    pm::Engine eng;
    setBand(eng, band, db0);
    prepareBrightVoicing(eng, 512);
    Stereo s;
    s.l.assign(n, 0.0f);
    s.r.assign(n, 0.0f);
    std::size_t next = 0;
    for (int pos = 0; pos < n; pos += block) {
        const int m = std::min(block, n - pos);
        if (pos <= at && at < pos + m) setBand(eng, band, db1);
        while (next < ev.size() && ev[next].time < eng.sampleTime() + static_cast<std::uint64_t>(m))
            eng.pushEvent(ev[next++]);
        eng.render(s.l.data() + pos, s.r.data() + pos, m, nullptr);
    }
    return s;
}

}  // namespace

// (a) 0 dB skips the filter: an engine that glides from -3 to 0 dB in the
// silence before the first note ends exactly in the bypass, equal sample for
// sample to an engine at 0 dB throughout. (The 0 dB render was also checked
// by hand at integration time to be bit-identical to the engine before the
// shelf was added.)
TEST(treble_zero_is_bypass) {
    const int n = 30000;
    const std::vector<pm::Event> ev = {noteOn(2048, 60, 90), noteOn(2050, 96, 100)};
    pm::Engine a;
    CHECK(a.trebleDb() == pm::Engine::kDefaultTrebleDb);
    a.setTrebleDb(0.0f);
    prepareBrightVoicing(a, 512);
    const Stereo ra = renderEngine(a, ev, n, 64);
    const Stereo rb = renderChange(-3.0f, 0.0f, 0, n, 64, ev);
    const Stereo rc = renderChange(-3.0f, -3.0f, 0, n, 64, ev);
    std::printf("  0 dB vs glide -3 -> 0 dB: max difference %.3g; vs -3 dB: %.3g (peak %.3g)\n", maxDiff(ra, rb),
                maxDiff(ra, rc), peakAbs(ra.l));
    CHECK(peakAbs(ra.l) > 1e-3);
    CHECK(maxDiff(ra, rb) == 0.0);
    CHECK(maxDiff(ra, rc) > 1e-4);
}

// (b) The shelf as heard on the engine output: band energies of a chord
// (C3, C4, C7) at -3 dB against 0 dB. The filter is linear and after the
// whole mix, so the ratio is the shelf's response, smeared over the band.
TEST(treble_shelf_response) {
    const int n = 24000;
    const Stereo r0 = renderChange(0.0f, 0.0f, 0, n, 64, chord());
    const Stereo r3 = renderChange(-3.0f, -3.0f, 0, n, 64, chord());
    for (double fc : {130.8, 261.6, 480.0, 2093.0, 4186.0, 8000.0, 12000.0}) {
        const double g = 10.0 * std::log10((bandEnergy(r3.l, fc) + bandEnergy(r3.r, fc)) /
                                           (bandEnergy(r0.l, fc) + bandEnergy(r0.r, fc)));
        std::printf("  %7.1f Hz: %+.2f dB\n", fc, g);
        if (fc < 500.0) CHECK(std::fabs(g) <= 0.3);
        if (fc == 8000.0) CHECK(std::fabs(g + 3.0) <= 0.5);
    }
}

// (c) A treble change in the middle of the scene keeps the buffer-size
// invariance (bit-identical for multiples of the 32-sample grid, within the
// -100 dB re peak limit of test_scheduling for an odd size).
TEST(treble_change_block_size_invariance) {
    const int n = 30000, at = 8192;
    const Stereo ref = renderChange(-3.0f, -9.0f, at, n, 64, chord());
    const Stereo flat = renderChange(-3.0f, -3.0f, at, n, 64, chord());
    CHECK(maxDiff(ref, flat) > 1e-4);
    for (int block : {32, 37, 128, 512}) {
        const Stereo s = renderChange(-3.0f, -9.0f, at, n, block, chord());
        const double d = maxDiff(s, ref);
        const double db = 20.0 * std::log10(d / peakAbs(ref.l) + 1e-30);
        std::printf("  treble change, block %3d: max difference %.1f dB re peak\n", block, db);
        if (block % 32 == 0) CHECK(d == 0.0);
        else CHECK_LE(db, -100.0);
    }
}

// (d) At the default -3 dB the engine goes idle as at 0 dB (the same
// number of fully idle buffers) and its output ends in exact zeros (the
// shelf's state is cleared once it has decayed below 1e-20 on silence).
namespace {
struct IdleRun {
    int idleBuffers = 0;
    bool idleAtEnd = false;
    int lastNonZero = -1;  // last sample that is not exactly zero
};
IdleRun idleRun(float db, Band band = kTreble) {
    const int n = static_cast<int>(6.0 * kFs), block = 64;
    pm::Engine eng;
    setBand(eng, band, db);
    eng.prepare(kFs, block);
    const std::vector<pm::Event> ev = {noteOn(0, 96, 80), noteOff(static_cast<std::uint64_t>(0.2 * kFs), 96)};
    std::vector<float> l(block), r(block);
    std::size_t next = 0;
    IdleRun out;
    for (int pos = 0; pos < n; pos += block) {
        while (next < ev.size() && ev[next].time < eng.sampleTime() + static_cast<std::uint64_t>(block))
            eng.pushEvent(ev[next++]);
        eng.render(l.data(), r.data(), block, nullptr);
        const pm::RenderInfo& info = eng.lastRender();
        out.idleAtEnd = info.idleSubBlocks == info.subBlocks;
        out.idleBuffers += out.idleAtEnd ? 1 : 0;
        for (int i = 0; i < block; ++i)
            if (l[i] != 0.0f || r[i] != 0.0f) out.lastNonZero = pos + i;
    }
    return out;
}
}  // namespace

TEST(treble_idle_exact_zero) {
    const IdleRun a = idleRun(-3.0f), b = idleRun(0.0f);
    std::printf("  idle buffers: %d at -3 dB, %d at 0 dB; last non-zero sample at %.3f s and %.3f s\n",
                a.idleBuffers, b.idleBuffers, a.lastNonZero / kFs, b.lastNonZero / kFs);
    CHECK(a.idleBuffers > static_cast<int>(2.0 * kFs / 64));
    CHECK(a.idleBuffers == b.idleBuffers);
    CHECK(a.idleAtEnd);
    // The 5 Hz DC blocker after the shelf reaches exact zero on its own
    // (as at 0 dB), from a slightly different last value.
    CHECK(a.lastNonZero > 0 && a.lastNonZero < static_cast<int>(5.9 * kFs));
    // Bass and mid at +-12 dB: the same idle behaviour and exact zeros.
    for (Band band : {kBass, kMid})
        for (float db : {-12.0f, 12.0f}) {
            const IdleRun c = idleRun(db, band);
            std::printf("  %s %+.0f dB: idle buffers %d, last non-zero sample at %.3f s\n", band == kBass ? "bass" : "mid",
                        db, c.idleBuffers, c.lastNonZero / kFs);
            CHECK(c.idleBuffers == b.idleBuffers);
            CHECK(c.idleAtEnd);
            CHECK(c.lastNonZero > 0 && c.lastNonZero < static_cast<int>(5.9 * kFs));
        }
}

// ---- Bass (200 Hz low shelf) and mid (1 kHz peak, Q 0.7) ----

// (e) 0 dB bypass: bass and mid set to 0 explicitly, and glided from -3 to
// 0 dB in the silence before the first note, are equal sample for sample to
// the engine that never sets them.
TEST(bass_mid_zero_is_bypass) {
    const int n = 30000;
    const std::vector<pm::Event> ev = {noteOn(2048, 36, 90), noteOn(2050, 72, 100)};
    pm::Engine a;
    CHECK(a.bassDb() == 0.0f && a.midDb() == 0.0f);
    prepareBrightVoicing(a, 512);
    const Stereo ra = renderEngine(a, ev, n, 64);
    for (Band band : {kBass, kMid}) {
        const Stereo r0 = renderChange(0.0f, 0.0f, 0, n, 64, ev, band);
        const Stereo rb = renderChange(-3.0f, 0.0f, 0, n, 64, ev, band);
        const Stereo rc = renderChange(-3.0f, -3.0f, 0, n, 64, ev, band);
        std::printf("  %s: 0 dB vs unset %.3g, glide -3 -> 0 dB %.3g; vs -3 dB: %.3g\n", band == kBass ? "bass" : "mid",
                    maxDiff(ra, r0), maxDiff(ra, rb), maxDiff(ra, rc));
        CHECK(maxDiff(ra, r0) == 0.0);
        CHECK(maxDiff(ra, rb) == 0.0);
        CHECK(maxDiff(ra, rc) > 1e-4);
    }
}

namespace {
// Gain of the band at fc (dB) measured as in (b): band energy of one held
// note with the EQ at `db` against 0 dB. The probe note puts a partial at
// (or next to) fc so that the band filter is dominated by it and not by
// partials on the EQ's slopes.
double bandGainDb(Band band, float db, int note, double fc) {
    const int n = 24000;
    const std::vector<pm::Event> ev = {noteOn(0, note, 100), noteOff(20000, note)};
    const Stereo r0 = renderChange(0.0f, 0.0f, 0, n, 64, ev, band);
    const Stereo r1 = renderChange(db, db, 0, n, 64, ev, band);
    return 10.0 * std::log10((bandEnergy(r1.l, fc) + bandEnergy(r1.r, fc)) / (bandEnergy(r0.l, fc) + bandEnergy(r0.r, fc)));
}
}  // namespace

// (f) Responses. Probes: G1 (49 Hz), G3 (196 Hz), C7 (2093 Hz), G2 (98 Hz),
// B5 (988 Hz) and the 5th partial of C7 (about 10.5 kHz) in a 10 kHz band.
// RBJ design values at +6 dB: low shelf 5.98 dB at 49 Hz, 3.12 at 196 Hz,
// 0.00 at 2 kHz; peak 6.00 at 988 Hz, 0.13 at 98 Hz, 0.10 at 10 kHz.
TEST(bass_mid_response) {
    for (float db : {6.0f, -6.0f}) {
        const double s = db > 0.0f ? 1.0 : -1.0;
        const double b50 = bandGainDb(kBass, db, 31, 49.0), b200 = bandGainDb(kBass, db, 55, 196.0),
                     b2k = bandGainDb(kBass, db, 96, 2093.0);
        const double m1k = bandGainDb(kMid, db, 83, 987.8), m100 = bandGainDb(kMid, db, 43, 98.0),
                     m10k = bandGainDb(kMid, db, 96, 10000.0);
        std::printf("  bass %+.0f dB: 49 Hz %+.2f, 196 Hz %+.2f, 2093 Hz %+.2f dB\n", db, b50, b200, b2k);
        std::printf("  mid  %+.0f dB: 988 Hz %+.2f, 98 Hz %+.2f, 10 kHz %+.2f dB\n", db, m1k, m100, m10k);
        CHECK(std::fabs(b50 - s * 6.0) <= 0.5);
        CHECK(std::fabs(b200 - s * 3.1) <= 0.5);
        CHECK(std::fabs(b2k) < 0.5);
        CHECK(std::fabs(m1k - s * 6.0) <= 0.5);
        CHECK(std::fabs(m100) < 1.5);
        CHECK(std::fabs(m10k) < 1.5);
    }
}

// (g) A step of the gain during a held note glides: the change keeps the
// buffer-size invariance, and the output's largest sample-to-sample step
// around the change is no larger than with the final gain held throughout
// (a click would show as a larger step).
TEST(bass_mid_change_glides) {
    const int n = 30000, at = 8192;
    for (Band band : {kBass, kMid}) {
        const char* name = band == kBass ? "bass" : "mid";
        const Stereo ref = renderChange(0.0f, 12.0f, at, n, 64, chord(), band);
        const Stereo flat = renderChange(0.0f, 0.0f, at, n, 64, chord(), band);
        const Stereo held = renderChange(12.0f, 12.0f, at, n, 64, chord(), band);
        CHECK(maxDiff(ref, flat) > 1e-4);
        for (int block : {32, 37, 512}) {
            const Stereo s = renderChange(0.0f, 12.0f, at, n, block, chord(), band);
            const double d = maxDiff(s, ref);
            const double db = 20.0 * std::log10(d / peakAbs(ref.l) + 1e-30);
            std::printf("  %s 0 -> +12 dB, block %3d: max difference %.1f dB re peak\n", name, block, db);
            if (block % 32 == 0) CHECK(d == 0.0);
            else CHECK_LE(db, -100.0);
        }
        auto maxStep = [&](const Stereo& x) {
            double m = 0.0;
            for (int i = at - 64; i < at + 2048; ++i)
                m = std::max(m, std::max(double(std::fabs(x.l[i] - x.l[i - 1])), double(std::fabs(x.r[i] - x.r[i - 1]))));
            return m;
        };
        const double sStep = maxStep(ref), sHeld = maxStep(held), sFlat = maxStep(flat);
        std::printf("  %s 0 -> +12 dB: max step %.4g (0 dB held %.4g, +12 dB held %.4g)\n", name, sStep, sFlat, sHeld);
        CHECK_LE(sStep, 1.05 * std::max(sHeld, sFlat));
    }
}

// (h) Range ends: every combination of -12 / +12 dB on the three bands,
// held and switched mid-note, stays finite and bounded at 44.1, 48 and 96 kHz
// (exact zeros at the end are checked in (d)).
TEST(eq_range_ends_stable) {
    for (double fs : {44100.0, 48000.0, 96000.0}) {
        const int n = static_cast<int>(1.5 * fs), block = 64;
        double worst = 0.0, ref = 0.0;
        bool finite = true;
        for (int combo = -1; combo < 8; ++combo) {
            pm::Engine eng;
            eng.prepare(fs, block);
            auto set = [&](int c) {
                if (c < 0) return;
                eng.setBassDb(c & 1 ? 12.0f : -12.0f);
                eng.setMidDb(c & 2 ? 12.0f : -12.0f);
                eng.setTrebleDb(c & 4 ? 12.0f : -12.0f);
            };
            set(combo);
            const std::vector<pm::Event> ev = {noteOn(0, 21, 127), noteOn(5, 60, 127), noteOn(9, 108, 127),
                                               noteOff(static_cast<std::uint64_t>(0.3 * fs), 21),
                                               noteOff(static_cast<std::uint64_t>(0.3 * fs), 60),
                                               noteOff(static_cast<std::uint64_t>(0.3 * fs), 108)};
            std::vector<float> l(block), r(block);
            std::size_t next = 0;
            double peak = 0.0;
            for (int pos = 0; pos < n; pos += block) {
                if (combo >= 0 && pos == (n / 6) / block * block) set(7 - combo);
                while (next < ev.size() && ev[next].time < eng.sampleTime() + static_cast<std::uint64_t>(block))
                    eng.pushEvent(ev[next++]);
                eng.render(l.data(), r.data(), block, nullptr);
                for (int i = 0; i < block; ++i) {
                    finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
                    peak = std::max(peak, double(std::max(std::fabs(l[i]), std::fabs(r[i]))));
                }
            }
            if (combo < 0) ref = peak;
            else worst = std::max(worst, peak);
        }
        std::printf("  fs %.0f: peak at 0 dB %.3g, worst at the range ends %.3g\n", fs, ref, worst);
        CHECK(finite);
        CHECK(ref > 1e-3);
        CHECK(worst < 64.0 * ref);  // three bands at +12 dB: at most 4x each
    }
}
