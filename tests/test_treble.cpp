// Treble shelf on the dry microphone pair (Engine::setTrebleDb): 0 dB is the
// unfiltered engine, the measured response is the 3 kHz high shelf, a change
// keeps the buffer-size invariance, and idle sleep still ends in exact zeros.
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
    eng.setSoundboardConfig(c);
    eng.setStereoWidth(0.35f);
    eng.prepare(kFs, maxFrames, t);
}

Stereo renderChange(float db0, float db1, int at, int n, int block, const std::vector<pm::Event>& ev) {
    pm::Engine eng;
    eng.setTrebleDb(db0);
    prepareBrightVoicing(eng, 512);
    Stereo s;
    s.l.assign(n, 0.0f);
    s.r.assign(n, 0.0f);
    std::size_t next = 0;
    for (int pos = 0; pos < n; pos += block) {
        const int m = std::min(block, n - pos);
        if (pos <= at && at < pos + m) eng.setTrebleDb(db1);
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
IdleRun idleRun(float db) {
    const int n = static_cast<int>(6.0 * kFs), block = 64;
    pm::Engine eng;
    eng.setTrebleDb(db);
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
}
