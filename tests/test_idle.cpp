// Idle sleep (engine.h, setIdleSleep): with nothing sounding the engine
// skips the bank, the soundboard loop, the board and the control ticks.
// The skipped work would compute zeros and change no state, so sleeping is
// exact: the output is identical with and without it, also across a pedal
// press during the silence and the next note-on (no click, nothing lost),
// and it does not depend on the buffer size.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
const int kN = static_cast<int>(10.0 * kFs);

std::vector<pm::Event> scene() {
    auto at = [](double s) { return static_cast<std::uint64_t>(s * kFs); };
    return {noteOn(at(0.2), 60, 90),  noteOn(at(0.21), 64, 80), noteOff(at(0.7), 60),  noteOff(at(0.7), 64),
            sustain(at(6.0), true),   sustain(at(6.5), false),  noteOn(at(7.0) + 13, 67, 100),
            noteOff(at(7.4), 67),     noteOn(at(9.5) + 5, 48, 70)};
}

struct Run {
    Stereo out;
    int firstIdle = -1;            // first buffer rendered entirely idle after the first note
    int idleBuffers = 0;
    std::uint64_t groupBlocksIdle = 0, boardStepsIdle = 0;  // work done in fully idle buffers
};

Run render(bool sleep, int block, bool liftNoise = true) {
    Run r;
    pm::Engine eng;
    eng.setIdleSleep(sleep);
    if (!liftNoise) eng.setNoiseLevels(eng.knockLevel(), 0.0f);
    eng.prepare(kFs, block);
    const std::vector<pm::Event> ev = scene();
    r.out.l.assign(kN, 0.0f);
    r.out.r.assign(kN, 0.0f);
    std::size_t next = 0;
    int b = 0;
    for (int pos = 0; pos < kN; pos += block, ++b) {
        const int n = std::min(block, kN - pos);
        while (next < ev.size() && ev[next].time < eng.sampleTime() + static_cast<std::uint64_t>(n))
            eng.pushEvent(ev[next++]);
        const std::uint64_t g0 = eng.bank().groupBlocksProcessed(), s0 = eng.soundboard().stepsProcessed();
        eng.render(r.out.l.data() + pos, r.out.r.data() + pos, n, nullptr);
        const pm::RenderInfo& info = eng.lastRender();
        if (info.idleSubBlocks == info.subBlocks) {
            if (r.firstIdle < 0 && pos > static_cast<int>(0.3 * kFs)) r.firstIdle = b;
            ++r.idleBuffers;
            r.groupBlocksIdle += eng.bank().groupBlocksProcessed() - g0;
            r.boardStepsIdle += eng.soundboard().stepsProcessed() - s0;
        }
    }
    return r;
}

double maxDiff(const Stereo& a, const Stereo& b, double* at = nullptr) {
    double d = 0.0;
    for (std::size_t i = 0; i < a.l.size(); ++i) {
        const double x = std::max(std::fabs(a.l[i] - b.l[i]), std::fabs(a.r[i] - b.r[i]));
        if (x > d) {
            d = x;
            if (at) *at = static_cast<double>(i) / kFs;
        }
    }
    return d;
}

}  // namespace

TEST(idle_sleep_is_exact_and_costs_nothing) {
    const Run on = render(true, 64), off = render(false, 64);
    const double peak = std::max(peakAbs(off.out.l), peakAbs(off.out.r));
    const double d = maxDiff(on.out, off.out);
    std::printf("  first fully idle buffer at %.2f s; %d of %d buffers idle; difference with sleep off %.3g "
                "(peak %.3g)\n",
                on.firstIdle * 64 / kFs, on.idleBuffers, kN / 64, d, peak);
    std::printf("  work in idle buffers: %llu group blocks, %llu board steps\n",
                static_cast<unsigned long long>(on.groupBlocksIdle), static_cast<unsigned long long>(on.boardStepsIdle));
    CHECK(peak > 1e-3);
    CHECK(d == 0.0);  // exact: the skipped work only ever computes zeros
    // The silences of the scene (from about 2-3 s after a release) are idle.
    CHECK(on.firstIdle * 64 > static_cast<int>(0.7 * kFs) && on.firstIdle * 64 < static_cast<int>(6.0 * kFs));
    CHECK(on.idleBuffers > static_cast<int>(2.0 * kFs / 64));
    CHECK(on.groupBlocksIdle == 0 && on.boardStepsIdle == 0);
    CHECK(off.idleBuffers == 0);
    // Exactly silent while idle (the output stage sleeps once its filters
    // have settled to zero).
    bool silent = true;
    for (int i = static_cast<int>(5.5 * kFs); i < static_cast<int>(6.0 * kFs); ++i)
        silent = silent && on.out.l[i] == 0.0f && on.out.r[i] == 0.0f;
    CHECK(silent);
}

// Sleep decisions are taken per sub-block, so they add no dependence on the
// buffer size: at every buffer size the output with sleep equals the output
// without it, and buffer sizes agree to rounding.
TEST(idle_sleep_is_buffer_size_invariant) {
    const Run ref = render(true, 64, false);
    const double peak = std::max(peakAbs(ref.out.l), peakAbs(ref.out.r));
    for (int block : {1, 37, 512}) {
        const Run r = render(true, block, false), rOff = render(false, block, false);
        double at = 0.0;
        const double d = maxDiff(ref.out, r.out, &at);
        const double dOff = maxDiff(r.out, rOff.out);
        std::printf("  block %3d: vs block 64 %.1f dB re peak (at %.3f s); sleep on vs off %.3g; %d idle buffers\n",
                    block, 20 * std::log10(d / peak + 1e-30), at, dOff, r.idleBuffers);
        CHECK(dOff == 0.0);
        // Block 1 differs from block 64 by -85 dB at the first hammer
        // contact (0.203 s), with or without sleep and in the engine before
        // idle sleep existed (a hammer-solver property, measured), so it is
        // only held to the sleep on/off identity above.
        if (block != 1) CHECK_LE(20 * std::log10(d / peak + 1e-30), -90.0);
    }
}


// The soundboard loop's zone kernel is updated in place, column by column,
// when only some keys' impulse responses change (wake-ups and cull
// decisions under a sustained chord); the result must be the full rebuild's
// bit for bit (CouplingLoop, soundboard.h).
TEST(kernel_column_update_matches_full_rebuild) {
    auto at = [](double s) { return static_cast<std::uint64_t>(s * kFs); };
    std::vector<pm::Event> ev = {sustain(0, true)};
    const int chords[3][4] = {{36, 48, 55, 64}, {41, 53, 60, 69}, {31, 43, 59, 74}};
    for (int c = 0; c < 3; ++c)
        for (int n : chords[c]) {
            ev.push_back(noteOn(at(0.05 + 0.6 * c) + 7 * n, n, 70 + 10 * c));
            ev.push_back(noteOff(at(0.35 + 0.6 * c), n));
        }
    std::stable_sort(ev.begin(), ev.end(), [](const pm::Event& x, const pm::Event& y) { return x.time < y.time; });
    const int n = static_cast<int>(2.4 * kFs), block = 256;
    pm::Engine ref, upd;
    ref.setKernelAlwaysRebuild(true);
    ref.prepare(kFs, block);
    upd.prepare(kFs, block);
    Stereo a, b;
    long rebuildsRef = 0, rebuildsUpd = 0, keyUpdates = 0;
    for (Stereo* s : {&a, &b}) {
        s->l.assign(static_cast<std::size_t>(n), 0.0f);
        s->r.assign(static_cast<std::size_t>(n), 0.0f);
    }
    std::size_t next = 0;
    for (int pos = 0; pos < n; pos += block) {
        const int m = std::min(block, n - pos);
        while (next < ev.size() && ev[next].time < ref.sampleTime() + static_cast<std::uint64_t>(m)) {
            ref.pushEvent(ev[next]);
            upd.pushEvent(ev[next++]);
        }
        ref.render(a.l.data() + pos, a.r.data() + pos, m, nullptr);
        upd.render(b.l.data() + pos, b.r.data() + pos, m, nullptr);
        rebuildsRef += ref.lastRender().kernelRebuilds;
        rebuildsUpd += upd.lastRender().kernelRebuilds;
        keyUpdates += upd.lastRender().kernelKeyUpdates;
    }
    int differ = 0;
    for (int i = 0; i < n; ++i) differ += (a.l[i] != b.l[i] || a.r[i] != b.r[i]) ? 1 : 0;
    std::printf("  full rebuilds %ld (reference) vs %ld + %ld in-place key updates; samples differing: %d of %d\n",
                rebuildsRef, rebuildsUpd, keyUpdates, differ, n);
    CHECK(differ == 0);
    CHECK(keyUpdates > 0 && rebuildsUpd < rebuildsRef);
    CHECK(peakAbs(a.l) > 1e-3);
}
