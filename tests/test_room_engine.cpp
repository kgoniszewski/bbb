// The room (pm_room) inside the engine: off is the dry engine, on keeps the
// engine's buffer-size invariance, its tail decays at the set T60, render
// does not allocate, and parameter changes do not click.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "piano_utils.h"
#include "pm_room/room.h"
#include "test_harness.h"

using namespace pm_test;

namespace {
std::atomic<bool> gArmed{false};
std::atomic<long> gAllocs{0};
void* counted(std::size_t n, std::size_t align) {
    if (gArmed.load(std::memory_order_relaxed)) gAllocs.fetch_add(1, std::memory_order_relaxed);
    if (n == 0) n = 1;
    void* p = nullptr;
    if (align <= alignof(std::max_align_t)) p = std::malloc(n);
    else if (posix_memalign(&p, align, n) != 0) p = nullptr;
    if (!p) std::abort();
    return p;
}
}  // namespace

void* operator new(std::size_t n) { return counted(n, 0); }
void* operator new[](std::size_t n) { return counted(n, 0); }
void* operator new(std::size_t n, std::align_val_t a) { return counted(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return counted(n, static_cast<std::size_t>(a)); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {

constexpr double kFs = 48000.0;

std::vector<pm::Event> scene(std::uint64_t t) {
    return {noteOn(t, 60, 90),         noteOn(t + 5, 64, 70),     sustain(t + 2000, true),
            noteOff(t + 3000, 60),     noteOn(t + 3001, 67, 110), noteOff(t + 7000, 64),
            sustain(t + 12345, false), noteOn(t + 12400, 60, 40), noteOff(t + 20000, 60),
            simple(t + 22222, pm::EventType::AllNotesOff)};
}

double maxDiff(const Stereo& a, const Stereo& b) {
    double d = 0.0;
    for (std::size_t i = 0; i < a.l.size(); ++i)
        d = std::max(d, std::max(double(std::fabs(a.l[i] - b.l[i])), double(std::fabs(a.r[i] - b.r[i]))));
    return d;
}

// RBJ band-pass (constant 0 dB peak gain), run twice: about a 1 kHz octave.
std::vector<double> octaveBand(const std::vector<double>& x, double fc) {
    const double w = 2.0 * 3.14159265358979 * fc / kFs, q = 1.414, al = std::sin(w) / (2.0 * q);
    const double a0 = 1.0 + al, b0 = al / a0, b2 = -al / a0, a1 = -2.0 * std::cos(w) / a0, a2 = (1.0 - al) / a0;
    std::vector<double> y = x;
    for (int pass = 0; pass < 2; ++pass) {
        double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        for (double& v : y) {
            const double o = b0 * v + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1, x1 = v, y2 = y1, y1 = o, v = o;
        }
    }
    return y;
}

// T60 from the Schroeder energy decay curve of x[from..]: line fit of the
// curve between -5 and -25 dB (T20), extrapolated to 60 dB.
double t60(const std::vector<double>& x, int from) {
    std::vector<double> edc(x.size() - from);
    double acc = 0.0;
    for (int i = static_cast<int>(x.size()) - 1; i >= from; --i) edc[i - from] = (acc += x[i] * x[i]);
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int m = 0;
    for (std::size_t i = 0; i < edc.size(); ++i) {
        const double db = 10.0 * std::log10(edc[i] / edc[0] + 1e-300);
        if (db > -5.0 || db < -25.0) continue;
        const double t = i / kFs;
        sx += t, sy += db, sxx += t * t, sxy += t * db, ++m;
    }
    const double slope = (m * sxy - sx * sy) / (m * sxx - sx * sx);
    return -60.0 / slope;
}

}  // namespace

// (a) Room off: compared with the room on at level -inf dB (wet gains
// exactly 0, so the wet signal is exactly 0 and the room path adds exact
// zeros): equal sample for sample (==; the sign of a zero may differ). The
// before/after-change render hash of the dry engine (room path compiled
// out vs present, off) was checked by hand at integration time.
TEST(room_off_equals_room_at_zero_level) {
    const int n = 30000;
    pm::Engine a;
    a.prepare(kFs, 512);
    const Stereo ra = renderEngine(a, scene(100), n, 64);
    pm::Engine b;
    b.setRoomEnabled(true);
    b.setRoomLevelDb(-INFINITY);
    b.prepare(kFs, 512);
    const Stereo rb = renderEngine(b, scene(100), n, 64);
    const double d = maxDiff(ra, rb);
    std::printf("  room off vs on at -inf dB: max difference %.3g (peak %.3g)\n", d, peakAbs(ra.l));
    CHECK(!a.roomEnabled());
    CHECK(peakAbs(ra.l) > 1e-3);
    CHECK(d == 0.0);
}

// (b) Room on: buffer-size invariance with the limits of test_scheduling
// (block_size_invariance), culling on.
TEST(room_on_block_size_invariance) {
    const int n = 30000;
    pm::Engine eng;
    eng.setRoomEnabled(true);
    eng.prepare(kFs, 512);
    const Stereo ref = renderEngine(eng, scene(100), n, 64);
    pm::Engine dry;
    dry.prepare(kFs, 512);
    const Stereo rd = renderEngine(dry, scene(100), n, 64);
    const double wet = maxDiff(ref, rd);
    std::printf("  wet signal peak (on minus off) %.3g, output peak %.3g\n", wet, peakAbs(ref.l));
    CHECK(wet > 1e-4);
    for (int block : {1, 32, 37, 128, 500, 512}) {
        eng.prepare(kFs, 512);
        const Stereo s = renderEngine(eng, scene(100), n, block);
        const double d = maxDiff(s, ref);
        const double db = 20.0 * std::log10(d / peakAbs(ref.l) + 1e-30);
        std::printf("  room on, block %3d: max difference %.1f dB re peak\n", block, db);
        if (block % 32 == 0) CHECK(d == 0.0);
        else CHECK_LE(db, -100.0);
    }
}

// (c) A mf C4, released after 0.5 s: the wet signal (room on minus room
// off, the dry path is identical) decays with the set T60 after note-off.
// Measured on the 1 kHz octave band (the room's "mid" decay; the HF ratio
// shortens the treble, so broadband is printed for information only).
TEST(room_tail_decays_at_set_t60) {
    for (double decay : {1.8, 3.0}) {
        const int n = static_cast<int>((0.5 + 2.5 * decay) * kFs);
        const std::vector<pm::Event> ev = {noteOn(0, 60, 64), noteOff(static_cast<std::uint64_t>(0.5 * kFs), 60)};
        pm::Engine on, off;
        on.setRoomEnabled(true);
        on.setRoomDecay(static_cast<float>(decay));
        on.prepare(kFs, 64);
        off.prepare(kFs, 64);
        const Stereo a = renderEngine(on, ev, n), b = renderEngine(off, ev, n);
        std::vector<double> w(n);
        for (int i = 0; i < n; ++i) w[i] = double(a.l[i] - b.l[i]) + double(a.r[i] - b.r[i]);
        const int from = static_cast<int>(0.8 * kFs);  // 0.3 s after note-off, past the damped string
        const double tBand = t60(octaveBand(w, 1000.0), from), tBroad = t60(w, from);
        std::printf("  decay %.1f s: T60 1 kHz octave %.2f s (%+.1f %%), broadband %.2f s\n", decay, tBand,
                    100.0 * (tBand / decay - 1.0), tBroad);
        CHECK_NEAR(tBand, decay, 0.15 * decay);
    }
}

// (d) No allocation in render with the room on, a size change (fade out,
// new taps and line lengths, fade in) and the room switched off and on.
TEST(room_render_does_not_allocate) {
    pm::Engine eng;
    eng.setRoomEnabled(true);
    eng.prepare(kFs, 256);
    const std::size_t bytes0 = pm::alignedLiveBytes();
    std::vector<float> l(256), r(256);
    eng.pushEvent(noteOn(10, 60, 100));
    eng.pushEvent(noteOn(20, 67, 90));
    gArmed = true;
    for (int b = 0; b < 600; ++b) {
        if (b == 100) eng.setRoomSize(1.7f);
        if (b == 250) eng.setRoomSize(0.6f);
        if (b == 350) eng.setRoomEnabled(false);
        if (b == 420) eng.setRoomEnabled(true);
        eng.render(l.data(), r.data(), 256, nullptr);
    }
    gArmed = false;
    std::printf("  allocations during render: %ld; aligned bytes %zu -> %zu\n", gAllocs.load(), bytes0,
                pm::alignedLiveBytes());
    CHECK(gAllocs.load() == 0);
    CHECK(pm::alignedLiveBytes() == bytes0);
}

// (e) Level and decay changed while a chord sounds: no click. Metric: the
// largest sample-to-sample step of the wet signal (on minus off) in the
// 100 ms after the change, against the larger of the same metric for the
// old and the new settings held throughout; a click (an unramped gain or
// state jump) would exceed both. Bound 1.1x.
TEST(room_parameter_changes_do_not_click) {
    const int n = static_cast<int>(2.0 * kFs), change = static_cast<int>(1.0 * kFs);
    const std::vector<pm::Event> ev = {sustain(0, true), noteOn(10, 48, 90), noteOn(12, 55, 80), noteOn(14, 64, 85),
                                       noteOn(16, 72, 70)};
    auto run = [&](int mode) {  // 0 old held, 1 new held, 2 change at 1 s, 3 room off
        pm::Engine eng;
        eng.setRoomEnabled(mode != 3);
        if (mode == 1) {
            eng.setRoomLevelDb(-3.0f);
            eng.setRoomDecay(3.5f);
        }
        eng.prepare(kFs, 64);
        Stereo s;
        s.l.assign(n, 0.0f);
        s.r.assign(n, 0.0f);
        std::size_t next = 0;
        for (int pos = 0; pos < n; pos += 64) {
            if (mode == 2 && pos == change) {
                eng.setRoomLevelDb(-3.0f);
                eng.setRoomDecay(3.5f);
            }
            while (next < ev.size() && ev[next].time < eng.sampleTime() + 64) eng.pushEvent(ev[next++]);
            eng.render(s.l.data() + pos, s.r.data() + pos, 64, nullptr);
        }
        return s;
    };
    const Stereo dry = run(3);
    double step[3] = {}, peak = 0.0;
    for (int mode = 0; mode < 3; ++mode) {
        const Stereo s = run(mode);
        for (int i = change; i < change + static_cast<int>(0.1 * kFs); ++i)
            for (int c = 0; c < 2; ++c) {
                const std::vector<float>& x = c ? s.r : s.l;
                const std::vector<float>& y = c ? dry.r : dry.l;
                const double d = std::fabs(double(x[i] - y[i]) - double(x[i - 1] - y[i - 1]));
                step[mode] = std::max(step[mode], d);
            }
        peak = std::max(peak, peakAbs(s.l));
    }
    std::printf("  max wet step after the change: %.3g (old held %.3g, new held %.3g); %.1f dB re output peak\n",
                step[2], step[0], step[1], 20.0 * std::log10(step[2] / peak));
    CHECK_LE(step[2], 1.1 * std::max(step[0], step[1]));
}

// The one-off clear of the room's late field (silence reached, size change
// completed, disabled): the longest single process() call, against the
// typical one, at 48 kHz and 64-frame calls. Printed; budget 3 % of a
// 64-frame buffer (40 us).
TEST(room_clear_spike) {
    pm::Room room;
    room.prepare(kFs, 64);
    std::vector<float> in(64), out(64), out2(64);
    std::uint32_t rng = 1;
    double worst = 0.0, typical = 0.0;
    int calls = 0;
    for (int rep = 0; rep < 3; ++rep) {
        for (int b = 0; b < 400; ++b) {  // 0.5 s of noise, then silence until the room sleeps
            for (float& v : in) v = b < 375 ? ((rng = rng * 1664525u + 1013904223u) >> 8) * (1.0f / 16777216.0f) - 0.5f : 0.0f;
            room.process(in.data(), in.data(), out.data(), out2.data(), 64);
        }
        for (int b = 0; b < 4000 && !room.silent(); ++b) {
            for (float& v : in) v = 0.0f;
            const auto t0 = std::chrono::steady_clock::now();
            room.process(in.data(), in.data(), out.data(), out2.data(), 64);
            const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            if (room.silent()) worst = std::max(worst, us);
            else typical += us, ++calls;
        }
        CHECK(room.silent());
    }
    std::printf("  call that clears the late field: %.1f us; mean sounding call %.2f us (64 frames, budget 40 us)\n",
                worst, typical / std::max(1, calls));
}
