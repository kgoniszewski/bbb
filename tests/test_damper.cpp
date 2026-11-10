// Dampers and sustain pedal (plan §1.9).
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;

// Short-time level, 10 ms windows, of the sum of both channels.
std::vector<double> envelopeDb(const Stereo& s) {
    std::vector<double> e;
    const int w = 480;
    for (std::size_t p = 0; p + w <= s.l.size(); p += w) {
        double acc = 0.0;
        for (int i = 0; i < w; ++i) {
            const double v = double(s.l[p + i]) + s.r[p + i];
            acc += v * v;
        }
        e.push_back(10.0 * std::log10(acc / w + 1e-30));
    }
    return e;
}

// Seconds after 'from' until the level is 60 dB below the level just before.
double timeTo60(const Stereo& s, int from) {
    const auto e = envelopeDb(s);
    const int i0 = from / 480;
    const double ref = e[i0 - 1];
    for (std::size_t i = i0; i < e.size(); ++i)
        if (e[i] <= ref - 60.0) return (double(i) * 480 - from) / kFs;
    return 1e9;
}

double maxStep(const std::vector<float>& y, int from, int to) {
    double m = 0.0;
    for (int k = from; k + 1 < to; ++k) m = std::max(m, double(std::fabs(y[k + 1] - y[k])));
    return m;
}

}  // namespace

TEST(note_off_decays_without_click) {
    pm::Engine eng;
    const int off = 48000 + 13;
    for (int note : {36, 48, 60, 72, 84}) {
        eng.prepare(kFs, 64);
        const Stereo s = renderEngine(eng, {noteOn(0, note, 80), noteOff(off, note)}, off + 48000);
        const double t60 = timeTo60(s, off);
        const double before = maxStep(s.l, off - 960, off);
        const double after = maxStep(s.l, off - 3, off + 3);
        std::printf("  note %d: -60 dB %.3f s after note-off; max step before %.4f, around the event %.4f\n", note,
                    t60, before, after);
        if (note >= 48) CHECK(t60 >= 0.1 && t60 <= 0.4);
        else CHECK(t60 >= 0.1 && t60 <= 1.0);
        // Steps around the event against the signal's own steps just before
        // (beating unisons make the envelope itself vary by a few %).
        CHECK_LE(after, 1.05 * before);
    }
}

// With the pedal down the note keeps ringing exactly as if the key were held.
TEST(pedal_keeps_ringing) {
    pm::Engine eng;
    const int n = 3 * 48000;
    eng.prepare(kFs, 64);
    const Stereo held = renderEngine(eng, {noteOn(0, 60, 80)}, n);
    eng.prepare(kFs, 64);
    const Stereo ped = renderEngine(eng, {sustain(100, true), noteOn(200, 60, 80), noteOff(48000, 60)}, n + 200);
    double d = 0.0;
    for (int i = 0; i < n - 200; ++i) d = std::max(d, double(std::fabs(ped.l[i + 200] - held.l[i])));
    std::printf("  pedal down vs key held: max difference %.3g\n", d);
    // Only the culling grid differs (the scene is shifted by 200 samples).
    CHECK(d <= 1e-4 * peakAbs(held.l));

    // Releasing the pedal later damps it.
    eng.prepare(kFs, 64);
    const Stereo rel = renderEngine(
        eng, {sustain(0, true), noteOn(0, 60, 80), noteOff(24000, 60), sustain(96000, false)}, n);
    const double t60 = timeTo60(rel, 96000);
    std::printf("  pedal released at 2 s: -60 dB after %.3f s\n", t60);
    CHECK(t60 <= 0.4);
}

// Keys from 69 up have no dampers: note-off changes nothing.
TEST(top_keys_have_no_dampers) {
    pm::Engine eng;
    for (int note : {89, 96, 108}) {
        eng.prepare(kFs, 64);
        const Stereo held = renderEngine(eng, {noteOn(0, note, 100)}, 48000);
        eng.prepare(kFs, 64);
        const Stereo off = renderEngine(eng, {noteOn(0, note, 100), noteOff(4800, note)}, 48000);
        double d = 0.0;
        for (std::size_t i = 0; i < held.l.size(); ++i) d = std::max(d, double(std::fabs(off.l[i] - held.l[i])));
        std::printf("  note %d: note-off vs held max difference %.3g\n", note, d);
        CHECK(d == 0.0);
    }
    // Key 68 (note 88) still has one.
    eng.prepare(kFs, 64);
    const Stereo s = renderEngine(eng, {noteOn(0, 88, 100), noteOff(24000, 88)}, 72000);
    CHECK(timeTo60(s, 24000) < 0.5);
}

// Panic silences everything (including undamped keys) quickly and leaves the
// engine playable.
TEST(panic_silences) {
    pm::Engine eng;
    eng.prepare(kFs, 64);
    std::vector<pm::Event> ev = {sustain(0, true)};
    for (int n = 21; n <= 108; n += 5) ev.push_back(noteOn(10, n, 100));
    ev.push_back(simple(24000, pm::EventType::Panic));
    ev.push_back(noteOn(48000, 60, 80));
    const Stereo s = renderEngine(eng, ev, 72000);
    const auto e = envelopeDb(s);
    std::printf("  level before panic %.1f dB, 300 ms after %.1f dB, new note %.1f dB\n", e[48],
                e[50 + 30], e[110]);
    CHECK(e[50 + 30] < e[48] - 60.0);
    CHECK(e[110] > e[50 + 30] + 40.0);
    CHECK(eng.sustain() == 0.0f);
}
