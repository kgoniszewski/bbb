// Engine benchmark (M3): % of one core at 48 kHz in 64-frame buffers.
//
//   worst:    all 88 keys struck (1 ms apart) and held, sustain pedal down
//   no-cull:  the same with culling disabled (every mode of every key)
//   typical:  a four-note chord re-struck every 0.5 s and released, pedal up
//   cluster:  a 12-note cluster (C3..B3 plus C5..B5 alternating) struck ff
//             every 100 ms: 12 hammers in contact at once, repeatedly
//   hammer:   cost of one active hammer per 64-frame buffer, per key, while
//             in contact and averaged over its life (contact plus release
//             wait), measured on the solver alone
//
//   bench_engine [--seconds 3] [--block 64] [--fs 48000]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "pm_base/denormals.h"

#include "pm_engine/engine.h"
#include "pm_voicing/key_loader.h"

namespace {

using Clock = std::chrono::steady_clock;

pm::Event ev(std::uint64_t t, pm::EventType type, int note, int v7) {
    pm::Event e;
    e.time = t;
    e.type = type;
    e.timeBase = pm::TimeBase::SampleTime;
    e.note = static_cast<std::uint8_t>(note);
    e.value = pm::value16FromMidi7(v7);
    return e;
}

struct Result {
    double avgPercent, p99Percent, maxPercent;
    double first100msPercent;  // average over the first 100 ms measured
    int awakeModesEnd;
    double meanAwakeModes;
};

// Renders 'warm' seconds unmeasured, then 'seconds' measured. 'events'
// returns the events due in [t0, t1).
template <typename Events>
Result run(pm::Engine& eng, double fs, int block, double warm, double seconds, Events events) {
    std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
    const int warmBlocks = static_cast<int>(warm * fs / block);
    const int blocks = static_cast<int>(seconds * fs / block);
    std::uint64_t t = 0;
    auto step = [&](double* dt) {
        for (const pm::Event& e : events(t, t + block)) eng.pushEvent(e);
        const auto a = Clock::now();
        eng.render(l.data(), r.data(), block, nullptr);
        if (dt) *dt = std::chrono::duration<double>(Clock::now() - a).count();
        t += static_cast<std::uint64_t>(block);
    };
    for (int b = 0; b < warmBlocks; ++b) step(nullptr);
    Result res{};
    const double period = block / fs;
    const int firstBlocks = std::max(1, static_cast<int>(0.1 * fs / block));
    std::vector<double> dts(static_cast<std::size_t>(blocks));
    double total = 0.0, first = 0.0, awake = 0.0;
    for (int b = 0; b < blocks; ++b) {
        step(&dts[b]);
        total += dts[b];
        if (b < firstBlocks) first += dts[b];
        awake += eng.stats().awakeModes;
    }
    res.meanAwakeModes = awake / blocks;
    res.first100msPercent = 100.0 * first / (firstBlocks * period);
    std::sort(dts.begin(), dts.end());
    res.avgPercent = 100.0 * total / (blocks * period);
    res.p99Percent = 100.0 * dts[static_cast<std::size_t>(0.99 * (blocks - 1))] / period;
    res.maxPercent = 100.0 * dts.back() / period;
    res.awakeModesEnd = eng.stats().awakeModes;
    return res;
}

}  // namespace

int main(int argc, char** argv) {
    double seconds = 3.0, fs = 48000.0;
    int block = 64;
    for (int i = 1; i + 1 < argc; i += 2) {
        const double v = std::atof(argv[i + 1]);
        if (!std::strcmp(argv[i], "--seconds")) seconds = v;
        else if (!std::strcmp(argv[i], "--block")) block = static_cast<int>(v);
        else if (!std::strcmp(argv[i], "--fs")) fs = v;
        else {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        }
    }

    auto allKeys = [](std::uint64_t t0, std::uint64_t t1) {
        std::vector<pm::Event> out;
        if (t0 == 0) out.push_back(ev(0, pm::EventType::Sustain, 0, 127));
        for (int n = 21; n <= 108; ++n) {
            const std::uint64_t t = static_cast<std::uint64_t>(n - 21) * 48;  // 1 ms apart
            if (t >= t0 && t < t1) out.push_back(ev(t, pm::EventType::NoteOn, n, 110));
        }
        return out;
    };
    const std::uint64_t period = static_cast<std::uint64_t>(0.5 * fs);
    auto chord = [period](std::uint64_t t0, std::uint64_t t1) {
        std::vector<pm::Event> out;
        const int notes[4] = {48, 60, 64, 67};
        for (std::uint64_t base = (t0 / period) * period; base < t1; base += period) {
            for (int n : notes) {
                if (base >= t0) out.push_back(ev(base, pm::EventType::NoteOn, n, 80));
                const std::uint64_t off = base + period * 3 / 4;
                if (off >= t0 && off < t1) out.push_back(ev(off, pm::EventType::NoteOff, n, 64));
            }
        }
        return out;
    };

    const std::uint64_t clusterPeriod = static_cast<std::uint64_t>(0.1 * fs);
    auto cluster = [clusterPeriod](std::uint64_t t0, std::uint64_t t1) {
        std::vector<pm::Event> out;
        for (std::uint64_t base = (t0 / clusterPeriod) * clusterPeriod; base < t1; base += clusterPeriod) {
            if (base < t0) continue;
            const int first = (base / clusterPeriod) % 2 ? 72 : 48;
            for (int n = first; n < first + 12; ++n) out.push_back(ev(base, pm::EventType::NoteOn, n, 120));
            for (int n = first; n < first + 12; ++n)
                out.push_back(ev(base + clusterPeriod / 2, pm::EventType::NoteOff, n, 64));
        }
        return out;
    };

    pm::Engine eng;
    if (!eng.prepare(fs, block)) return 1;
    std::printf("engine benchmark: fs %.0f Hz, block %d, kernel %s, %.1f s measured; %d normal modes "
                "in %d groups of %d\n",
                fs, block, pm::simdKernelName(), seconds, eng.stats().totalModes, eng.stats().groups,
                pm::ModeBank::kGroupSize);
    struct Case {
        const char* name;
        bool cull;
        int scene;  // 0 worst, 1 typical, 2 cluster
    };
    for (const Case& c : {Case{"worst (88 keys held, pedal)", true, 0},
                          Case{"worst, culling off", false, 0},
                          Case{"typical (4-note chord, pedal up)", true, 1},
                          Case{"cluster (12 notes ff every 100 ms)", true, 2}}) {
        eng.setCulling(c.cull, -100.0f);
        if (!eng.prepare(fs, block)) return 1;
        // The worst cases are measured from the first strike, so the max
        // includes the burst of 88 hammers (1 ms apart, pool of 32).
        const Result r = c.scene == 0   ? run(eng, fs, block, 0.0, seconds, allKeys)
                         : c.scene == 1 ? run(eng, fs, block, 0.5, seconds, chord)
                                        : run(eng, fs, block, 0.5, seconds, cluster);
        std::printf("  %-35s %6.2f %% of one core; first 100 ms %6.2f %%; buffers p99 %6.2f %%, "
                    "max %6.2f %%; awake modes mean %5.0f, at end %5d\n",
                    c.name, r.avgPercent, r.first100msPercent, r.p99Percent, r.maxPercent,
                    r.meanAwakeModes, r.awakeModesEnd);
    }
    std::printf("  (awake modes exclude group padding; with culling on, partials more than 100 dB\n"
                "   down are never processed. The max of a single buffer includes any preemption\n"
                "   by the OS.)\n");

    // One hammer alone, per key: median over 200 strikes at mf.
    std::printf("  hammer solver, one hammer, us per %d-frame buffer (median of 200 mf strikes):\n", block);
    pm::PianoTable table;
    pm::buildPianoTable(table);
    for (int key : {1, 16, 28, 40, 52, 64, 76, 88}) {
        const pm::KeyParams& kp = table.keys[key - 1];
        const pm::KeyModeSet ms = pm::computeKeyModes(kp, fs);
        pm::ModeBank bank;
        pm::ContactTables contact;
        const int n = ms.count;
        if (!bank.allocateSegments(&n, 1) || !contact.allocate(bank.capacity(), 1)) return 1;
        pm::loadKeyModes(bank, contact, 0, kp, ms, fs);
        bank.setSegmentDamper(0, 0.0f);
        const pm::HammerParams hp = pm::hammerParams(kp);
        std::vector<double> firstBuf, life;
        std::vector<float> out(static_cast<std::size_t>(block));
        pm::ScopedFlushDenormals ftz;
        for (int rep = 0; rep < 200; ++rep) {
            bank.clearState();
            pm::HammerState hs;
            const pm::HammerKeyModes m = contact.view(bank, 0);
            pm::launchHammer(hs, hp, m, 2.2);
            double tot = 0.0;
            int b = 0;
            while (!pm::hammerFinished(hs, m)) {
                const auto a = Clock::now();
                pm::runHammer(hs, hp, m, out.data(), block);
                const double dt = std::chrono::duration<double>(Clock::now() - a).count();
                if (b == 0) firstBuf.push_back(dt);
                tot += dt;
                ++b;
            }
            life.push_back(tot / b);
        }
        std::sort(firstBuf.begin(), firstBuf.end());
        std::sort(life.begin(), life.end());
        std::printf("    key %2d (%d strings, %4d modes, R=%d): %6.1f in contact, %6.1f averaged over its life\n",
                    key, ms.strings, ms.count, kp.contactOversampling, 1e6 * firstBuf[100], 1e6 * life[100]);
    }
    return 0;
}
