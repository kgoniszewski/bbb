// Engine benchmark (M4): % of one core at 48 kHz in 64-frame buffers, with
// the soundboard coupling loop (the "board off" lines are the A/B mode).
// The scenarios live in core/pm_bench (bench.h), shared with the app's
// Benchmark screen (C API pm_bench_*), which runs the same cases on the
// device without the per-phase breakdown.
//
// Board-on cases run at the quality tier given by --tier (default full);
// --tiers runs the pedal-down and cluster cases at every tier.
//
// After the summary lines, a per-phase breakdown of every case (us per
// buffer): the most expensive buffer, the mean over the top 1 % of buffers
// (those at or above p99) and the mean over all buffers, split into event
// handling (incl. control tick, wake-ups, impulse-response cache), hammer
// solves, the bank's free pass, the forced pass, zone kernel + board step,
// and the rest (output, filters, cull passes). The breakdown uses the
// engine's run-time phase timing (Engine::setPhaseTiming, the same switch as
// the app's): about five clock reads per 32-frame sub-block plus two per
// sample while a hammer is active, included in all times. --timing off runs
// without it (no breakdown), --timing both runs every case both ways and
// prints the difference, which is what the timers cost.
//
// First line: the speed probe (pm_host/speed_probe.h) on this thread, the
// reference the app compares its audio thread with. Then, per case, the
// most expensive buffer's hammer and kernel detail.
//
//   bench_engine [--seconds 3] [--block 64] [--fs 48000] [--tier full|balanced|eco] [--tiers]
//                [--timing on|off|both]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "pm_bench/bench.h"
#include "pm_host/speed_probe.h"

int main(int argc, char** argv) {
    pm::bench::Config cfg;
    int timingMode = 1;  // 0 off, 1 on, 2 both
    for (int i = 1; i < argc; i += 2) {
        if (!std::strcmp(argv[i], "--tiers")) {
            cfg.allTiers = true;
            --i;
            continue;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", argv[i]);
            return 2;
        }
        const double v = std::atof(argv[i + 1]);
        if (!std::strcmp(argv[i], "--tier")) {
            if (!std::strcmp(argv[i + 1], "full")) cfg.tier = pm::QualityTier::Full;
            else if (!std::strcmp(argv[i + 1], "balanced")) cfg.tier = pm::QualityTier::Balanced;
            else if (!std::strcmp(argv[i + 1], "eco")) cfg.tier = pm::QualityTier::Eco;
            else {
                std::fprintf(stderr, "unknown tier %s\n", argv[i + 1]);
                return 2;
            }
        } else if (!std::strcmp(argv[i], "--timing")) {
            if (!std::strcmp(argv[i + 1], "on")) timingMode = 1;
            else if (!std::strcmp(argv[i + 1], "off")) timingMode = 0;
            else if (!std::strcmp(argv[i + 1], "both")) timingMode = 2;
            else {
                std::fprintf(stderr, "unknown timing mode %s\n", argv[i + 1]);
                return 2;
            }
        } else if (!std::strcmp(argv[i], "--seconds")) cfg.seconds = v;
        else if (!std::strcmp(argv[i], "--block")) cfg.block = static_cast<int>(v);
        else if (!std::strcmp(argv[i], "--fs")) cfg.sampleRate = v;
        else {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        }
    }
    const double fs = cfg.sampleRate;
    const int block = cfg.block;

    auto eng = std::make_unique<pm::Engine>();
    if (!eng->prepare(fs, block)) return 1;
    std::printf("engine benchmark: fs %.0f Hz, block %d, kernel %s, %.1f s measured; %d normal modes "
                "in %d groups of %d\n",
                fs, block, pm::simdKernelName(), cfg.seconds, eng->stats().totalModes, eng->stats().groups,
                pm::ModeBank::kGroupSize);
    {
        std::vector<double> t(501);
        float sink = 0.0f;
        for (double& x : t) {
            float r = 0.0f;
            x = pm::host::timeSpeedProbe(&r);
            sink += r;
        }
        std::sort(t.begin(), t.end());
        std::printf("  speed probe on this thread: min %.2f / median %.2f us (checksum %g)\n", t[0], t[t.size() / 2],
                    static_cast<double>(sink));
    }
    const char* tierName[3] = {"full", "balanced", "eco"};
    std::printf("  board-on cases at tier %s unless named; phase timing %s\n", tierName[static_cast<int>(cfg.tier)],
                timingMode == 0 ? "off" : (timingMode == 1 ? "on" : "off, then on"));
    const int nCases = pm::bench::caseCount(cfg);
    std::vector<pm::bench::ScenarioResult> results(static_cast<std::size_t>(nCases));
    std::vector<pm::bench::ScenarioResult> untimed(static_cast<std::size_t>(nCases));
    for (int i = 0; i < nCases; ++i) {
        const pm::bench::Case c = pm::bench::caseAt(cfg, i);
        pm::bench::ScenarioResult& r = results[static_cast<std::size_t>(i)];
        if (timingMode == 2) {
            pm::bench::Config off = cfg;
            off.phaseTiming = false;
            if (!pm::bench::runScenario(off, c, *eng, untimed[static_cast<std::size_t>(i)])) return 1;
        }
        pm::bench::Config run = cfg;
        run.phaseTiming = timingMode != 0;
        if (!pm::bench::runScenario(run, c, *eng, r)) return 1;
        std::printf("  %-35s %6.2f %% of one core; first 100 ms %6.2f %%; buffers p99 %6.2f %%, "
                    "max %6.2f %%; awake modes mean %5.0f, at end %5d\n",
                    c.name, r.avgPercent, r.first100msPercent, r.p99Percent, r.maxPercent,
                    r.meanAwakeModes, r.awakeModesEnd);
    }
    if (timingMode == 2) {
        std::printf("  phase timing off vs on, us per buffer (mean / p99 / max):\n");
        for (int i = 0; i < nCases; ++i) {
            const pm::bench::ScenarioResult& a = untimed[static_cast<std::size_t>(i)];
            const pm::bench::ScenarioResult& b = results[static_cast<std::size_t>(i)];
            std::printf("    %-35s off %7.1f / %7.1f / %7.1f   on %7.1f / %7.1f / %7.1f   mean +%.1f %%\n",
                        pm::bench::caseAt(cfg, i).name, a.meanUs, a.p99Us, a.maxUs, b.meanUs, b.p99Us, b.maxUs,
                        a.meanUs > 0.0 ? 100.0 * (b.meanUs / a.meanUs - 1.0) : 0.0);
        }
    }
    std::printf("  (awake modes exclude group padding; with culling on, partials more than 100 dB\n"
                "   down are never processed. The max of a single buffer includes any preemption\n"
                "   by the OS.)\n");

    std::printf("  most expensive buffer: hammers that touched a string / most at once / contact samples,\n"
                "  Newton evaluations / fallbacks; zone kernel per buffer on average: full rebuilds, key updates\n");
    for (int i = 0; i < nCases; ++i) {
        const pm::bench::ScenarioResult& r = results[static_cast<std::size_t>(i)];
        const pm::bench::BufferDetail& d = r.maxDetail;
        std::printf("    %-35s hammers %2d / %2d / %5d, Newton %6d / %d; kernel %.2f rebuilds, %.2f key updates\n",
                    pm::bench::caseAt(cfg, i).name, d.hammersInContact, d.maxHammersInContact, d.hammerContactSamples,
                    d.newtonEvals, d.fallbackSolves, r.kernelRebuildsPerBuffer, r.kernelKeyUpdatesPerBuffer);
    }

    if (timingMode != 0) {
        std::printf("  per-phase breakdown, us per %d-frame buffer (%.0f us = 100 %% of one core):\n"
                    "    %-35s %-14s %7s %7s %7s %7s %7s %7s %8s\n",
                    block, 1e6 * block / fs, "case", "buffers", "events", "hammer", "free", "forced", "board",
                    "other", "total");
        for (int i = 0; i < nCases; ++i) {
            const pm::bench::ScenarioResult& r = results[static_cast<std::size_t>(i)];
            char label[3][32];
            std::snprintf(label[0], sizeof label[0], "max (#%d)", r.maxIndex);
            std::snprintf(label[1], sizeof label[1], "top 1%% (%d)", r.topCount);
            std::snprintf(label[2], sizeof label[2], "mean");
            const pm::bench::Phases* rows[3] = {&r.maxBuf, &r.top1, &r.all};
            for (int j = 0; j < 3; ++j) {
                std::printf("    %-35s %-14s", j == 0 ? pm::bench::caseAt(cfg, i).name : "", label[j]);
                for (int p = 0; p < pm::bench::kPhaseCount; ++p) std::printf(" %7.1f", rows[j]->us[p]);
                std::printf(" %8.1f\n", rows[j]->total);
            }
        }
        std::printf("  (events: event handling, control tick, wake-ups, impulse-response cache and kernel\n"
                    "   updates; hammer: contact solves; free: the bank's block pass (board off: the whole\n"
                    "   bank); forced: input added to the coupled modes; board: zone kernel, noise, knocks\n"
                    "   and board step; other: output, filters, limiter, cull passes, hammer release.)\n");
    }

    // One hammer alone, per key: median over the strikes at mf.
    std::printf("  hammer solver, one hammer, us per %d-frame buffer (median of %d mf strikes):\n", block,
                cfg.hammerStrikes);
    for (int key : pm::bench::kHammerKeys) {
        pm::bench::HammerResult h;
        if (!pm::bench::runHammer(cfg, key, h)) return 1;
        std::printf("    key %2d (%d strings, %4d modes, R=%d): %6.1f in contact, %6.1f averaged over its life\n",
                    h.key, h.strings, h.modes, h.oversampling, h.contactUs, h.lifeUs);
    }
    return 0;
}
