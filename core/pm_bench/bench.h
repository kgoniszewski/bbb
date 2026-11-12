// Offline engine benchmark (the scenarios of tools/bench/bench_engine), as a
// library so that the app can run exactly the same measurements on the
// device (C API: pm_bench_*). Non-real-time: every call prepares its own
// engine and renders on the calling thread, as fast as it can.
//
//   worst:    all 88 keys struck (1 ms apart) and held, sustain pedal down
//   no-cull:  the same with culling disabled (every mode of every key)
//   typical:  a four-note chord re-struck every 0.5 s and released, pedal up
//   cluster:  a 12-note cluster (C3..B3 plus C5..B5 alternating) struck ff
//             every 100 ms: 12 hammers in contact at once, repeatedly
//   bloom:    sustain pedal down, a staccato four-note chord every 0.5 s
//   board off: worst and cluster without the soundboard (A/B)
//   hammer:   cost of one active hammer per buffer, per key, while in
//             contact and averaged over its life (from the launch until
//             hammerFinished: the release wait, or since the sample path's
//             early release, until the hammer provably cannot return),
//             measured on the solver alone
//
// Compiled twice: pm_bench (the real-time engine library, for the C API and
// the app) and pm_bench_profile (with PM_PROFILE against the profiling
// engine, for bench_engine's per-phase breakdown). The Engine class differs
// between the two, so each variant must be linked with its own engine.
#pragma once

#include <cstdint>

#include "pm_engine/engine.h"

namespace pm::bench {

struct Config {
    double sampleRate = 48000.0;
    int block = 64;          // frames per render call
    double seconds = 3.0;    // measured per scenario
    QualityTier tier = QualityTier::Full;  // board-on scenarios
    bool allTiers = false;   // also worst/cluster/bloom at Balanced and Eco
    int hammerStrikes = 200; // per key, hammer micro-benchmark
};

enum class Scene : int { Worst = 0, Typical = 1, Cluster = 2, Bloom = 3 };

struct Case {
    const char* name;  // static string
    bool cull;
    Scene scene;
    bool board;
    QualityTier tier;
};

// Engine scenarios, in bench_engine's order.
int caseCount(const Config& c) noexcept;
Case caseAt(const Config& c, int index) noexcept;

// Keys of the hammer micro-benchmark.
inline constexpr int kHammerKeys[8] = {1, 16, 28, 40, 52, 64, 76, 88};
inline constexpr int kHammerKeyCount = 8;
const char* hammerName(int index) noexcept;  // "hammer, key 40" (static)

// Per-phase breakdown of one buffer (us): events, hammer, free, forced,
// board, other (EngineProfile, engine.h). Only measured in the profiling
// build (hasPhases).
inline constexpr int kPhaseCount = 6;
struct Phases {
    double us[kPhaseCount] = {};
    double total = 0.0;
};

struct ScenarioResult {
    double periodUs = 0.0;
    double avgPercent = 0.0, p99Percent = 0.0, maxPercent = 0.0;
    double first100msPercent = 0.0;  // mean over the first 100 ms measured
    double meanUs = 0.0, p99Us = 0.0, maxUs = 0.0;  // render time per buffer
    double meanAwakeModes = 0.0;
    int awakeModesEnd = 0;
    int buffers = 0;
    bool hasPhases = false;
    Phases maxBuf, top1, all;  // the most expensive buffer, mean of the top 1 %, mean of all
    int maxIndex = 0, topCount = 0;
};

// Prepares 'eng' for the case (culling, soundboard, tier) and runs it.
// Returns false if the engine could not be prepared.
bool runScenario(const Config& c, const Case& k, Engine& eng, ScenarioResult& out);

struct HammerResult {
    int key = 0, strings = 0, modes = 0, oversampling = 0;
    double contactUs = 0.0;  // median over the strikes: first buffer (in contact)
    double lifeUs = 0.0;     // median over the strikes: mean per buffer until finished
};

bool runHammer(const Config& c, int key, HammerResult& out);

}  // namespace pm::bench
