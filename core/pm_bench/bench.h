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
//   (with allTiers: worst, cluster and bloom at Balanced and Eco)
// then, after those, scenes with the events of live playing:
//   legato:   a melody with overlapping notes, each released while it
//             sounds, and a bass line, pedal up
//   pedalling: chords every 400-800 ms with legato pedalling, CC64 as
//             ramps of many values and as plain on/off
//   chord release: ten notes struck and released together, every 1.5 s
//   idle:     nothing sounding (the cost of an idle engine)
//   hammer burst: ten bass and middle notes at velocity 127 struck at the
//             same sample (as a chord played hard arrives in one buffer),
//             every second, released after half a second, pedal up
//   pedalled chords: chords on eight different roots across the keyboard,
//             one every 0.7 s, with legato pedalling (CC64 as plain on/off,
//             as a switch pedal sends it): the sustained strings' wake-ups
//             and cull decisions change the loop's set of awake modes on
//             most sub-blocks (zone-kernel updates), as in live playing
//   fresh-key burst: pedal down, a sustained bass and chord sounding, and
//             every 0.6 s a burst of six notes at velocity 100-127 struck in
//             one buffer, on keys drawn at random (fixed seed) from those not
//             struck for at least 6 s, released 0.25 s later (some before
//             their hammers have finished, so dampers land on active
//             hammers); legato pedal changes every 2.4 s
//   same-key burst: identical, except that every burst strikes the same six
//             keys (the warm counterpart: compare the hammer phase per
//             hammer-sample of the two)
//   hammer:   cost of one active hammer per buffer, per key, while in
//             contact and averaged over its life (from the launch until
//             hammerFinished: the release wait, or since the sample path's
//             early release, until the hammer provably cannot return),
//             measured on the solver alone
//
// Pacing (Config::paced): by default the buffers are rendered back to back,
// as fast as the thread can, with everything the engine touches hot in the
// caches. Paced, the thread sleeps after each buffer until that buffer's
// real-time deadline (start + (b + 1) * period), as an audio thread does, so
// the engine wakes up every period with whatever caches, predictors and core
// state the gap left; only the render call is timed. evictBytes > 0 also
// walks a scratch buffer of that size (read-modify-write, one access per
// cache line) after each render, outside the timing, to emulate other work
// on the core between buffers (pick it larger than the L2 cache). spin
// busy-waits to the deadline instead of sleeping: the core stays awake and
// at speed and the caches keep what the engine left, so paced-spin against
// back to back isolates nothing but the gap, spin + eviction the caches
// alone, and sleep against spin the core's power state and clock.
//
// Phase timing (Config::phaseTiming) is the engine's run-time switch, the
// same one the app's diagnostics use; bench_engine turns it on for its
// per-phase breakdown and can run every case with it off as well, to
// measure what the timers themselves cost.
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

#include "pm_engine/engine.h"

namespace pm::bench {

struct Config {
    double sampleRate = 48000.0;
    int block = 64;          // frames per render call
    double seconds = 3.0;    // measured per scenario
    QualityTier tier = QualityTier::Full;  // board-on scenarios
    bool allTiers = false;   // also worst/cluster/bloom at Balanced and Eco
    int hammerStrikes = 200; // per key, hammer micro-benchmark
    bool phaseTiming = false; // Engine::setPhaseTiming during the scenarios (always on in the profiling build)
    bool paced = false;       // sleep to each buffer's real-time deadline (see above)
    bool spin = false;        // paced only: busy-wait to the deadline instead of sleeping (core stays awake)
    std::size_t evictBytes = 0;  // paced only: scratch walked between buffers (0: none)
    int probeEvery = 0;       // > 0: the speed probe right before every Nth render call (untimed), as the host does
};

// Default eviction buffer of --paced-evict / PM_BENCH_PACED_EVICT: larger
// than the L2 of the cores in question (Apple M-series P-cluster 16 MiB, x86
// 1-2 MiB per core) and than most last-level caches' share per core.
inline constexpr std::size_t kDefaultEvictBytes = std::size_t{32} << 20;

enum class Scene : int {
    Worst = 0, Typical = 1, Cluster = 2, Bloom = 3,
    Legato = 4, Pedalling = 5, ChordRelease = 6, Idle = 7,
    HammerBurst = 8, PedalChords = 9, FreshBurst = 10, SameBurst = 11
};

// Every event of a scene up to sample 'end' (sample-time stamps), in time
// order; events at the same time keep the order in which they are generated.
void sceneEvents(Scene s, double sampleRate, std::uint64_t end, std::vector<Event>& out);

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

// Load diagnostics of a scenario's most expensive buffer (RenderInfo).
struct BufferDetail {
    int hammersInContact = 0, maxHammersInContact = 0, hammerContactSamples = 0;
    int newtonEvals = 0, fallbackSolves = 0;
    int kernelRebuilds = 0, kernelKeyUpdates = 0, responseGroups = 0, damperUpdates = 0;
    int hammerSamples = 0, hammerDampedSamples = 0;  // RenderInfo: hammer-samples stepped / with a damper on
    int awakeModes = 0, freeGroupBlocks = 0, forcedGroupBlocks = 0;
    int loopMembers = 0, loopListeners = 0, loopEntries = 0, outZones = 0;
    int noteOns = 0, noteOffs = 0, pedalEvents = 0;
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
    BufferDetail maxDetail;    // the most expensive buffer by wall time
    double kernelRebuildsPerBuffer = 0.0, kernelKeyUpdatesPerBuffer = 0.0;
    // Means per buffer over the measured buffers (RenderInfo).
    double freeGroupBlocksPerBuffer = 0.0, forcedGroupBlocksPerBuffer = 0.0;
    double loopMembersMean = 0.0, loopListenersMean = 0.0;
    // Hammer phase normalised (phase timing only; 0 without hammer work):
    // us of the hammers phase per hammer-sample stepped (every active
    // hammer, in flight, in contact or waiting for its release) and per
    // hammer-sample in contact, summed over the measured buffers.
    long hammerSamples = 0, hammerContactSamples = 0, hammerDampedSamples = 0;
    double hammerPhaseUs = 0.0;
    double hammerUsPerSample = 0.0, hammerUsPerContactSample = 0.0;
    int overruns = 0;  // buffers whose render took longer than the period
    // Speed probe before the render calls (Config::probeEvery): runs, median
    // and mean (us). After a sleep it shows whether the core itself is
    // slower on waking (clock, power state) rather than its caches colder.
    int probeRuns = 0;
    double probeMedianUs = 0.0, probeMeanUs = 0.0;
    bool paced = false, spin = false;
    std::size_t evictBytes = 0;
};

// Configures 'eng' for the case (culling, soundboard, tier) and prepares it
// (non-real-time, about a second).
bool prepareForCase(const Case& k, Engine& eng, double sampleRate, int maxFrames);

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
