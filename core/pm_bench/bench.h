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
//   pedal lift: many strings sounding under the held sustain pedal (a
//             two-hand chord of 12 notes struck every 1.2 s and released
//             into the pedal, four bass and middle keys held throughout);
//             every 0.6 s the pedal comes up for 150 ms and goes down
//             again (alternately CC64 on/off and ramps of many values), and
//             three notes are struck as it goes down: the dampers land on
//             and lift off about 9000 sounding modes, as in the iPad's
//             worst live buffer (the damper amounts change on every control
//             tick while the felts pass through their transition)
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
// Scheduling class: the paced loop runs on the calling thread, in whatever
// scheduling class that thread has. A real audio thread is a time-constraint
// (real-time) thread in the audio device's workgroup, whose deadlines the
// system's performance controller sees; an ordinary thread that sleeps
// between buffers is not. Config::paceHook lets the caller put the paced
// loop into an audio thread's class (the app: time-constraint policy and an
// audio work interval, start / finish per buffer with its deadline) without
// platform code here.
//
// Load multiplier (Config::load = K, 1..kMaxLoad): every buffer is rendered
// by K independent engines, each prepared for the case and fed the same
// events, one after the other inside the timed interval; the first engine's
// output and work counts are reported, the others render into scratch. The
// work per buffer is then exactly K times the scene's, with K times its
// working set (about 6 MB per engine), which shows whether a thread whose
// core and clock the system chooses gets K times the work done in time.
// Phase splits are the first engine's.
//
// CPU numbers (cpu_id.h) are read right before and after every measured
// render call (outside the timed interval) and tallied per CPU.
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
#include "pm_host/cpu_id.h"

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
    int load = 1;             // engines rendered per buffer (load multiplier, see above), 1..kMaxLoad
    // Paced only: called on this thread (event: 0 begin, ns = period; 1
    // before a render call, ns = time to its deadline; 2 after it; 3 end).
    void (*paceHook)(void* ctx, int event, std::int64_t ns) = nullptr;
    void* paceCtx = nullptr;
};

// Default eviction buffer of --paced-evict / PM_BENCH_PACED_EVICT: larger
// than the L2 of the cores in question (Apple M-series P-cluster 16 MiB, x86
// 1-2 MiB per core) and than most last-level caches' share per core.
inline constexpr std::size_t kDefaultEvictBytes = std::size_t{32} << 20;

// Largest load multiplier: 8 engines of about 6 MB each.
inline constexpr int kMaxLoad = 8;

enum class Scene : int {
    Worst = 0, Typical = 1, Cluster = 2, Bloom = 3,
    Legato = 4, Pedalling = 5, ChordRelease = 6, Idle = 7,
    HammerBurst = 8, PedalChords = 9, FreshBurst = 10, SameBurst = 11, PedalLift = 12
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
    int approxResponses = 0;  // coupled-group responses from the damper expansion (over budget)
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
    // Most damper updates and exact / expanded coupled-group responses in
    // one buffer over the run (RenderInfo).
    int maxDamperUpdates = 0, maxResponseGroups = 0, maxApproxResponses = 0;
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
    double probeMedianUs = 0.0, probeMeanUs = 0.0;  // the whole probe
    struct ProbeStats {
        int runs = 0;
        double minUs = 0.0, medianUs = 0.0, meanUs = 0.0, maxUs = 0.0;
    };
    ProbeStats probeLatency, probeThroughput;  // its two parts (speed_probe.h)
    host::CpuUsage cpu;  // CPUs of the measured render calls
    int load = 1;                    // engines rendered per buffer
    std::size_t engineBytes = 0;     // load > 1: memory of one prepared engine (object + aligned arrays)
    bool paced = false, spin = false;
    std::size_t evictBytes = 0;
};

// Configures 'eng' for the case (culling, soundboard, tier) and prepares it
// (non-real-time, about a second). bytes (optional): the memory the prepared
// engine holds, sizeof(Engine) plus its aligned arrays (measured as the
// change of the process-wide aligned-array total, so only exact while no
// other thread allocates aligned arrays at the same time).
bool prepareForCase(const Case& k, Engine& eng, double sampleRate, int maxFrames, std::size_t* bytes = nullptr);

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
