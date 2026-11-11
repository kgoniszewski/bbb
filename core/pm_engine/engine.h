// Piano engine, M4 (plan §4.2): 88 keys of coupled unisons (1-3 strings in
// two polarizations, as normal modes), a hammer pool, dampers, continuous
// sustain (half-pedal), sostenuto and una corda pedals, a modal soundboard
// coupled to every string through the bridge, sample-accurate event
// scheduling and a placeholder output stage.
//
// Soundboard loop (plan §1.6, pm_board/soundboard.h, soundboard_model.h).
// Per sample: the hammers and the coupled string modes (partials inside the
// loop's band) advance with the bridge displacement computed at the
// previous sample; every key's bridge force goes to the zone forces; the
// board advances and returns zone displacements, mixed into each key's
// input for the next sample (times the sympathetic gain). String modes
// above the loop's band only drive the board. The recursion is evaluated
// exactly but block-wise (CouplingLoop): only the hammers, an 18-zone core
// and the board are stepped per sample. The bake of every key excludes exactly what the loop feeds
// back (no double counting). Sustain-pedal bloom, sympathetic resonance and
// string-board hybridisation all come out of this loop; there is no special
// case for any of them.
//
// Output (PLACEHOLDER until the radiation model, M5/M8): the board velocity
// at two pickup points (bass side -> left, treble side -> right), plus a
// high-frequency direct path for partials above the board's modal range:
// the panned bridge forces, high-passed at 0.9 times the modal range and
// scaled by the board's mean transfer admittance to the pickups near the
// top of that range. Stereo width scales the side signal. Level (with the
// MIDI dynamics mf = 80, ff = 112): mf single notes C3-C5 peak at -33 to
// -22 dBFS, velocity 127 at -17 dBFS at most; 12-note clusters every 100 ms
// at -7 (ff) to -5 dBFS (127, pedal down); all 88 keys at velocity 127 at
// once -1.4 dBFS, just below the limiter.
//
// Pedals. Sustain (CC64) is continuous (plan §1.9). The pedal position is
// smoothed (pedal and lever inertia, 10 ms) and mapped per register to a
// felt position: from pressed on the string (full damping) through light
// contact, where the felt damps a vibration only for the part of the cycle
// it reaches the felt (amplitude-dependent, 2/pi acos(gap / amplitude) of
// the contact damping), to clear. Bass dampers lift later than treble ones.
// The key's own damper motion (lift on note-on, landing curve on note-off)
// multiplies the pedal's factor. Nothing is ever reset: only decay rates
// change. Sostenuto (CC66): when it goes down (half travel), the keys whose
// dampers are lifted by their own keys at that moment are latched and stay
// undamped until it comes up. Una corda (CC67) is continuous; it is latched
// per strike when the hammer is launched (offsets and felt of that strike),
// so moving it never changes a sounding note or a contact in progress.
//
// Mechanical noises (plan §1.8, synthesized, switchable): a hammer-knock
// force pulse into the key's bridge zones (1.5 ms raised cosine, peak
// proportional to hammer velocity), and low-passed noise into every zone
// while the sustain pedal moves the dampers (proportional to pedal speed
// and to the share of dampers it moves).
//
// Threads. prepare() and the velocity-curve setter are non-real-time.
// pushEvent() may be called from ONE producer thread concurrently with
// render(). The other parameter setters are atomic and may be called from
// any thread. render() is real-time safe: no allocation, locks, exceptions
// or I/O, and its cost does not depend on the buffer size beyond the work
// itself (the buffer is cut into sub-blocks of at most kSubBlock frames).
//
// Determinism. The buffer is split at event times, on an absolute grid of
// kSubBlock frames (control-rate state: damper envelopes) and on a grid
// relative to each active hammer's launch (hammer release), and every
// decision with a lasting effect (wake-ups, hammer release, damper motion)
// is taken at a sample fixed relative to those. Output therefore does not
// depend on how the host cuts the stream into buffers, and an event lands
// on its exact sample: a note-on moved by k samples gives the same output
// moved by k samples. Both hold to rounding (-100 to -110 dB re peak), not
// bit-exactly: the soundboard loop sums its forced response per sub-block,
// so where a buffer boundary or an event cuts a sub-block changes the
// rounding. (With culling, whose 64-sample grid is absolute, a moved note
// also differs to within the culling threshold.)
//
// Soundboard off (SoundboardConfig::enabled = false, for A/B, or prepare()
// with an explicit bridge): the M3 output, the sum of the keys' bridge
// forces (times outputGain) through a fixed stereo spread, with the board's
// full driving-point admittance baked and no loop.
//
// Both outputs pass a 5 Hz DC blocker and a soft clipper that is transparent
// below -1 dBFS (0.89) and never reaches 1.0.
#pragma once

#include <atomic>
#include <cstdint>

#include "pm_base/aligned.h"
#include "pm_base/spsc_queue.h"
#include "pm_board/soundboard.h"
#include "pm_engine/events.h"
#include "pm_hammer/hammer.h"
#include "pm_string/mode_bank.h"
#include "pm_voicing/bridge_admittance.h"
#include "pm_voicing/piano_table.h"
#include "pm_voicing/soundboard_model.h"

namespace pm {

enum class SchedulingMode : int {
    LowestLatency = 0,    // events already due go to the start of the current buffer
    ConstantLatency = 1,  // host-time events are delayed by a fixed latency, then exact
};

struct EngineStats {
    int totalModes = 0;           // in use (excluding group padding)
    int groups = 0;
    int awakeGroups = 0;
    int awakeModes = 0;           // in awake groups, excluding padding
    int activeHammers = 0;
    int peakActiveHammers = 0;
    std::uint64_t droppedEvents = 0;     // queue or pending list full
    std::uint64_t deferredStrikes = 0;   // note-ons that waited for a free hammer
    std::uint64_t lateEvents = 0;        // constant-latency events that arrived too late
    std::uint64_t limitedSamples = 0;    // samples where the soft clipper acted
    std::uint64_t nonFiniteSamples = 0;  // replaced by zero (should stay 0)
    float peakPreLimiter = 0.0f;
};

#if defined(PM_PROFILE)
// Per-phase wall time of the last render() call, in microseconds. Only in
// the profiling build of the engine (pm_engine_profile, which bench_engine
// links); the real-time library has no timers. Everything not listed
// (output mixing and filters, limiter, cull passes, hammer release,
// statistics) is totalUs minus the sum of the phases.
struct EngineProfile {
    enum Phase : int {
        Events = 0,  // event handling, control tick (dampers, pedal), loop begin: wake-ups, impulse-response cache, kernel rebuild
        Hammers,     // hammer contact solves (board on: per sample, inside the loop)
        FreePass,    // bank block pass (board on: free responses; board off: the whole bank)
        ForcedPass,  // loop end: input added to the coupled modes (forced kernel), sub-block wake-ups
        Board,       // zone kernel (free forces, convolution), noise, knocks, board step
        kPhases
    };
    double us[kPhases] = {};
    double totalUs = 0.0;
};
#endif

// Reported once per strike, when its hammer has left the string for good.
struct StrikeInfo {
    std::uint64_t sampleTime = 0;  // launch
    int note = 0;
    float velocity = 0.0f;         // m/s
    float contactSpanMs = 0.0f;    // first to last contact
    float contactTimeMs = 0.0f;    // total time in contact
    int contacts = 0;              // separate contact episodes
    float peakForce = 0.0f;        // N
    float feltLossFraction = 0.0f; // of the hammer's kinetic energy
    float unaCorda = 0.0f;         // pedal amount latched for this strike
    int fallbackSolves = 0;        // contact substeps solved by the fallback
};

// Quality tiers of the string-board loop (plan §4.1): which partials are
// fed back by the loop (sympathetic exchange, bloom, hybridisation with
// board modes) and which bake the full driving-point admittance and only
// drive the board. The cost of the loop is proportional to the fed-back
// partials. Bands: Full 5 kHz (the board's modal range), Balanced 2 kHz
// (half the fed-back modes; pedal bloom within 0.7 dB per third octave of
// Full), Eco 1 kHz (a quarter; within 1.3 dB, 2.8 dB one second later).
enum class QualityTier : int { Full = 0, Balanced = 1, Eco = 2 };
// Loop band of a tier (Hz; Full: the board's whole modal range).
double loopCutoffForTier(QualityTier t, const PlateParams& plate) noexcept;

// Prepare-time soundboard settings (table class: they need prepare(), i.e.
// a rebuild on a worker thread in an app, because the bake depends on them).
struct SoundboardConfig {
    bool enabled = true;       // false: no board, M3 output (A/B)
    double sympathetic = 1.0;  // loop gain g, 0..1.5: 0 = the board listens, nothing is fed back
    QualityTier tier = QualityTier::Full;
    double loopCutoffHz = 0.0;  // > 0: overrides the tier's loop band
    PlateParams plate{};
};

// Half-pedal lift curve (plan §1.9). Pedal position P in 0..1; the key's
// felt starts to move at p0 (p0Bass for key 1, p0Treble for key 88, linear
// in between), passes through light contact up to p0 + width (smoothstep
// in between) and lifts clear above.
struct HalfPedalCurve {
    float p0Treble = 0.30f, p0Bass = 0.40f, width = 0.25f;
};

class Engine {
public:
    static constexpr int kHammerPool = 32;
    static constexpr int kSubBlock = 32;     // also the control-rate grid
    static_assert(kSubBlock <= CouplingLoop::kMaxBlock, "sub-block longer than the loop's kernel");
    static constexpr int kQueueSize = 1024;
    static constexpr int kMaxPending = 1024;
    static constexpr double kPanicSigma = 70.0;  // 1/s: every partial T60 <= 0.1 s
    static constexpr double kPanicSeconds = 0.3;
    // Felt geometry of the half-pedal model (m) and the damping fraction at
    // the point of light contact.
    // Through the transition zone the felt rises from kFeltPress into the
    // string to kFeltZone above it (comparable to the strings' vibration,
    // so that contact is partial across the zone); above the zone it lifts
    // on to kFeltGap at full pedal.
    static constexpr float kFeltPress = 0.15e-3f, kFeltZone = 0.1e-3f, kFeltGap = 3.0e-3f, kLightContact = 0.15f;
    static constexpr int kMaxKnocks = 32;
    // Output scale of the board path, per m/s of pickup velocity.
    static constexpr float kBoardOutputGain = 2.4f;

    Engine() = default;
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Non-real-time. Builds the default table (A4 = 440 Hz, default stretch
    // and unison width) or uses 'table' (with the soundboard, its stretch
    // tuning is redone on the sounding partials, key_loader.h
    // tuneToSoundingPartials; table() holds the result), computes every key's normal modes
    // (with the placeholder bridge admittance unless 'bridge' is given),
    // allocates everything and resets all state. maxFrames is the largest
    // buffer the host will render (used as the default constant latency).
    bool prepare(double sampleRate, int maxFrames) noexcept;
    bool prepare(double sampleRate, int maxFrames, const PianoTable& table) noexcept;
    // Soundboard off, with the given bridge admittance baked (tests, A/B).
    bool prepare(double sampleRate, int maxFrames, const PianoTable& table,
                 const BridgeAdmittance& bridge) noexcept;
    // Non-real-time, before prepare().
    void setSoundboardConfig(const SoundboardConfig& c) noexcept { boardCfg_ = c; }
    const SoundboardConfig& soundboardConfig() const noexcept { return boardCfg_; }
    bool prepared() const noexcept { return prepared_; }

    // Producer thread. Returns false (and counts a drop) when the queue is full.
    bool pushEvent(const Event& e) noexcept;

    // Real-time. Any nFrames (no maximum). clock may be null: host-time events
    // are then due immediately.
    void render(float* left, float* right, int nFrames, const ClockMapping* clock) noexcept;

    // Atomic parameters, any thread.
    void setMasterGainDb(float db) noexcept { masterGainDb_.store(db, std::memory_order_relaxed); }
    void setStereoWidth(float w) noexcept { width_.store(w, std::memory_order_relaxed); }
    void setSchedulingMode(SchedulingMode m) noexcept {
        schedMode_.store(static_cast<int>(m), std::memory_order_relaxed);
    }
    void setConstantLatencyFrames(int frames) noexcept {
        latency_.store(frames < 0 ? 0 : frames, std::memory_order_relaxed);
    }
    // Mechanical noises: hammer knock (N of peak force per m/s of hammer
    // velocity) and damper-lift noise (N rms per unit of pedal travel per
    // second, at full share of dampers moved); 0 switches a noise off.
    void setNoiseLevels(float knock, float damperLift) noexcept {
        knockLevel_.store(knock < 0.0f ? 0.0f : knock, std::memory_order_relaxed);
        liftNoiseLevel_.store(damperLift < 0.0f ? 0.0f : damperLift, std::memory_order_relaxed);
    }
    float knockLevel() const noexcept { return knockLevel_.load(std::memory_order_relaxed); }
    float damperNoiseLevel() const noexcept { return liftNoiseLevel_.load(std::memory_order_relaxed); }
    void setHalfPedalCurve(const HalfPedalCurve& c) noexcept {
        hpP0T_.store(c.p0Treble, std::memory_order_relaxed);
        hpP0B_.store(c.p0Bass, std::memory_order_relaxed);
        hpW_.store(c.width < 0.01f ? 0.01f : c.width, std::memory_order_relaxed);
    }
    HalfPedalCurve halfPedalCurve() const noexcept {
        return {hpP0T_.load(std::memory_order_relaxed), hpP0B_.load(std::memory_order_relaxed),
                hpW_.load(std::memory_order_relaxed)};
    }
    // Keys whose damper is seated are woken (or kept awake) by sympathetic
    // input only above the culling threshold plus this many dB (plan §1.5:
    // with the pedal up only held notes and the undamped top section stay
    // awake). The bank's input bound is frequency-blind, so it cannot tell
    // that a damped string hardly responds away from its partials; the
    // default 120 dB means a seated damper's strings are never woken by the
    // bridge (they drop out of the loop until struck or lifted). Their own
    // decay (a release tail) is culled at the normal threshold.
    void setDampedCullOffsetDb(float db) noexcept {
        dampedCullDb_.store(db < 0.0f ? 0.0f : db, std::memory_order_relaxed);
    }
    float dampedCullOffsetDb() const noexcept { return dampedCullDb_.load(std::memory_order_relaxed); }
    void setCulling(bool enabled, float thresholdDb) noexcept {
        cullOn_.store(enabled, std::memory_order_relaxed);
        cullDb_.store(thresholdDb, std::memory_order_relaxed);
    }
    float masterGainDb() const noexcept { return masterGainDb_.load(std::memory_order_relaxed); }
    float stereoWidth() const noexcept { return width_.load(std::memory_order_relaxed); }
    SchedulingMode schedulingMode() const noexcept {
        return static_cast<SchedulingMode>(schedMode_.load(std::memory_order_relaxed));
    }
    int constantLatencyFrames() const noexcept { return latency_.load(std::memory_order_relaxed); }
    float cullThresholdDb() const noexcept { return cullDb_.load(std::memory_order_relaxed); }
    bool cullingEnabled() const noexcept { return cullOn_.load(std::memory_order_relaxed); }

    // Non-real-time; at most one thread. Replaces the velocity -> hammer speed
    // mapping (normalised velocity 0..1 -> m/s). The function is sampled into
    // a table, so it is never called on the audio thread.
    void setVelocityFunction(VelocityFn fn, const void* user) noexcept;
    double hammerVelocityFor(double normalizedVelocity) const noexcept;

    // Consumer side of the strike log (one non-real-time reader).
    bool popStrikeInfo(StrikeInfo& info) noexcept { return strikeLog_.pop(info); }

    // Safe to read from another thread (relaxed snapshot).
    EngineStats stats() const noexcept;
#if defined(PM_PROFILE)
    // Profiling build only: the phases of the last render() (same thread).
    const EngineProfile& profile() const noexcept { return prof_; }
#endif

    // Inspection, for tools and tests (not thread-safe against render()).
    double sampleRate() const noexcept { return fs_; }
    std::uint64_t sampleTime() const noexcept { return sampleTime_; }
    const PianoTable& table() const noexcept { return table_; }
    const ModeBank& bank() const noexcept { return bank_; }
    int keyModeCount(int key) const noexcept { return keyModes_[key]; }
    int freeHammers() const noexcept { return freeCount_; }
    float sustain() const noexcept { return sustain_; }        // pedal target 0..1
    float pedalPosition() const noexcept { return pedal_; }    // smoothed
    bool soundboardActive() const noexcept { return boardOn_; }
    const Soundboard& soundboard() const noexcept { return board_; }
    const BoardModel& boardModel() const noexcept { return boardModel_; }
    float sympatheticGain() const noexcept { return symGain_; }
    // Key segment layout (bank modes incl. padding) and the key's coupled modes.
    int keyCoupledModes(int key) const noexcept { return keyCoupled_[key]; }
    bool sostenutoDown() const noexcept { return sostenuto_; }
    bool sostenutoLatched(int key) const noexcept { return sostLatched_[key]; }
    float unaCorda() const noexcept { return unaCorda_; }
    // Normal modes that needed a perturbed eigen-solve / fell back to
    // uncoupled oscillators, over all keys (from prepare).
    int perturbedPartials() const noexcept { return perturbedPartials_; }
    int uncoupledPartials() const noexcept { return uncoupledPartials_; }
    // Damper amount applied to the key's modes (0 lifted, 1 seated).
    float damperAmount(int key) const noexcept { return applied_[key]; }

private:
    struct Pending {
        std::int64_t when;
        std::uint32_t seq;
        Event e;
    };
    struct Damper {
        float cur, from, to;
        std::int64_t start;
        std::int32_t len;
        bool ramping, landing;
    };
    struct Hammer {
        HammerState state;
        HammerParams params;  // latched at launch (una corda)
        int key;
        std::uint64_t launchTime;
        float unaCorda;
    };
    static constexpr std::uint8_t kStrikeOnly = 100;  // internal: deferred hammer launch

    void resetState() noexcept;
    void readParams() noexcept;
    void drainQueue(const ClockMapping* clock) noexcept;
    bool insertPending(std::int64_t when, const Event& e) noexcept;
    void applyEvent(const Event& e, std::int64_t now) noexcept;
    void noteOn(int k, std::uint16_t vel, std::int64_t now) noexcept;
    bool strike(int k, double velocity, std::int64_t now) noexcept;
    void releaseHammer(int slot, bool report) noexcept;
    float damperTarget(int k, std::int64_t now) const noexcept;
    void retargetDamper(int k, std::int64_t now, float releaseVelocity) noexcept;
    void controlTick(std::int64_t now) noexcept;
    void processSubBlock(float* left, float* right, int n, std::int64_t now) noexcept;
    void processBoard(int n) noexcept;
    bool prepareImpl(double sampleRate, int maxFrames, const PianoTable& table, const BridgeAdmittance& bridge,
                     bool board) noexcept;
    float pedalFactor(int k) noexcept;
    void joinLoop(int k) noexcept;
    float keyAmplitude(int k) const noexcept;

    bool prepared_ = false;
    double fs_ = 48000.0;
    int maxFrames_ = 0;
    PianoTable table_{};
    HammerParams hammerParams_[kNumKeys]{};
    // Una corda: offset growth of one string and felt softening, per key.
    int unaString_[kNumKeys]{};
    double unaMiss_[kNumKeys]{}, unaSoften_[kNumKeys]{};
    int keyModes_[kNumKeys]{};
    ModeBank bank_;
    ContactTables contact_;
    AlignedArray<SegmentIO> io_;
    AlignedArray<std::uint8_t> groupModes_;  // modes in use per bank group
    AlignedArray<float> keyBuf_;   // kNumKeys x kSubBlock
    AlignedArray<float> mixL_, mixR_;
    AlignedArray<float> hfL_, hfR_;       // high-frequency direct bus, kSubBlock
    AlignedArray<float> dampShape_;       // per bank mode: displacement at the damper per |z|

    // Soundboard.
    SoundboardConfig boardCfg_{};
    BoardModel boardModel_{};
    Soundboard board_;
    bool boardOn_ = false;
    float symGain_ = 0.0f;
    KeyZone keyZone_[kNumKeys]{};
    int keyCoupled_[kNumKeys]{};
    CouplingLoop loop_;
    float uPrev_[kNumKeys]{};  // last input of hammer-owned keys
    float statK0_[kNumKeys]{}, statK1_[kNumKeys]{};  // static correction taps (soundboard_model.h)
    float zoneF_[Soundboard::kMaxZones]{}, zoneD_[Soundboard::kMaxZones]{}, pick_[Soundboard::kMaxPickups]{};
    float hfGain_ = 0.0f;
    // The culling threshold refers to the output; through the board a bank
    // output unit can reach the output amplified by up to this (dB).
    float cullOffsetDb_ = 0.0f;
    float hpB_[3]{}, hpA_[2]{};           // high-pass biquad (direct form I)
    float hpX_[2][2]{}, hpY_[2][2]{};
    HammerKeyModes hView_[kHammerPool]{};
    struct Knock {
        int key;
        float amp;
        int age;
    };
    Knock knocks_[kMaxKnocks]{};
    int nKnocks_ = 0;
    int knockLen_ = 72;
    std::uint32_t rng_ = 0x9E3779B9u;
    float noiseLp_[Soundboard::kMaxZones]{};
    float noiseCoef_ = 0.2f;
    float noiseAmp_ = 0.0f;
    // Smoothed pedal: an exponential from pedalFrom_ at the exact time of the
    // last pedal event towards sustain_, sampled on the control grid.
    float pedal_ = 0.0f, pedalPrev_ = 0.0f, pedalFrom_ = 0.0f;
    std::int64_t pedalStart_ = 0;
    double pedalTau_ = 480.0;  // samples
    float pedalAt(std::int64_t t) const noexcept;
    float applied_[kNumKeys]{};

    // Keys, dampers, pedal.
    bool keyDown_[kNumKeys]{};
    bool hasDamper_[kNumKeys]{};
    float panicAmount_[kNumKeys]{};
    Damper damp_[kNumKeys]{};
    float sustain_ = 0.0f;
    bool sostenuto_ = false;
    bool sostLatched_[kNumKeys]{};
    float unaCorda_ = 0.0f;
    std::int64_t panicUntil_ = -1;
    int perturbedPartials_ = 0, uncoupledPartials_ = 0;

    // Hammers.
    Hammer hammers_[kHammerPool]{};
    int hammerOfKey_[kNumKeys]{};
    int freeList_[kHammerPool]{};
    int freeCount_ = 0;
    int active_[kHammerPool]{};
    int activeCount_ = 0;

    // Events.
    SpscQueue<Event, kQueueSize> queue_;
    Pending pending_[kMaxPending]{};
    int pendingCount_ = 0;
    std::uint32_t seq_ = 0;
    std::uint64_t sampleTime_ = 0;

    // Output stage.
    float gainL_[kNumKeys]{}, gainR_[kNumKeys]{};
    float appliedWidth_ = -1.0f;
    float masterGain_ = 1.0f, masterTarget_ = 1.0f;
    float dcX_[2]{}, dcY_[2]{};
    float dcCoef_ = 0.9995f;
    bool appliedCullOn_ = true;
    float appliedCullDb_ = 0.0f;

    // Velocity mapping, double-buffered.
    static constexpr int kVelTable = 129;
    float velTable_[2][kVelTable]{};
    std::atomic<int> velActive_{0};
    bool velocitySet_ = false;
    VelocityCurve defaultCurve_{};

    // Parameters.
    std::atomic<float> masterGainDb_{0.0f};
    std::atomic<float> width_{0.35f};
    std::atomic<int> schedMode_{0};
    std::atomic<int> latency_{0};
    std::atomic<bool> cullOn_{true};
    std::atomic<float> cullDb_{-100.0f};
    std::atomic<float> dampedCullDb_{120.0f};
    float cullOffsetApplied_[kNumKeys]{};
    std::atomic<float> knockLevel_{0.02f};
    std::atomic<float> liftNoiseLevel_{0.004f};
    std::atomic<float> hpP0T_{0.30f}, hpP0B_{0.40f}, hpW_{0.25f};

    // Statistics.
    SpscQueue<StrikeInfo, 256> strikeLog_;
    std::atomic<std::uint64_t> dropped_{0};
    std::uint64_t deferred_ = 0, late_ = 0, limited_ = 0, nonFinite_ = 0;
    float peakPre_ = 0.0f;
    int peakHammers_ = 0;
    int totalModes_ = 0;
    struct Published {
        std::atomic<int> awakeGroups{0}, awakeModes{0}, activeHammers{0}, peakHammers{0};
        std::atomic<std::uint64_t> deferred{0}, late{0}, limited{0}, nonFinite{0};
        std::atomic<float> peakPre{0.0f};
    } pub_;
#if defined(PM_PROFILE)
    EngineProfile prof_;
#endif
};

}  // namespace pm
