// Piano engine, M3 (plan §4.2): 88 keys of coupled unisons (1-3 strings in
// two polarizations, as normal modes), a hammer pool, dampers, sustain,
// sostenuto and una corda pedals, sample-accurate event scheduling and a
// fixed stereo output stage (no soundboard yet).
//
// Pedals. Sustain (CC64) lifts every damper at half travel. Sostenuto (CC66,
// plan §1.9): when it goes down (half travel), the keys whose dampers are
// lifted by their own keys at that moment are latched and stay undamped
// until it comes up. Una corda (CC67) is continuous; it is latched per
// strike when the hammer is launched (offsets and felt of that strike), so
// moving it never changes a sounding note or a contact in progress.
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
// relative to each active hammer's launch (hammer release). Output therefore
// does not depend on how the host cuts the stream into buffers (bit-exact),
// and an event lands on its exact sample: a note-on moved by k samples gives
// the same output moved by k samples, bit-exact without culling (with
// culling, whose 64-sample grid is absolute, to within the culling
// threshold).
//
// Output level. The output is the sum of the keys' (vertical) bridge forces
// (times outputGain from the table) through a fixed stereo spread and a 5 Hz
// DC blocker. Dense pedalled ff chords can exceed full scale, so the output
// passes a soft clipper that is transparent below -1 dBFS (0.89) and never
// reaches 1.0.
#pragma once

#include <atomic>
#include <cstdint>

#include "pm_base/aligned.h"
#include "pm_base/spsc_queue.h"
#include "pm_engine/events.h"
#include "pm_hammer/hammer.h"
#include "pm_string/mode_bank.h"
#include "pm_voicing/bridge_admittance.h"
#include "pm_voicing/piano_table.h"

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

class Engine {
public:
    static constexpr int kHammerPool = 32;
    static constexpr int kSubBlock = 32;     // also the control-rate grid
    static constexpr int kQueueSize = 1024;
    static constexpr int kMaxPending = 1024;
    static constexpr double kPanicSigma = 70.0;  // 1/s: every partial T60 <= 0.1 s
    static constexpr double kPanicSeconds = 0.3;

    Engine() = default;
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Non-real-time. Builds the default table (A4 = 440 Hz, default stretch
    // and unison width) or uses 'table', computes every key's normal modes
    // (with the placeholder bridge admittance unless 'bridge' is given),
    // allocates everything and resets all state. maxFrames is the largest
    // buffer the host will render (used as the default constant latency).
    bool prepare(double sampleRate, int maxFrames) noexcept;
    bool prepare(double sampleRate, int maxFrames, const PianoTable& table) noexcept;
    bool prepare(double sampleRate, int maxFrames, const PianoTable& table,
                 const BridgeAdmittance& bridge) noexcept;
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

    // Inspection, for tools and tests (not thread-safe against render()).
    double sampleRate() const noexcept { return fs_; }
    std::uint64_t sampleTime() const noexcept { return sampleTime_; }
    const PianoTable& table() const noexcept { return table_; }
    const ModeBank& bank() const noexcept { return bank_; }
    int keyModeCount(int key) const noexcept { return keyModes_[key]; }
    int freeHammers() const noexcept { return freeCount_; }
    float sustain() const noexcept { return sustain_; }
    bool sostenutoDown() const noexcept { return sostenuto_; }
    bool sostenutoLatched(int key) const noexcept { return sostLatched_[key]; }
    float unaCorda() const noexcept { return unaCorda_; }
    // Normal modes that needed a perturbed eigen-solve / fell back to
    // uncoupled oscillators, over all keys (from prepare).
    int perturbedPartials() const noexcept { return perturbedPartials_; }
    int uncoupledPartials() const noexcept { return uncoupledPartials_; }
    float damperAmount(int key) const noexcept { return damp_[key].cur; }

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
};

}  // namespace pm
