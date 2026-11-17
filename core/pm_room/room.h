// Room: image-source early reflections plus a 16-line feedback delay network
// (plan §1.7). Stereo in (the dry microphone pair), stereo out: the WET
// signal only, the caller mixes it with the dry signal.
//
// Threading. prepare() and configure() allocate and must not run concurrently
// with process(). The set* functions may be called from any thread; they
// write atomics that process() latches at the next control-block boundary
// (every kControlBlock samples of absolute stream position), so the output
// is bit-identical for any split of the same input into process() calls.
#pragma once

#include <atomic>
#include <cstdint>

#include "pm_base/aligned.h"

namespace pm {

struct RoomGeometry {
    // Shoebox dimensions (x, y, z) in metres. Default: a medium recital hall.
    double dims[3] = {18.0, 12.0, 7.0};
    // The piano (sound source), off-centre so that the two microphones do not
    // see mirror-identical reflection patterns.
    double source[3] = {6.0, 5.0, 1.0};
    // Spaced pair (left, right), 0.8 m apart, about 1.9 m from the piano.
    double mic[2][3] = {{7.6, 4.6, 2.0}, {7.6, 5.4, 2.0}};
    // Broadband energy absorption coefficient per surface pair (x walls,
    // y walls, floor/ceiling). Reflection amplitude per bounce: sqrt(1 - a).
    double absorption[3] = {0.15, 0.15, 0.20};
    // Corner frequency of the one-pole low-pass shared by all early taps of a
    // channel (air and wall high-frequency absorption, v0).
    double earlyHfCutoffHz = 6000.0;
};

class Room {
public:
    static constexpr int kLines = 16;
    static constexpr int kTaps = 32;          // early taps per channel
    static constexpr int kControlBlock = 32;  // samples per parameter update
    static constexpr double kMinSize = 0.5, kMaxSize = 2.0;
    static constexpr double kSpeedOfSound = 343.0;

    Room() noexcept = default;
    Room(const Room&) = delete;
    Room& operator=(const Room&) = delete;

    // Non-real-time. Allocates for the largest room (size scale kMaxSize) of
    // the current geometry and resets. Returns false on allocation failure.
    bool prepare(double sampleRate, int maxFrames) noexcept;
    // Non-real-time (may reallocate). Takes effect immediately and resets.
    bool configure(const RoomGeometry& g) noexcept;

    // Real-time.
    void process(const float* inL, const float* inR, float* outL, float* outR, int n) noexcept;
    void reset() noexcept;
    bool silent() const noexcept { return silent_; }

    // Real-time parameters (any thread).
    void setDecay(double seconds) noexcept;        // T60 at mid frequencies, 0.3..6 s (default 1.8)
    void setHfRatio(double r) noexcept;            // T60(8 kHz) / T60(mid), 0.1..1 (default 0.5)
    void setLevelDb(double db) noexcept;           // 0 dB = physical reverberant-to-direct ratio
    void setEarlyLateBalance(double b) noexcept;   // 0 = early only, 1 = late only, 0.5 = both at physical level
    void setEnabled(bool on) noexcept;
    void setSize(double scale) noexcept;           // 0.5..2, scales the room about its corner

    static constexpr double kDefaultLevelDb = -9.0;

    // Introspection (tests, UI); valid after prepare.
    int earlyTap(int ch, int k, float* gain) const noexcept;  // returns delay in samples
    int lineLength(int i) const noexcept { return len_[i]; }
    int lateInputDelay() const noexcept { return preDelay_; }
    // Physical reverberant-to-direct energy ratio at level 0 dB for the
    // current size and decay: (r / r_c)^2, r_c = 0.057 sqrt(V / T60).
    double reverbToDirectDb() const noexcept;

private:
    struct Tap {
        int delay;
        float gain;
    };
    struct Derived {
        Tap taps[2][kTaps];
        int len[kLines];
        int pre;
        double volume, micDist, lateStart;
    };
    void derive(double scale, Derived& d) const noexcept;
    void applySize(double scale) noexcept;
    void snapAll() noexcept;
    void computeGainTargets() noexcept;
    void computeLineCoefficients() noexcept;
    void control() noexcept;
    void clearLate() noexcept;
    void processRun(const float* inL, const float* inR, float* outL, float* outR, int n) noexcept;

    double fs_ = 48000.0;
    RoomGeometry geom_;
    bool prepared_ = false;

    // Input history per channel (early taps and the late-field pre-delay).
    AlignedArray<float> inBuf_[2];
    uint32_t inMask_ = 0, inW_ = 0;
    // Late field: kLines lines of fdnSize_ samples each, one shared write index.
    AlignedArray<float> lines_;
    uint32_t fdnSize_ = 0, fdnMask_ = 0, w_ = 0;

    Tap taps_[2][kTaps] = {};
    float lpA_ = 0.5f, lp_[2] = {0.0f, 0.0f};
    int len_[kLines] = {};
    int preDelay_ = 0;
    alignas(64) float b0_[kLines] = {};  // feedback gain times 1/4 (Hadamard scale)
    alignas(64) float pole_[kLines] = {};
    alignas(64) float st_[kLines] = {};

    // Geometry derived for the current size.
    double size_ = 1.0, volume_ = 0.0, micDist_ = 1.0, lateStart_ = 0.0;
    double meanG2_ = 0.0;

    // Smoothed parameters (control rate) and per-sample gain ramps.
    double logT_ = 0.0, hf_ = 0.5, level_ = 1.0, bal_ = 0.5;
    int fadeCnt_ = 0, fadeBlocks_ = 30;
    double alpha_ = 0.0;
    Derived scratch_ = {};
    bool enabledNow_ = true;
    float gE_ = 0.0f, dgE_ = 0.0f, gL_ = 0.0f, dgL_ = 0.0f, gETarget_ = 0.0f, gLTarget_ = 0.0f;

    // Idle detection.
    uint64_t pos_ = 0;
    uint64_t zeroRun_ = 0, quietRun_ = 0;
    float blockPeak_ = 0.0f;
    bool silent_ = true;

    std::atomic<float> aDecay_{1.8f}, aHf_{0.5f}, aLevelDb_{float(kDefaultLevelDb)}, aBal_{0.5f}, aSize_{1.0f};
    std::atomic<bool> aEnabled_{true};
};

}  // namespace pm
