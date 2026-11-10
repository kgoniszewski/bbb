// Bank of complex one-pole resonators (plan §1.5), structure of arrays.
//
//   z[n+1] = c * z[n] + wIn * u[n]          (wIn complex; real for a plain string)
//   y[n]  += wOut * Im(z[n+1])               (wOut real)
//
// Output weights are real. A mode whose physical output weight is complex
// (a coupled unison normal mode, pm_voicing/unison.h) is stored with its
// state rotated by the weight's phase, which makes the weight real at no
// run-time cost; its input weight then carries the inverse rotation, so the
// input weight is complex. Without an input (the engine: hammers drive their
// keys' modes themselves) the kernel skips the input entirely.
//
// Coefficients are computed in double and stored as float; state runs in
// float. The real part is stored as d = Re(c) - 1 and the update evaluated as
// z + d*z + j*Im(c)*z: a float near 1 resolves |c| only to ~6e-8, which at
// 48 kHz is a decay-rate step of ~3e-3 1/s (about 1 % of T60 for a 30 s bass
// partial), whereas d keeps full relative precision. With FMA the cost is
// the same as the plain form.
//
// Changing c while a mode rings is click-free because z carries the
// amplitude and phase.
//
// Modes are processed in groups of kGroupSize. With culling enabled, every
// kCullInterval samples a group whose output bound falls below the threshold
// and that received no input is zeroed and put to sleep; a sleeping group is
// skipped entirely until input large enough to matter arrives.
//
// Segments. A bank may be laid out as several segments (one per piano key),
// each starting on a group boundary, so that a group never mixes keys. A
// segment has its own input and output buffer, can be skipped for a
// sub-block (while a hammer solver owns its modes), and carries a damper
// amount d that scales a per-mode extra decay rate:
//   c_eff = c_free * exp(-d * sigmaDamp_n / fs)
// The kernels only ever see c_eff; the free coefficient is kept separately so
// d can change at any time without accumulating rounding error.
//
// The bank laid out by allocate() has a single segment covering all groups.
//
// process(), processSegments() and the setters marked RT are real-time
// safe: no allocation, locks or I/O. Call them from one thread. The caller
// is responsible for denormal flushing (pm::ScopedFlushDenormals at the top
// of the render callback).
#pragma once

#include <complex>
#include <cstdint>

#include "pm_base/aligned.h"

namespace pm {

enum class Kernel { Scalar, Simd };

// Name of the instruction set the Simd kernel was compiled for.
const char* simdKernelName() noexcept;

// Per-segment I/O for ModeBank::processSegments().
struct SegmentIO {
    const float* in = nullptr;  // force input u[0..n), or null for none
    float* out = nullptr;       // receives the segment output (see processSegments)
    bool skip = false;          // leave this segment untouched for this call
    bool wrote = false;         // set by the bank: out was written
};

class ModeBank {
public:
    static constexpr int kGroupSize = 16;
    static constexpr int kCullInterval = 64;

    ModeBank() = default;
    ModeBank(ModeBank&&) noexcept = default;
    ModeBank& operator=(ModeBank&&) noexcept = default;

    // Non-real-time. Reserves storage for up to maxModes modes in a single
    // segment and clears everything. Returns false on allocation failure.
    bool allocate(int maxModes) noexcept;

    // Non-real-time. Lays out nSegments segments; segment s holds
    // modesPerSegment[s] modes starting on a group boundary (the rest of its
    // last group is silent padding). All modes are in use (size() is the
    // padded total) and cleared. Returns false on allocation failure.
    bool allocateSegments(const int* modesPerSegment, int nSegments) noexcept;

    int capacity() const noexcept { return capacity_; }
    int size() const noexcept { return size_; }
    int groupCount() const noexcept { return (size_ + kGroupSize - 1) / kGroupSize; }

    int segmentCount() const noexcept { return nSeg_; }
    int segmentFirstMode(int s) const noexcept { return segFirstGroup(s) * kGroupSize; }
    // Padded: a whole number of groups.
    int segmentModeCount(int s) const noexcept {
        return (segFirstGroup(s + 1) - segFirstGroup(s)) * kGroupSize;
    }

    // Sets the number of modes in use (<= capacity). Modes beyond the new size
    // are cleared, so padding lanes in the last group stay silent. Only for
    // the single-segment layout.
    void resize(int count) noexcept;

    // c = exp((-sigma + j*2*pi*freqHz) / fs), computed in double. Sets the
    // free coefficient; the segment's current damper amount is applied on top.
    void setMode(int i, double freqHz, double sigma, double fs) noexcept;
    void setWeights(int i, float wIn, float wOut) noexcept { setWeights(i, wIn, 0.0f, wOut); }
    void setWeights(int i, float wInRe, float wInIm, float wOut) noexcept;
    // Extra decay rate (1/s) of mode i when its segment's damper amount is 1.
    void setDamperRate(int i, double sigmaDamp, double fs) noexcept;
    // Writing a non-zero state wakes the mode's group.
    void setState(int i, float re, float im) noexcept;
    void clearState() noexcept;

    // RT. Sets the damper amount (>= 0) of segment s and recomputes the
    // coefficients of its modes. Cheap (a short polynomial per mode); call it
    // at control rate, only when the amount changes.
    void setSegmentDamper(int s, float amount) noexcept;
    float segmentDamper(int s) const noexcept { return segDamp_[s]; }
    // RT. Marks every group of segment s awake (after its state was written
    // directly through the state pointers below).
    void wakeSegment(int s) noexcept;

    // The coefficient exactly as the kernels apply it.
    std::complex<double> coefficient(int i) const noexcept {
        return {1.0 + static_cast<double>(dr_[i]), static_cast<double>(ci_[i])};
    }
    float stateRe(int i) const noexcept { return zr_[i]; }
    float stateIm(int i) const noexcept { return zi_[i]; }
    float weightIn(int i) const noexcept { return wIn_[i]; }
    float weightInIm(int i) const noexcept { return wInIm_[i]; }
    float weightOut(int i) const noexcept { return wOut_[i]; }

    // Direct access for a solver that advances one segment's modes itself
    // (the hammer contact solver). 64-byte aligned, padded with zeros.
    float* stateReData() noexcept { return zr_.data(); }
    float* stateImData() noexcept { return zi_.data(); }
    const float* weightOutData() const noexcept { return wOut_.data(); }
    const float* weightInReData() const noexcept { return wIn_.data(); }
    const float* weightInImData() const noexcept { return wInIm_.data(); }
    // exp(-d * sigmaDamp_n / fs) - 1 for the current damper amount d.
    const float* damperFactorMinusOneData() const noexcept { return dampAm1_.data(); }

    // thresholdDb is relative to an output amplitude of 1.0 (full scale).
    // Disabling culling wakes every group.
    void setCulling(bool enabled, float thresholdDb = -100.0f) noexcept;
    // RT. Raises segment s's culling thresholds (dB >= 0, default 0): its
    // groups sleep when their output bound is below threshold * 10^(stateDb
    // / 20), and input keeps them awake or wakes them only when its bound
    // exceeds threshold * 10^(inputDb / 20).
    void setSegmentCullOffset(int s, float stateDb, float inputDb) noexcept;
    bool cullingEnabled() const noexcept { return cullEnabled_; }

    // Advances all awake modes by nFrames samples. 'in' is the force input
    // u[n] (may be null for no input), shared by all segments. The bank output
    // is ADDED to out[0..n).
    void process(const float* in, float* out, int nFrames, Kernel kernel = Kernel::Simd) noexcept;

    // Advances every segment not marked skip by nFrames samples, each with
    // its own input. When a segment has awake groups its output OVERWRITES
    // io[s].out[0..nFrames) and io[s].wrote is set; otherwise io[s].out is
    // untouched and wrote is false (the segment is silent). io has
    // segmentCount() entries. Skipped segments are excluded from culling.
    void processSegments(SegmentIO* io, int nFrames, Kernel kernel = Kernel::Simd) noexcept;

    // Coupled processing (soundboard loop, pm_board/soundboard.h). The first
    // segmentCoupledGroups(s) groups of segment s are "coupled": they take
    // a per-sample input from the loop and must be advanced one sample at a
    // time with stepCoupled(); the remaining groups are advanced a sub-block
    // at a time by processSegmentsUncoupled() (no input). Per sub-block of
    // n samples (n <= kCullInterval, never crossing a multiple of
    // kCullInterval in absolute time):
    //   processSegmentsUncoupled(io, n); beginCoupled(io);
    //   n x stepCoupled(in, out);        endCoupled(io, n);
    // Segments marked io[s].skip are left alone by all of these. Culling:
    // a sleeping coupled group wakes as soon as the input accumulated since
    // the last cull pass could have moved its output above the threshold
    // (the same bound as process()); the cull pass runs in endCoupled() on
    // the absolute kCullInterval grid.
    void setSegmentCoupledGroups(int s, int groups) noexcept;  // non-RT
    // Static correction of the coupled input (pm_voicing/soundboard_model.h):
    // stepCoupled() subtracts k0 u[n] + k1 u[n-1] per awake coupled group
    // from its segment's output, so that sleeping groups (which do not
    // respond) are not corrected. computeStaticCorrection() derives both
    // taps from the current coefficients and input/output weights (damper
    // amount 0): k0 + k1 = sum wOut Im(wIn / (1 - c)) (static gain),
    // -k1 = sum wOut Im(-wIn c / (1 - c)^2) (its slope in j omega T).
    void computeStaticCorrection(int s) noexcept;  // non-RT, after the modes and setSegmentCoupledGroups
    // Sum over all coupled groups of segment s (awake or not).
    float staticCorrection0(int s) const noexcept;
    float staticCorrection1(int s) const noexcept;
    // The previous input of segment s as stepCoupled() remembers it (state
    // of the static correction; tests that set the whole state use these).
    float coupledInputHistory(int s) const noexcept { return segInPrev_[s]; }
    void setCoupledInputHistory(int s, float u) noexcept { segInPrev_[s] = u; }
    int segmentCoupledGroups(int s) const noexcept { return segCoupled_[s]; }
    // RT. Overwrites io[s].out[0..n) with the uncoupled groups' output where
    // they are awake (io[s].wrote), like processSegments.
    void processSegmentsUncoupled(SegmentIO* io, int nFrames, Kernel kernel = Kernel::Simd) noexcept;
    void beginCoupled(const SegmentIO* io) noexcept;
    // RT. One sample: in[s] is segment s's input, out[s] receives its coupled
    // groups' output for every segment with awake coupled groups (the
    // segments listed by coupledSegments()); other entries are untouched.
    void stepCoupled(const float* in, float* out, Kernel kernel = Kernel::Simd) noexcept;
    // RT, valid after beginCoupled() until the next change of the awake
    // set: segments with awake coupled groups, and segments that are asleep
    // (in part) but can still be woken by their input.
    int coupledSegments(const int*& list) noexcept;
    // Whether stepCoupled() writes out[s] (segment has awake coupled groups).
    bool segmentHasAwakeCoupled(int s) const noexcept { return segHasRuns_[s] != 0; }
    int wakeableSegments(const int*& list) noexcept;
    void endCoupled(const SegmentIO* io, int nFrames) noexcept;

    int activeGroups() const noexcept;
    bool groupActive(int g) const noexcept { return active_[g] != 0; }
    // Total number of (group x sub-block) kernel invocations so far; a silent
    // bank leaves it unchanged.
    std::uint64_t groupBlocksProcessed() const noexcept { return groupBlocks_; }

private:
    int segFirstGroup(int s) const noexcept {
        return singleSegment_ ? (s == 0 ? 0 : groupCount()) : segGroups_[s];
    }
    bool allocateArrays(int groups, int segments) noexcept;
    void updateGroupGain(int g) noexcept;
    void applyDamper(int i) noexcept;
    void cullPass(const SegmentIO* io) noexcept;
    void run(SegmentIO* io, int nFrames, Kernel kernel, bool overwrite) noexcept;
    void rebuildRuns() noexcept;
    void wakeCoupled(int s) noexcept;

    AlignedArray<float> zr_, zi_, dr_, ci_, wIn_, wInIm_, wOut_;  // dr = Re(c) - 1
    AlignedArray<float> dr0_, ci0_;   // free-string coefficient
    AlignedArray<float> dampRate_;    // sigmaDamp / fs per mode
    AlignedArray<float> dampAm1_;     // current damper factor - 1
    AlignedArray<float> groupInOut_;  // sum |wIn| |wOut| over the group
    AlignedArray<std::uint8_t> active_;
    AlignedArray<int> activeList_;
    AlignedArray<int> segGroups_;     // first group of each segment, plus end
    AlignedArray<float> segDamp_;     // damper amount per segment
    AlignedArray<float> segInput_;    // sum |u| since the last cull pass, per segment
    AlignedArray<float> scratch_;     // kCullInterval * SIMD width
    // Coupled processing.
    AlignedArray<int> segCoupled_;    // coupled groups per segment
    AlignedArray<std::uint8_t> segSkip_, segSleeping_, segHasRuns_;
    AlignedArray<float> segWakeGain_; // max groupInOut over the coupled groups
    AlignedArray<int> runSeg_, runM0_, runM1_, runSegList_, wakeList_;
    int nRunSegs_ = 0, nWake_ = 0;
    AlignedArray<float> groupK0_, groupK1_, segK0_, segK1_, segInPrev_;
    AlignedArray<float> segCullScale_, segInputScale_;  // per-segment threshold factors
    int nRuns_ = 0;
    bool runsDirty_ = true;

    int capacity_ = 0;
    int size_ = 0;
    int nSeg_ = 0;
    bool singleSegment_ = true;
    bool cullEnabled_ = false;
    float cullThreshold_ = 1e-5f;
    int sinceCull_ = 0;
    std::uint64_t groupBlocks_ = 0;
};

}  // namespace pm
