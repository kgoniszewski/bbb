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
    // Taps of coupledResponse() (the longest coupled sub-block).
    static constexpr int kResponseTaps = 32;

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
    // coefficients of its awake modes (a short polynomial per mode); call it
    // at control rate, only when the amount changes. Sleeping groups are
    // brought up to date when they wake (their state is zero, so nothing
    // reads their coefficients before), which makes a pedal movement cost
    // nothing for the strings that are silent.
    void setSegmentDamper(int s, float amount) noexcept;
    float segmentDamper(int s) const noexcept { return segDamp_[s]; }
    // RT. Marks every group of segment s awake (after its state was written
    // directly through the state pointers below).
    void wakeSegment(int s) noexcept;

    // The coefficient exactly as the kernels apply it (for a sleeping
    // group: as they will apply it when it wakes).
    std::complex<double> coefficient(int i) const noexcept;
    float stateRe(int i) const noexcept { return zr_[i]; }
    float stateIm(int i) const noexcept { return zi_[i]; }
    float weightIn(int i) const noexcept { return wIn_[i]; }
    float weightInIm(int i) const noexcept { return wInIm_[i]; }
    float weightOut(int i) const noexcept { return wOut_[i]; }

    // Direct access for a solver that advances one segment's modes itself
    // (the hammer contact solver). 64-byte aligned, padded with zeros.
    float* stateReData() noexcept { return zr_.data(); }
    float* stateImData() noexcept { return zi_.data(); }
    const float* stateReData() const noexcept { return zr_.data(); }
    const float* stateImData() const noexcept { return zi_.data(); }
    const float* weightOutData() const noexcept { return wOut_.data(); }
    const float* weightInReData() const noexcept { return wIn_.data(); }
    const float* weightInImData() const noexcept { return wInIm_.data(); }
    // exp(-d * sigmaDamp_n / fs) - 1 for the current damper amount d
    // (up to date for awake groups only; see setSegmentDamper).
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
    // segmentCoupledGroups(s) groups of segment s are "coupled": they take an
    // input u from the loop, one sample after the board produced it. The
    // loop is solved exactly per sub-block of n samples (n <= kCullInterval,
    // never crossing a multiple of kCullInterval in absolute time):
    //   processSegmentsFree(io, n)      every awake group without input: the
    //                                   free response F0 (overwrites io[s].out)
    //   loop, sample by sample:         F[i] = F0[i] + sum_j h[j] u[i-j] - k1 uPrev [i = 0]
    //                                   (h, k1 from coupledResponse())
    //   applyCoupledInput(s, u, n)      adds the input's share to the state
    //   endCoupled(io, n)               culling on the absolute grid
    // In exact arithmetic this is the same recursion as stepping every coupled
    // mode one sample at a time with its input (superposition), so it has the
    // per-sample loop's stability and passivity, but each mode is touched
    // twice per sub-block in registers instead of once per sample in memory.
    // Culling: a sleeping coupled group wakes at the end of the sub-block in
    // which the input accumulated since the last cull pass could have moved
    // its output above the threshold (the same bound as process()).
    void setSegmentCoupledGroups(int s, int groups) noexcept;  // non-RT
    // Static correction of the coupled input (pm_voicing/soundboard_model.h):
    // the segment's output is corrected by -(k0 u[n] + k1 u[n-1]), summed
    // over its awake coupled groups, so that sleeping groups (which do not
    // respond) are not corrected. computeStaticCorrection() derives both
    // taps from the current coefficients and input/output weights (damper
    // amount 0): k0 + k1 = sum wOut Im(wIn / (1 - c)) (static gain),
    // -k1 = sum wOut Im(-wIn c / (1 - c)^2) (its slope in j omega T).
    void computeStaticCorrection(int s) noexcept;  // non-RT, after the modes and setSegmentCoupledGroups
    // Sum over all coupled groups of segment s (awake or not).
    float staticCorrection0(int s) const noexcept;
    float staticCorrection1(int s) const noexcept;
    // The last input of segment s (state of the static correction).
    float coupledInputHistory(int s) const noexcept { return segInPrev_[s]; }
    void setCoupledInputHistory(int s, float u) noexcept { segInPrev_[s] = u; }
    int segmentCoupledGroups(int s) const noexcept { return segCoupled_[s]; }
    // Changes whenever coupledResponse(s) would change (damper amount,
    // coefficients, weights, or the awake set of the coupled groups).
    std::uint32_t coupledVersion(int s) const noexcept { return segVersion_[s]; }
    // RT. The response of segment s's output to a unit input at sample 0,
    // h[0..kResponseTaps), over its awake coupled groups, static correction
    // included; *k1 = the correction's tap on the previous input. Returns
    // false (h untouched) when no coupled group is awake. Call it only when
    // coupledVersion() changed. *exact (optional) is set to whether every
    // group's response in h is exact (see "Damper responses" below).
    //
    // Damper responses. Each coupled group's exact response is cached
    // together with the damper amount it was computed for (computing one
    // costs about one free pass of the group over kResponseTaps samples), so
    // a group going to sleep or waking costs a re-sum, and a damper that
    // returns to an amount it had keeps its cache. The lifted response
    // (amount 0) is precomputed. The groups whose cache does not match the
    // segment's current amount are recomputed if the response budget left
    // in the current control tick (beginResponseTick) covers all of them,
    // except for a segment whose damper moved in this tick while the tick's
    // damper changes together need more than the whole budget (a pedal
    // stroke: the budget is better spent on keys at rest). Otherwise each of
    // them contributes its second-order expansion in the damper amount d,
    //   h_k(d) = H0_k + d D1_k + d^2 D2_k,
    //   D1_k = -k sum wOut r Im(c0^k wIn),  D2_k = k^2 / 2 sum wOut r^2 Im(c0^k wIn)
    // (r = damper rate per sample, c0 the free coefficient; precomputed;
    // summed over the awake groups once per awake set when all of them are
    // expanded), the exact response's Taylor series in d, truncated: the
    // relative error of a mode's tap k is at most about (d r k)^3 / 6, under
    // 4.2e-4 for the strongest damper (192 1/s: r k <= 0.135 at tap 31 at
    // 44.1 kHz, d <= 1). Amounts above 1 (panic only) use the expansion at
    // d = 1. Such groups are marked approximate; refineResponses() replaces
    // them by exact ones once their segment's amount has stopped moving,
    // within the budget, so at rest every cache is exact and h is what an
    // unlimited budget gives.
    bool coupledResponse(int s, float* h, float* k1, Kernel kernel = Kernel::Simd, bool* exact = nullptr) noexcept;
    // RT. Starts a control tick: up to 'groups' exact group responses may be
    // computed (by refineResponses and coupledResponse) until the next call;
    // groups < 0: unlimited (every response exact, as before budgets).
    void beginResponseTick(int groups) noexcept {
        budget_ = tickBudget_ = groups;
        tickDemand_ = 0;
        ++tick_;
    }
    // RT, after the tick's damper changes and before the loop reads the
    // responses. Spends the tick's budget, less what the tick's own damper
    // changes need when they fit in it, on the awake approximate groups of
    // segments whose damper amount has not changed during the last two
    // ticks, round-robin over the segments; such a segment is re-summed at
    // its next coupledResponse (its version changes).
    void refineResponses(Kernel kernel = Kernel::Simd) noexcept;
    // Awake coupled groups whose response is currently approximate.
    int approximateGroups() const noexcept;
    // RT. Forgets the damper-amount caches, the approximations and the tick
    // count (what allocate and load leave), keeping the lifted responses and
    // the expansion rows: an engine reset then renders as a fresh one.
    void resetResponses() noexcept;
    // Non-RT: fills the damper-lifted response cache of every coupled group
    // and its expansion rows D1, D2 (after the modes, damper rates, weights
    // and coupled groups are set), so that lifting a damper never costs a
    // recompute on the audio thread and every amount has its expansion.
    void precomputeCoupledResponses() noexcept;
    bool segmentHasAwakeCoupled(int s) const noexcept;
    // Has sleeping coupled groups that input could wake (culling on and the
    // segment's input threshold not raised out of reach).
    bool segmentWakeable(int s) const noexcept;
    // RT. Every awake group (coupled or not) of each segment not marked
    // skip, without input; outputs overwrite io[s].out where io[s].wrote.
    void processSegmentsFree(SegmentIO* io, int nFrames, Kernel kernel = Kernel::Simd) noexcept;
    // RT. Adds input u[0..n) to the awake coupled groups of segment s (their
    // state was advanced freely over the same n samples), records u for the
    // static correction and the wake-up bound, and wakes sleeping coupled
    // groups whose bound was exceeded during the sub-block. A woken group
    // gets the state it would have had had it woken at the sample where its
    // bound was crossed (quasi-static deflection under that sample's input,
    // then the remaining input), so wake-ups happen at the same point of the
    // music however the stream is cut into sub-blocks.
    void applyCoupledInput(int s, const float* u, int n, Kernel kernel = Kernel::Simd) noexcept;
    void endCoupled(const SegmentIO* io, int nFrames) noexcept;
    // RT. Wakes every sleeping coupled group of segment s at its
    // quasi-static deflection under input u (a key that starts to take
    // input: its strings follow the bridge statically, and their static
    // correction joins at the same time). Groups that stay silent go back
    // to sleep at the next cull pass.
    void wakeAllCoupled(int s, float u) noexcept;

    int activeGroups() const noexcept { return activeCount_; }
    // Any group of segment s awake (coupled or not).
    bool segmentAwake(int s) const noexcept;
    bool groupActive(int g) const noexcept { return active_[g] != 0; }
    // Total number of (group x sub-block) kernel invocations so far; a silent
    // bank leaves it unchanged.
    std::uint64_t groupBlocksProcessed() const noexcept { return groupBlocks_; }
    // Coupled groups whose exact impulse response was computed on the audio
    // thread (coupledResponse, refineResponses), so far; and group responses
    // coupledResponse() took from the expansion instead (over budget).
    std::uint64_t responseGroupsComputed() const noexcept { return responseGroups_; }
    std::uint64_t responseGroupsApproximated() const noexcept { return approxGroups_; }
    // Group x sub-block invocations of the forced kernel (applyCoupledInput), so far.
    std::uint64_t forcedGroupBlocks() const noexcept { return forcedBlocks_; }
    // Sleeping coupled groups woken by loop input (applyCoupledInput,
    // wakeAllCoupled), so far: each costs a double-precision start-up of its
    // modes over the rest of the sub-block.
    std::uint64_t groupsWoken() const noexcept { return groupsWoken_; }

private:
    int segFirstGroup(int s) const noexcept {
        return singleSegment_ ? (s == 0 ? 0 : groupCount()) : segGroups_[s];
    }
    bool allocateArrays(int groups, int segments) noexcept;
    void updateGroupGain(int g) noexcept;
    void applyDamper(int i) noexcept;
    void cullPass(const SegmentIO* io) noexcept;
    void run(SegmentIO* io, int nFrames, Kernel kernel, bool overwrite) noexcept;
    void wakeCoupled(int s, float inputBefore, const float* u, int n) noexcept;
    void setActive(int g, std::uint8_t on) noexcept {
        if (active_[g] == on) return;
        if (on && groupStale_[g]) refreshGroup(g);
        active_[g] = on;
        activeCount_ += on ? 1 : -1;
        const int s = groupSeg_[g];
        if (g < segFirstGroup(s) + segCoupled_[s]) {
            ++segVersion_[s];
            ++segAwakeVer_[s];
        }
    }
    // Applies the segment's damper amount to group g's coefficients.
    void refreshGroup(int g) noexcept;
    // After active_ was written directly: recount and refresh awake groups.
    void syncActive() noexcept;
    void touchAll() noexcept {
        for (int s = 0; s < nSeg_; ++s) {
            ++segVersion_[s];
            ++segAwakeVer_[s];
        }
    }
    // The free coefficients or weights of group g changed (non-RT setters):
    // every cached response of it is out of date.
    void invalidateResponse(int g) noexcept {
        groupHValid_[g] = groupH0Valid_[g] = groupTaylorValid_[g] = 0;
        ++segVersion_[groupSeg_[g]];
        ++segAwakeVer_[groupSeg_[g]];
    }
    // Exact responses of the listed groups (consecutive runs in one kernel
    // call) at the current coefficients, into the lifted or the current cache.
    void computeResponses(const int* list, int n, bool lifted, Kernel kernel) noexcept;

    AlignedArray<float> zr_, zi_, dr_, ci_, wIn_, wInIm_, wOut_;  // dr = Re(c) - 1
    AlignedArray<float> dr0_, ci0_;   // free-string coefficient
    AlignedArray<float> dampRate_;    // sigmaDamp / fs per mode
    AlignedArray<float> dampAm1_;     // current damper factor - 1
    AlignedArray<float> groupInOut_;  // sum |wIn| |wOut| over the group
    AlignedArray<std::uint8_t> active_;
    AlignedArray<std::uint8_t> groupStale_;  // coefficients lag the segment's damper amount
    AlignedArray<int> activeList_;
    AlignedArray<int> segGroups_;     // first group of each segment, plus end
    AlignedArray<float> segDamp_;     // damper amount per segment
    AlignedArray<float> segInput_;    // sum |u| since the last cull pass, per segment
    AlignedArray<float> scratch_;     // kCullInterval * SIMD width
    // Coupled processing.
    AlignedArray<int> segCoupled_;    // coupled groups per segment
    AlignedArray<int> groupSeg_;      // segment of each group
    AlignedArray<std::uint32_t> segVersion_;
    AlignedArray<float> segWakeGain_; // max groupInOut over the coupled groups
    AlignedArray<float> groupK0_, groupK1_, segInPrev_;
    // Cached impulse responses, groups x kResponseTaps: at the current damper
    // amount, and at amount 0 (damper lifted: held keys, sustain pedal),
    // which survives damper changes.
    AlignedArray<float> groupH_, groupH0_;
    AlignedArray<std::uint8_t> groupHValid_, groupH0Valid_;
    AlignedArray<float> groupHAmount_;            // damper amount of groupH_'s row
    AlignedArray<float> groupD1_, groupD2_;       // expansion rows (see coupledResponse)
    AlignedArray<std::uint8_t> groupTaylorValid_;
    AlignedArray<std::uint8_t> groupApprox_;      // last coupledResponse used the expansion
    AlignedArray<std::uint8_t> segApprox_;        // some awake group of the segment is approximate
    AlignedArray<std::uint32_t> segDampTick_;     // tick of the segment's last damper change
    // Per segment, the expansion rows summed over its awake coupled groups
    // (3 x kResponseTaps), valid while the awake set is the one counted by
    // segAwakeVer_ when they were summed.
    AlignedArray<float> segT_;
    AlignedArray<std::uint32_t> segAwakeVer_, segTVer_;
    AlignedArray<std::uint8_t> segTValid_;
    std::uint32_t tick_ = 0;
    int budget_ = -1;                             // exact responses left this tick (< 0: unlimited)
    int tickBudget_ = -1;                         // the tick's whole budget
    int tickDemand_ = 0;                          // awake coupled groups whose damper moved this tick
    int refineCursor_ = 0;
    AlignedArray<int> hList_;
    AlignedArray<float> segCullScale_, segInputScale_;  // per-segment threshold factors

    int capacity_ = 0;
    int size_ = 0;
    int nSeg_ = 0;
    bool singleSegment_ = true;
    bool cullEnabled_ = false;
    float cullThreshold_ = 1e-5f;
    int sinceCull_ = 0;
    int activeCount_ = 0;
    std::uint64_t groupBlocks_ = 0;
    std::uint64_t responseGroups_ = 0;
    std::uint64_t approxGroups_ = 0;
    std::uint64_t forcedBlocks_ = 0;
    std::uint64_t groupsWoken_ = 0;
};

}  // namespace pm
