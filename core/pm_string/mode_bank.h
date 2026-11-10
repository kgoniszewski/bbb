// Bank of complex one-pole resonators (plan §1.5), structure of arrays.
//
//   z[n+1] = c * z[n] + wIn * u[n]          (u enters the real part)
//   y[n]  += wOut * Im(z[n+1])
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
// process() and all setters are real-time safe: no allocation, locks or I/O.
// Call them from one thread. The caller is responsible for denormal flushing
// (pm::ScopedFlushDenormals at the top of the render callback).
#pragma once

#include <complex>
#include <cstdint>

#include "pm_base/aligned.h"

namespace pm {

enum class Kernel { Scalar, Simd };

// Name of the instruction set the Simd kernel was compiled for.
const char* simdKernelName() noexcept;

class ModeBank {
public:
    static constexpr int kGroupSize = 16;
    static constexpr int kCullInterval = 64;

    ModeBank() = default;
    ModeBank(ModeBank&&) noexcept = default;
    ModeBank& operator=(ModeBank&&) noexcept = default;

    // Non-real-time. Reserves storage for up to maxModes modes and clears
    // everything. Returns false on allocation failure.
    bool allocate(int maxModes) noexcept;

    int capacity() const noexcept { return capacity_; }
    int size() const noexcept { return size_; }
    int groupCount() const noexcept { return (size_ + kGroupSize - 1) / kGroupSize; }

    // Sets the number of modes in use (<= capacity). Modes beyond the new size
    // are cleared, so padding lanes in the last group stay silent.
    void resize(int count) noexcept;

    // c = exp((-sigma + j*2*pi*freqHz) / fs), computed in double.
    void setMode(int i, double freqHz, double sigma, double fs) noexcept;
    void setWeights(int i, float wIn, float wOut) noexcept;
    // Writing a non-zero state wakes the mode's group.
    void setState(int i, float re, float im) noexcept;
    void clearState() noexcept;

    // The coefficient exactly as the kernels apply it.
    std::complex<double> coefficient(int i) const noexcept {
        return {1.0 + static_cast<double>(dr_[i]), static_cast<double>(ci_[i])};
    }
    float stateRe(int i) const noexcept { return zr_[i]; }
    float stateIm(int i) const noexcept { return zi_[i]; }
    float weightIn(int i) const noexcept { return wIn_[i]; }
    float weightOut(int i) const noexcept { return wOut_[i]; }

    // thresholdDb is relative to an output amplitude of 1.0 (full scale).
    // Disabling culling wakes every group.
    void setCulling(bool enabled, float thresholdDb = -100.0f) noexcept;
    bool cullingEnabled() const noexcept { return cullEnabled_; }

    // Advances all awake modes by nFrames samples. 'in' is the force input
    // u[n] (may be null for no input). The bank output is ADDED to out[0..n).
    void process(const float* in, float* out, int nFrames, Kernel kernel = Kernel::Simd) noexcept;

    int activeGroups() const noexcept;
    bool groupActive(int g) const noexcept { return active_[g] != 0; }
    // Total number of (group x sub-block) kernel invocations so far; a silent
    // bank leaves it unchanged.
    std::uint64_t groupBlocksProcessed() const noexcept { return groupBlocks_; }

private:
    void updateGroupGain(int g) noexcept;
    void cullPass() noexcept;

    AlignedArray<float> zr_, zi_, dr_, ci_, wIn_, wOut_;  // dr = Re(c) - 1
    AlignedArray<float> groupInOut_;  // max |wIn * wOut| over the group
    AlignedArray<std::uint8_t> active_;
    AlignedArray<int> activeList_;
    AlignedArray<float> scratch_;     // kCullInterval * SIMD width
    AlignedArray<float> zeros_;       // kCullInterval zero samples

    int capacity_ = 0;
    int size_ = 0;
    bool cullEnabled_ = false;
    float cullThreshold_ = 1e-5f;
    int sinceCull_ = 0;
    float intervalInput_ = 0.0f;      // sum |u| since the last cull pass
    std::uint64_t groupBlocks_ = 0;
};

}  // namespace pm
