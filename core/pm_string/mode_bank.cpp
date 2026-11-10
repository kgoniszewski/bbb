#include "pm_string/mode_bank.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_base/simd.h"
#include "pm_string/modal_kernels.h"

namespace pm {

bool ModeBank::allocate(int maxModes) noexcept {
    if (maxModes < 0) return false;
    const int groups = (maxModes + kGroupSize - 1) / kGroupSize;
    const std::size_t padded = static_cast<std::size_t>(groups) * kGroupSize;
    const bool ok = zr_.allocate(padded) && zi_.allocate(padded) && dr_.allocate(padded) &&
                    ci_.allocate(padded) && wIn_.allocate(padded) && wOut_.allocate(padded) &&
                    groupInOut_.allocate(static_cast<std::size_t>(groups)) &&
                    active_.allocate(static_cast<std::size_t>(groups)) &&
                    activeList_.allocate(static_cast<std::size_t>(groups)) &&
                    scratch_.allocate(static_cast<std::size_t>(kCullInterval) * simd::kWidth) &&
                    zeros_.allocate(kCullInterval);
    capacity_ = ok ? static_cast<int>(padded) : 0;
    size_ = 0;
    sinceCull_ = 0;
    intervalInput_ = 0.0f;
    groupBlocks_ = 0;
    return ok;
}

void ModeBank::resize(int count) noexcept {
    count = std::clamp(count, 0, capacity_);
    for (int i = count; i < capacity_; ++i) {
        zr_[i] = zi_[i] = dr_[i] = ci_[i] = wIn_[i] = wOut_[i] = 0.0f;
    }
    size_ = count;
    for (int g = 0; g < capacity_ / kGroupSize; ++g) {
        updateGroupGain(g);
        if (g >= groupCount()) active_[g] = 0;
        else if (!cullEnabled_) active_[g] = 1;
    }
}

void ModeBank::setMode(int i, double freqHz, double sigma, double fs) noexcept {
    const double theta = 2.0 * std::numbers::pi * freqHz / fs;
    const double rm1 = std::expm1(-sigma / fs);  // |c| - 1 without cancellation
    const double half = std::sin(0.5 * theta);
    // Re(c) - 1 = (|c| - 1) cos(theta) + (cos(theta) - 1)
    dr_[i] = static_cast<float>(rm1 * std::cos(theta) - 2.0 * half * half);
    ci_[i] = static_cast<float>((1.0 + rm1) * std::sin(theta));
}

void ModeBank::setWeights(int i, float wIn, float wOut) noexcept {
    wIn_[i] = wIn;
    wOut_[i] = wOut;
    updateGroupGain(i / kGroupSize);
}

void ModeBank::setState(int i, float re, float im) noexcept {
    zr_[i] = re;
    zi_[i] = im;
    if (re != 0.0f || im != 0.0f) active_[i / kGroupSize] = 1;
}

void ModeBank::clearState() noexcept {
    for (int i = 0; i < capacity_; ++i) zr_[i] = zi_[i] = 0.0f;
    for (int g = 0; g < groupCount(); ++g) active_[g] = cullEnabled_ ? 0 : 1;
}

void ModeBank::setCulling(bool enabled, float thresholdDb) noexcept {
    cullEnabled_ = enabled;
    cullThreshold_ = std::pow(10.0f, thresholdDb / 20.0f);
    for (int g = 0; g < groupCount(); ++g) {
        bool any = !enabled;
        for (int m = g * kGroupSize; m < (g + 1) * kGroupSize && !any; ++m)
            any = zr_[m] != 0.0f || zi_[m] != 0.0f;
        active_[g] = any ? 1 : 0;
    }
}

int ModeBank::activeGroups() const noexcept {
    int n = 0;
    for (int g = 0; g < groupCount(); ++g) n += active_[g];
    return n;
}

void ModeBank::updateGroupGain(int g) noexcept {
    float m = 0.0f;
    for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i)
        m = std::max(m, std::abs(wIn_[i] * wOut_[i]));
    groupInOut_[g] = m;
}

// The sleep test uses sum(|wOut| * |z|), an upper bound on the group's output
// amplitude, rather than the plan's raw energy sum(|z|^2): it makes the
// threshold a guaranteed bound on the output error caused by culling.
// When further outputs exist (bridge force, hammer contact point), the bound
// must take the largest of their weights.
void ModeBank::cullPass() noexcept {
    for (int g = 0; g < groupCount(); ++g) {
        if (!active_[g]) continue;
        if (intervalInput_ * groupInOut_[g] > cullThreshold_) continue;
        float bound = 0.0f;
        const int m0 = g * kGroupSize;
        for (int m = m0; m < m0 + kGroupSize; ++m)
            bound += std::abs(wOut_[m]) * std::sqrt(zr_[m] * zr_[m] + zi_[m] * zi_[m]);
        if (bound < cullThreshold_) {
            for (int m = m0; m < m0 + kGroupSize; ++m) zr_[m] = zi_[m] = 0.0f;
            active_[g] = 0;
        }
    }
}

void ModeBank::process(const float* in, float* out, int nFrames, Kernel kernel) noexcept {
    const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),
                                ci_.data(), wIn_.data(), wOut_.data()};
    const int nGroups = groupCount();
    int pos = 0;
    while (pos < nFrames) {
        const int len = std::min(nFrames - pos, kCullInterval - sinceCull_);
        const float* inChunk = in ? in + pos : zeros_.data();

        if (cullEnabled_ && in) {
            // |z| gained from input over the chunk is at most |wIn| * sum|u|.
            float l1 = 0.0f;
            for (int k = 0; k < len; ++k) l1 += std::abs(inChunk[k]);
            intervalInput_ += l1;
            if (l1 > 0.0f)
                for (int g = 0; g < nGroups; ++g)
                    if (!active_[g] && l1 * groupInOut_[g] > cullThreshold_) active_[g] = 1;
        }

        int nActive = 0;
        for (int g = 0; g < nGroups; ++g)
            if (active_[g]) activeList_[nActive++] = g;

        if (nActive > 0) {
            if (kernel == Kernel::Simd)
                detail::runModesSimd(view, activeList_.data(), nActive, inChunk, out + pos, len,
                                     scratch_.data());
            else
                detail::runModesScalar(view, activeList_.data(), nActive, inChunk, out + pos, len);
            groupBlocks_ += static_cast<std::uint64_t>(nActive);
        }

        sinceCull_ += len;
        if (sinceCull_ == kCullInterval) {
            if (cullEnabled_) cullPass();
            sinceCull_ = 0;
            intervalInput_ = 0.0f;
        }
        pos += len;
    }
}

}  // namespace pm
