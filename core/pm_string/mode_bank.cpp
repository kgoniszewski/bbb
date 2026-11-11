#include "pm_string/mode_bank.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

#include "pm_base/simd.h"
#include "pm_string/modal_kernels.h"

namespace pm {

bool ModeBank::allocateArrays(int groups, int segments) noexcept {
    const std::size_t padded = static_cast<std::size_t>(groups) * kGroupSize;
    const std::size_t ng = static_cast<std::size_t>(groups);
    const std::size_t ns = static_cast<std::size_t>(std::max(segments, 1));
    const bool ok = zr_.allocate(padded) && zi_.allocate(padded) && dr_.allocate(padded) &&
                    ci_.allocate(padded) && wIn_.allocate(padded) && wInIm_.allocate(padded) &&
                    wOut_.allocate(padded) &&
                    dr0_.allocate(padded) && ci0_.allocate(padded) &&
                    dampRate_.allocate(padded) && dampAm1_.allocate(padded) &&
                    groupInOut_.allocate(ng) && active_.allocate(ng) &&
                    activeList_.allocate(ng) && segGroups_.allocate(ns + 1) &&
                    segCoupled_.allocate(ns) && segSkip_.allocate(ns) && segSleeping_.allocate(ns) && segHasRuns_.allocate(ns) &&
                    segWakeGain_.allocate(ns) && runSeg_.allocate(ng) && runM0_.allocate(ng) &&
                    runM1_.allocate(ng) && runSegList_.allocate(ns) && wakeList_.allocate(ns) &&
                    groupK0_.allocate(ng) && groupK1_.allocate(ng) &&
                    segK0_.allocate(ns) && segK1_.allocate(ns) && segInPrev_.allocate(ns) &&
                    segCullScale_.allocate(ns) && segInputScale_.allocate(ns) &&
                    segDamp_.allocate(ns) && segInput_.allocate(ns) &&
                    scratch_.allocate(static_cast<std::size_t>(kCullInterval) * simd::kWidth);
    capacity_ = ok ? static_cast<int>(padded) : 0;
    if (ok)
        for (std::size_t i = 0; i < ns; ++i) segCullScale_[i] = segInputScale_[i] = 1.0f;
    size_ = 0;
    nSeg_ = ok ? segments : 0;
    sinceCull_ = 0;
    groupBlocks_ = 0;
    nRuns_ = 0;
    runsDirty_ = true;
    return ok;
}

bool ModeBank::allocate(int maxModes) noexcept {
    if (maxModes < 0) return false;
    singleSegment_ = true;
    return allocateArrays((maxModes + kGroupSize - 1) / kGroupSize, 1);
}

bool ModeBank::allocateSegments(const int* modesPerSegment, int nSegments) noexcept {
    if (nSegments <= 0) return false;
    int groups = 0;
    for (int s = 0; s < nSegments; ++s) {
        if (modesPerSegment[s] < 0) return false;
        groups += (modesPerSegment[s] + kGroupSize - 1) / kGroupSize;
    }
    singleSegment_ = false;
    if (!allocateArrays(groups, nSegments)) return false;
    int g = 0;
    for (int s = 0; s < nSegments; ++s) {
        segGroups_[s] = g;
        g += (modesPerSegment[s] + kGroupSize - 1) / kGroupSize;
    }
    segGroups_[nSegments] = g;
    size_ = capacity_;
    for (int i = 0; i < groups; ++i) active_[i] = cullEnabled_ ? 0 : 1;
    return true;
}

void ModeBank::resize(int count) noexcept {
    if (!singleSegment_) return;
    count = std::clamp(count, 0, capacity_);
    for (int i = count; i < capacity_; ++i) {
        zr_[i] = zi_[i] = dr_[i] = ci_[i] = wIn_[i] = wInIm_[i] = wOut_[i] = 0.0f;
        dr0_[i] = ci0_[i] = dampRate_[i] = dampAm1_[i] = 0.0f;
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
    dr0_[i] = static_cast<float>(rm1 * std::cos(theta) - 2.0 * half * half);
    ci0_[i] = static_cast<float>((1.0 + rm1) * std::sin(theta));
    applyDamper(i);
}

void ModeBank::setDamperRate(int i, double sigmaDamp, double fs) noexcept {
    dampRate_[i] = static_cast<float>(std::max(sigmaDamp, 0.0) / fs);
    applyDamper(i);
}

namespace {
// a - 1 with a = exp(-x), x = d * sigmaDamp / fs >= 0. x is at most a few
// 1e-3 for any damper (sigma of a few hundred 1/s), where a 4-term series is
// exact to float precision; expm1 covers anything larger.
inline float damperAm1(float x) noexcept {
    if (x < 0.05f) return -x * (1.0f - x * (0.5f - x * (1.0f / 6.0f - x * (1.0f / 24.0f))));
    return std::expm1(-x);
}
}  // namespace

// c_eff = c0 * a, written as Re(c_eff) - 1 = dr0 + am1 + dr0 * am1 to keep the
// precision of the dr form.
void ModeBank::applyDamper(int i) noexcept {
    const int g = i / kGroupSize;
    int s = 0;
    if (!singleSegment_)
        s = static_cast<int>(std::upper_bound(segGroups_.data(), segGroups_.data() + nSeg_ + 1, g) -
                             segGroups_.data()) - 1;
    const float am1 = damperAm1(segDamp_[s] * dampRate_[i]);
    dampAm1_[i] = am1;
    dr_[i] = dr0_[i] + am1 + dr0_[i] * am1;
    ci_[i] = ci0_[i] * (1.0f + am1);
}

void ModeBank::setSegmentDamper(int s, float amount) noexcept {
    amount = std::max(amount, 0.0f);
    if (segDamp_[s] == amount) return;
    segDamp_[s] = amount;
    const int m0 = segmentFirstMode(s), m1 = m0 + segmentModeCount(s);
    for (int i = m0; i < m1; ++i) {
        const float am1 = damperAm1(amount * dampRate_[i]);
        dampAm1_[i] = am1;
        dr_[i] = dr0_[i] + am1 + dr0_[i] * am1;
        ci_[i] = ci0_[i] * (1.0f + am1);
    }
}

void ModeBank::wakeSegment(int s) noexcept {
    for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g) active_[g] = 1;
    runsDirty_ = true;
}

void ModeBank::setWeights(int i, float wInRe, float wInIm, float wOut) noexcept {
    wIn_[i] = wInRe;
    wInIm_[i] = wInIm;
    wOut_[i] = wOut;
    updateGroupGain(i / kGroupSize);
    runsDirty_ = true;
}

void ModeBank::setState(int i, float re, float im) noexcept {
    zr_[i] = re;
    zi_[i] = im;
    if (re != 0.0f || im != 0.0f) active_[i / kGroupSize] = 1;
    runsDirty_ = true;
}

void ModeBank::clearState() noexcept {
    for (int i = 0; i < capacity_; ++i) zr_[i] = zi_[i] = 0.0f;
    for (int g = 0; g < groupCount(); ++g) active_[g] = cullEnabled_ ? 0 : 1;
    runsDirty_ = true;
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
    runsDirty_ = true;
}

void ModeBank::setSegmentCullOffset(int s, float stateDb, float inputDb) noexcept {
    const float c = std::pow(10.0f, std::max(stateDb, 0.0f) / 20.0f);
    const float i = std::pow(10.0f, std::max(inputDb, 0.0f) / 20.0f);
    if (c == segCullScale_[s] && i == segInputScale_[s]) return;
    segCullScale_[s] = c;
    segInputScale_[s] = i;
    runsDirty_ = true;  // the wake list depends on the input threshold
}

int ModeBank::activeGroups() const noexcept {
    int n = 0;
    for (int g = 0; g < groupCount(); ++g) n += active_[g];
    return n;
}

// Bound on the group's output gained per unit of summed |input|: each mode's
// |z| grows by at most |wIn| sum|u|, so the output by sum |wIn| |wOut|.
void ModeBank::updateGroupGain(int g) noexcept {
    float m = 0.0f;
    for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i)
        m += std::hypot(wIn_[i], wInIm_[i]) * std::abs(wOut_[i]);
    groupInOut_[g] = m;
}

// The sleep test uses sum(|wOut| * |z|), an upper bound on the group's output
// amplitude, rather than the plan's raw energy sum(|z|^2): it makes the
// threshold a guaranteed bound on the output error caused by culling.
// The hammer solver also reads the modes (displacement at the strike point),
// but it only ever meets culled modes whose displacement is below
// threshold / |wOut|, far below any felt compression, so wOut alone is used.
void ModeBank::cullPass(const SegmentIO* io) noexcept {
    for (int s = 0; s < nSeg_; ++s) {
        if (io && io[s].skip) continue;
        const float input = segInput_[s];
        const float threshold = cullThreshold_ * segCullScale_[s];
        const float inThreshold = cullThreshold_ * segInputScale_[s];
        for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g) {
            if (!active_[g]) continue;
            if (input * groupInOut_[g] > inThreshold) continue;
            float bound = 0.0f;
            const int m0 = g * kGroupSize;
            for (int m = m0; m < m0 + kGroupSize; ++m)
                bound += std::abs(wOut_[m]) * std::sqrt(zr_[m] * zr_[m] + zi_[m] * zi_[m]);
            if (bound < threshold) {
                for (int m = m0; m < m0 + kGroupSize; ++m) zr_[m] = zi_[m] = 0.0f;
                active_[g] = 0;
                runsDirty_ = true;
            }
        }
    }
}

void ModeBank::run(SegmentIO* io, int nFrames, Kernel kernel, bool overwrite) noexcept {
    const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),  ci_.data(),
                                wIn_.data(), wInIm_.data(), wOut_.data()};
    for (int s = 0; s < nSeg_; ++s) io[s].wrote = false;
    int pos = 0;
    while (pos < nFrames) {
        const int len = std::min(nFrames - pos, kCullInterval - sinceCull_);
        for (int s = 0; s < nSeg_; ++s) {
            SegmentIO& sio = io[s];
            if (sio.skip) continue;
            const int g0 = segFirstGroup(s), g1 = segFirstGroup(s + 1);
            const float* inChunk = sio.in ? sio.in + pos : nullptr;

            if (cullEnabled_ && sio.in) {
                // |z| gained from input over the chunk is at most |wIn| * sum|u|.
                float l1 = 0.0f;
                for (int k = 0; k < len; ++k) l1 += std::abs(inChunk[k]);
                segInput_[s] += l1;
                if (l1 > 0.0f)
                    for (int g = g0; g < g1; ++g)
                        if (!active_[g] && l1 * groupInOut_[g] > cullThreshold_ * segInputScale_[s]) active_[g] = 1;
            }

            int nActive = 0;
            for (int g = g0; g < g1; ++g)
                if (active_[g]) activeList_[nActive++] = g;
            if (nActive == 0) continue;

            if (overwrite && !sio.wrote) std::fill(sio.out, sio.out + nFrames, 0.0f);
            sio.wrote = true;
            if (kernel == Kernel::Simd)
                detail::runModesSimd(view, activeList_.data(), nActive, inChunk, sio.out + pos,
                                     len, scratch_.data());
            else
                detail::runModesScalar(view, activeList_.data(), nActive, inChunk,
                                       sio.out + pos, len);
            groupBlocks_ += static_cast<std::uint64_t>(nActive);
        }

        sinceCull_ += len;
        if (sinceCull_ == kCullInterval) {
            if (cullEnabled_) cullPass(io);
            sinceCull_ = 0;
            for (int s = 0; s < nSeg_; ++s) segInput_[s] = 0.0f;
        }
        pos += len;
    }
}

void ModeBank::process(const float* in, float* out, int nFrames, Kernel kernel) noexcept {
    if (!singleSegment_) return;  // segmented banks use processSegments()
    SegmentIO io;
    io.in = in;
    io.out = out;
    run(&io, nFrames, kernel, false);
}

void ModeBank::processSegments(SegmentIO* io, int nFrames, Kernel kernel) noexcept {
    run(io, nFrames, kernel, true);
}

// ---------------------------------------------------------------- coupled

void ModeBank::setSegmentCoupledGroups(int s, int groups) noexcept {
    segCoupled_[s] = std::clamp(groups, 0, segFirstGroup(s + 1) - segFirstGroup(s));
    float m = 0.0f;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) m = std::max(m, groupInOut_[g]);
    segWakeGain_[s] = m;
    runsDirty_ = true;
}

void ModeBank::computeStaticCorrection(int s) noexcept {
    for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g) {
        double K = 0.0, D1 = 0.0;
        if (g < segFirstGroup(s) + segCoupled_[s]) {
            for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i) {
                const std::complex<double> w(wIn_[i], wInIm_[i]);
                if (w == 0.0) continue;
                const std::complex<double> c(1.0 + double(dr0_[i]), double(ci0_[i]));
                K += wOut_[i] * std::imag(w / (1.0 - c));
                D1 += wOut_[i] * std::imag(-w * c / ((1.0 - c) * (1.0 - c)));
            }
        }
        groupK0_[g] = static_cast<float>(K + D1);
        groupK1_[g] = static_cast<float>(-D1);
    }
    runsDirty_ = true;
}

float ModeBank::staticCorrection0(int s) const noexcept {
    float k = 0.0f;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) k += groupK0_[g];
    return k;
}
float ModeBank::staticCorrection1(int s) const noexcept {
    float k = 0.0f;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) k += groupK1_[g];
    return k;
}

void ModeBank::processSegmentsUncoupled(SegmentIO* io, int nFrames, Kernel kernel) noexcept {
    const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),  ci_.data(),
                                wIn_.data(), wInIm_.data(), wOut_.data()};
    for (int s = 0; s < nSeg_; ++s) {
        SegmentIO& sio = io[s];
        sio.wrote = false;
        if (sio.skip) continue;
        int nActive = 0;
        for (int g = segFirstGroup(s) + segCoupled_[s]; g < segFirstGroup(s + 1); ++g)
            if (active_[g]) activeList_[nActive++] = g;
        if (nActive == 0) continue;
        std::fill(sio.out, sio.out + nFrames, 0.0f);
        sio.wrote = true;
        if (kernel == Kernel::Simd)
            detail::runModesSimd(view, activeList_.data(), nActive, nullptr, sio.out, nFrames, scratch_.data());
        else
            detail::runModesScalar(view, activeList_.data(), nActive, nullptr, sio.out, nFrames);
        groupBlocks_ += static_cast<std::uint64_t>(nActive);
    }
}

void ModeBank::beginCoupled(const SegmentIO* io) noexcept {
    for (int s = 0; s < nSeg_; ++s) {
        const std::uint8_t k = io[s].skip ? 1 : 0;
        if (segSkip_[s] != k) runsDirty_ = true;
        segSkip_[s] = k;
    }
}

void ModeBank::rebuildRuns() noexcept {
    nRuns_ = nRunSegs_ = nWake_ = 0;
    for (int s = 0; s < nSeg_; ++s) {
        segSleeping_[s] = 0;
        segHasRuns_[s] = 0;
        segK0_[s] = segK1_[s] = 0.0f;
        if (segSkip_[s]) continue;
        const int g0 = segFirstGroup(s), g1 = g0 + segCoupled_[s];
        const int runsBefore = nRuns_;
        int g = g0;
        while (g < g1) {
            if (!active_[g]) {
                segSleeping_[s] = 1;
                ++g;
                continue;
            }
            const int start = g;
            while (g < g1 && active_[g]) {
                segK0_[s] += groupK0_[g];
                segK1_[s] += groupK1_[g];
                ++g;
            }
            runSeg_[nRuns_] = s;
            runM0_[nRuns_] = start * kGroupSize;
            runM1_[nRuns_] = g * kGroupSize;
            ++nRuns_;
        }
        if (nRuns_ > runsBefore) {
            runSegList_[nRunSegs_++] = s;
            segHasRuns_[s] = 1;
        }
        // A sleeping group that no input could wake (a seated damper with
        // the default offset) need not be watched.
        if (segSleeping_[s] && segInputScale_[s] < 1e5f) wakeList_[nWake_++] = s;
    }
    runsDirty_ = false;
}

int ModeBank::coupledSegments(const int*& list) noexcept {
    if (runsDirty_) rebuildRuns();
    list = runSegList_.data();
    return nRunSegs_;
}

int ModeBank::wakeableSegments(const int*& list) noexcept {
    if (runsDirty_) rebuildRuns();
    list = wakeList_.data();
    return nWake_;
}

// A woken group starts from its quasi-static deflection under the current
// input u, z = wIn u / (1 - c), not from rest: its static correction (which
// joins at the same time) then cancels exactly, and no force step reaches
// the board.
void ModeBank::wakeCoupled(int s, float u) noexcept {
    const float input = segInput_[s];
    bool sleeping = false;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) {
        if (active_[g]) continue;
        if (input * groupInOut_[g] > cullThreshold_ * segInputScale_[s]) {
            for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i) {
                const std::complex<double> w(wIn_[i], wInIm_[i]);
                const std::complex<double> c(1.0 + double(dr_[i]), double(ci_[i]));
                const std::complex<double> z = w == 0.0 ? 0.0 : w * double(u) / (1.0 - c);
                zr_[i] = static_cast<float>(z.real());
                zi_[i] = static_cast<float>(z.imag());
            }
            active_[g] = 1;
            runsDirty_ = true;
        } else {
            sleeping = true;
        }
    }
    segSleeping_[s] = sleeping ? 1 : 0;
}

void ModeBank::stepCoupled(const float* in, float* out, Kernel kernel) noexcept {
    if (runsDirty_) rebuildRuns();
    for (int i = 0; i < nRunSegs_; ++i) out[runSegList_[i]] = 0.0f;
    const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),  ci_.data(),
                                wIn_.data(), wInIm_.data(), wOut_.data()};
    if (kernel == Kernel::Simd)
        detail::stepRunsSimd(view, runSeg_.data(), runM0_.data(), runM1_.data(), nRuns_, in, out);
    else
        detail::stepRunsScalar(view, runSeg_.data(), runM0_.data(), runM1_.data(), nRuns_, in, out);
    for (int i = 0; i < nRunSegs_; ++i) {
        const int s = runSegList_[i];
        out[s] -= segK0_[s] * in[s] + segK1_[s] * segInPrev_[s];
        segInPrev_[s] = in[s];
        if (cullEnabled_) segInput_[s] += std::fabs(in[s]);
    }
    if (!cullEnabled_) return;
    bool woke = false;
    for (int i = 0; i < nWake_; ++i) {
        const int s = wakeList_[i];
        if (!segHasRuns_[s]) {  // segments with awake runs were handled above
            segInPrev_[s] = in[s];
            segInput_[s] += std::fabs(in[s]);
        }
        if (segInput_[s] * segWakeGain_[s] > cullThreshold_ * segInputScale_[s]) woke = true;
    }
    if (woke)
        for (int i = 0; i < nWake_; ++i) {
            const int s = wakeList_[i];
            if (segInput_[s] * segWakeGain_[s] > cullThreshold_ * segInputScale_[s]) wakeCoupled(s, in[s]);
        }
}

void ModeBank::endCoupled(const SegmentIO* io, int nFrames) noexcept {
    for (int r = 0; r < nRuns_; ++r)
        groupBlocks_ += static_cast<std::uint64_t>((runM1_[r] - runM0_[r]) / kGroupSize);
    sinceCull_ += nFrames;
    if (sinceCull_ >= kCullInterval) {
        if (cullEnabled_) cullPass(io);
        sinceCull_ = 0;
        for (int s = 0; s < nSeg_; ++s) segInput_[s] = 0.0f;
    }
}

}  // namespace pm
