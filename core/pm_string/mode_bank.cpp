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
                    groupInOut_.allocate(ng) && active_.allocate(ng) && groupStale_.allocate(ng) &&
                    activeList_.allocate(ng) && segGroups_.allocate(ns + 1) &&
                    segCoupled_.allocate(ns) && groupSeg_.allocate(ng) && segVersion_.allocate(ns) &&
                    segWakeGain_.allocate(ns) && groupK0_.allocate(ng) && groupK1_.allocate(ng) &&
                    segInPrev_.allocate(ns) && groupH_.allocate(ng * kResponseTaps) &&
                    groupHValid_.allocate(ng) && groupH0_.allocate(ng * kResponseTaps) && groupH0Valid_.allocate(ng) && hList_.allocate(ng) &&
                    groupHAmount_.allocate(ng) && groupD1_.allocate(ng * kResponseTaps) &&
                    groupD2_.allocate(ng * kResponseTaps) && groupTaylorValid_.allocate(ng) &&
                    groupApprox_.allocate(ng) && segApprox_.allocate(ns) && segDampTick_.allocate(ns) &&
                    segT_.allocate(ns * 3 * kResponseTaps) && segAwakeVer_.allocate(ns) && segTVer_.allocate(ns) &&
                    segTValid_.allocate(ns) &&
                    segCullScale_.allocate(ns) && segInputScale_.allocate(ns) &&
                    segDamp_.allocate(ns) && segInput_.allocate(ns) &&
                    scratch_.allocate(static_cast<std::size_t>(kCullInterval) * simd::kWidth);
    capacity_ = ok ? static_cast<int>(padded) : 0;
    if (ok) {
        for (std::size_t i = 0; i < ns; ++i) segCullScale_[i] = segInputScale_[i] = 1.0f;
        for (std::size_t g = 0; g < ng; ++g) {
            groupStale_[g] = 0;
            groupSeg_[g] = 0;
            active_[g] = 0;
            groupHValid_[g] = groupH0Valid_[g] = groupTaylorValid_[g] = groupApprox_[g] = 0;
        }
        for (std::size_t i = 0; i < ns; ++i) {
            segApprox_[i] = segTValid_[i] = 0;
            segDampTick_[i] = segAwakeVer_[i] = segTVer_[i] = 0;
        }
    }
    activeCount_ = 0;
    size_ = 0;
    nSeg_ = ok ? segments : 0;
    sinceCull_ = 0;
    groupBlocks_ = 0;
    forcedBlocks_ = 0;
    groupsWoken_ = 0;
    responseGroups_ = approxGroups_ = 0;
    tick_ = 0;
    budget_ = tickBudget_ = -1;
    tickDemand_ = 0;
    refineCursor_ = 0;
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
        for (int q = segGroups_[s]; q < g; ++q) groupSeg_[q] = s;
    }
    segGroups_[nSegments] = g;
    size_ = capacity_;
    for (int i = 0; i < groups; ++i) active_[i] = cullEnabled_ ? 0 : 1;
    syncActive();
    touchAll();
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
    syncActive();
    touchAll();
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
    invalidateResponse(g);
    dampAm1_[i] = am1;
    dr_[i] = dr0_[i] + am1 + dr0_[i] * am1;
    ci_[i] = ci0_[i] * (1.0f + am1);
}

void ModeBank::setSegmentDamper(int s, float amount) noexcept {
    amount = std::max(amount, 0.0f);
    if (segDamp_[s] == amount) return;
    segDamp_[s] = amount;
    // The cached responses keep the amount they were computed for (see
    // coupledResponse); only the segment's response is out of date.
    ++segVersion_[s];
    segDampTick_[s] = tick_;
    if (amount != 0.0f)
        for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) tickDemand_ += active_[g];
    for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g) {
        if (active_[g]) refreshGroup(g);
        else groupStale_[g] = 1;
    }
}

void ModeBank::refreshGroup(int g) noexcept {
    groupStale_[g] = 0;
    const float amount = segDamp_[singleSegment_ ? 0 : groupSeg_[g]];
    const int i0 = g * kGroupSize;
    float x[kGroupSize];
    int outside = 0;  // an integer OR (not a short-circuit chain), so the loop vectorises
    for (int j = 0; j < kGroupSize; ++j) {
        x[j] = amount * dampRate_[i0 + j];
        outside |= x[j] >= 0.05f ? 1 : 0;
    }
    // The usual case (every x in the series range) as a branch-free loop
    // the compiler vectorises; damperAm1() evaluates the same expression.
    float am1[kGroupSize];
    if (!outside)
        for (int j = 0; j < kGroupSize; ++j)
            am1[j] = -x[j] * (1.0f - x[j] * (0.5f - x[j] * (1.0f / 6.0f - x[j] * (1.0f / 24.0f))));
    else
        for (int j = 0; j < kGroupSize; ++j) am1[j] = damperAm1(x[j]);
    for (int j = 0; j < kGroupSize; ++j) {
        const int i = i0 + j;
        dampAm1_[i] = am1[j];
        dr_[i] = dr0_[i] + am1[j] + dr0_[i] * am1[j];
        ci_[i] = ci0_[i] * (1.0f + am1[j]);
    }
}

void ModeBank::syncActive() noexcept {
    activeCount_ = 0;
    for (int g = 0; g < groupCount(); ++g) {
        if (!active_[g]) continue;
        ++activeCount_;
        if (groupStale_[g]) refreshGroup(g);
    }
}

std::complex<double> ModeBank::coefficient(int i) const noexcept {
    const int g = i / kGroupSize;
    if (!groupStale_[g]) return {1.0 + static_cast<double>(dr_[i]), static_cast<double>(ci_[i])};
    // The same float operations as refreshGroup().
    const float am1 = damperAm1(segDamp_[singleSegment_ ? 0 : groupSeg_[g]] * dampRate_[i]);
    const float dr = dr0_[i] + am1 + dr0_[i] * am1;
    const float ci = ci0_[i] * (1.0f + am1);
    return {1.0 + static_cast<double>(dr), static_cast<double>(ci)};
}

bool ModeBank::segmentAwake(int s) const noexcept {
    for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g)
        if (active_[g]) return true;
    return false;
}

void ModeBank::wakeSegment(int s) noexcept {
    for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g) setActive(g, 1);
}

void ModeBank::setWeights(int i, float wInRe, float wInIm, float wOut) noexcept {
    wIn_[i] = wInRe;
    wInIm_[i] = wInIm;
    wOut_[i] = wOut;
    updateGroupGain(i / kGroupSize);
    invalidateResponse(i / kGroupSize);
}

void ModeBank::setState(int i, float re, float im) noexcept {
    zr_[i] = re;
    zi_[i] = im;
    if (re != 0.0f || im != 0.0f) setActive(i / kGroupSize, 1);
}

void ModeBank::clearState() noexcept {
    for (int i = 0; i < capacity_; ++i) zr_[i] = zi_[i] = 0.0f;
    for (int g = 0; g < groupCount(); ++g) active_[g] = cullEnabled_ ? 0 : 1;
    syncActive();
    touchAll();
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
    syncActive();
    touchAll();
}

void ModeBank::setSegmentCullOffset(int s, float stateDb, float inputDb) noexcept {
    const float c = std::pow(10.0f, std::max(stateDb, 0.0f) / 20.0f);
    const float i = std::pow(10.0f, std::max(inputDb, 0.0f) / 20.0f);
    if (c == segCullScale_[s] && i == segInputScale_[s]) return;
    segCullScale_[s] = c;
    segInputScale_[s] = i;
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
    if (activeCount_ == 0) return;
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
                setActive(g, 0);
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
                        if (!active_[g] && l1 * groupInOut_[g] > cullThreshold_ * segInputScale_[s]) setActive(g, 1);
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
    ++segVersion_[s];
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
    ++segVersion_[s];
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

bool ModeBank::segmentHasAwakeCoupled(int s) const noexcept {
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g)
        if (active_[g]) return true;
    return false;
}

bool ModeBank::segmentWakeable(int s) const noexcept {
    // A sleeping group that no input could wake (a seated damper with the
    // engine's default offset) need not be watched.
    if (!cullEnabled_ || segInputScale_[s] >= 1e5f) return false;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g)
        if (!active_[g]) return true;
    return false;
}

void ModeBank::processSegmentsFree(SegmentIO* io, int nFrames, Kernel kernel) noexcept {
    const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),  ci_.data(),
                                wIn_.data(), wInIm_.data(), wOut_.data()};
    for (int s = 0; s < nSeg_; ++s) {
        SegmentIO& sio = io[s];
        sio.wrote = false;
        if (sio.skip) continue;
        int nActive = 0;
        for (int g = segFirstGroup(s); g < segFirstGroup(s + 1); ++g)
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

void ModeBank::computeResponses(const int* list, int n, bool lifted, Kernel kernel) noexcept {
    constexpr int T = kResponseTaps;
    float* cache = lifted ? groupH0_.data() : groupH_.data();
    std::uint8_t* valid = lifted ? groupH0Valid_.data() : groupHValid_.data();
    const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),  ci_.data(),
                                wIn_.data(), wInIm_.data(), wOut_.data()};
    responseGroups_ += static_cast<std::uint64_t>(n);
    // Responses of consecutive groups land in consecutive rows (the usual
    // case: a damper change), one kernel call per run.
    int a = 0;
    while (a < n) {
        int b = a + 1;
        while (b < n && list[b] == list[b - 1] + 1) ++b;
        float* row = cache + std::size_t(list[a]) * T;
        if (kernel == Kernel::Simd)
            detail::impulseSimd(view, list + a, b - a, row + 1, T, T - 1);
        else
            detail::impulseScalar(view, list + a, b - a, row + 1, T, T - 1);
        for (int q = a; q < b; ++q) {
            const int g = list[q];
            float h0 = 0.0f;
            for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i) h0 += wOut_[i] * wInIm_[i];
            cache[std::size_t(g) * T] = h0;
            valid[g] = 1;
            if (!lifted) groupHAmount_[g] = segDamp_[groupSeg_[g]];
        }
        a = b;
    }
}

bool ModeBank::coupledResponse(int s, float* h, float* k1, Kernel kernel, bool* exact) noexcept {
    constexpr int T = kResponseTaps;
    const float amount = segDamp_[s];
    // With the damper lifted the coefficients are the free ones: the lifted
    // cache applies (its rows were computed with the same float values).
    const bool lifted = amount == 0.0f;
    int nActive = 0, nStale = 0;
    bool expandable = true;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) {
        if (!active_[g]) continue;
        activeList_[nActive++] = g;
        groupApprox_[g] = 0;
        const bool stale = lifted ? !groupH0Valid_[g] : !groupHValid_[g] || groupHAmount_[g] != amount;
        if (stale) {
            hList_[nStale++] = g;
            expandable = expandable && groupTaylorValid_[g];
        }
    }
    // Exact responses for the stale groups, if the tick's budget covers all
    // of them: not for a key whose damper moved this tick when the tick's
    // damper changes as a whole exceed the budget (a pedal stroke: the
    // budget then goes to keys at rest, refineResponses). Without the
    // expansion rows (a bank not set up by precomputeCoupledResponses) and
    // for the lifted cache, always exact.
    bool approx = false;
    if (nStale > 0 && !lifted && expandable && budget_ >= 0) {
        const bool moving = segDampTick_[s] == tick_;
        approx = nStale > budget_ || (moving && tickDemand_ > tickBudget_);
    }
    segApprox_[s] = approx ? 1 : 0;
    if (exact) *exact = !approx;
    if (nActive == 0) return false;
    if (nStale > 0 && !approx) {
        computeResponses(hList_.data(), nStale, lifted, kernel);
        if (budget_ > 0) budget_ = std::max(0, budget_ - nStale);
    }
    float K0 = 0.0f, K1 = 0.0f;
    for (int q = 0; q < nActive; ++q) {
        K0 += groupK0_[activeList_[q]];
        K1 += groupK1_[activeList_[q]];
    }
    const float d = std::min(amount, 1.0f);
    if (approx && nStale == nActive) {
        // Every awake group expanded (a key whose damper moves): the
        // expansion rows summed over the awake set, kept while it lasts.
        float* t = segT_.data() + std::size_t(s) * 3 * T;
        if (!segTValid_[s] || segTVer_[s] != segAwakeVer_[s]) {
            for (int k = 0; k < 3 * T; ++k) t[k] = 0.0f;
            for (int q = 0; q < nActive; ++q) {
                const std::size_t r = std::size_t(activeList_[q]) * T;
                const float* h0 = groupH0_.data() + r;
                const float* d1 = groupD1_.data() + r;
                const float* d2 = groupD2_.data() + r;
                for (int k = 0; k < T; ++k) {
                    t[k] += h0[k];
                    t[T + k] += d1[k];
                    t[2 * T + k] += d2[k];
                }
            }
            segTValid_[s] = 1;
            segTVer_[s] = segAwakeVer_[s];
        }
        for (int k = 0; k < T; ++k) h[k] = t[k] + d * (t[T + k] + d * t[2 * T + k]);
        for (int q = 0; q < nActive; ++q) groupApprox_[activeList_[q]] = 1;
        approxGroups_ += static_cast<std::uint64_t>(nActive);
    } else {
        if (approx)
            for (int q = 0; q < nStale; ++q) groupApprox_[hList_[q]] = 1;
        const float* cache = lifted ? groupH0_.data() : groupH_.data();
        for (int k = 0; k < T; ++k) h[k] = 0.0f;
        for (int q = 0; q < nActive; ++q) {
            const int g = activeList_[q];
            const std::size_t r = std::size_t(g) * T;
            if (groupApprox_[g]) {
                ++approxGroups_;
                const float* h0 = groupH0_.data() + r;
                const float* d1 = groupD1_.data() + r;
                const float* d2 = groupD2_.data() + r;
                for (int k = 0; k < T; ++k) h[k] += h0[k] + d * (d1[k] + d * d2[k]);
            } else {
                const float* row = cache + r;
                for (int k = 0; k < T; ++k) h[k] += row[k];
            }
        }
    }
    h[0] -= K0;
    h[1] -= K1;
    *k1 = K1;
    return true;
}

void ModeBank::refineResponses(Kernel kernel) noexcept {
    if (budget_ <= 0 || nSeg_ == 0) return;
    // Keep what this tick's own damper changes need, when they fit.
    const int reserve = tickDemand_ <= tickBudget_ ? tickDemand_ : 0;
    int avail = budget_ - reserve;
    if (avail <= 0) return;
    budget_ = reserve;
    for (int c = 0; c < nSeg_; ++c) {
        const int s = (refineCursor_ + c) % nSeg_;
        // Still moving: the sustain pedal moves a damper on every second tick.
        if (!segApprox_[s] || tick_ - segDampTick_[s] < 2u) continue;
        const float amount = segDamp_[s];
        int n = 0;
        bool more = false;
        for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) {
            if (!active_[g] || (groupHValid_[g] && groupHAmount_[g] == amount)) continue;
            if (n == avail) {
                more = true;
                break;
            }
            hList_[n++] = g;
        }
        if (n > 0) {
            computeResponses(hList_.data(), n, false, kernel);
            ++segVersion_[s];  // re-summed at its next coupledResponse
            avail -= n;
        } else {
            segApprox_[s] = 0;  // its approximate groups went to sleep
        }
        if (more || avail == 0) {
            refineCursor_ = more ? s : (s + 1) % nSeg_;
            break;
        }
    }
    budget_ += avail;  // unspent: left to the loop's wake-ups this tick
}

void ModeBank::resetResponses() noexcept {
    for (int g = 0; g < groupCount(); ++g) groupHValid_[g] = groupApprox_[g] = 0;
    for (int s = 0; s < nSeg_; ++s) {
        segApprox_[s] = 0;
        segDampTick_[s] = 0;
    }
    tick_ = 0;
    budget_ = -1;
    refineCursor_ = 0;
}

int ModeBank::approximateGroups() const noexcept {
    int n = 0;
    for (int s = 0; s < nSeg_; ++s) {
        if (!segApprox_[s]) continue;
        for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) n += active_[g] && groupApprox_[g];
    }
    return n;
}

void ModeBank::precomputeCoupledResponses() noexcept {
    const detail::BankView view{zr_.data(), zi_.data(), dr0_.data(), ci0_.data(),
                                wIn_.data(), wInIm_.data(), wOut_.data()};
    constexpr int T = kResponseTaps;
    for (int s = 0; s < nSeg_; ++s)
        for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) {
            float* row = groupH0_.data() + std::size_t(g) * T;
            detail::impulseSimd(view, &g, 1, row + 1, T, T - 1);
            float h0 = 0.0f;
            for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i) h0 += wOut_[i] * wInIm_[i];
            row[0] = h0;
            groupH0Valid_[g] = 1;
            // Expansion rows, in double from the float coefficients the
            // kernels use: m_k = sum wOut r^j Im(c0^k wIn), j = 1, 2.
            double m1[T] = {}, m2[T] = {};
            for (int i = g * kGroupSize; i < (g + 1) * kGroupSize; ++i) {
                const double r = dampRate_[i];
                if (wOut_[i] == 0.0f || r == 0.0) continue;
                const std::complex<double> c(1.0 + double(dr0_[i]), double(ci0_[i]));
                std::complex<double> z(wIn_[i], wInIm_[i]);
                for (int k = 0; k < T; ++k) {
                    const double y = double(wOut_[i]) * z.imag();
                    m1[k] += r * y;
                    m2[k] += r * r * y;
                    z *= c;
                }
            }
            float* d1 = groupD1_.data() + std::size_t(g) * T;
            float* d2 = groupD2_.data() + std::size_t(g) * T;
            for (int k = 0; k < T; ++k) {
                d1[k] = static_cast<float>(-double(k) * m1[k]);
                d2[k] = static_cast<float>(0.5 * double(k) * double(k) * m2[k]);
            }
            groupTaylorValid_[g] = 1;
        }
}

void ModeBank::applyCoupledInput(int s, const float* u, int n, Kernel kernel) noexcept {
    if (n <= 0) return;
    segInPrev_[s] = u[n - 1];
    int nActive = 0;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g)
        if (active_[g]) activeList_[nActive++] = g;
    if (nActive > 0) {
        const detail::BankView view{zr_.data(), zi_.data(), dr_.data(),  ci_.data(),
                                    wIn_.data(), wInIm_.data(), wOut_.data()};
        if (kernel == Kernel::Simd)
            detail::forcedSimd(view, activeList_.data(), nActive, u, n);
        else
            detail::forcedScalar(view, activeList_.data(), nActive, u, n);
        forcedBlocks_ += static_cast<std::uint64_t>(nActive);
    }
    if (!cullEnabled_) return;
    float l1 = 0.0f;
    for (int k = 0; k < n; ++k) l1 += std::fabs(u[k]);
    const float before = segInput_[s];
    segInput_[s] += l1;
    if (nActive < segCoupled_[s] && segInput_[s] * segWakeGain_[s] > cullThreshold_ * segInputScale_[s])
        wakeCoupled(s, before, u, n);
}

// A group wakes at the first sample i where the input accumulated since the
// last cull pass could have moved its output above the threshold, from its
// quasi-static deflection under u[i], z = wIn u / (1 - c) (its static
// correction, which joins at the same time, then cancels it: no force step
// reaches the board), advanced with the rest of the sub-block's input. The
// group's output over the rest of the sub-block is missing from the loop.
// Double precision. Not rare: when the pedal lifts the dampers off strings
// that still ring, a hundred groups or more can wake in one sub-block (the
// input summed since the cull pass crosses their bounds as soon as the
// damper's raised input threshold drops), and done one mode after the
// other each mode's recursion is a serial chain of dependent double
// operations, latency-bound: about 150 us extra in such a buffer on an M2
// performance core, back to back (plan §4.1). The group's modes are therefore advanced together, as
// independent lanes the compiler vectorises, with exactly the operations
// of the complex expression z = c z + w u (product terms rounded the same
// way, the same sum order), so the state is bit for bit what the per-mode
// std::complex loop gave. The start-up value keeps std::complex division.
void ModeBank::wakeCoupled(int s, float inputBefore, const float* u, int n) noexcept {
    const float thr = cullThreshold_ * segInputScale_[s];
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) {
        if (active_[g]) continue;
        float acc = inputBefore;
        int i = 0;
        for (; i < n; ++i) {
            acc += std::fabs(u[i]);
            if (acc * groupInOut_[g] > thr) break;
        }
        if (i == n) continue;
        if (groupStale_[g]) refreshGroup(g);
        const int m0 = g * kGroupSize;
        alignas(64) double zr[kGroupSize], zi[kGroupSize], cr[kGroupSize], cim[kGroupSize], wr[kGroupSize],
            wi[kGroupSize];
        for (int j = 0; j < kGroupSize; ++j) {
            const int m = m0 + j;
            const std::complex<double> w(wIn_[m], wInIm_[m]);
            const std::complex<double> c(1.0 + double(dr_[m]), double(ci_[m]));
            const std::complex<double> z = w == 0.0 ? std::complex<double>(0.0) : w * double(u[i]) / (1.0 - c);
            zr[j] = z.real();
            zi[j] = z.imag();
            cr[j] = c.real();
            cim[j] = c.imag();
            wr[j] = w.real();
            wi[j] = w.imag();
        }
        // A mode with w = 0 starts at +0 and stays there (every product and
        // sum below is then +-0, and +0 + -0 = +0), as it was never advanced.
        // Separate statements: c z is rounded term by term as the complex
        // product is (no contraction across statements where only
        // in-expression contraction is allowed; where any is, it is applied
        // as it was to the lowered complex product).
        for (int k = i + 1; k < n; ++k) {
            const double uk = double(u[k]);
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC unroll 1
#endif
            for (int j = 0; j < kGroupSize; ++j) {
                const double t1 = cr[j] * zr[j];
                const double t2 = cim[j] * zi[j];
                const double rr = t1 - t2;
                const double t3 = cr[j] * zi[j];
                const double t4 = cim[j] * zr[j];
                const double ri = t3 + t4;
                const double pr = wr[j] * uk;
                const double pi = wi[j] * uk;
                zr[j] = rr + pr;
                zi[j] = ri + pi;
            }
        }
        ++groupsWoken_;
        for (int j = 0; j < kGroupSize; ++j) {
            zr_[m0 + j] = static_cast<float>(zr[j]);
            zi_[m0 + j] = static_cast<float>(zi[j]);
        }
        setActive(g, 1);
    }
}

void ModeBank::wakeAllCoupled(int s, float u) noexcept {
    if (!cullEnabled_) return;
    for (int g = segFirstGroup(s); g < segFirstGroup(s) + segCoupled_[s]; ++g) {
        if (active_[g]) continue;
        if (groupStale_[g]) refreshGroup(g);
        for (int m = g * kGroupSize; m < (g + 1) * kGroupSize; ++m) {
            const std::complex<double> w(wIn_[m], wInIm_[m]);
            const std::complex<double> c(1.0 + double(dr_[m]), double(ci_[m]));
            const std::complex<double> z = w == 0.0 ? 0.0 : w * double(u) / (1.0 - c);
            zr_[m] = static_cast<float>(z.real());
            zi_[m] = static_cast<float>(z.imag());
        }
        ++groupsWoken_;
        setActive(g, 1);
    }
}

void ModeBank::endCoupled(const SegmentIO* io, int nFrames) noexcept {
    sinceCull_ += nFrames;
    if (sinceCull_ >= kCullInterval) {
        if (cullEnabled_) cullPass(io);
        sinceCull_ = 0;
        for (int s = 0; s < nSeg_; ++s) segInput_[s] = 0.0f;
    }
}

}  // namespace pm
