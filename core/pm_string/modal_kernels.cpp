#include "pm_string/modal_kernels.h"

#include "pm_base/simd.h"
#include "pm_string/mode_bank.h"

namespace pm::detail {

void runModesScalar(const BankView& b, const int* groups, int nGroups, const float* in,
                    float* out, int n) noexcept {
    for (int gi = 0; gi < nGroups; ++gi) {
        const int m0 = groups[gi] * ModeBank::kGroupSize;
        for (int m = m0; m < m0 + ModeBank::kGroupSize; ++m) {
            const float dr = b.dr[m], ci = b.ci[m], wr = b.wInRe[m], wi = b.wInIm[m], wo = b.wOut[m];
            float zr = b.zr[m], zi = b.zi[m];
            for (int k = 0; k < n; ++k) {
                // Same evaluation order as the SIMD kernel.
                const float ar = in ? zr + wr * in[k] : zr;
                const float ai = in ? zi + wi * in[k] : zi;
                const float nr = (ar + dr * zr) - ci * zi;
                const float ni = (ai + dr * zi) + ci * zr;
                zr = nr;
                zi = ni;
                out[k] += wo * zi;
            }
            b.zr[m] = zr;
            b.zi[m] = zi;
        }
    }
}

void impulseScalar(const BankView& b, const int* groups, int nGroups, float* out, int n) noexcept {
    for (int gi = 0; gi < nGroups; ++gi) {
        const int m0 = groups[gi] * ModeBank::kGroupSize;
        for (int m = m0; m < m0 + ModeBank::kGroupSize; ++m) {
            const float dr = b.dr[m], ci = b.ci[m], wo = b.wOut[m];
            float zr = b.wInRe[m], zi = b.wInIm[m];
            for (int k = 0; k < n; ++k) {
                const float nr = (zr + dr * zr) - ci * zi;
                const float ni = (zi + dr * zi) + ci * zr;
                zr = nr;
                zi = ni;
                out[k] += wo * zi;
            }
        }
    }
}

void forcedScalar(const BankView& b, const int* groups, int nGroups, const float* u, int n) noexcept {
    for (int gi = 0; gi < nGroups; ++gi) {
        const int m0 = groups[gi] * ModeBank::kGroupSize;
        for (int m = m0; m < m0 + ModeBank::kGroupSize; ++m) {
            const float dr = b.dr[m], ci = b.ci[m];
            float ar = 0.0f, ai = 0.0f;
            for (int k = 0; k < n; ++k) {
                const float nr = ((ar + u[k]) + dr * ar) - ci * ai;
                const float ni = (ai + dr * ai) + ci * ar;
                ar = nr;
                ai = ni;
            }
            const float wr = b.wInRe[m], wi = b.wInIm[m];
            b.zr[m] += wr * ar - wi * ai;
            b.zi[m] += wr * ai + wi * ar;
        }
    }
}

namespace {

using namespace pm::simd;

constexpr int kVecPerGroup = ModeBank::kGroupSize / kWidth;
static_assert(ModeBank::kGroupSize % kWidth == 0, "group must be a whole number of vectors");

// Number of groups advanced together per time step. One mode's recursion is a
// serial chain (z[n+1] needs z[n]: three dependent FMA latencies per
// sample), so enough independent vectors must be in flight to hide that
// latency; past that point extra vectors only add register spills (each
// vector needs zr, zi, dr, ci, wOut, plus wInRe, wInIm with an input).
//   NEON: 2 groups = 8 vectors (8 x 5 fills 32 registers less a few). NOT yet
//         measured on Apple silicon; tune with -DPM_GROUPS_PER_STEP=N on device.
//   AVX2: 4 groups = 8 vectors of 8 (measured best on x86: 1/2/4 groups gave
//         0.27/0.13/0.115 ns per mode-sample).
//   SSE2: 2 groups (1/2/4 measured within 15 %; no FMA, throughput-bound).
#if defined(PM_GROUPS_PER_STEP)
constexpr int kGroupsPerStep = PM_GROUPS_PER_STEP;
#elif defined(PM_SIMD_AVX2)
constexpr int kGroupsPerStep = 4;
#else
constexpr int kGroupsPerStep = 2;
#endif

// HasInput = false is the free-running case (every key in the engine: the
// hammer solver drives its modes itself): four FMAs and one output FMA per
// mode and sample. With an input the complex input weight adds two.
// Impulse = true: start from the input weight instead of the state and do
// not store the state (impulse response, see modal_kernels.h).
template <int NG, bool HasInput, bool Impulse = false>
inline void step(const BankView& b, const int* groups, const float* in, float* scratch,
                 int n) noexcept {
    constexpr int NV = NG * kVecPerGroup;
    constexpr int NI = HasInput ? NV : 1;
    vf zr[NV], zi[NV], dr[NV], ci[NV], wo[NV];
    [[maybe_unused]] vf wr[NI], wi[NI];
    int base[NV];
    for (int v = 0; v < NV; ++v) {
        base[v] = groups[v / kVecPerGroup] * ModeBank::kGroupSize + (v % kVecPerGroup) * kWidth;
        zr[v] = load((Impulse ? b.wInRe : b.zr) + base[v]);
        zi[v] = load((Impulse ? b.wInIm : b.zi) + base[v]);
        dr[v] = load(b.dr + base[v]);
        ci[v] = load(b.ci + base[v]);
        wo[v] = load(b.wOut + base[v]);
        if constexpr (HasInput) {
            wr[v] = load(b.wInRe + base[v]);
            wi[v] = load(b.wInIm + base[v]);
        }
    }
    for (int k = 0; k < n; ++k) {
        [[maybe_unused]] const vf u = HasInput ? set1(in[k]) : zero();
        // Two accumulators so the output sum is a short tree, not a chain.
        vf acc0 = load(scratch + k * kWidth);
        vf acc1 = zero();
        for (int v = 0; v < NV; ++v) {
            // z + (c - 1) z: four FMAs per mode, like the plain c*z form.
            vf ar = zr[v], ai = zi[v];
            if constexpr (HasInput) {
                ar = fmadd(wr[v], u, ar);
                ai = fmadd(wi[v], u, ai);
            }
            const vf nr = fnmadd(ci[v], zi[v], fmadd(dr[v], zr[v], ar));
            const vf ni = fmadd(ci[v], zr[v], fmadd(dr[v], zi[v], ai));
            zr[v] = nr;
            zi[v] = ni;
            if (v & 1)
                acc1 = fmadd(wo[v], ni, acc1);
            else
                acc0 = fmadd(wo[v], ni, acc0);
        }
        store(scratch + k * kWidth, add(acc0, acc1));
    }
    if constexpr (!Impulse)
        for (int v = 0; v < NV; ++v) {
            store(b.zr + base[v], zr[v]);
            store(b.zi + base[v], zi[v]);
        }
}

// Horner accumulation of the input in c for NG groups at once (in registers).
template <int NG>
inline void forcedStep(const BankView& b, const int* groups, const float* u, int n) noexcept {
    constexpr int NV = NG * kVecPerGroup;
    vf ar[NV], ai[NV], dr[NV], ci[NV];
    int base[NV];
    for (int v = 0; v < NV; ++v) {
        base[v] = groups[v / kVecPerGroup] * ModeBank::kGroupSize + (v % kVecPerGroup) * kWidth;
        dr[v] = load(b.dr + base[v]);
        ci[v] = load(b.ci + base[v]);
        ar[v] = zero();
        ai[v] = zero();
    }
    for (int k = 0; k < n; ++k) {
        const vf x = set1(u[k]);
        for (int v = 0; v < NV; ++v) {
            const vf nr = fnmadd(ci[v], ai[v], fmadd(dr[v], ar[v], add(ar[v], x)));
            const vf ni = fmadd(ci[v], ar[v], fmadd(dr[v], ai[v], ai[v]));
            ar[v] = nr;
            ai[v] = ni;
        }
    }
    for (int v = 0; v < NV; ++v) {
        const vf wr = load(b.wInRe + base[v]), wi = load(b.wInIm + base[v]);
        store(b.zr + base[v], fnmadd(wi, ai[v], fmadd(wr, ar[v], load(b.zr + base[v]))));
        store(b.zi + base[v], fmadd(wi, ar[v], fmadd(wr, ai[v], load(b.zi + base[v]))));
    }
}

template <bool HasInput>
void runAll(const BankView& b, const int* groups, int nGroups, const float* in, float* scratch,
            int n) noexcept {
    int gi = 0;
    for (; gi + kGroupsPerStep <= nGroups; gi += kGroupsPerStep)
        step<kGroupsPerStep, HasInput>(b, groups + gi, in, scratch, n);
    for (; gi < nGroups; ++gi) step<1, HasInput>(b, groups + gi, in, scratch, n);
}

}  // namespace

void impulseSimd(const BankView& b, const int* groups, int nGroups, float* out, int n,
                 float* scratch) noexcept {
    for (int k = 0; k < n; ++k) store(scratch + k * kWidth, zero());
    int gi = 0;
    for (; gi + kGroupsPerStep <= nGroups; gi += kGroupsPerStep)
        step<kGroupsPerStep, false, true>(b, groups + gi, nullptr, scratch, n);
    for (; gi < nGroups; ++gi) step<1, false, true>(b, groups + gi, nullptr, scratch, n);
    for (int k = 0; k < n; ++k) out[k] += hsum(load(scratch + k * kWidth));
}

// No output and no input weights in the loop: four FMAs and an add per mode
// and sample, so more groups fit in registers than in step().
void forcedSimd(const BankView& b, const int* groups, int nGroups, const float* u, int n) noexcept {
    constexpr int G = kGroupsPerStep;
    int gi = 0;
    for (; gi + G <= nGroups; gi += G) forcedStep<G>(b, groups + gi, u, n);
    for (; gi < nGroups; ++gi) forcedStep<1>(b, groups + gi, u, n);
}

void runModesSimd(const BankView& b, const int* groups, int nGroups, const float* in,
                  float* out, int n, float* scratch) noexcept {
    // Per-lane partial sums live in scratch; one horizontal sum per sample at
    // the end instead of one per group per sample.
    for (int k = 0; k < n; ++k) store(scratch + k * kWidth, zero());
    if (in)
        runAll<true>(b, groups, nGroups, in, scratch, n);
    else
        runAll<false>(b, groups, nGroups, in, scratch, n);
    for (int k = 0; k < n; ++k) out[k] += hsum(load(scratch + k * kWidth));
}

}  // namespace pm::detail

namespace pm {
const char* simdKernelName() noexcept { return simd::kIsaName; }
}  // namespace pm
