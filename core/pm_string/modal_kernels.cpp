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

void impulseScalar(const BankView& b, const int* groups, int nGroups, float* out, int stride, int n) noexcept {
    for (int gi = 0; gi < nGroups; ++gi) {
        float* o = out + gi * stride;
        for (int k = 0; k < n; ++k) o[k] = 0.0f;
        const int m0 = groups[gi] * ModeBank::kGroupSize;
        for (int m = m0; m < m0 + ModeBank::kGroupSize; ++m) {
            const float dr = b.dr[m], ci = b.ci[m], wo = b.wOut[m];
            float zr = b.wInRe[m], zi = b.wInIm[m];
            for (int k = 0; k < n; ++k) {
                const float nr = (zr + dr * zr) - ci * zi;
                const float ni = (zi + dr * zi) + ci * zr;
                zr = nr;
                zi = ni;
                o[k] += wo * zi;
            }
        }
    }
}

void forcedScalar(const BankView& b, const int* groups, int nGroups, const float* u, int n) noexcept {
    for (int gi = 0; gi < nGroups; ++gi) {
        const int m0 = groups[gi] * ModeBank::kGroupSize;
        for (int m = m0; m < m0 + ModeBank::kGroupSize; ++m) {
            // Same evaluation order as the SIMD kernel.
            const float cr = 1.0f + b.dr[m], ci = b.ci[m];
            const float qr = cr * cr - ci * ci, qi = cr * ci + cr * ci;
            float ar = 0.0f, ai = 0.0f;
            int k = 0;
            for (; k + 2 <= n; k += 2) {
                const float nr = qr * ar + (((cr * u[k]) + u[k + 1]) - qi * ai);
                const float ni = qr * ai + (qi * ar + ci * u[k]);
                ar = nr;
                ai = ni;
            }
            if (k < n) {
                const float nr = cr * ar + (u[k] - ci * ai);
                const float ni = cr * ai + ci * ar;
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
// Mode vectors per step of the forced pass (six registers each). Measured
// on x86 (forced pass alone, ns per mode and sample for 1 / 2 / 4 vectors):
//   SSE2: 0.27 / 0.20 / 0.18 -> 4 (the free pass: 0.31)
//   AVX2: 0.09 / 0.13 / 0.11 -> 1 (the free pass: 0.19)
//   NEON (32 registers): 4. NOT yet measured on Apple silicon; tune with
//   -DPM_FORCED_VECTORS_PER_STEP=N (1, 2 or a multiple of the group).
#if defined(PM_FORCED_VECTORS_PER_STEP)
constexpr int kForcedVectorsPerStep = PM_FORCED_VECTORS_PER_STEP;
#elif defined(PM_SIMD_AVX2)
constexpr int kForcedVectorsPerStep = 1;
#else
constexpr int kForcedVectorsPerStep = 4;
#endif

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
template <int NG, bool HasInput>
inline void step(const BankView& b, const int* groups, const float* in, float* scratch,
                 int n) noexcept {
    constexpr int NV = NG * kVecPerGroup;
    constexpr int NI = HasInput ? NV : 1;
    vf zr[NV], zi[NV], dr[NV], ci[NV], wo[NV];
    [[maybe_unused]] vf wr[NI], wi[NI];
    int base[NV];
    for (int v = 0; v < NV; ++v) {
        base[v] = groups[v / kVecPerGroup] * ModeBank::kGroupSize + (v % kVecPerGroup) * kWidth;
        zr[v] = load(b.zr + base[v]);
        zi[v] = load(b.zi + base[v]);
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
    for (int v = 0; v < NV; ++v) {
        store(b.zr + base[v], zr[v]);
        store(b.zi + base[v], zi[v]);
    }
}

// Impulse responses of NG groups at once, one output row per group (same
// per-mode operation order as step()).
template <int NG>
inline void impulseStep(const BankView& b, const int* groups, float* out, int stride, int n) noexcept {
    constexpr int NV = NG * kVecPerGroup;
    vf zr[NV], zi[NV], dr[NV], ci[NV], wo[NV];
    int base[NV];
    for (int v = 0; v < NV; ++v) {
        base[v] = groups[v / kVecPerGroup] * ModeBank::kGroupSize + (v % kVecPerGroup) * kWidth;
        zr[v] = load(b.wInRe + base[v]);
        zi[v] = load(b.wInIm + base[v]);
        dr[v] = load(b.dr + base[v]);
        ci[v] = load(b.ci + base[v]);
        wo[v] = load(b.wOut + base[v]);
    }
    for (int k = 0; k < n; ++k) {
        for (int q = 0; q < NG; ++q) {
            vf acc = zero();
            for (int v = q * kVecPerGroup; v < (q + 1) * kVecPerGroup; ++v) {
                const vf nr = fnmadd(ci[v], zi[v], fmadd(dr[v], zr[v], zr[v]));
                const vf ni = fmadd(ci[v], zr[v], fmadd(dr[v], zi[v], zi[v]));
                zr[v] = nr;
                zi[v] = ni;
                acc = fmadd(wo[v], ni, acc);
            }
            out[q * stride + k] = hsum(acc);
        }
    }
}

// Horner accumulation of the input for NV mode vectors at once (in
// registers), two samples per step:
//   acc <- c^2 acc + (c u[k] + u[k+1])
// which needs three operations per component and sample pair against four
// per sample, with a dependent chain of two operations per pair. It uses c
// itself rather than the d = c - 1 form of the state recursion: the sum runs
// over one sub-block only, where |c| resolved to 6e-8 changes the result by
// about 1e-6 relative.
template <int NV>
inline void forcedStep(const BankView& b, const int* base, const float* u, int n) noexcept {
    vf ar[NV], ai[NV], cr[NV], ci[NV], qr[NV], qi[NV];
    const vf one = set1(1.0f);
    for (int v = 0; v < NV; ++v) {
        cr[v] = add(one, load(b.dr + base[v]));
        ci[v] = load(b.ci + base[v]);
        qr[v] = fnmadd(ci[v], ci[v], mul(cr[v], cr[v]));  // c^2
        qi[v] = add(mul(cr[v], ci[v]), mul(cr[v], ci[v]));
        ar[v] = zero();
        ai[v] = zero();
    }
    int k = 0;
    for (; k + 2 <= n; k += 2) {
        const vf x0 = set1(u[k]), x1 = set1(u[k + 1]);
        for (int v = 0; v < NV; ++v) {
            const vf nr = fmadd(qr[v], ar[v], fnmadd(qi[v], ai[v], fmadd(cr[v], x0, x1)));
            const vf ni = fmadd(qr[v], ai[v], fmadd(qi[v], ar[v], mul(ci[v], x0)));
            ar[v] = nr;
            ai[v] = ni;
        }
    }
    if (k < n) {
        const vf x = set1(u[k]);
        for (int v = 0; v < NV; ++v) {
            const vf nr = fmadd(cr[v], ar[v], fnmadd(ci[v], ai[v], x));
            const vf ni = fmadd(cr[v], ai[v], mul(ci[v], ar[v]));
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

void impulseSimd(const BankView& b, const int* groups, int nGroups, float* out, int stride, int n) noexcept {
    int gi = 0;
    for (; gi + kGroupsPerStep <= nGroups; gi += kGroupsPerStep)
        impulseStep<kGroupsPerStep>(b, groups + gi, out + gi * stride, stride, n);
    for (; gi < nGroups; ++gi) impulseStep<1>(b, groups + gi, out + gi * stride, stride, n);
}

// Six vectors live per mode vector (ar, ai, c, c^2), so the register block
// is counted in vectors, not groups.
void forcedSimd(const BankView& b, const int* groups, int nGroups, const float* u, int n) noexcept {
    constexpr int NV = kForcedVectorsPerStep;
    static_assert(kVecPerGroup % NV == 0 || NV % kVecPerGroup == 0, "register block must tile groups");
    int base[NV];
    if constexpr (NV <= kVecPerGroup) {
        for (int gi = 0; gi < nGroups; ++gi)
            for (int v0 = 0; v0 < kVecPerGroup; v0 += NV) {
                for (int v = 0; v < NV; ++v) base[v] = groups[gi] * ModeBank::kGroupSize + (v0 + v) * kWidth;
                forcedStep<NV>(b, base, u, n);
            }
    } else {
        constexpr int G = NV / kVecPerGroup;
        int gi = 0;
        for (; gi + G <= nGroups; gi += G) {
            for (int v = 0; v < NV; ++v)
                base[v] = groups[gi + v / kVecPerGroup] * ModeBank::kGroupSize + (v % kVecPerGroup) * kWidth;
            forcedStep<NV>(b, base, u, n);
        }
        for (; gi < nGroups; ++gi) {
            for (int v = 0; v < kVecPerGroup; ++v) base[v] = groups[gi] * ModeBank::kGroupSize + v * kWidth;
            forcedStep<kVecPerGroup>(b, base, u, n);
        }
    }
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
