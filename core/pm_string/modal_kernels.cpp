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

void stepRunsScalar(const BankView& b, const int* seg, const int* m0, const int* m1, int nRuns,
                    const float* in, float* out) noexcept {
    for (int r = 0; r < nRuns; ++r) {
        const float u = in[seg[r]];
        float acc = 0.0f;
        for (int m = m0[r]; m < m1[r]; ++m) {
            const float zr = b.zr[m], zi = b.zi[m];
            const float ar = zr + b.wInRe[m] * u;
            const float ai = zi + b.wInIm[m] * u;
            const float nr = (ar + b.dr[m] * zr) - b.ci[m] * zi;
            const float ni = (ai + b.dr[m] * zi) + b.ci[m] * zr;
            b.zr[m] = nr;
            b.zi[m] = ni;
            acc += b.wOut[m] * ni;
        }
        out[seg[r]] += acc;
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

template <bool HasInput>
void runAll(const BankView& b, const int* groups, int nGroups, const float* in, float* scratch,
            int n) noexcept {
    int gi = 0;
    for (; gi + kGroupsPerStep <= nGroups; gi += kGroupsPerStep)
        step<kGroupsPerStep, HasInput>(b, groups + gi, in, scratch, n);
    for (; gi < nGroups; ++gi) step<1, HasInput>(b, groups + gi, in, scratch, n);
}

}  // namespace

// Memory-bound (seven loads and two stores per vector for six FMAs), so the
// loop is kept simple; two accumulators shorten the output chain.
void stepRunsSimd(const BankView& b, const int* seg, const int* m0, const int* m1, int nRuns,
                  const float* in, float* out) noexcept {
    for (int r = 0; r < nRuns; ++r) {
        const vf u = set1(in[seg[r]]);
        vf acc0 = zero(), acc1 = zero();
        int m = m0[r];
        const int end = m1[r];
        for (; m + 2 * kWidth <= end; m += 2 * kWidth) {
            for (int h = 0; h < 2; ++h) {
                const int i = m + h * kWidth;
                const vf zr = load(b.zr + i), zi = load(b.zi + i);
                const vf dr = load(b.dr + i), ci = load(b.ci + i);
                const vf ar = fmadd(load(b.wInRe + i), u, zr);
                const vf ai = fmadd(load(b.wInIm + i), u, zi);
                const vf nr = fnmadd(ci, zi, fmadd(dr, zr, ar));
                const vf ni = fmadd(ci, zr, fmadd(dr, zi, ai));
                store(b.zr + i, nr);
                store(b.zi + i, ni);
                if (h == 0) acc0 = fmadd(load(b.wOut + i), ni, acc0);
                else acc1 = fmadd(load(b.wOut + i), ni, acc1);
            }
        }
        for (; m < end; m += kWidth) {
            const vf zr = load(b.zr + m), zi = load(b.zi + m);
            const vf dr = load(b.dr + m), ci = load(b.ci + m);
            const vf ar = fmadd(load(b.wInRe + m), u, zr);
            const vf ai = fmadd(load(b.wInIm + m), u, zi);
            const vf nr = fnmadd(ci, zi, fmadd(dr, zr, ar));
            const vf ni = fmadd(ci, zr, fmadd(dr, zi, ai));
            store(b.zr + m, nr);
            store(b.zi + m, ni);
            acc0 = fmadd(load(b.wOut + m), ni, acc0);
        }
        out[seg[r]] += hsum(add(acc0, acc1));
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
