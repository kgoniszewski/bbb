#include "pm_string/modal_kernels.h"

#include "pm_base/simd.h"
#include "pm_string/mode_bank.h"

namespace pm::detail {

void runModesScalar(const BankView& b, const int* groups, int nGroups, const float* in,
                    float* out, int n) noexcept {
    for (int gi = 0; gi < nGroups; ++gi) {
        const int m0 = groups[gi] * ModeBank::kGroupSize;
        for (int m = m0; m < m0 + ModeBank::kGroupSize; ++m) {
            const float dr = b.dr[m], ci = b.ci[m], wi = b.wIn[m], wo = b.wOut[m];
            float zr = b.zr[m], zi = b.zi[m];
            for (int k = 0; k < n; ++k) {
                // Same evaluation order as the SIMD kernel.
                const float nr = ((zr + wi * in[k]) + dr * zr) - ci * zi;
                const float ni = (zi + dr * zi) + ci * zr;
                zr = nr;
                zi = ni;
                out[k] += wo * zi;
            }
            b.zr[m] = zr;
            b.zi[m] = zi;
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
// vector needs zr, zi, dr, ci, wIn, wOut).
//   NEON: 2 groups = 8 vectors (8 x 6 fills 32 registers less a few). NOT yet
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

template <int NG>
inline void step(const BankView& b, const int* groups, const float* in, float* scratch,
                 int n) noexcept {
    constexpr int NV = NG * kVecPerGroup;
    vf zr[NV], zi[NV], dr[NV], ci[NV], wi[NV], wo[NV];
    int base[NV];
    for (int v = 0; v < NV; ++v) {
        base[v] = groups[v / kVecPerGroup] * ModeBank::kGroupSize + (v % kVecPerGroup) * kWidth;
        zr[v] = load(b.zr + base[v]);
        zi[v] = load(b.zi + base[v]);
        dr[v] = load(b.dr + base[v]);
        ci[v] = load(b.ci + base[v]);
        wi[v] = load(b.wIn + base[v]);
        wo[v] = load(b.wOut + base[v]);
    }
    for (int k = 0; k < n; ++k) {
        const vf u = set1(in[k]);
        // Two accumulators so the output sum is a short tree, not a chain.
        vf acc0 = load(scratch + k * kWidth);
        vf acc1 = zero();
        for (int v = 0; v < NV; ++v) {
            // z + (c - 1) z: four FMAs per mode, like the plain c*z form.
            const vf nr = fnmadd(ci[v], zi[v], fmadd(dr[v], zr[v], fmadd(wi[v], u, zr[v])));
            const vf ni = fmadd(ci[v], zr[v], fmadd(dr[v], zi[v], zi[v]));
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

}  // namespace

void runModesSimd(const BankView& b, const int* groups, int nGroups, const float* in,
                  float* out, int n, float* scratch) noexcept {
    // Per-lane partial sums live in scratch; one horizontal sum per sample at
    // the end instead of one per group per sample.
    for (int k = 0; k < n; ++k) store(scratch + k * kWidth, zero());

    int gi = 0;
    for (; gi + kGroupsPerStep <= nGroups; gi += kGroupsPerStep)
        step<kGroupsPerStep>(b, groups + gi, in, scratch, n);
    for (; gi < nGroups; ++gi) step<1>(b, groups + gi, in, scratch, n);

    for (int k = 0; k < n; ++k) out[k] += hsum(load(scratch + k * kWidth));
}

}  // namespace pm::detail

namespace pm {
const char* simdKernelName() noexcept { return simd::kIsaName; }
}  // namespace pm
