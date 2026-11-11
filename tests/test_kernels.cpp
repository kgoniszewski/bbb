// Scalar reference kernel versus SIMD kernel (plan §4.2, §5.4).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <utility>
#include <vector>

#include "pm_base/aligned.h"
#include "signal_utils.h"
#include "test_harness.h"

using namespace pm;
using pm_test::render;

namespace {

constexpr double kFs = 48000.0;

// A bank with 'modes' random modes and random weights, plus a pluck-like
// initial state, so every lane differs.
void fillRandom(ModeBank& bank, int modes, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uf(30.0, 20000.0), us(0.5, 40.0), uw(-1.0, 1.0);
    bank.allocate(modes);
    bank.resize(modes);
    for (int i = 0; i < modes; ++i) {
        const double f = uf(rng), sigma = us(rng);
        const double wIn = 0.01 * uw(rng), wInIm = 0.01 * uw(rng), wOut = uw(rng) / modes;
        const double re = uw(rng), im = uw(rng);
        bank.setMode(i, f, sigma, kFs);
        bank.setWeights(i, static_cast<float>(wIn), static_cast<float>(wInIm), static_cast<float>(wOut));
        bank.setState(i, static_cast<float>(re), static_cast<float>(im));
    }
}

std::vector<float> noise(int n, unsigned seed, float amp) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(-amp, amp);
    std::vector<float> v(static_cast<std::size_t>(n));
    for (auto& x : v) x = u(rng);
    return v;
}

double maxAbs(const std::vector<float>& v) {
    double m = 0.0;
    for (float x : v) m = std::max(m, static_cast<double>(std::fabs(x)));
    return m;
}

double maxDiff(const std::vector<float>& a, const std::vector<float>& b) {
    double m = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        m = std::max(m, static_cast<double>(std::fabs(a[i] - b[i])));
    return m;
}

void compareKernels(int modes, bool withInput = true) {
    const int n = static_cast<int>(kFs);  // 1 s
    const auto u = noise(n, 7, 1.0f);
    ModeBank a, b;
    fillRandom(a, modes, 42);
    fillRandom(b, modes, 42);
    const auto ya = render(a, n, Kernel::Scalar, withInput ? &u : nullptr);
    const auto yb = render(b, n, Kernel::Simd, withInput ? &u : nullptr);
    const double peak = maxAbs(ya);
    // First block: only a few roundings apart.
    double firstBlock = 0.0;
    for (int k = 0; k < 64; ++k) firstBlock = std::max(firstBlock, double(std::fabs(ya[k] - yb[k])));
    const double whole = maxDiff(ya, yb);
    double stateDiff = 0.0;
    for (int i = 0; i < modes; ++i)
        stateDiff = std::max({stateDiff, double(std::fabs(a.stateRe(i) - b.stateRe(i))),
                              double(std::fabs(a.stateIm(i) - b.stateIm(i)))});
    std::printf("  %d modes (%s, %s): peak %.3f, first block diff %.2e, 1 s diff %.2e (rel %.2e), "
                "state diff %.2e\n",
                modes, simdKernelName(), withInput ? "complex input" : "free-running", peak, firstBlock,
                whole, whole / peak, stateDiff);
    CHECK(peak > 0.01);
    CHECK_LE(firstBlock / peak, 1e-6);
    CHECK_LE(whole / peak, 2e-5);
    CHECK_LE(stateDiff, 1e-4);
}

}  // namespace

TEST(scalar_and_simd_agree_even_group_count) { compareKernels(128); }
// Odd group count exercises the single-group remainder path.
TEST(scalar_and_simd_agree_odd_group_count) { compareKernels(80); }
// Partial last group (padding lanes must stay silent).
TEST(scalar_and_simd_agree_partial_group) { compareKernels(37); }
// The free-running kernel (no input).
TEST(scalar_and_simd_agree_free_running) {
    compareKernels(128, false);
    compareKernels(80, false);
}

// No input is bit-identical to a zero input.
TEST(free_running_equals_zero_input) {
    const int n = 4800;
    const std::vector<float> zeros(static_cast<std::size_t>(n), 0.0f);
    for (Kernel k : {Kernel::Scalar, Kernel::Simd}) {
        ModeBank a, b;
        fillRandom(a, 100, 9);
        fillRandom(b, 100, 9);
        const auto ya = render(a, n, k, &zeros);
        const auto yb = render(b, n, k, nullptr);
        CHECK(maxDiff(ya, yb) == 0.0);
    }
}

TEST(string_pluck_kernels_agree) {
    StringParams p;
    ModeBank a, b;
    a.allocate(256);
    b.allocate(256);
    loadString(a, p, kFs);
    loadString(b, p, kFs);
    pluck(a, 0.12, 1e-3);
    pluck(b, 0.12, 1e-3);
    const int n = static_cast<int>(2 * kFs);
    const auto ya = render(a, n, Kernel::Scalar);
    const auto yb = render(b, n, Kernel::Simd);
    const double rel = maxDiff(ya, yb) / maxAbs(ya);
    std::printf("  C4 pluck, 2 s: relative max diff %.2e\n", rel);
    CHECK_LE(rel, 2e-5);
}

// Splitting the same signal into different block sizes must not change the
// result at all: sub-block boundaries do not alter any arithmetic.
TEST(block_size_does_not_change_output) {
    const int n = 10000;
    const auto u = noise(n, 3, 0.5f);
    for (Kernel k : {Kernel::Scalar, Kernel::Simd}) {
        ModeBank a, b;
        fillRandom(a, 100, 5);
        fillRandom(b, 100, 5);
        const auto ya = render(a, n, k, &u, 64);
        const auto yb = render(b, n, k, &u, 37);
        CHECK(maxDiff(ya, yb) == 0.0);
    }
}

// Block-coupled kernels (mode_bank.h, "Coupled processing"): the free pass
// plus the forced pass reproduce stepping every mode with its input sample
// by sample (runModes with input), and the output predicted from the free
// response and coupledResponse() (no static correction set here) matches;
// for the scalar references and the SIMD kernels of both new passes
// (impulse response, forced update).
TEST(block_coupled_matches_per_sample_input) {
    for (Kernel k : {Kernel::Scalar, Kernel::Simd}) {
        const int modes = 16 * 7;  // a segment of 7 groups, all coupled
        std::mt19937 rng(9);
        std::uniform_real_distribution<double> uf(30.0, 5000.0), us(0.2, 20.0), uw(-1.0, 1.0);
        ModeBank a, b;
        const int nm = modes;
        a.allocateSegments(&nm, 1);
        b.allocateSegments(&nm, 1);
        for (int i = 0; i < modes; ++i) {
            const double f = uf(rng), s = us(rng);
            const float wr = static_cast<float>(0.01 * uw(rng)), wi = static_cast<float>(0.01 * uw(rng));
            const float wo = static_cast<float>(uw(rng) / modes);
            const float zr = static_cast<float>(uw(rng)), zi = static_cast<float>(uw(rng));
            for (ModeBank* m : {&a, &b}) {
                m->setMode(i, f, s, kFs);
                m->setWeights(i, wr, wi, wo);
                m->setState(i, zr, zi);
            }
        }
        for (ModeBank* m : {&a, &b}) {
            m->setCulling(false);
            m->setSegmentDamper(0, 0.3f);
            m->setSegmentCoupledGroups(0, 7);
        }
        float h[ModeBank::kResponseTaps], k1 = 0.0f;
        CHECK(a.coupledResponse(0, h, &k1, k));
        double worstOut = 0.0, worstState = 0.0, peak = 0.0;
        std::vector<float> ref(32), blk(32), u = noise(32, 3, 1.0f);
        for (int rep = 0; rep < 50; ++rep) {
            // Reference: per-sample stepping with input.
            SegmentIO ioa;
            ioa.in = u.data();
            ioa.out = ref.data();
            a.processSegments(&ioa, 32, k);
            // Block: free pass, then the forced pass; output from the response.
            SegmentIO iob;
            iob.out = blk.data();
            b.processSegmentsFree(&iob, 32, k);
            for (int i = 0; i < 32; ++i) {
                double y = blk[i];
                for (int j = 0; j <= i; ++j) y += double(h[j]) * u[i - j];
                blk[i] = static_cast<float>(y);
            }
            b.applyCoupledInput(0, u.data(), 32, k);
            for (int i = 0; i < 32; ++i) peak = std::max(peak, double(std::fabs(ref[i])));
            for (int i = 0; i < modes; ++i)
                worstState = std::max(worstState, double(std::fabs(a.stateRe(i) - b.stateRe(i))) +
                                                      std::fabs(a.stateIm(i) - b.stateIm(i)));
            for (int i = 0; i < 32; ++i) worstOut = std::max(worstOut, double(std::fabs(blk[i] - ref[i])));
            u = noise(32, 100 + rep, 1.0f);
        }
        std::printf("  %s: block vs per-sample: worst state difference %.2e, worst output difference %.2e (peak %.2e)\n",
                    k == Kernel::Simd ? simdKernelName() : "scalar", worstState, worstOut, peak);
        CHECK_LE(worstState, 1e-5);
        CHECK_LE(worstOut, 1e-4 * peak);
    }
}

// Large arrays allocated together start at distinct cache-line offsets
// modulo 4 KiB and 16 KiB (aligned.h), so kernels that stream them in
// lockstep at equal indices never map them all to one cache set.
TEST(large_arrays_are_staggered_across_cache_sets) {
    constexpr int kArrays = 20;
    constexpr std::size_t kCount = 18336;  // a full-piano bank's capacity
    AlignedArray<float> a[kArrays];
    bool seen4k[64] = {}, seen16k[256] = {};
    for (int j = 0; j < kArrays; ++j) {
        CHECK(a[j].allocate(kCount));
        const auto addr = reinterpret_cast<std::uintptr_t>(a[j].data());
        CHECK(addr % kCacheLine == 0);
        const std::size_t l4 = (addr % 4096) / kCacheLine, l16 = (addr % 16384) / kCacheLine;
        CHECK(!seen4k[l4]);
        CHECK(!seen16k[l16]);
        seen4k[l4] = seen16k[l16] = true;
        bool zero = true;
        for (std::size_t i = 0; i < kCount; ++i) zero = zero && a[j][i] == 0.0f;
        CHECK(zero);
        a[j][kCount - 1] = 1.0f;
    }
    AlignedArray<float> moved(std::move(a[0]));
    CHECK(moved.size() == kCount && moved[kCount - 1] == 1.0f && a[0].data() == nullptr);
    AlignedArray<float> small;
    CHECK(small.allocate(100));
    CHECK(reinterpret_cast<std::uintptr_t>(small.data()) % kCacheLine == 0);
}
