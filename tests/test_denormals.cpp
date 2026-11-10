// Denormal guard and long decays to silence (plan §4.2).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "pm_base/denormals.h"
#include "signal_utils.h"
#include "test_harness.h"

using namespace pm;

namespace {

constexpr double kFs = 48000.0;

float tinyProduct() {
    volatile float a = 1e-30f;
    volatile float b = 1e-10f;
    return a * b;  // 1e-40: subnormal unless flushed
}

bool isSubnormal(float x) { return std::fpclassify(x) == FP_SUBNORMAL; }

StringParams fastDecay() {
    StringParams p;
    p.b1 = 200.0;  // falls 700 dB in under 0.5 s: deep into the subnormal range
    return p;
}

int countSubnormalStates(const ModeBank& bank) {
    int n = 0;
    for (int i = 0; i < bank.size(); ++i)
        n += isSubnormal(bank.stateRe(i)) + isSubnormal(bank.stateIm(i));
    return n;
}

}  // namespace

TEST(guard_flushes_and_restores) {
    if (!kFlushDenormalsSupported) {
        std::printf("  flush-to-zero not supported on this target; skipped\n");
        return;
    }
    const bool subnormalBefore = isSubnormal(tinyProduct());
    {
        ScopedFlushDenormals ftz;
        CHECK(tinyProduct() == 0.0f);
    }
    CHECK(isSubnormal(tinyProduct()) == subnormalBefore);
    std::printf("  subnormal arithmetic outside the guard: %s\n", subnormalBefore ? "yes" : "no");
}

TEST(decay_to_silence_produces_no_subnormals) {
    if (!kFlushDenormalsSupported) {
        std::printf("  flush-to-zero not supported on this target; skipped\n");
        return;
    }
    for (Kernel kernel : {Kernel::Scalar, Kernel::Simd}) {
        ModeBank bank;
        bank.allocate(256);
        loadString(bank, fastDecay(), kFs);
        pluck(bank, 0.12, 1e-3);
        int subnormalOut = 0, subnormalState = 0;
        {
            ScopedFlushDenormals ftz;
            std::vector<float> y(64);
            for (int blk = 0; blk < static_cast<int>(kFs) / 64; ++blk) {
                std::fill(y.begin(), y.end(), 0.0f);
                bank.process(nullptr, y.data(), 64, kernel);
                for (float v : y) subnormalOut += isSubnormal(v);
                subnormalState += countSubnormalStates(bank);
            }
        }
        // Under flush-to-zero the increment (c - 1) z underflows once |z| falls
        // below about FLT_MIN / |c - 1|, so a mode can freeze at a normal value
        // of order 1e-36..1e-33 instead of reaching exactly zero. That costs
        // nothing (no subnormal operands, ~-600 dBFS) and culling removes it.
        double largest = 0.0;
        for (int i = 0; i < bank.size(); ++i)
            largest = std::max({largest, double(std::fabs(bank.stateRe(i))),
                                double(std::fabs(bank.stateIm(i)))});
        std::printf("  %s: subnormal outputs %d, subnormal states %d, largest |state| after 1 s %.2e\n",
                    kernel == Kernel::Simd ? "simd" : "scalar", subnormalOut, subnormalState, largest);
        CHECK(subnormalOut == 0);
        CHECK(subnormalState == 0);
        CHECK_LE(largest, 1e-30);
    }
}

// Without the guard the same decay does pass through subnormals (shows the
// test above exercises the guard rather than passing trivially). Reported,
// not asserted, since some toolchains enable flush-to-zero at startup.
TEST(unguarded_decay_reaches_subnormal_range) {
    ModeBank bank;
    bank.allocate(256);
    loadString(bank, fastDecay(), kFs);
    pluck(bank, 0.12, 1e-3);
    int subnormalState = 0;
    std::vector<float> y(64);
    for (int blk = 0; blk < static_cast<int>(kFs) / 64; ++blk) {
        bank.process(nullptr, y.data(), 64, Kernel::Scalar);
        subnormalState += countSubnormalStates(bank);
    }
    std::printf("  unguarded: subnormal states seen %d\n", subnormalState);
}

TEST(culling_puts_decayed_string_to_sleep) {
    ModeBank bank;
    bank.allocate(256);
    loadString(bank, fastDecay(), kFs);
    bank.setCulling(true, -100.0f);
    pluck(bank, 0.12, 1e-3);
    // No guard here on purpose: culling alone must stop the decay before it
    // reaches the subnormal range.
    std::vector<float> y(64);
    int subnormalState = 0;
    for (int blk = 0; blk < static_cast<int>(kFs) / 64; ++blk) {
        bank.process(nullptr, y.data(), 64, Kernel::Simd);
        subnormalState += countSubnormalStates(bank);
    }
    std::printf("  culled: active groups after 1 s %d, subnormal states %d\n", bank.activeGroups(),
                subnormalState);
    CHECK(bank.activeGroups() == 0);
    CHECK(subnormalState == 0);
}
