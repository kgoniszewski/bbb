// Energy-based group culling (plan §1.5).
#include <cmath>
#include <cstdio>
#include <vector>

#include "signal_utils.h"
#include "test_harness.h"

using namespace pm;
using pm_test::render;

namespace {

constexpr double kFs = 48000.0;

StringParams bassish() {
    // Many groups (120 modes), moderate decay.
    StringParams p;
    p.f0 = 65.41;
    p.B = 7.4e-5;
    p.b1 = 1.5;
    p.b3 = 2e-8;
    p.length = 1.9;
    p.linearDensity = 0.035;
    return p;
}

}  // namespace

TEST(culled_output_matches_unculled_below_minus_90_db) {
    for (const StringParams& p : {StringParams{}, bassish()}) {
        ModeBank a, b;
        a.allocate(256);
        b.allocate(256);
        loadString(a, p, kFs);
        loadString(b, p, kFs);
        b.setCulling(true, -100.0f);
        pluck(a, 0.12, 1e-3);
        pluck(b, 0.12, 1e-3);
        const int n = static_cast<int>(6 * kFs);
        const auto ya = render(a, n, Kernel::Simd);
        const auto yb = render(b, n, Kernel::Simd);
        double peak = 0.0, diff = 0.0;
        for (int k = 0; k < n; ++k) {
            peak = std::max(peak, double(std::fabs(ya[k])));
            diff = std::max(diff, double(std::fabs(ya[k] - yb[k])));
        }
        const double diffDb = 20.0 * std::log10(std::max(diff, 1e-30));
        std::printf("  f0=%.2f: %d groups, peak %.3f, max |culled - unculled| %.2e (%.1f dBFS, "
                    "%.1f dB re peak), active groups at end %d\n",
                    p.f0, b.groupCount(), peak, diff, diffDb,
                    diffDb - 20.0 * std::log10(peak), b.activeGroups());
        CHECK(peak > 0.1);
        CHECK_LE(diffDb, -90.0);
        CHECK(b.groupBlocksProcessed() < a.groupBlocksProcessed());
    }
}

TEST(silent_bank_does_no_work) {
    ModeBank bank;
    bank.allocate(256);
    StringParams p = bassish();
    p.b1 = 8.0;  // decays to -100 dB within about 1.5 s
    loadString(bank, p, kFs);
    bank.setCulling(true, -100.0f);

    // Freshly loaded, nothing excited: no group is awake.
    CHECK(bank.activeGroups() == 0);
    render(bank, 4800, Kernel::Simd);
    CHECK(bank.groupBlocksProcessed() == 0);

    pluck(bank, 0.12, 1e-3);
    CHECK(bank.activeGroups() == bank.groupCount());
    render(bank, static_cast<int>(3 * kFs), Kernel::Simd);
    std::printf("  after 3 s decay: %d of %d groups awake\n", bank.activeGroups(),
                bank.groupCount());
    CHECK(bank.activeGroups() == 0);

    const auto before = bank.groupBlocksProcessed();
    const auto y = render(bank, static_cast<int>(kFs), Kernel::Simd);
    CHECK(bank.groupBlocksProcessed() == before);
    bool allZero = true;
    for (float v : y) allZero = allZero && v == 0.0f;
    CHECK(allZero);
}

TEST(input_wakes_sleeping_groups) {
    ModeBank bank;
    bank.allocate(256);
    loadString(bank, StringParams{}, kFs);
    bank.setCulling(true, -100.0f);
    CHECK(bank.activeGroups() == 0);

    // A force far too small to reach the threshold leaves the bank asleep.
    std::vector<float> u(256, 0.0f);
    u[10] = 1e-9f;
    render(bank, 256, Kernel::Simd, &u);
    CHECK(bank.activeGroups() == 0);
    CHECK(bank.groupBlocksProcessed() == 0);

    // A real hammer-sized force wakes every group in the same sub-block, so
    // the attack is not delayed.
    u[10] = 20.0f;
    const auto y = render(bank, 256, Kernel::Simd, &u);
    std::printf("  after 20 N impulse: %d of %d groups awake\n", bank.activeGroups(),
                bank.groupCount());
    CHECK(bank.activeGroups() == bank.groupCount());
    CHECK(std::fabs(y[12]) > 0.0f);
    for (int k = 0; k <= 10; ++k) CHECK(y[k] == 0.0f);

    // Waking happens before the sub-block is processed: identical to a bank
    // that never culled.
    ModeBank ref;
    ref.allocate(256);
    loadString(ref, StringParams{}, kFs);
    const auto yr = render(ref, 256, Kernel::Simd, &u);
    double d = 0.0;
    for (int k = 0; k < 256; ++k) d = std::max(d, double(std::fabs(y[k] - yr[k])));
    CHECK(d == 0.0);
}

TEST(disabling_culling_wakes_everything) {
    ModeBank bank;
    bank.allocate(256);
    loadString(bank, StringParams{}, kFs);
    bank.setCulling(true, -100.0f);
    CHECK(bank.activeGroups() == 0);
    bank.setCulling(false);
    CHECK(bank.activeGroups() == bank.groupCount());
}
