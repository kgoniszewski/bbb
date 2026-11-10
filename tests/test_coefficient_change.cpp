// Changing pitch and decay while a string rings must not click (plan §1.5).
#include <cmath>
#include <cstdio>
#include <vector>

#include "signal_utils.h"
#include "test_harness.h"

using namespace pm;

namespace {

constexpr double kFs = 48000.0;

double maxStep(const std::vector<float>& y, int from, int to) {
    double m = 0.0;
    for (int k = from; k + 1 < to; ++k) m = std::max(m, double(std::fabs(y[k + 1] - y[k])));
    return m;
}

void runChange(const char* label, const StringParams& before, const StringParams& after) {
    for (Kernel kernel : {Kernel::Scalar, Kernel::Simd}) {
        ModeBank bank;
        bank.allocate(256);
        loadString(bank, before, kFs);
        pluck(bank, 0.12, 1e-3);

        // Change at a sample that is not a block or cull boundary.
        const int changeAt = static_cast<int>(0.5 * kFs) + 23;
        const int total = changeAt + static_cast<int>(0.1 * kFs);
        std::vector<float> y(static_cast<std::size_t>(total), 0.0f);
        {
            ScopedFlushDenormals ftz;
            bank.process(nullptr, y.data(), changeAt, kernel);
            retuneString(bank, after, kFs);
            bank.process(nullptr, y.data() + changeAt, total - changeAt, kernel);
        }
        const int win = static_cast<int>(0.02 * kFs);
        const double ref = maxStep(y, changeAt - win, changeAt);
        const double at = maxStep(y, changeAt - 2, changeAt + 3);
        const double afterMax = maxStep(y, changeAt, changeAt + win);
        double level = 0.0;
        for (int k = changeAt - win; k < changeAt; ++k) level = std::max(level, double(std::fabs(y[k])));
        std::printf("  %s (%s): level %.3f, max step before %.4f, at change %.4f, after %.4f\n",
                    label, kernel == Kernel::Simd ? "simd" : "scalar", level, ref, at, afterMax);
        CHECK(ref > 0.0);
        // The step across the change stays within the signal's own
        // sample-to-sample motion just before it (a click would be a step of
        // the order of the signal level).
        CHECK_LE(at, 1.25 * ref);
        CHECK_LE(afterMax, 1.25 * ref);
        CHECK_LE(ref, 0.5 * level);
    }
}

}  // namespace

TEST(retune_mid_note_is_click_free) {
    StringParams a;
    StringParams b = a;
    b.f0 *= std::pow(2.0, 30.0 / 1200.0);  // +30 cents
    b.B *= 1.2;
    runChange("+30 cents, B x1.2", a, b);
}

TEST(damping_change_mid_note_is_click_free) {
    StringParams a;
    StringParams b = a;
    b.b1 = 40.0;  // damper landing: T60 drops to ~0.17 s
    b.b3 *= 4.0;
    runChange("b1 0.5 -> 40, b3 x4", a, b);
}
