// Hammer-string contact (plan §1.4): energy balance, convergence against a
// much smaller time step, contact durations, stability with stiff felt.
#include <cmath>
#include <cstdio>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
const double kVel[3] = {kPP, kMF, kFF};
const char* kVelName[3] = {"pp", "mf", "ff"};

void strikeFor(SingleKey& k, double v, int samples) {
    pm::ScopedFlushDenormals ftz;
    k.strike(v);
    const pm::HammerKeyModes m = k.view();
    float y;
    for (int i = 0; i < samples; ++i) pm::runHammer(k.hs, k.hp, m, &y, 1);
}

}  // namespace

// Hammer kinetic energy lost = string modal energy gained + felt
// dissipation, with the table's own string losses (which take their small
// share during the contact).
TEST(energy_balance_with_losses) {
    for (int key : {1, 16, 40, 64, 76, 88}) {
        for (int iv = 0; iv < 3; ++iv) {
            SingleKey k(tableKey(key), kFs);
            const StrikeResult r = runStrike(k, kVel[iv]);
            const double lost = r.ke0 - r.keEnd;
            const double err = (lost - r.stringEnergy - r.feltLoss) / lost;
            std::printf("  key %2d %s: KE0 %.3e J, lost %5.1f %%, to string %5.1f %%, felt %4.1f %%, "
                        "imbalance %+.3f %% of lost\n",
                        key, kVelName[iv], r.ke0, 100 * lost / r.ke0, 100 * r.stringEnergy / r.ke0,
                        100 * r.feltLoss / r.ke0, 100 * err);
            CHECK(lost > 0.0);
            CHECK_LE(std::fabs(err), 0.01);
        }
    }
}

// Lossless string: the balance is exact up to float rounding of the state.
// With mu = 0 the felt is a pure spring: the collision is lossless.
TEST(energy_balance_lossless) {
    for (int key : {16, 40, 76, 88}) {
        for (double mu : {0.04, 0.0}) {
            for (int iv = 0; iv < 3; ++iv) {
                pm::KeyParams p = tableKey(key);
                p.b1 = 0.0;
                p.b3 = 0.0;
                p.feltMu = mu;
                SingleKey k(p, kFs);
                const StrikeResult r = runStrike(k, kVel[iv]);
                const double lost = r.ke0 - r.keEnd;
                const double err = (lost - r.stringEnergy - r.feltLoss) / r.ke0;
                std::printf("  key %2d mu %.2f %s: to string %5.1f %%, felt %.2e of KE0, "
                            "imbalance %+.2e of KE0\n",
                            key, mu, kVelName[iv], 100 * r.stringEnergy / r.ke0, r.feltLoss / r.ke0, err);
                CHECK_LE(std::fabs(err), 1e-4);
                if (mu == 0.0) CHECK_LE(std::fabs(r.feltLoss), 1e-9 * r.ke0);
            }
        }
    }
}

// The scheme is passive for any stiffness: total energy (hammer + modes +
// felt potential) never rises, even far outside the table's range, with no
// oversampling.
TEST(stiff_felt_is_stable) {
    for (double K : {3e12, 3e13, 3e14}) {
        pm::KeyParams p = tableKey(88);
        p.b1 = p.b3 = 0.0;
        p.feltK = K;
        p.feltMu = 0.0;
        SingleKey k(p, kFs, 1);
        pm::ScopedFlushDenormals ftz;
        k.strike(10.0);
        const pm::HammerKeyModes m = k.view();
        const double e0 = k.hs.kineticEnergy0;
        double worst = 0.0;
        float y;
        bool finite = true;
        for (int i = 0; i < 400; ++i) {
            pm::runHammer(k.hs, k.hp, m, &y, 1);
            finite = finite && std::isfinite(y);
            const double xi = k.hs.x - pm::strikeDisplacement(m);
            const double e = 0.5 * k.hp.mass * k.hs.v * k.hs.v + k.stringEnergy() +
                             pm::contactPotential(k.hp, m, xi);
            worst = std::max(worst, e / e0 - 1.0);
        }
        std::printf("  C8, K = %.0e, 10 m/s, R = 1: contact %.3f ms, peak force %.0f N, "
                    "max energy rise %.2e\n",
                    K, (k.hs.lastContact - k.hs.firstContact + 1) * m.h * 1e3, k.hs.peakForce, worst);
        CHECK(finite);
        CHECK_LE(worst, 1e-4);
    }
}

// Real-time result (table oversampling) against the same model with a 16x
// smaller step: partial amplitudes (first 30, those within 40 dB of the
// strongest) within 1 dB, contact duration within 3 %.
TEST(convergence_against_16x_smaller_step) {
    const int samples = 2400;  // 50 ms: every hammer has left
    for (int key : {1, 16, 28, 40, 52, 64, 70, 76, 82, 88}) {
        for (int iv = 0; iv < 3; ++iv) {
            const pm::KeyParams p = tableKey(key);
            SingleKey a(p, kFs);
            SingleKey b(p, kFs, 16 * p.contactOversampling);
            strikeFor(a, kVel[iv], samples);
            strikeFor(b, kVel[iv], samples);
            const int np = std::min(30, a.modes.count);
            double amax = 0.0;
            for (int i = 0; i < np; ++i) amax = std::max(amax, b.modeAmplitude(i));
            double worst = 0.0;
            int worstN = 0, used = 0;
            for (int i = 0; i < np; ++i) {
                const double B = b.modeAmplitude(i);
                if (B < amax * 1e-2) continue;
                ++used;
                const double d = 20.0 * std::log10(a.modeAmplitude(i) / B);
                if (std::fabs(d) > std::fabs(worst)) {
                    worst = d;
                    worstN = i + 1;
                }
            }
            const double ta = (a.hs.lastContact - a.hs.firstContact + 1) * a.view().h;
            const double tb = (b.hs.lastContact - b.hs.firstContact + 1) * b.view().h;
            std::printf("  key %2d %s R=%d: %2d partials compared, worst %+.2f dB (n=%d), "
                        "contact %.3f vs %.3f ms (%+.1f %%)\n",
                        key, kVelName[iv], p.contactOversampling, used, worst, worstN, ta * 1e3,
                        tb * 1e3, 100 * (ta / tb - 1));
            CHECK_LE(std::fabs(worst), 1.0);
            CHECK_LE(std::fabs(ta / tb - 1.0), 0.03);
        }
    }
}

// Informational: why the table oversamples the contact above C4. With R = 1
// everywhere the error exceeds 1 dB in the upper treble.
TEST(convergence_without_oversampling_info) {
    for (int key : {40, 52, 64, 70, 76, 82, 88}) {
        double worstAll = 0.0;
        for (int iv = 0; iv < 3; ++iv) {
            const pm::KeyParams p = tableKey(key);
            SingleKey a(p, kFs, 1), b(p, kFs, 16);
            strikeFor(a, kVel[iv], 2400);
            strikeFor(b, kVel[iv], 2400);
            const int np = std::min(30, a.modes.count);
            double amax = 0.0;
            for (int i = 0; i < np; ++i) amax = std::max(amax, b.modeAmplitude(i));
            for (int i = 0; i < np; ++i) {
                const double B = b.modeAmplitude(i);
                if (B < amax * 1e-2) continue;
                const double d = 20.0 * std::log10(a.modeAmplitude(i) / B);
                if (std::fabs(d) > std::fabs(worstAll)) worstAll = d;
            }
        }
        std::printf("  key %2d, R = 1: worst partial error over pp/mf/ff %+.2f dB\n", key, worstAll);
    }
}

// Contact durations in physically sensible ranges; multiple contacts in the
// bass.
TEST(contact_durations_plausible) {
    struct Range {
        int key;
        double lo, hi;  // ms, at mf
    };
    for (const Range& r : {Range{1, 2.0, 8.0}, Range{16, 2.0, 6.0}, Range{40, 1.0, 3.0},
                           Range{64, 0.6, 1.6}, Range{76, 0.3, 1.0}, Range{88, 0.3, 1.0}}) {
        double ms[3];
        int contacts[3];
        for (int iv = 0; iv < 3; ++iv) {
            SingleKey k(tableKey(r.key), kFs);
            const StrikeResult s = runStrike(k, kVel[iv]);
            ms[iv] = s.contactSpan * 1e3;
            contacts[iv] = s.contacts;
        }
        std::printf("  key %2d: contact pp %.2f ms (%d), mf %.2f ms (%d), ff %.2f ms (%d)\n", r.key,
                    ms[0], contacts[0], ms[1], contacts[1], ms[2], contacts[2]);
        CHECK(ms[1] >= r.lo && ms[1] <= r.hi);
        CHECK(ms[2] <= ms[1] * 1.05 || contacts[2] > 1);  // harder strike, shorter contact
        if (r.key >= 76) {
            CHECK_LE(ms[1], 1.0);
            CHECK_LE(ms[2], 1.0);
            CHECK_LE(ms[0], 1.5);
        }
        if (r.key == 1) CHECK(contacts[1] >= 2 && contacts[2] >= 2);
    }
}
