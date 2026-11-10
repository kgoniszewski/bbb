// Hammer-string contact (plan §1.4) with one hammer on the S strings of a
// coupled unison: energy balance, convergence against a much smaller time
// step, contact durations, stability with stiff felt, una corda contact,
// accuracy of the fast power function.
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "pm_base/fast_math.h"
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

// A purely reactive (lossless) bridge: the strings exchange energy through
// it but none leaves.
class ReactiveBridge final : public pm::BridgeAdmittance {
public:
    pm::Admittance2 at(int key, double f) const noexcept override {
        const pm::Admittance2 y = pm::defaultBridge().at(key, f);
        const double x = std::abs(y.vv);
        return {{0.0, 0.5 * x}, {0.0, 0.1 * x}, {0.0, 0.08 * x}};
    }
};

pm::KeyParams lossless(int key) {
    pm::KeyParams p = tableKey(key);
    p.b1 = 0.0;
    p.b3 = 0.0;
    return p;
}

// Hammer + strings + felt potentials.
double totalEnergy(SingleKey& k) {
    const pm::HammerKeyModes m = k.view();
    double e = 0.5 * k.hp.mass * k.hs.v * k.hs.v + k.stringEnergy();
    for (int s = 0; s < m.strings; ++s)
        e += pm::contactPotential(k.hp, m, k.hs.x - k.hp.offset[s] - pm::strikeDisplacement(m, s));
    return e;
}

}  // namespace

// Hammer kinetic energy lost = string energy gained + energy the strings
// lost meanwhile (internal and bridge losses, integrated from the state) +
// felt dissipation, with the table's coupled unisons and mating offsets.
TEST(energy_balance_with_losses) {
    for (int key : {1, 16, 40, 64, 76, 88}) {
        for (int iv = 0; iv < 3; ++iv) {
            SingleKey k(tableKey(key), kFs);
            const StrikeResult r = runStrike(k, kVel[iv]);
            const double lost = r.ke0 - r.keEnd;
            const double err = (lost - r.stringEnergy - r.stringLoss - r.feltLoss) / lost;
            std::printf("  key %2d (S=%d) %s: KE0 %.3e J, lost %5.1f %%, to strings %5.1f %% "
                        "(+%.2f %% already dissipated), felt %4.1f %%, imbalance %+.3f %% of lost, "
                        "fallback solves %d\n",
                        key, k.modes.strings, kVelName[iv], r.ke0, 100 * lost / r.ke0,
                        100 * r.stringEnergy / r.ke0, 100 * r.stringLoss / r.ke0,
                        100 * r.feltLoss / r.ke0, 100 * err, k.hs.fallbackSolves);
            CHECK(lost > 0.0);
            // 1 % of the energy lost by the hammer, or 0.5 % of its initial
            // energy where very little reaches the strings (treble): the
            // energy of a lossy oscillator held deflected by the felt is only
            // defined to O(sigma / omega), and the bridge makes sigma up to
            // ~20 1/s in the treble. The exact checks are the lossless and
            // reactive-bridge tests below.
            CHECK(std::fabs(err) <= 0.01 || std::fabs(err * lost) <= 0.005 * r.ke0);
        }
    }
}

// Lossless, uncoupled strings (rigid bridge): the balance is exact up to
// float rounding of the state, with 1, 2 and 3 strings and mating offsets.
// With mu = 0 the felt is a pure spring: the collision is lossless.
TEST(energy_balance_lossless) {
    for (int key : {1, 16, 40, 76, 88}) {
        for (double mu : {0.04, 0.0}) {
            for (int iv = 0; iv < 3; ++iv) {
                pm::KeyParams p = lossless(key);
                p.bridgeCoupling = false;
                p.feltMu = mu;
                SingleKey k(p, kFs);
                const StrikeResult r = runStrike(k, kVel[iv]);
                const double lost = r.ke0 - r.keEnd;
                const double err = (lost - r.stringEnergy - r.feltLoss) / r.ke0;
                std::printf("  key %2d (S=%d) mu %.2f %s: to strings %5.1f %%, felt %.2e of KE0, "
                            "imbalance %+.2e of KE0\n",
                            key, k.modes.strings, mu, kVelName[iv], 100 * r.stringEnergy / r.ke0,
                            r.feltLoss / r.ke0, err);
                CHECK_LE(std::fabs(err), 1e-4);
                if (mu == 0.0) CHECK_LE(std::fabs(r.feltLoss), 1e-9 * r.ke0);
            }
        }
    }
}

// Lossless strings on a purely reactive bridge: the normal modes are
// coupled but conserve energy, so the balance still holds (to 1e-3: the
// oscillator energies weight |z|^2 by omega_k^2, which differ by the
// detuning, while the reactive coupling conserves sum |z|^2).
TEST(energy_balance_reactive_bridge) {
    const ReactiveBridge bridge;
    for (int key : {16, 40, 64}) {
        for (int iv = 0; iv < 3; ++iv) {
            pm::KeyParams p = lossless(key);
            p.feltMu = 0.0;
            SingleKey k(p, kFs, 0, &bridge);
            const StrikeResult r = runStrike(k, kVel[iv]);
            const double lost = r.ke0 - r.keEnd;
            const double err = (lost - r.stringEnergy - r.feltLoss) / r.ke0;
            std::printf("  key %2d %s, reactive bridge: to strings %5.1f %%, imbalance %+.2e of KE0\n",
                        key, kVelName[iv], 100 * r.stringEnergy / r.ke0, err);
            CHECK_LE(std::fabs(err), 1e-3);
        }
    }
}

// The scheme is passive for any stiffness: total energy (hammer + strings +
// felt potentials) never rises, even far outside the table's range, with no
// oversampling, three strings and mating offsets.
TEST(stiff_felt_is_stable) {
    for (double K : {3e12, 3e13, 3e14}) {
        pm::KeyParams p = lossless(88);
        p.bridgeCoupling = false;
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
            worst = std::max(worst, totalEnergy(k) / e0 - 1.0);
        }
        std::printf("  C8, K = %.0e, 10 m/s, R = 1: contact %.3f ms, peak force %.0f N, "
                    "max energy rise %.2e, fallback solves %d\n",
                    K, (k.hs.lastContact - k.hs.firstContact + 1) * m.h * 1e3, k.hs.peakForce, worst,
                    k.hs.fallbackSolves);
        CHECK(finite);
        CHECK_LE(worst, 1e-4);
    }
}

// Real-time result (table oversampling) against the same model with a 16x
// smaller step: partial energies (first 30 partials, all strings and
// polarizations) within 1 dB for those within 30 dB of the strongest and
// 1.5 dB for those 30-40 dB down (weak partials sit near nulls of the
// contact spectrum, where a 1 % change of contact time moves the null),
// contact duration within 3 %.
TEST(convergence_against_16x_smaller_step) {
    const int samples = 2400;  // 50 ms: every hammer has left
    for (int key : {1, 16, 28, 40, 52, 64, 70, 76, 82, 88}) {
        for (int iv = 0; iv < 3; ++iv) {
            const pm::KeyParams p = tableKey(key);
            SingleKey a(p, kFs);
            SingleKey b(p, kFs, 16 * p.contactOversampling);
            strikeFor(a, kVel[iv], samples);
            strikeFor(b, kVel[iv], samples);
            const int np = std::min<int>(30, static_cast<int>(a.modes.partials.size()));
            double amax = 0.0;
            for (int i = 0; i < np; ++i) amax = std::max(amax, b.partialAmplitude(i));
            double worst = 0.0, excess = 0.0;
            int worstN = 0, used = 0;
            for (int i = 0; i < np; ++i) {
                const double B = b.partialAmplitude(i);
                if (B < amax * 1e-2) continue;
                ++used;
                const double d = 20.0 * std::log10(a.partialAmplitude(i) / B);
                excess = std::max(excess, std::fabs(d) - (B < amax * std::pow(10.0, -30.0 / 20.0) ? 1.5 : 1.0));
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
            CHECK_LE(excess, 0.0);
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
            const int np = std::min<int>(30, static_cast<int>(a.modes.partials.size()));
            double amax = 0.0;
            for (int i = 0; i < np; ++i) amax = std::max(amax, b.partialAmplitude(i));
            for (int i = 0; i < np; ++i) {
                const double B = b.partialAmplitude(i);
                if (B < amax * 1e-2) continue;
                const double d = 20.0 * std::log10(a.partialAmplitude(i) / B);
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
        // A second contact at A0 is a threshold effect: at mf it occurs with
        // equal temperament but not with the stretched (24 cents lower,
        // heavier) A0 string; at ff it always does.
        if (r.key == 1) CHECK(contacts[2] >= 2);
    }
}

// One hammer, three strings: each string receives energy according to its
// mating offset; at full una corda the shifted string is never touched (it
// only picks up energy through the bridge, orders of magnitude less), and
// the hammer still conserves energy.
TEST(una_corda_misses_one_string) {
    // With the smooth placeholder admittance: the soundboard's reactance
    // passes more energy between the strings of a unison through the bridge
    // within the observation time, which is physical but not what this
    // checks.
    const pm::PlaceholderBridgeAdmittance smooth;
    for (double u : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        pm::KeyParams p = tableKey(40);
        SingleKey k(p, kFs, 0, &smooth);
        k.hp = pm::unaCordaHammerParams(p, u);
        const StrikeResult r = runStrike(k, kMF);
        // Vertical energy of each string over the first 10 partials.
        double es[3] = {};
        for (int pi = 0; pi < 10; ++pi)
            for (int s = 0; s < 3; ++s) es[s] += k.partialEnergy(pi, s);
        const double lost = r.ke0 - r.keEnd;
        const double err = (lost - r.stringEnergy - r.stringLoss - r.feltLoss) / lost;
        std::printf("  C4 una corda %.1f: string energies %.3e %.3e %.3e J (string 0 at %+.1f dB "
                    "re string 1), imbalance %+.3f %%\n",
                    u, es[0], es[1], es[2], 10 * std::log10(es[0] / es[1]), 100 * err);
        CHECK(std::fabs(err) <= 0.01 || std::fabs(err * lost) <= 0.005 * r.ke0);
        if (u == 0.0) CHECK(es[0] > 0.5 * es[1] && es[0] < 2.0 * es[1]);
        if (u == 1.0) CHECK(es[0] < 1e-4 * es[1]);
    }
}

// The fast power function used by the felt law.
TEST(fast_pow_accuracy) {
    double worst = 0.0;
    for (int i = 0; i < 200000; ++i) {
        const double x = std::exp(-30.0 + 32.0 * i / 200000.0);
        const double p = 2.0 + 1.6 * (i % 101) / 101.0;
        worst = std::max(worst, std::fabs(pm::fastPow(x, p) / std::pow(x, p) - 1.0));
    }
    std::printf("  fastPow: worst relative error %.2e over x in [1e-13, 7], p in [2, 3.6]\n", worst);
    CHECK_LE(worst, 1e-13);
}
