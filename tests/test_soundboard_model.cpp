// Soundboard mode set v0 (plan §1.6): targets of the default plate, the
// discrete loop's passivity per mode, and the admittance interface against a
// direct time-domain drive of the run-time modal bank.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_board/soundboard.h"
#include "pm_voicing/soundboard_model.h"
#include "test_harness.h"

namespace {

constexpr double kFs = 48000.0;
constexpr double kPi = std::numbers::pi;
using cd = std::complex<double>;

// Mean over a log-spaced grid of Re Y at a key's driving point.
double meanRe(const pm::BoardModel& b, int key, double f0, double f1) {
    double s = 0.0;
    int n = 0;
    for (double f = f0; f < f1; f *= 1.002, ++n) s += pm::boardDrivingPoint(b, key, f).real();
    return s / n;
}

}  // namespace

// First mode 50-60 Hz, modal density approaching one per 20-30 Hz above
// 1 kHz, loss factors 1-3.5 % above 1 kHz and up to about 10 % for the lowest modes
// (soundboard_model.h: rim and case participation), 300-600 modes is the plan's range but the
// density target with the modal range 5 kHz implies about 200 (see the
// README); driving-point impedance of order 1e3 kg/s in the mid range.
TEST(default_board_meets_targets) {
    const pm::BoardModel& b = pm::defaultBoard();
    int n13 = 0, n35 = 0;
    for (double f : b.freq) {
        n13 += f >= 1000.0 && f < 3000.0;
        n35 += f >= 3000.0 && f < 5000.0;
    }
    double etaMin = 1.0, etaMax = 0.0, etaMid = 0.0;
    for (int i = 0; i < b.modes; ++i) {
        const double eta = b.sigma[i] / (kPi * b.freq[i]);
        etaMin = std::min(etaMin, eta);
        etaMax = std::max(etaMax, eta);
        if (b.freq[i] >= 1000.0) etaMid = std::max(etaMid, eta);
    }
    std::printf("  %d modes below %.0f Hz, first %.1f %.1f %.1f Hz; 1-3 kHz: one per %.1f Hz, 3-5 kHz: one per "
                "%.1f Hz; asymptotic one per %.1f Hz; loss factor %.3f-%.3f (above 1 kHz at most %.3f); mass %.1f kg\n",
                b.modes, b.params.maxFreqHz, b.freq[0], b.freq[1], b.freq[2], 2000.0 / n13, 2000.0 / n35,
                1.0 / b.density, etaMin, etaMax, etaMid, b.mass);
    CHECK(b.freq[0] >= 50.0 && b.freq[0] <= 60.0);
    CHECK(2000.0 / n13 >= 18.0 && 2000.0 / n13 <= 32.0);
    CHECK(2000.0 / n35 >= 18.0 && 2000.0 / n35 <= 32.0);
    CHECK(etaMin >= 0.01 && etaMid <= 0.035 && etaMax <= 0.11);
    CHECK(b.modes >= 150 && b.modes <= 600);
    for (int key : {16, 28, 40, 52, 64, 76}) {
        const double zMid = 1.0 / meanRe(b, key, 200.0, 2000.0);
        const double zHigh = 1.0 / meanRe(b, key, 2000.0, 8000.0);
        std::printf("  key %2d: mean conductance -> impedance %5.0f kg/s (0.2-2 kHz), %5.0f kg/s (2-8 kHz)\n", key,
                    zMid, zHigh);
        CHECK(zMid >= 500.0 && zMid <= 4000.0);
        CHECK(zHigh >= 500.0 && zHigh <= 4000.0);
    }
    // Above the modal range the continuation takes over at the mean value.
    const double cont = b.continuation(8000.0).real() / (b.meanConductance * b.bridgeFactor(8000.0));
    std::printf("  continuation at 8 kHz: %.2f x the mean-value conductance\n", cont);
    CHECK(cont > 0.8 && cont < 1.2);
}

// Every board mode inside the one-sample loop acts on the strings as a
// positive-real admittance (Re >= 0 at every frequency), so the zone-port
// loop is passive; and the driving point at every key has Re Y >= 0.
TEST(loop_is_positive_real) {
    const pm::BoardModel& b = pm::defaultBoard();
    double worst = 0.0;
    for (int i = 0; i < b.modes; i += 3)
        for (double f = 20.0; f < 0.49 * kFs; f *= 1.01) {
            const cd y = pm::loopModeAdmittance(b.freq[i], b.sigma[i], f, kFs);
            const cd peak = pm::loopModeAdmittance(b.freq[i], b.sigma[i], b.freq[i], kFs);
            worst = std::min(worst, y.real() / std::abs(peak));
        }
    double worstDp = 0.0;
    for (int key = 1; key <= 88; key += 5)
        for (double f = 20.0; f < 20000.0; f *= 1.01) worstDp = std::min(worstDp, pm::boardDrivingPoint(b, key, f).real());
    std::printf("  worst Re of a mode's loop admittance: %.2e of its peak; worst driving-point Re Y %.2e s/kg\n",
                worst, worstDp);
    CHECK(worst >= -1e-9);
    CHECK(worstDp >= 0.0);
}

// The run-time bank driven in the time domain by a unit impulse at a zone
// reproduces the model's (continuous) transfer admittance from that zone to
// another and to a pickup: within 3 % in magnitude below 2 kHz.
TEST(admittance_matches_time_domain_drive) {
    pm::ScopedFlushDenormals ftz;
    const pm::BoardModel& b = pm::defaultBoard();
    pm::Soundboard sb;
    CHECK(pm::loadSoundboard(sb, b, kFs));
    const int n = 1 << 16;  // 1.37 s: the slowest mode has decayed by 60 dB
    std::vector<double> disp(n), vel(n);
    std::vector<float> F(static_cast<std::size_t>(b.zones), 0.0f), d(static_cast<std::size_t>(b.zones));
    float pk[pm::Soundboard::kMaxPickups];
    const int zin = 6, zout = 9;
    for (int i = 0; i < n; ++i) {
        F[zin] = i == 0 ? 1.0f : 0.0f;
        sb.step(F.data(), d.data(), pk);
        disp[i] = d[zout];
        vel[i] = pk[0];
    }
    // Response to one sample of 1 N (an impulse of 1/fs N s): its DFT, a sum
    // over samples (fs times the continuous Fourier transform), is the
    // continuous transfer function. Magnitudes are compared, relative to the
    // local magnitude plus 5 % of the mean (deep anti-resonances excluded).
    std::vector<double> fr, magD, magV, modD, modV;
    for (double f = 80.0; f < 2000.0; f *= 1.03) {
        cd D = 0.0, V = 0.0;
        for (int i = 0; i < n; ++i) {
            const cd e = std::polar(1.0, -2.0 * kPi * f * i / kFs);
            D += disp[i] * e;
            V += vel[i] * e;
        }
        const double* sa = b.shapeOfZone(zin);
        fr.push_back(f);
        magD.push_back(std::abs(D));
        magV.push_back(std::abs(V));
        modD.push_back(std::abs(pm::boardModalAdmittance(b, sa, b.shapeOfZone(zout), f)) / (2.0 * kPi * f));
        modV.push_back(std::abs(pm::boardModalAdmittance(b, sa, b.pickupShape.data(), f)));
    }
    double meanD = 0.0, meanV = 0.0;
    for (std::size_t i = 0; i < fr.size(); ++i) {
        meanD += modD[i] / fr.size();
        meanV += modV[i] / fr.size();
    }
    double worstD = 0.0, worstV = 0.0;
    for (std::size_t i = 0; i < fr.size(); ++i) {
        worstD = std::max(worstD, std::fabs(magD[i] - modD[i]) / (modD[i] + 0.05 * meanD));
        worstV = std::max(worstV, std::fabs(magV[i] - modV[i]) / (modV[i] + 0.05 * meanV));
    }
    std::printf("  time-domain drive vs admittance model, 80 Hz - 2 kHz: zone displacement %.2f %%, pickup "
                "velocity %.2f %%\n",
                100 * worstD, 100 * worstV);
    CHECK_LE(worstD, 0.03);
    CHECK_LE(worstV, 0.03);
}

// The bake removes exactly the loop's share: driving point = baked + loop
// (where the bake is not clamped), and the loop share at a zone centre is
// close to the true driving point (interpolated shapes elsewhere).
TEST(bake_plus_loop_is_driving_point) {
    const pm::BoardModel& b = pm::defaultBoard();
    const pm::SoundboardBridge loop(b, kFs, 1.0);
    double worst = 0.0;
    int clamped = 0, total = 0;
    for (int key = 1; key <= 88; key += 3)
        for (double f = 60.0; f < 4800.0; f *= 1.01) {
            const cd y = pm::boardDrivingPoint(b, key, f);
            const cd l = pm::loopSelfAdmittance(b, key, f, kFs);
            const cd baked = loop.baked(key, f);
            ++total;
            if ((y - l).real() < 0.0) {
                ++clamped;
                continue;
            }
            worst = std::max(worst, std::abs(baked + l - y) / std::abs(y));
        }
    std::printf("  baked + loop vs driving point: worst relative error %.1e; clamped at %.1f %% of points\n", worst,
                100.0 * clamped / total);
    CHECK_LE(worst, 1e-9);
    CHECK_LE(double(clamped) / total, 0.05);
}

// Bass driving-point impedance in a plausible range (soundboard_model.h,
// board revision): published bridge impedances of grands (Wogram 1980;
// Giordano 1998 for an upright; Conklin 1996, part II) are of order
// 1e3 kg/s from 50 to 500 Hz with resonance dips of a few hundred kg/s,
// i.e. conductance peaks within about 10 dB of the local mean. The uniform
// plate with a 3 % loss factor dipped to 50-70 kg/s at its lowest modes
// (peaks 18-25 times the mean), which made bass fundamentals there hybridise
// into string-board doublets. Targets over keys 1-52, 40-500 Hz: median
// |Z| 700-2000 kg/s, every key's minimum |Z| above 120 kg/s and its largest
// Re Y within 9 times its mean (19 dB).
TEST(bass_driving_point_not_peaky) {
    const pm::BoardModel& b = pm::defaultBoard();
    std::vector<double> all, mins;
    double worstPeak = 0.0;
    for (int key = 1; key <= 52; ++key) {
        double mn = 1e30, sum = 0.0, peak = 0.0;
        int n = 0;
        for (double f = 40.0; f <= 500.0; f *= 1.003, ++n) {
            const cd y = pm::boardDrivingPoint(b, key, f);
            const double z = 1.0 / std::abs(y);
            mn = std::min(mn, z);
            all.push_back(z);
            sum += y.real();
            peak = std::max(peak, y.real());
        }
        mins.push_back(mn);
        worstPeak = std::max(worstPeak, peak / (sum / n));
    }
    std::sort(all.begin(), all.end());
    std::sort(mins.begin(), mins.end());
    const double median = all[all.size() / 2];
    std::printf("  keys 1-52, 40-500 Hz: |Z| median %.0f kg/s (5 %% %.0f, 95 %% %.0f); per-key minimum |Z|: median "
                "%.0f, lowest %.0f kg/s; largest Re Y / mean Re Y %.1f\n",
                median, all[all.size() / 20], all[all.size() * 19 / 20], mins[mins.size() / 2], mins.front(),
                worstPeak);
    CHECK(median >= 700.0 && median <= 2000.0);
    CHECK(mins.front() >= 120.0);
    CHECK(worstPeak <= 9.0);
}
