// Stereo microphone radiation v0 (core/pm_voicing/radiation_model.h).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "piano_utils.h"
#include "pm_board/soundboard.h"
#include "pm_voicing/radiation_model.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
constexpr double kPi = std::numbers::pi;

int findMode(const pm::BoardModel& b, int m, int n) {
    for (int i = 0; i < b.modes; ++i)
        if (!b.lowRegime[i] && b.mIndex[i] == m && b.nIndex[i] == n) return i;
    return -1;
}

}  // namespace

// A low mode (k L << 1) with zero-mean shape is a dipole and radiates far
// less than one with non-zero mean (a monopole); the monopole's gain at a far
// point on the axis is the point-source value (j w rho / 2 pi) Q / R,
// Q = Int phi dA. Both modes are moved to 20 Hz (k L = 0.7).
TEST(low_modes_monopole_and_dipole) {
    pm::BoardModel b = pm::defaultBoard();
    const int i11 = findMode(b, 1, 1), i21 = findMode(b, 2, 1);
    CHECK(i11 >= 0 && i21 >= 0);
    if (i11 < 0 || i21 < 0) return;
    b.freq[i11] = b.freq[i21] = 20.0;
    pm::RadiationParams r;
    r.lid = false;
    const double L = b.params.lengthX, W = b.params.widthY;
    // Far off-axis point: the dipole does not cancel by symmetry.
    r.micX[0] = 0.5 * L + 6.0;
    r.micY[0] = 0.5 * W + 3.0;
    r.micZ[0] = 8.0;
    // Far on-axis point.
    const double R = 20.0;
    r.micX[1] = 0.5 * L;
    r.micY[1] = 0.5 * W;
    r.micZ[1] = R;
    const double mono = std::abs(pm::micModeGain(b, r, 0, i11));
    const double di = std::abs(pm::micModeGain(b, r, 0, i21));
    std::printf("  20 Hz, 10.4 m off axis: |G(2,1)| / |G(1,1)| = %.4f\n", di / mono);
    CHECK_LE(di / mono, 0.15);
    const double w = 2.0 * kPi * 20.0;
    const double wd = std::sqrt(w * w - b.sigma[i11] * b.sigma[i11]);
    const double q = pm::boardModeShape(b, i11, 0.5 * L, 0.5 * W) * 4.0 * L * W / (kPi * kPi);
    const double analytic = wd * r.rho / (2.0 * kPi) * q / R;
    const double num = std::abs(pm::micModeGain(b, r, 1, i11));
    std::printf("  monopole on axis at 20 m: numeric / analytic = %.5f\n", num / analytic);
    // 1/r and phase variation over the board at 20 m: < 0.1 %; midpoint
    // rule with 16 patches per half wave: 0.3 %.
    CHECK_NEAR(num / analytic, 1.0, 0.01);
}

// Halving the patch size changes the gains of the highest modes little.
TEST(grid_convergence) {
    const pm::BoardModel& b = pm::defaultBoard();
    pm::RadiationParams r1, r2;
    r2.gridScale = 2.0;
    std::vector<double> rel;
    for (int i = b.modes - 24; i < b.modes; ++i)
        for (int mic = 0; mic < 2; ++mic) {
            const std::complex<double> g1 = pm::micModeGain(b, r1, mic, i), g2 = pm::micModeGain(b, r2, mic, i);
            rel.push_back(std::abs(std::abs(g1) - std::abs(g2)) / std::abs(g2));
        }
    std::sort(rel.begin(), rel.end());
    const double med = rel[rel.size() / 2], mx = rel.back();
    std::printf("  top 24 modes (%.0f-%.0f Hz): |G| change on halving: median %.4f, max %.4f\n",
                b.freq[b.modes - 24], b.freq[b.modes - 1], med, mx);
    CHECK_LE(med, 0.02);
    CHECK_LE(mx, 0.10);
    const auto t0 = std::chrono::steady_clock::now();
    const pm::MicGains g = pm::computeMicGains(b, r1);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("  computeMicGains: %d modes, %.0f ms, scale %.3g\n", b.modes, ms, g.scale);
    // Level relative to the v0 pickups by band (geometric mean of |G| / |phi|).
    const double edges[] = {50, 100, 200, 500, 1000, 2000, 3500, 5000};
    for (int e = 0; e + 1 < 8; ++e) {
        double lg = 0.0, lp = 0.0;
        int n = 0;
        for (int mic = 0; mic < 2; ++mic)
            for (int i = 0; i < b.modes; ++i)
                if (b.freq[i] >= edges[e] && b.freq[i] < edges[e + 1]) {
                    lg += std::log(std::abs(g.row(mic)[i]) + 1e-30);
                    lp += std::log(std::fabs(b.pickupShape[std::size_t(mic) * b.modes + i]) + 1e-30);
                    ++n;
                }
        if (n > 0)
            std::printf("  %4.0f-%4.0f Hz: %3d modes, mics / pickups %+.1f dB\n", edges[e], edges[e + 1], n / 2,
                        20.0 / std::log(10.0) * (lg - lp) / n);
    }
}

// setPickupGain(phi, 0) is setPickupShape(phi) bit for bit, and the complex
// gain follows the e^{+j w t} convention: gain j advances the readout by a
// quarter period.
TEST(pickup_gain_mapping) {
    const pm::BoardModel& b = pm::defaultBoard();
    pm::Soundboard s1, s2;
    CHECK(pm::loadSoundboard(s1, b, kFs) && pm::loadSoundboard(s2, b, kFs));
    for (int p = 0; p < 2; ++p)
        for (int i = 0; i < b.modes; ++i) s2.setPickupGain(p, i, b.pickupShape[std::size_t(p) * b.modes + i], 0.0);
    s2.updatePickupBound();
    std::vector<float> f(static_cast<std::size_t>(b.zones)), d(f.size());
    float v1[2], v2[2];
    unsigned seed = 1;
    bool same = true;
    for (int t = 0; t < 4800; ++t) {
        for (float& x : f) {
            seed = seed * 1664525u + 1013904223u;
            x = t < 2400 ? static_cast<float>(int(seed >> 9) - (1 << 22)) * 1e-7f : 0.0f;
        }
        s1.step(f.data(), d.data(), v1, pm::Kernel::Scalar);
        s2.step(f.data(), d.data(), v2, pm::Kernel::Scalar);
        same = same && v1[0] == v2[0] && v1[1] == v2[1];
    }
    CHECK(same);

    pm::Soundboard s;
    CHECK(s.allocate(1, 1, 2));
    const double f0 = 1000.0;
    s.setMode(0, f0, 2.0, kFs);
    s.setZoneShape(0, 0, 1.0);
    s.setPickupGain(0, 0, 1.0, 0.0);
    s.setPickupGain(1, 0, 0.0, 1.0);
    s.setSleepThresholds(0.0, 0.0);
    const int n = 4800, q = static_cast<int>(kFs / f0 / 4.0);  // 12 samples = T/4
    std::vector<double> y0(n), y1(n);
    float zf = 1.0f, zd = 0.0f, v[2];
    for (int t = 0; t < n; ++t) {
        s.step(&zf, &zd, v, pm::Kernel::Scalar);
        zf = 0.0f;
        y0[t] = v[0];
        y1[t] = v[1];
    }
    double c = 0.0, e0 = 0.0, e1 = 0.0;
    for (int t = 100; t + q < n; ++t) {
        c += y1[t] * y0[t + q];
        e0 += y0[t + q] * y0[t + q];
        e1 += y1[t] * y1[t];
    }
    std::printf("  gain j vs gain 1 advanced by T/4: correlation %.4f\n", c / std::sqrt(e0 * e1));
    CHECK(c / std::sqrt(e0 * e1) > 0.99);
}

// Engine: the v0 pickups (enabled = false) and the microphones differ; the
// microphones give different channels whose mono sum does not cancel.
TEST(engine_stereo) {
    for (int note : {36, 60, 84}) {
        pm::Engine a, v0;
        a.setNoiseLevels(0.0f, 0.0f);
        v0.setNoiseLevels(0.0f, 0.0f);
        pm::SoundboardConfig c = v0.soundboardConfig();
        c.radiation.enabled = false;
        v0.setSoundboardConfig(c);
        a.prepare(kFs, 64);
        v0.prepare(kFs, 64);
        const int len = static_cast<int>(1.5 * kFs);
        const Stereo s = renderEngine(a, {noteOn(0, note, 80)}, len);
        const Stereo o = renderEngine(v0, {noteOn(0, note, 80)}, len);
        bool differ = false;
        for (int i = 0; i < len && !differ; ++i) differ = s.l[i] != o.l[i];
        CHECK(differ);
        double ll = 0, rr = 0, lr = 0;
        for (int i = 0; i < len; ++i) {
            ll += double(s.l[i]) * s.l[i];
            rr += double(s.r[i]) * s.r[i];
            lr += double(s.l[i]) * s.r[i];
        }
        // Lag of the cross-correlation peak within +-1 ms.
        int bestLag = 0;
        double best = -1e300;
        for (int lag = -48; lag <= 48; ++lag) {
            double x = 0.0;
            for (int i = 4800; i < len - 48; ++i) x += double(s.l[i]) * s.r[i + lag];
            if (x > best) best = x, bestLag = lag;
        }
        const double corr = lr / std::sqrt(ll * rr), ild = 10.0 * std::log10(ll / rr);
        const double mono = 10.0 * std::log10((ll + rr + 2.0 * lr) / (ll + rr));
        std::printf("  note %d: L-R %.1f dB, correlation %.3f, peak lag %d samples, mono sum %+.1f dB re L+R power\n",
                    note, ild, corr, bestLag, mono);
        CHECK(std::fabs(ild) > 0.1 || std::fabs(corr) < 0.99);
        CHECK(mono > -3.0);  // correlation > -0.5: no broadband cancellation
    }
}

namespace {

// GCC-PHAT lag (s) of the right channel against the left, transfer from the
// bridge point of a key, 100 Hz-5 kHz.
double gccLag(const pm::BoardModel& b, const pm::MicGains& g, int key) {
    const double* sh = b.shapeOfKey(key);
    std::vector<std::complex<double>> x;
    std::vector<double> w;
    for (double f = 100.0; f < 5000.0; f += 2.0) {
        const std::complex<double> c = pm::micModalTransfer(b, sh, g.row(1), f) *
                                       std::conj(pm::micModalTransfer(b, sh, g.row(0), f));
        x.push_back(c / std::max(std::abs(c), 1e-300));
        w.push_back(2.0 * kPi * f);
    }
    double best = -1e300, lag = 0.0;
    for (double tau = -3e-3; tau <= 3e-3; tau += 1e-5) {
        double acc = 0.0;
        for (std::size_t k = 0; k < x.size(); ++k) acc += (x[k] * std::polar(1.0, w[k] * tau)).real();
        if (acc > best) best = acc, lag = tau;
    }
    return lag;
}

}  // namespace

// One bulk delay for both microphones keeps the inter-microphone time
// difference: the channels' GCC-PHAT lag is that of per-microphone bulk
// delays plus the difference of the microphones' distances to the board
// centre (0.63 ms). For a bass key the lag is the path difference from its
// bridge point; elsewhere the whole board radiates and no single path rules.
TEST(inter_microphone_delay) {
    const pm::BoardModel& b = pm::defaultBoard();
    pm::RadiationParams r, own;
    own.commonBulkDelay = false;
    const pm::MicGains g = pm::computeMicGains(b, r), go = pm::computeMicGains(b, own);
    double d[2];
    for (int q = 0; q < 2; ++q) {
        const double cx = 0.5 * b.params.lengthX - r.micX[q], cy = 0.5 * b.params.widthY - r.micY[q];
        d[q] = std::sqrt(cx * cx + cy * cy + r.micZ[q] * r.micZ[q]);
    }
    const double centre = (d[1] - d[0]) / r.c;
    for (int key : {10, 40, 70}) {
        const double lag = gccLag(b, g, key), lagOwn = gccLag(b, go, key);
        double p[2];
        for (int q = 0; q < 2; ++q) {
            const double dx = b.keyX[key - 1] - r.micX[q], dy = b.keyY[key - 1] - r.micY[q];
            p[q] = std::sqrt(dx * dx + dy * dy + r.micZ[q] * r.micZ[q]);
        }
        std::printf("  key %d: R-L lag %.3f ms (own bulk delays %.3f ms), bridge-point path difference %.3f ms\n", key,
                    1e3 * lag, 1e3 * lagOwn, 1e3 * (p[1] - p[0]) / r.c);
        CHECK_NEAR(lag - lagOwn, centre, 0.05e-3);
        if (key == 10) CHECK_NEAR(lag, (p[1] - p[0]) / r.c, 0.1e-3);
    }
}
