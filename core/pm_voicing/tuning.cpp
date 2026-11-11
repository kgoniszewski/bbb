#include "pm_voicing/tuning.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pm {

namespace {

// 0 at a, 1 at b (a < b), raised cosine in between.
double ramp(double x, double a, double b) noexcept {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return 0.5 - 0.5 * std::cos(std::numbers::pi * t);
}

double etHz(int key, double a4Hz) noexcept { return a4Hz * std::pow(2.0, (key - 49.0) / 12.0); }

}  // namespace

double partialRatio(double B, int n) noexcept {
    const double nn = static_cast<double>(n);
    return nn * std::sqrt((1.0 + B * nn * nn) / (1.0 + B));
}

OctaveWeights octaveWeights(int key) noexcept {
    OctaveWeights w;
    if (key > 49) {
        const double to21 = ramp(key, 56.0, 68.0);
        const double to41 = 0.5 * ramp(key, 76.0, 88.0);
        w.w42 = 1.0 - to21;
        w.w21 = to21 * (1.0 - to41);
        w.w41 = to21 * to41;
    } else if (key < 37) {
        const double to63 = ramp(-key, -28.0, -18.0);
        const double to105 = ramp(-key, -16.0, -1.0);
        w.w42 = 1.0 - to63;
        w.w63 = to63 * (1.0 - to105);
        w.w105 = to63 * to105;
    }
    return w;
}

void virtualTuner(const double* B, const TuningOptions& opt, double* f1, const PartialRatios* ratios) noexcept {
    const double a4 = opt.a4Hz;
    if (opt.equalTemperament) {
        for (int k = 1; k <= kNumKeys; ++k) f1[k - 1] = etHz(k, a4);
        return;
    }
    auto lf = [&](int k) { return std::log(f1[k - 1]); };
    auto pr = [&](int k, int n) {
        const double r = ratios && n <= kTunerPartials ? ratios->r[k - 1][n] : 0.0;
        return r > 0.0 ? r : partialRatio(B[k - 1], n);
    };
    f1[48] = a4;
    // A3 from A4, 4:2.
    f1[36] = a4 * pr(49, 2) / pr(37, 4);
    // Equal division on the second partials: with sounding ratios, a first
    // partial pulled by a board resonance does not move its neighbours.
    const double l37 = lf(37) + std::log(pr(37, 2)), l49 = lf(49) + std::log(pr(49, 2));
    for (int k = 38; k <= 48; ++k) f1[k - 1] = std::exp(l37 + (l49 - l37) * (k - 37) / 12.0) / pr(k, 2);
    for (int k = 50; k <= kNumKeys; ++k) {
        const OctaveWeights w = octaveWeights(k);
        const int lo = k - 12;
        double acc = 0.0;
        acc += w.w42 * (lf(lo) + std::log(pr(lo, 4) / pr(k, 2)));
        acc += w.w21 * (lf(lo) + std::log(pr(lo, 2)));
        if (w.w41 > 0.0) acc += w.w41 * (lf(k - 24) + std::log(pr(k - 24, 4)));
        f1[k - 1] = std::exp(acc);
    }
    for (int k = 36; k >= 1; --k) {
        const OctaveWeights w = octaveWeights(k);
        const int hi = k + 12;
        double acc = 0.0;
        acc += w.w42 * (lf(hi) + std::log(pr(hi, 2) / pr(k, 4)));
        acc += w.w63 * (lf(hi) + std::log(pr(hi, 3) / pr(k, 6)));
        acc += w.w105 * (lf(hi) + std::log(pr(hi, 5) / pr(k, 10)));
        f1[k - 1] = std::exp(acc);
    }
    const double s = std::clamp(opt.stretch, 0.0, 1.5);
    for (int k = 1; k <= kNumKeys; ++k) {
        const double et = etHz(k, a4);
        f1[k - 1] = et * std::pow(f1[k - 1] / et, s);
    }
}

double centsFromEqualTemperament(double f1, int key, double a4Hz) noexcept {
    return 1200.0 * std::log2(f1 / etHz(key, a4Hz));
}

double railsbackReferenceCents(int key) noexcept {
    static const double kKeys[10] = {1, 4, 16, 28, 40, 49, 52, 64, 76, 88};
    static const double kCents[10] = {-25.0, -16.0, -7.0, -2.5, -0.5, 0.0, 1.0, 5.0, 13.0, 30.0};
    return interpMonotone(kKeys, kCents, 10, key);
}

}  // namespace pm
