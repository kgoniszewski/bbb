#include "pm_voicing/piano_table.h"

#include <algorithm>
#include <cmath>

namespace pm {

namespace {

constexpr int kAnchors = 6;
constexpr double kAnchorKey[kAnchors] = {1, 16, 40, 64, 76, 88};  // A0 C2 C4 C6 C7 C8

// Plan §2.1 anchors. Hammer masses: the table's simulation values are
// effective per-string masses (C2 4.9 g, C4 2.97 g, C7 2.2 g); with one
// string per key those are used, and the estimates for the other anchors are
// divided by the strings they would drive (C6 ~6 g / 3, C8 ~4.5 g / 3; A0 has
// one string). Loss values: C4 is the given anchor, the others are chosen
// for plausible single-string decay (longer in the bass, shorter in the
// treble). Damper strength and felt width are estimates for M2.
constexpr double kLength[kAnchors] = {2.01, 1.90, 0.62, 0.19, 0.09, 0.05};
constexpr double kTension[kAnchors] = {1300, 750, 670, 720, 750, 700};
constexpr double kB[kAnchors] = {2.5e-4, 7.4e-5, 3.8e-4, 2.5e-3, 8.6e-3, 1.5e-2};
constexpr double kMassG[kAnchors] = {11.0, 4.9, 2.97, 2.4, 2.2, 1.6};
constexpr double kP[kAnchors] = {2.2, 2.3, 2.5, 2.8, 3.0, 3.5};
constexpr double kK[kAnchors] = {2e8, 4e8, 4.5e9, 1e11, 1e12, 3e12};
constexpr double kStrike[kAnchors] = {0.12, 0.12, 0.12, 0.09, 0.0625, 0.06};
constexpr double kB1[kAnchors] = {0.2, 0.3, 0.5, 1.5, 2.5, 4.0};
constexpr double kB3[kAnchors] = {2e-8, 1.5e-8, 6.25e-9, 3e-9, 2e-9, 1.5e-9};
constexpr double kDamperSigma[kAnchors] = {16.0, 24.0, 56.0, 90.0, 110.0, 120.0};
constexpr double kDamperWidth[kAnchors] = {0.02, 0.02, 0.04, 0.08, 0.12, 0.15};

// Fritsch-Carlson monotone cubic through the anchors: smooth (C1), and no
// overshoot between anchors.
double pchip(const double* y, double key) noexcept {
    const double* x = kAnchorKey;
    double d[kAnchors - 1], m[kAnchors];
    for (int i = 0; i < kAnchors - 1; ++i) d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    m[0] = d[0];
    m[kAnchors - 1] = d[kAnchors - 2];
    for (int i = 1; i < kAnchors - 1; ++i) {
        if (d[i - 1] * d[i] <= 0.0) {
            m[i] = 0.0;
        } else {
            const double w1 = 2.0 * (x[i + 1] - x[i]) + (x[i] - x[i - 1]);
            const double w2 = (x[i + 1] - x[i]) + 2.0 * (x[i] - x[i - 1]);
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
        }
    }
    int i = 0;
    while (i < kAnchors - 2 && key > x[i + 1]) ++i;
    const double hseg = x[i + 1] - x[i];
    const double t = std::clamp((key - x[i]) / hseg, 0.0, 1.0);
    const double t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * y[i] + (t3 - 2 * t2 + t) * hseg * m[i] +
           (-2 * t3 + 3 * t2) * y[i + 1] + (t3 - t2) * hseg * m[i + 1];
}

double interpLin(const double (&a)[kAnchors], double key) noexcept { return pchip(a, key); }
// Scale-like quantities (lengths, tensions, stiffnesses, losses): geometric.
double interpLog(const double (&a)[kAnchors], double key) noexcept {
    double l[kAnchors];
    for (int i = 0; i < kAnchors; ++i) l[i] = std::log(a[i]);
    return std::exp(pchip(l, key));
}

}  // namespace

void buildPianoTable(PianoTable& table, double a4Hz) noexcept {
    for (int i = 0; i < kNumKeys; ++i) {
        KeyParams& k = table.keys[i];
        const double key = i + 1;
        k = KeyParams{};
        k.key = i + 1;
        k.f1 = a4Hz * std::pow(2.0, (key - 49.0) / 12.0);
        k.length = interpLog(kLength, key);
        k.tension = interpLog(kTension, key);
        k.B = interpLog(kB, key);
        k.f0 = k.f1 / std::sqrt(1.0 + k.B);
        const double c2L = 2.0 * k.length * k.f0;  // wave speed
        k.linearDensity = k.tension / (c2L * c2L);
        k.b1 = interpLog(kB1, key);
        k.b3 = interpLog(kB3, key);

        k.hammerMass = 1e-3 * interpLog(kMassG, key);
        k.feltK = interpLog(kK, key);
        k.feltP = interpLin(kP, key);
        k.feltMu = 0.04;
        k.strikePos = interpLin(kStrike, key);
        // Contact oversampling: keeps the substep short against the contact
        // time and the highest partials. With R = 1 throughout, partial
        // levels miss the 1 dB target from about C4 up (up to 3 dB around C7);
        // R = 2 meets it to key 75, R = 4 in the top octave (convergence test).
        k.contactOversampling = key >= 76 ? 4 : (key >= 40 ? 2 : 1);

        // Keys 69 and up have no dampers (plan §1.9); the rate is still set so
        // a panic can damp them.
        k.hasDamper = k.key < 69;
        k.damperSigma = interpLog(kDamperSigma, key);
        k.damperWidth = interpLog(kDamperWidth, key);
        k.damperPos = k.strikePos + 0.5 * k.damperWidth + 0.01;
        k.damperFloor = 0.4;

        k.outputGain = 0.01;
        k.pan = (key - 44.5) / 43.5;
    }
}

double velocityCurveFn(double x, const void* user) noexcept {
    const VelocityCurve& c = *static_cast<const VelocityCurve*>(user);
    x = std::clamp(x, 0.0, 1.0);
    return c.vMin + (c.vMax - c.vMin) * std::pow(x, c.exponent);
}

}  // namespace pm
