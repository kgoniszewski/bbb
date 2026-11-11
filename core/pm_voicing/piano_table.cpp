#include "pm_voicing/piano_table.h"

#include <algorithm>
#include <cmath>

#include "pm_voicing/tuning.h"

namespace pm {

namespace {

constexpr int kAnchors = 6;
constexpr double kAnchorKey[kAnchors] = {1, 16, 40, 64, 76, 88};  // A0 C2 C4 C6 C7 C8

// Plan §2.1 anchors.
//
// Hammer mass: the whole hammer, now that it strikes all strings of the
// unison (the "real D" column of the plan; per string this is close to the
// simulation masses used as single-string values in M2: C2 10 g / 2, C4
// 9 g / 3, C7 5 g / 3). The felt law stays per string.
//
// Losses. With the bridge coupling of the unison model (bridge_admittance.h)
// supplying the bridge losses, b1 and b3 represent internal and air losses
// only (plan §1.2). Split used here, for the fundamental of a struck note:
//   - b1, b3 set the decay of the modes that hardly move the bridge
//     (antisymmetric and horizontal ones), i.e. the aftersound: target T60
//     of the fundamental about 60, 45, 20, 7, 3.5, 1.5 s at A0, C2, C4, C6,
//     C7, C8 (b1 = 6.91 / T60 - b3 (2 pi f1)^2);
//   - the placeholder admittance's conductance adds S kappa Re Yvv to the
//     in-phase vertical mode, the prompt sound: target T60 of the
//     fundamental about 25, 14, 3.5, 1.2, 0.6, 0.3 s.
// b3 is about half the M2 value: the admittance also rises with frequency.
// These are plausible starting values, not measurements.
constexpr double kLength[kAnchors] = {2.01, 1.90, 0.62, 0.19, 0.09, 0.05};
constexpr double kTension[kAnchors] = {1300, 750, 670, 720, 750, 700};
constexpr double kB[kAnchors] = {2.5e-4, 7.4e-5, 3.8e-4, 2.5e-3, 8.6e-3, 1.5e-2};
constexpr double kMassG[kAnchors] = {11.0, 10.0, 9.0, 6.0, 5.0, 4.5};
constexpr double kP[kAnchors] = {2.2, 2.3, 2.5, 2.8, 3.0, 3.5};
constexpr double kK[kAnchors] = {2e8, 4e8, 4.5e9, 1e11, 1e12, 3e12};
constexpr double kStrike[kAnchors] = {0.12, 0.12, 0.12, 0.09, 0.0625, 0.06};
constexpr double kB1[kAnchors] = {0.115, 0.15, 0.335, 0.92, 1.8, 4.0};
constexpr double kB3[kAnchors] = {1e-8, 8e-9, 3.5e-9, 1.5e-9, 1e-9, 8e-10};
constexpr double kDamperSigma[kAnchors] = {16.0, 24.0, 56.0, 90.0, 110.0, 120.0};
constexpr double kDamperWidth[kAnchors] = {0.02, 0.02, 0.04, 0.08, 0.12, 0.15};

double interpLin(const double (&a)[kAnchors], double key) noexcept {
    return interpMonotone(kAnchorKey, a, kAnchors, key);
}

// splitmix64: deterministic, platform-independent pseudo-random numbers.
std::uint64_t mix(std::uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}
// Uniform in [-1, 1).
double uniform(std::uint64_t seed, int key, int stream, int index) noexcept {
    const std::uint64_t h = mix(mix(mix(seed) ^ std::uint64_t(key)) ^ (std::uint64_t(stream) << 32 | std::uint64_t(index)));
    return double(h >> 11) * (2.0 / 9007199254740992.0) - 1.0;
}

}  // namespace

double interpMonotone(const double* x, const double* y, int n, double at) noexcept {
    constexpr int kMax = 16;
    if (n < 2 || n > kMax || at <= x[0]) return y[0];
    if (at >= x[n - 1]) return y[n - 1];
    double d[kMax] = {}, m[kMax] = {};
    for (int i = 0; i < n - 1; ++i) d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    m[0] = d[0];
    m[n - 1] = d[n - 2];
    for (int i = 1; i < n - 1; ++i) {
        if (d[i - 1] * d[i] <= 0.0) {
            m[i] = 0.0;
        } else {
            const double w1 = 2.0 * (x[i + 1] - x[i]) + (x[i] - x[i - 1]);
            const double w2 = (x[i + 1] - x[i]) + 2.0 * (x[i] - x[i - 1]);
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
        }
    }
    int i = 0;
    while (i < n - 2 && at > x[i + 1]) ++i;
    const double hseg = x[i + 1] - x[i];
    const double t = std::clamp((at - x[i]) / hseg, 0.0, 1.0);
    const double t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * y[i] + (t3 - 2 * t2 + t) * hseg * m[i] +
           (-2 * t3 + 3 * t2) * y[i + 1] + (t3 - t2) * hseg * m[i + 1];
}

double interpAnchorsLog(const double* values6, double key) noexcept {
    double l[kAnchors];
    for (int i = 0; i < kAnchors; ++i) l[i] = std::log(values6[i]);
    return std::exp(interpMonotone(kAnchorKey, l, kAnchors, key));
}

int defaultStringCount(int key) noexcept { return key <= 8 ? 1 : (key <= 20 ? 2 : 3); }

void buildPianoTable(PianoTable& table, double a4Hz) noexcept {
    TableOptions o;
    o.a4Hz = a4Hz;
    buildPianoTable(table, o);
}

void buildPianoTable(PianoTable& table, const TableOptions& opt) noexcept {
    double B[kNumKeys], f1[kNumKeys];
    for (int i = 0; i < kNumKeys; ++i) B[i] = interpAnchorsLog(kB, i + 1);
    TuningOptions tun;
    tun.a4Hz = opt.a4Hz;
    tun.stretch = opt.stretch;
    tun.equalTemperament = opt.equalTemperament;
    virtualTuner(B, tun, f1);
    table.tuning = tun;

    for (int i = 0; i < kNumKeys; ++i) {
        KeyParams& k = table.keys[i];
        const double key = i + 1;
        k = KeyParams{};
        k.key = i + 1;
        k.f1 = f1[i];
        k.length = interpAnchorsLog(kLength, key);
        k.tension = interpAnchorsLog(kTension, key);
        k.B = B[i];
        k.f0 = k.f1 / std::sqrt(1.0 + k.B);
        const double c2L = 2.0 * k.length * k.f0;  // wave speed
        k.linearDensity = k.tension / (c2L * c2L);
        k.b1 = interpAnchorsLog(kB1, key);
        k.b3 = interpAnchorsLog(kB3, key);

        k.strings = defaultStringCount(k.key);
        if (k.strings > 1) {
            // A spread pattern (randomly permuted) with +-30 % jitter, zero
            // mean: every unison is detuned by about the spread, none
            // accidentally in perfect tune.
            static const double kPattern2[2] = {-0.5, 0.5}, kPattern3[3] = {-0.6, 0.0, 0.6};
            const double* pat = k.strings == 2 ? kPattern2 : kPattern3;
            const int shift = static_cast<int>(std::floor((uniform(opt.seed, k.key, 3, 0) + 1.0) * 0.5 * k.strings)) % k.strings;
            const bool flip = uniform(opt.seed, k.key, 3, 1) < 0.0;
            double d[kMaxStrings], mean = 0.0;
            for (int s = 0; s < k.strings; ++s) {
                const double base = pat[(s + shift) % k.strings] * (flip ? -1.0 : 1.0);
                mean += (d[s] = base + 0.3 * std::fabs(base + 0.5) * uniform(opt.seed, k.key, 1, s));
            }
            mean /= k.strings;
            for (int s = 0; s < k.strings; ++s)
                k.detuneCents[s] = opt.unisonWidth * opt.unisonSpreadCents * 2.0 * (d[s] - mean);
        }
        // Horizontal modes a little sharp (stiffer horizontal termination).
        k.polarizationSplitCents = 0.2;
        for (int s = 0; s < k.strings; ++s)
            k.strikeOffset[s] = opt.matingSpreadM * uniform(opt.seed, k.key, 2, s);
        k.strikeHorizontal = 0.1;

        k.hammerMass = 1e-3 * interpAnchorsLog(kMassG, key);
        k.feltK = interpAnchorsLog(kK, key);
        k.feltP = interpLin(kP, key);
        k.feltMu = 0.04;
        k.strikePos = interpLin(kStrike, key);
        // Contact oversampling: keeps the substep short against the contact
        // time and the highest partials. With R = 1 throughout, partial
        // levels miss the 1 dB target from about C4 up (up to 3 dB around C7);
        // R = 2 meets it to key 75, R = 4 in the top octave (convergence test).
        k.contactOversampling = key >= 76 ? 4 : (key >= 40 ? 2 : 1);
        // Una corda: on trichords the shifted hammer misses string 0 (the
        // offset grows beyond any felt compression); every key meets softer
        // felt.
        k.unaCordaString = k.strings == 3 ? 0 : -1;
        k.unaCordaMiss = 2.5e-3;
        k.unaCordaSoften = 0.35;

        // Keys 69 and up have no dampers (plan §1.9); the rate is still set so
        // a panic can damp them.
        k.hasDamper = k.key < 69;
        k.damperSigma = interpAnchorsLog(kDamperSigma, key);
        k.damperWidth = interpAnchorsLog(kDamperWidth, key);
        k.damperPos = k.strikePos + 0.5 * k.damperWidth + 0.01;
        k.damperFloor = 0.4;

        k.outputGain = 0.004;
        k.pan = (key - 44.5) / 43.5;
    }
}

double velocityCurveFn(double x, const void* user) noexcept {
    const VelocityCurve& c = *static_cast<const VelocityCurve*>(user);
    x = std::clamp(x, 0.0, 1.0);
    return c.vMin + (c.vMax - c.vMin) * std::pow(x, c.exponent);
}

}  // namespace pm
