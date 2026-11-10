// Partial frequency and decay accuracy of the modal string (plan §1.2, M1).
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "signal_utils.h"
#include "test_harness.h"

using namespace pm;

namespace {

constexpr double kFs = 48000.0;
constexpr double kLn1000 = 6.907755278982137;  // T60 = ln(1000) / sigma

StringParams c2() {
    StringParams p;
    p.f0 = 65.41;
    p.B = 7.4e-5;
    p.b1 = 0.25;
    p.b3 = 7.5e-9;
    p.length = 1.90;
    p.linearDensity = 0.035;
    return p;
}

StringParams c4() { return StringParams{}; }

StringParams c7() {
    StringParams p;
    p.f0 = 2093.0;
    p.B = 8.6e-3;
    p.b1 = 1.0;
    p.b3 = 2.7e-9;
    p.length = 0.09;
    p.linearDensity = 3.9e-3;
    p.inputPos = 0.0625;
    return p;
}

// Coefficient phase and magnitude against the formulas, for every mode.
void checkCoefficients(const StringParams& p) {
    ModeBank bank;
    CHECK(bank.allocate(256));
    const int count = loadString(bank, p, kFs);
    CHECK(count > 0);
    double worstCents = 0.0, worstSigma = 0.0;
    for (int i = 0; i < count; ++i) {
        const std::complex<double> c = bank.coefficient(i);
        const double f = std::arg(c) * kFs / (2.0 * std::numbers::pi);
        const double sigma = -std::log(std::abs(c)) * kFs;
        const double fx = partialFrequency(p.f0, p.B, i + 1);
        const double sx = partialDecayRate(p.b1, p.b3, fx);
        worstCents = std::max(worstCents, std::fabs(pm_test::cents(f, fx)));
        worstSigma = std::max(worstSigma, std::fabs(sigma / sx - 1.0));
    }
    std::printf("  f0=%.2f: %d modes, worst coefficient error %.2e cents, %.3f %% decay\n", p.f0,
                count, worstCents, 100.0 * worstSigma);
    CHECK_LE(worstCents, 0.01);
    CHECK_LE(worstSigma, 0.005);
}

// Pluck, render, and measure partials 1..20 from the signal.
void checkRenderedPartials(const StringParams& p, double seconds) {
    ModeBank bank;
    CHECK(bank.allocate(256));
    const int count = loadString(bank, p, kFs);
    pluck(bank, 0.12, 1e-3);
    const auto y = pm_test::render(bank, static_cast<int>(seconds * kFs), Kernel::Simd);
    const int nCheck = std::min(20, count);
    double worstCents = 0.0, worstT60 = 0.0;
    for (int n = 1; n <= nCheck; ++n) {
        const double fx = partialFrequency(p.f0, p.B, n);
        const double sx = partialDecayRate(p.b1, p.b3, fx);
        const auto fit = pm_test::measurePartial(y, kFs, fx, p.f0, 60.0);
        CHECK(fit.frames >= 8);
        const double ce = pm_test::cents(fit.freqHz, fx);
        const double t60 = kLn1000 / fit.sigma, t60x = kLn1000 / sx;
        const double te = t60 / t60x - 1.0;
        worstCents = std::max(worstCents, std::fabs(ce));
        worstT60 = std::max(worstT60, std::fabs(te));
        CHECK_LE(std::fabs(ce), 0.5);
        CHECK_LE(std::fabs(te), 0.03);
    }
    std::printf("  f0=%.2f: partials 1..%d worst |error| %.4f cents, T60 %.3f %%\n", p.f0, nCheck,
                worstCents, 100.0 * worstT60);
}

}  // namespace

TEST(coefficients_match_formula) {
    checkCoefficients(c2());
    checkCoefficients(c4());
    checkCoefficients(c7());
}

TEST(partial_count_respects_limits) {
    StringParams p = c4();
    p.maxPartials = 10;
    CHECK(countStringModes(p, kFs) == 10);
    p.maxPartials = 1000;
    p.maxFreqHz = 1e9;
    const int n = countStringModes(p, kFs);
    CHECK(partialFrequency(p.f0, p.B, n) < 0.45 * kFs);
    CHECK(partialFrequency(p.f0, p.B, n + 1) >= 0.45 * kFs);
}

TEST(rendered_pluck_c4) { checkRenderedPartials(c4(), 4.0); }
TEST(rendered_pluck_c2) { checkRenderedPartials(c2(), 6.0); }
TEST(rendered_pluck_c7) { checkRenderedPartials(c7(), 2.0); }

// The force-input path: a unit force impulse must produce
//   y[k] = sum_n wOut_n * wIn_n * F * exp(-sigma_n t) * sin(omega_n t),  t = k/fs
// with wIn_n = phi_n(x_in) / (m_n omega_n fs): the textbook impulse response
// of a mass-spring mode, sampled.
TEST(force_impulse_matches_analytic_response) {
    const StringParams p = c4();
    ModeBank bank;
    CHECK(bank.allocate(256));
    const int count = loadString(bank, p, kFs);
    std::vector<ModeSpec> spec(static_cast<std::size_t>(count));
    computeStringModes(p, kFs, spec.data(), count);

    const int nSamples = static_cast<int>(0.5 * kFs);
    const double force = 10.0;  // N
    std::vector<float> u(static_cast<std::size_t>(nSamples), 0.0f);
    u[0] = static_cast<float>(force);
    const auto y = pm_test::render(bank, nSamples, Kernel::Simd, &u);

    double peak = 0.0, worst = 0.0;
    for (int k = 0; k < nSamples; ++k) {
        const double t = k / kFs;  // y[k] reads Im(z[k+1]) = Im(c^k * wIn * F)
        double ref = 0.0;
        for (const auto& m : spec)
            ref += m.wOut * m.wIn * force * std::exp(-m.sigma * t) *
                   std::sin(2.0 * std::numbers::pi * m.freqHz * t);
        peak = std::max(peak, std::fabs(ref));
        worst = std::max(worst, std::fabs(ref - y[k]));
    }
    std::printf("  impulse response: peak %.3e, max error %.2e (relative %.2e)\n", peak, worst,
                worst / peak);
    CHECK(peak > 0.0);
    CHECK_LE(worst / peak, 1e-4);
}
