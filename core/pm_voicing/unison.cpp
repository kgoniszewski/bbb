#include "pm_voicing/unison.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_voicing/complex_eigen.h"

namespace pm {

namespace {
constexpr double kTwoPi = 2.0 * std::numbers::pi;
using cd = std::complex<double>;

// Deterministic perturbation pattern for the oscillator frequencies.
constexpr double kPattern[kMaxOscillators] = {0.37, -0.61, 0.13, 0.89, -0.29, 0.53};
}  // namespace

void unisonMatrix(const UnisonPartialSpec& s, cd* A) noexcept {
    const int S = std::clamp(s.strings, 1, kMaxStrings);
    const int n = 2 * S;
    const double w = kTwoPi * s.freqHz;
    for (int k = 0; k < n; ++k) {
        const int str = k % S;
        const bool horiz = k >= S;
        const int pk = horiz ? 1 : 0;
        for (int l = 0; l < n; ++l) {
            const int pl = l >= S ? 1 : 0;
            const cd y = pk == 0 ? (pl == 0 ? s.Y.vv : s.Y.vh) : (pl == 0 ? s.Y.vh : s.Y.hh);
            A[k * n + l] = -s.kappa * y;
        }
        const double wk = w * s.detuneRatio[str] * (horiz ? s.horizontalRatio : 1.0);
        A[k * n + k] += cd(-s.sigma, wk);
    }
}

UnisonPartial solveUnisonPartial(const UnisonPartialSpec& s) noexcept {
    UnisonPartial r;
    const int S = std::clamp(s.strings, 1, kMaxStrings);
    const int n = 2 * S;
    r.oscillators = n;
    cd A0[kMaxOscillators * kMaxOscillators];
    unisonMatrix(s, A0);
    for (int k = 0; k < n; ++k) {
        r.oscFreqHz[k] = A0[k * n + k].imag() / kTwoPi;
        r.oscSigma[k] = -A0[k * n + k].real();
    }

    // Coupling strength (1/s), the scale of the perturbations.
    const double coupling =
        std::max(std::fabs(s.kappa) * std::max({std::abs(s.Y.vv), std::abs(s.Y.vh), std::abs(s.Y.hh)}),
                 1e-12 * kTwoPi * s.freqHz);

    cd lambda[kMaxOscillators], V[kMaxOscillators * kMaxOscillators], Vi[kMaxOscillators * kMaxOscillators];
    bool done = false;
    const double scales[] = {0.0, 1e-6, 1e-5, 1e-4, 1e-3};
    for (double sc : scales) {
        for (int i = 0; i < n * n; ++i) r.A[i] = A0[i];
        for (int k = 0; k < n; ++k) r.A[k * n + k] += cd(0.0, sc * coupling * kPattern[k]);
        const EigenResult e = eigenDecompose(n, r.A, lambda, V, Vi);
        if (e.ok && e.condition <= UnisonPartial::kMaxCondition && e.residual <= 1e-10) {
            r.status = sc == 0.0 ? UnisonStatus::Exact : UnisonStatus::Perturbed;
            r.condition = e.condition;
            r.residual = e.residual;
            done = true;
            break;
        }
    }
    if (!done) {
        // Uncoupled fallback: each oscillator keeps its own diagonal entry
        // (including its own share of the bridge loss and frequency shift).
        for (int i = 0; i < n * n; ++i) {
            r.A[i] = (i / n == i % n) ? A0[i] : cd(0.0);
            V[i] = Vi[i] = (i / n == i % n) ? cd(1.0) : cd(0.0);
        }
        for (int k = 0; k < n; ++k) lambda[k] = A0[k * n + k];
        r.status = UnisonStatus::Uncoupled;
        r.condition = 1.0;
        r.residual = 0.0;
    }

    // Rotate each mode so its output weight is real and non-negative.
    const double beta = s.phi / (s.modalMass * kTwoPi * s.freqHz);
    for (int j = 0; j < n; ++j) {
        cd c = 0.0;
        for (int k = 0; k < n; ++k) c += (k < S ? s.outV : s.outH) * V[k * n + j];
        const double ac = std::abs(c);
        const cd alpha = ac > 0.0 ? ac / c : cd(1.0);
        for (int k = 0; k < n; ++k) {
            V[k * n + j] *= alpha;
            Vi[j * n + k] /= alpha;
        }
        UnisonMode& m = r.mode[j];
        m.lambda = lambda[j];
        m.wOut = ac;
        double exc = 0.0;
        for (int st = 0; st < S; ++st) {
            m.proj[st] = s.phi * (V[st * n + j] + s.strikeHorizontal * V[(S + st) * n + j]);
            m.input[st] = beta * (Vi[j * n + st] + s.strikeHorizontal * Vi[j * n + S + st]);
            exc += std::abs(m.input[st]);
        }
        m.score = ac * exc;
    }
    for (int i = 0; i < n * n; ++i) {
        r.V[i] = V[i];
        r.Vinv[i] = Vi[i];
    }
    return r;
}

}  // namespace pm
