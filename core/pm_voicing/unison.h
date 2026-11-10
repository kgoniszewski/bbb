// Coupled unison normal modes for one partial of one key (plan §1.3).
// Non-real-time.
//
// Oscillators. Partial n of each of the key's S strings in two
// polarizations: 2S complex amplitudes z_k (the bank's convention,
// z = qdot/omega + j q, so Im z is the modal displacement q in metres),
// ordered k = s (vertical, string s) and k = S + s (horizontal, string s).
//
// Coupling (Weinreich). String s exerts on the bridge the force g q_k in
// polarization p(k), g = T (n pi / L)(1 + B n^2) (tension plus shear at a
// pinned end), and the bridge moves with velocity v = Y(omega) F_bridge.
// The moving termination drives each string with the force +g u_p(k).
// Keeping only the resonant (positive-frequency) part of that force, which
// is exact to first order in the weak coupling, gives
//
//   dz/dt = A z,   A = diag(j omega_k - sigma_k) - kappa E^T Y(omega_n) E,
//   kappa = g^2 / (2 m omega_n^2) = (T / L)(1 + B n^2)
//
// with E selecting each oscillator's polarization and m = mu L / 2. A single
// string on a resistive bridge thus decays at kappa Re Y = 2 f0 Z0 Re Y. With
// Y complex symmetric and Re Y positive semi-definite, the Hermitian part of
// A is negative semi-definite: every normal mode decays.
//
// Normal modes. A = V diag(lambda) V^-1 (complex_eigen.h). Each column of V
// is normalised to unit 2-norm and then rotated by a unit phase so that the
// mode's output weight c_j = sum_k o_k V_kj becomes real and non-negative:
// the bank then needs no complex output weight (the phase lives in the
// normal mode's state). Per mode j, for a force F_s on string s along the
// hammer direction (vertical plus strikeHorizontal times horizontal) at the
// strike point (mode shape phi):
//
//   dw_j/dt = lambda_j w_j + input_js F_s,   input_js = phi / (m omega_n) (V^-1 e_s)_j
//   y_s     = sum_j Im(proj_sj w_j),         proj_sj  = phi (e_s^T V)_j
//   output  = sum_j wOut_j Im(w_j)
//
// Near-degenerate eigenvalues of a non-normal A (exceptional points, e.g.
// detuning comparable to the coupling) make V ill-conditioned. If its
// condition number exceeds kMaxCondition, the oscillator frequencies are
// perturbed by a deterministic pattern of growing size (1e-6 .. 1e-3 times
// the coupling strength) until it does not; if that fails the partial falls
// back to uncoupled oscillators (V = I).
#pragma once

#include <complex>

#include "pm_hammer/hammer.h"
#include "pm_voicing/bridge_admittance.h"

namespace pm {

inline constexpr int kMaxOscillators = 2 * kMaxStrings;

struct UnisonPartialSpec {
    int strings = 1;
    double freqHz = 0.0;                                  // nominal partial frequency
    double detuneRatio[kMaxStrings] = {1.0, 1.0, 1.0};    // per-string frequency ratio
    double horizontalRatio = 1.0;                         // horizontal / vertical frequency
    double sigma = 0.0;                                   // internal loss, 1/s
    double kappa = 0.0;                                   // N/m, see above
    Admittance2 Y{};                                      // at freqHz
    double phi = 0.0;                                     // mode shape at the strike point
    double strikeHorizontal = 0.0;                        // horizontal part of the hammer direction
    double modalMass = 1.0;                               // kg
    std::complex<double> outV = 1.0, outH = 0.0;          // output per metre, per oscillator
};

struct UnisonMode {
    std::complex<double> lambda;                // -sigma + j omega, 1/s
    double wOut = 0.0;                          // real, >= 0
    std::complex<double> proj[kMaxStrings];     // strike-point displacement of string s
    std::complex<double> input[kMaxStrings];    // 1/(N s)
    double score = 0.0;                         // wOut * sum_s |input_s|
};

enum class UnisonStatus { Exact, Perturbed, Uncoupled };

struct UnisonPartial {
    static constexpr double kMaxCondition = 1e3;

    int oscillators = 0;  // 2S, also the number of normal modes
    UnisonMode mode[kMaxOscillators];
    // Physical <-> normal coordinates (rotated basis): z = V w, w = Vinv z.
    std::complex<double> V[kMaxOscillators * kMaxOscillators];
    std::complex<double> Vinv[kMaxOscillators * kMaxOscillators];
    std::complex<double> A[kMaxOscillators * kMaxOscillators];  // as decomposed
    double oscFreqHz[kMaxOscillators] = {};
    double oscSigma[kMaxOscillators] = {};
    UnisonStatus status = UnisonStatus::Exact;
    double condition = 1.0;
    double residual = 0.0;
};

// The 2S x 2S system matrix, row-major.
void unisonMatrix(const UnisonPartialSpec& s, std::complex<double>* A) noexcept;

UnisonPartial solveUnisonPartial(const UnisonPartialSpec& s) noexcept;

}  // namespace pm
