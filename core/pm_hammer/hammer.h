// Nonlinear hammer and hammer-string contact solver (plan §1.4).
//
// Felt: Hunt-Crossley, F = K xi^p (1 + mu dxi/dt) for compression xi > 0,
// one felt contact per string. One hammer, a point mass M, strikes the S
// strings of a unison (S <= kMaxStrings): string s is met at its own offset
// o_s (hammer-string mating; una corda moves one offset out of reach), so
// the felt compression at string s is
//     xi_s = x - o_s - y_s,     y_s = displacement of string s at the strike point.
//
// Modes. The solver works directly on the key's normal modes (the coupled
// unison modes of pm_voicing/unison.h), complex amplitudes w_j with
//     dw_j/dt = lambda_j w_j + sum_s input_js F_s,    y_s = sum_j Im(proj_sj w_j).
// With one uncoupled string this reduces to the M2 formulation
// (input = phi / (m omega), proj = phi, real).
//
// Discretization. While a hammer is active, the solver owns its key's modes
// and advances them itself in substeps h = T / R (R = contact oversampling):
//
//  * Modes are propagated exactly (the same exponential as the bank), and
//    the forces are held constant over each substep (zero-order hold) and
//    integrated exactly:
//        w <- C w + sum_s b_s F_s,   C = exp(lambda h),  b_s = input_s (C - 1) / lambda
//    so the strike-point displacements at the end of the substep are
//        y_s = y_s,free + sum_t G_st F_t,   G_st = sum_j Im(proj_sj b_tj)
//    (G is diagonal-dominant: the bridge coupling acts over seconds, a
//    substep over microseconds).
//  * The hammer moves exactly under the same constant total force:
//        x = x + v h - P h^2 / (2M),   v = v - P h / M,   P = sum_s F_s
//  * Each felt force is the discrete gradient of its felt potential
//    Phi(xi) = K xi+^(p+1) / (p+1) between the compressions at the start and
//    end of the substep, times the Hunt-Crossley factor:
//        F_s = max(0, (Phi(xi_s1) - Phi(xi_s0)) / (xi_s1 - xi_s0) * (1 + mu (xi_s1 - xi_s0) / h))
//    with xi_s1 = xi_s,free - (h^2/2M) P - sum_t G_st F_t: S coupled scalar
//    equations, solved by Newton's method in S dimensions (each F_s is
//    nondecreasing in its own compression and the coupling matrix
//    h^2/2M 11^T + G is positive definite, so the root is unique). A
//    bracketed nested solve (outer: total force; inner: the M2 scalar
//    safeguarded Newton per string, ignoring the off-diagonal of G) is the
//    fallback if Newton has not converged after 12 steps; it is counted.
//
// Fused passes. Each substep's force is not applied in a pass of its own:
// it is kept pending and added to the state at the start of the next
// substep's propagation (w <- C (w + b F)), which is the same arithmetic
// with one pass over the modes per substep instead of two. stepHammer()
// leaves the last force pending in the HammerState between calls (the
// engine calls it once per sample); runHammer() and flushHammer() apply it.
// Anything that reads the key's state directly must flush first.
//
// Sympathetic input (soundboard loop). With m.wInRe/wInIm set and an input
// u[k] per base sample, the key's modes also receive wIn u[k] at the end of
// the sample's last substep, as in the bank kernel (z <- c z + wIn u).
//
// Because both sides integrate exactly under the same constant forces, the
// work sum_s F_s dxi_s equals the change of felt potential plus a
// non-negative dissipation term, so the discrete hammer + string energy can
// only fall: the scheme is unconditionally stable for any felt stiffness,
// and with mu = 0 and lossless, uncoupled modes it conserves energy to
// rounding. Accuracy (not stability) sets R: see the convergence test.
//
// Residual compliance. Partials above the bank's limit (0.45 fs) are not
// simulated, but they still make the string softer at the strike point. They
// lie far above the contact's spectrum, so they respond quasi-statically and
// act as a spring of compliance cRes (the static compliance of the omitted
// modes) in series with each felt. The series pair is treated as one
// nonlinear spring in the total compression d = x - o - y, with potential
// Phi(xi_felt) + cRes F^2 / 2, so the scheme above stays energy-exact; the
// Hunt-Crossley factor then uses the rate of the total compression.
//
// The felt power law uses fastPow() (pm_base/fast_math.h, ~1e-13 relative
// error); the energy identity holds exactly for whatever potential that
// evaluates, since force and potential come from the same value.
//
// Everything here except the ContactTables setup is real-time safe.
#pragma once

#include <complex>
#include <cstdint>

#include "pm_base/aligned.h"
#include "pm_string/mode_bank.h"

namespace pm {

inline constexpr int kMaxStrings = 3;

struct HammerParams {
    double mass = 2.97e-3;  // kg, the whole hammer
    double K = 4.5e9;       // N / m^p, per string contact
    double p = 2.5;
    double mu = 0.0;        // s / m, Hunt-Crossley damping
    int strings = 1;
    double offset[kMaxStrings] = {};  // m, string s is reached when x - y_s > offset_s
};

// Read-only description of one key's modes for the solver, plus pointers to
// the bank state it advances. Arrays are 64-byte aligned and count is a
// multiple of the SIMD width (padding lanes are all zero).
struct HammerKeyModes {
    int count = 0;
    int strings = 1;
    float* zr = nullptr;            // bank state (written)
    float* zi = nullptr;
    const float* wOut = nullptr;    // bank output weights
    const float* dampAm1 = nullptr; // bank damper factor - 1, per base sample
    bool damped = false;            // segment damper amount > 0
    const float* dSub = nullptr;    // Re(C) - 1 for one substep
    const float* sSub = nullptr;    // Im(C)
    const float* pRe[kMaxStrings] = {};  // strike-point projection of string s
    const float* pIm[kMaxStrings] = {};
    const float* bRe[kMaxStrings] = {};  // ZOH force weights for string s, one substep
    const float* bIm[kMaxStrings] = {};
    const float* wInRe = nullptr;   // bank input weights (sympathetic input), or null
    const float* wInIm = nullptr;
    double G[kMaxStrings][kMaxStrings] = {};  // metres per newton, see above
    double gOut[kMaxStrings] = {};  // sum wOut Im(b_s): output units per newton
    double cRes = 0.0;              // residual compliance of omitted modes, m/N
    int oversampling = 1;
    double h = 0.0;                 // substep, s
    std::int64_t releaseSubsteps = 0;  // out of contact this long -> finished
    std::int64_t maxSubsteps = 0;      // hard lifetime limit
};

struct HammerState {
    double x = 0.0;       // hammer position, m (same axis as y_s)
    double v = 0.0;       // m/s, positive towards the strings
    // Per string, at the end of the previous substep:
    double xiPrev[kMaxStrings] = {};    // total compression x - o_s - y_s
    double feltPrev[kMaxStrings] = {};  // felt part of it
    double wPrev[kMaxStrings] = {};     // contact potential
    double fPrev[kMaxStrings] = {};     // elastic felt force
    double dxiPrev[kMaxStrings] = {};   // last change of the felt compression
    double dxiPrev2[kMaxStrings] = {};  // the one before
    double yPrev[kMaxStrings] = {};     // y_s
    double fPend[kMaxStrings] = {};     // force of the last substep, not yet applied
    bool pending = false;

    // Statistics, from the last launch.
    double launchVelocity = 0.0;
    double kineticEnergy0 = 0.0;
    double feltLoss = 0.0;       // J dissipated in the felts
    double workOnString = 0.0;   // J of work done by the forces on the strings
    double peakForce = 0.0;      // N, total
    std::int64_t substeps = 0;   // since launch
    std::int64_t firstContact = -1, lastContact = -1;  // substep indices, any string
    std::int64_t contactSubsteps = 0;
    int contacts = 0;            // separate contact episodes (any string touching)
    bool inContact = false;
    int fallbackSolves = 0;      // substeps solved by the nested fallback
};

// Per-mode contact coefficients for every key of a segmented bank, parallel
// to the bank's mode arrays. Non-real-time setup.
class ContactTables {
public:
    bool allocate(int modeCapacity, int segments) noexcept;

    // Mode i of the bank: complex rate lambda = -sigma + j omega (1/s), the
    // strike-point projection proj[s] and the force input input[s] (1/(N s))
    // of each of the key's 'strings' strings, for substep 1 / (fs R).
    void setMode(int i, std::complex<double> lambda, int strings, const std::complex<double>* proj,
                 const std::complex<double>* input, double fs, int oversampling) noexcept;
    // Uncoupled single string (M2 form): proj = phi, input = phi / (m omega).
    void setMode(int i, double freqHz, double sigma, double phi, double modalMass, double fs,
                 int oversampling) noexcept;
    // After all modes of segment s are set (and the bank's output weights):
    // computes G and gOut. residualCompliance: see cRes above. releaseSeconds:
    // how long the hammer must stay off the strings before it is considered
    // gone for good.
    void finishSegment(const ModeBank& bank, int s, int strings, double fs, int oversampling,
                       double residualCompliance, double releaseSeconds,
                       double maxSeconds) noexcept;

    // RT.
    HammerKeyModes view(ModeBank& bank, int s) const noexcept;

private:
    AlignedArray<float> dSub_, sSub_;
    AlignedArray<float> pRe_[kMaxStrings], pIm_[kMaxStrings], bRe_[kMaxStrings], bIm_[kMaxStrings];
    struct Seg {
        double G[kMaxStrings][kMaxStrings] = {};
        double gOut[kMaxStrings] = {};
        double h = 0.0, cRes = 0.0;
        int oversampling = 1, strings = 1;
        std::int64_t release = 0, maxSteps = 0;
    };
    AlignedArray<Seg> seg_;
};

// String s displacement at the strike point, sum_j Im(proj_sj w_j).
double strikeDisplacement(const HammerKeyModes& m, int s = 0) noexcept;

// Starts (or restarts) a strike (flushing a pending force first): the hammer is placed half a substep of
// travel below the first string it will meet, moving up at 'velocity' m/s.
// Strings already vibrating keep their state; the contact sees their true
// motion. The offsets of 'p' are latched by the caller for the strike.
void launchHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m,
                  double velocity) noexcept;

// Advances hammer and the key's modes by n base samples, writing the key's
// output (sum wOut Im w after each sample, like the bank) to out[0..n).
// 'in' (may be null) is the sympathetic input per base sample. runHammer
// leaves the state complete (flushed); stepHammer leaves the last force
// pending.
void runHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, float* out,
               int n, const float* in = nullptr) noexcept;
void stepHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in,
                float* out, int n) noexcept;
// Applies a pending force to the key's modes.
void flushHammer(HammerState& h, const HammerKeyModes& m) noexcept;

// True once the hammer has been off the strings for longer than the key's
// release time (or reached its lifetime limit): the bank may take the modes
// back.
bool hammerFinished(const HammerState& h, const HammerKeyModes& m) noexcept;

// Potential energy stored in one felt contact at total compression
// d = x - o - y: felt K d_f^(p+1) / (p+1) plus residual spring.
double contactPotential(const HammerParams& p, const HammerKeyModes& m, double d) noexcept;

// Energy of an uncoupled mode, (m/2) (qdot^2 + (omega^2 + sigma^2) q^2), from
// its state (Im z = q, Re z = (qdot + sigma q) / omega).
double modalEnergy(double zr, double zi, double freqHz, double sigma, double modalMass) noexcept;

}  // namespace pm
