// Nonlinear hammer and hammer-string contact solver (plan §1.4).
//
// Felt: Hunt-Crossley, F = K xi^p (1 + mu dxi/dt) for compression xi > 0.
// The hammer is a point mass M moving on the same axis as the string's
// displacement at the strike point, y_s = sum_n phi_n q_n.
//
// Discretization. While a hammer is active, the solver owns its key's modes
// and advances them itself in substeps h = T / R (R = contact oversampling):
//
//  * Modes are propagated exactly (the same exponential as the bank), and the
//    force is held constant over each substep (zero-order hold) and
//    integrated exactly:
//        z <- C z + b F,   C = exp(s h),  b = beta (C - 1) / s,
//        s = -sigma + j omega,  beta = phi / (m omega)
//    Unlike the bank's impulse-invariant input (b real, so Im(z) - the
//    displacement - does not respond to the current sample's force at all),
//    Im(b) > 0, so the strike-point displacement at the end of the substep is
//        y_s = y_free + g F,    g = sum phi_n Im(b_n)  (~ h^2 sum phi^2 / 2m)
//  * The hammer moves exactly under the same constant force:
//        x = x + v h - F h^2 / (2M),   v = v - F h / M
//  * The felt force is the discrete gradient of the felt potential
//    Phi(xi) = K xi+^(p+1) / (p+1) between the compressions at the start and
//    end of the substep, times the Hunt-Crossley factor:
//        F = max(0, (Phi(xi1) - Phi(xi0)) / (xi1 - xi0) * (1 + mu (xi1 - xi0) / h))
//    (Phi including the residual spring, see below)
//    with xi1 = xi_free - (h^2/2M + g) F, a scalar equation solved by a
//    safeguarded Newton iteration (its residual is monotonic: unique root).
//
// Because both sides integrate exactly under the same constant force, the
// work F * dxi equals the change of felt potential plus a non-negative
// dissipation term, so the discrete hammer + string energy can only fall:
// the scheme is unconditionally stable for any felt stiffness, and with
// mu = 0 and lossless modes it conserves energy to rounding. Accuracy (not
// stability) sets R: see the convergence test.
//
// Residual compliance. Partials above the bank's limit (0.45 fs) are not
// simulated, but they still make the string softer at the strike point. They
// lie far above the contact's spectrum, so they respond quasi-statically and
// act as a spring of compliance cRes (the static compliance of the omitted
// modes) in series with the felt. This matters in the top two octaves, where
// only 3-10 partials are kept (for C8 they carry about a third of the
// string's static compliance). The series pair is treated as one nonlinear
// spring in the total compression xi = x - sum phi q, with potential
// Phi(xi_felt) + cRes F^2 / 2, so the scheme above stays energy-exact; the
// Hunt-Crossley factor then uses the rate of the total compression.
//
// Everything here except the ContactTables setup is real-time safe.
#pragma once

#include <cstdint>

#include "pm_base/aligned.h"
#include "pm_string/mode_bank.h"

namespace pm {

struct HammerParams {
    double mass = 2.97e-3;  // kg
    double K = 4.5e9;       // N / m^p
    double p = 2.5;
    double mu = 0.0;        // s / m, Hunt-Crossley damping
};

// Read-only description of one key's modes for the solver, plus pointers to
// the bank state it advances. Arrays are 64-byte aligned and count is a
// multiple of the SIMD width (padding lanes are all zero).
struct HammerKeyModes {
    int count = 0;
    float* zr = nullptr;            // bank state (written)
    float* zi = nullptr;
    const float* wOut = nullptr;    // bank output weights
    const float* dampAm1 = nullptr; // bank damper factor - 1, per base sample
    bool damped = false;            // segment damper amount > 0
    const float* dSub = nullptr;    // Re(C) - 1 for one substep
    const float* sSub = nullptr;    // Im(C)
    const float* bRe = nullptr;     // ZOH force weights for one substep
    const float* bIm = nullptr;
    const float* phi = nullptr;     // mode shape at the strike point
    double g = 0.0;                 // sum phi Im(b): metres per newton
    double cRes = 0.0;              // residual compliance of omitted modes, m/N
    double gOut = 0.0;              // sum wOut Im(b): output units per newton
    int oversampling = 1;
    double h = 0.0;                 // substep, s
    std::int64_t releaseSubsteps = 0;  // out of contact this long -> finished
    std::int64_t maxSubsteps = 0;      // hard lifetime limit
};

struct HammerState {
    double x = 0.0;       // hammer position, m (same axis as y_s)
    double v = 0.0;       // m/s, positive towards the string
    double xiPrev = 0.0;  // total compression x - y_s at the end of the previous substep
    double feltPrev = 0.0;  // felt part of it
    double wPrev = 0.0;   // contact potential there
    double fPrev = 0.0;   // elastic felt force there
    double dxiPrev = 0.0;
    double yPrev = 0.0;   // y_s at the end of the previous substep

    // Statistics, from the last launch.
    double launchVelocity = 0.0;
    double kineticEnergy0 = 0.0;
    double feltLoss = 0.0;       // J dissipated in the felt
    double workOnString = 0.0;   // J of work done by the force on the string
    double peakForce = 0.0;      // N
    std::int64_t substeps = 0;   // since launch
    std::int64_t firstContact = -1, lastContact = -1;  // substep indices
    std::int64_t contactSubsteps = 0;
    int contacts = 0;            // separate contact episodes
    bool inContact = false;
};

// Per-mode contact coefficients for every key of a segmented bank, parallel
// to the bank's mode arrays. Non-real-time setup.
class ContactTables {
public:
    bool allocate(int modeCapacity, int segments) noexcept;

    // Mode i of the bank: free frequency and decay, mode shape at the strike
    // point, modal mass, for substep h = 1 / (fs * oversampling).
    void setMode(int i, double freqHz, double sigma, double phi, double modalMass, double fs,
                 int oversampling) noexcept;
    // After all modes of segment s are set (and the bank's output weights):
    // computes g and gOut. residualCompliance: see cRes above. releaseSeconds:
    // how long the hammer must stay off the string before it is considered
    // gone for good.
    void finishSegment(const ModeBank& bank, int s, double fs, int oversampling,
                       double residualCompliance, double releaseSeconds,
                       double maxSeconds) noexcept;

    // RT.
    HammerKeyModes view(ModeBank& bank, int s) const noexcept;

private:
    AlignedArray<float> dSub_, sSub_, bRe_, bIm_, phi_;
    struct Seg {
        double g = 0.0, gOut = 0.0, h = 0.0, cRes = 0.0;
        int oversampling = 1;
        std::int64_t release = 0, maxSteps = 0;
    };
    AlignedArray<Seg> seg_;
};

// String displacement at the strike point, sum phi_n Im(z_n).
double strikeDisplacement(const HammerKeyModes& m) noexcept;

// Starts (or restarts) a strike: the hammer is placed half a substep of
// travel below the string's current strike-point displacement, moving up at
// 'velocity' m/s. A string already vibrating keeps its state; the contact
// sees its true motion.
void launchHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m,
                  double velocity) noexcept;

// Advances hammer and the key's modes by n base samples, writing the key's
// output (sum wOut Im z after each sample, like the bank) to out[0..n).
void runHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, float* out,
               int n) noexcept;

// True once the hammer has been off the string for longer than the key's
// release time (or reached its lifetime limit): the bank may take the modes
// back.
bool hammerFinished(const HammerState& h, const HammerKeyModes& m) noexcept;

// Potential energy stored in the contact at total compression
// xi = x - sum phi q: felt K xi_f^(p+1) / (p+1) plus residual spring.
double contactPotential(const HammerParams& p, const HammerKeyModes& m, double xi) noexcept;

// Energy of mode i, (m/2) (qdot^2 + (omega^2 + sigma^2) q^2), from its state
// (Im z = q, Re z = (qdot + sigma q) / omega).
double modalEnergy(double zr, double zi, double freqHz, double sigma, double modalMass) noexcept;

}  // namespace pm
