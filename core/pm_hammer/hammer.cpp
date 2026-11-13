#include "pm_hammer/hammer.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

#include "pm_base/fast_math.h"
#include "pm_base/simd.h"

namespace pm {

namespace {

constexpr double kPi = std::numbers::pi;
using cd = std::complex<double>;

// Relative tolerance of the contact solves (on the compression). A residual
// of 1e-8 of the compression changes the force by at most about 3e-8 of
// itself, below the float rounding (6e-8) with which the force enters the
// modal state; the energy identity is then off by about 1e-8 of the contact
// energy per substep, far below what the tests resolve (1e-6). Rendered
// output is the same as with 1e-10 to within float rounding (-130 dB rms re
// peak against the previous solver; 1e-10 costs about 20 % more Newton
// evaluations at ff, each S pow calls).
constexpr double kSolveTol = 1e-8;


// Sample-path record of one block of simd::kWidth modes, in vectors:
//   B_tq (re, im) for t < S, q < R    2SR   pending forces
//   wIn (re, im)                      2     pending input
//   P_sr (re, im) for s < S, r < R    2SR   projections
//   C^R (Re - 1, Im)                  2     rotation over the sample
//   wOut                              1     output
constexpr int recVectors(int S, int R) noexcept { return 4 * S * R + 5; }
constexpr int recB(int k) noexcept { return 2 * k; }
constexpr int recIn(int S, int R) noexcept { return 2 * S * R; }
constexpr int recP(int S, int R, int k) noexcept { return 2 * S * R + 2 + 2 * k; }
constexpr int recC(int S, int R) noexcept { return 4 * S * R + 2; }
constexpr int recOut(int S, int R) noexcept { return 4 * S * R + 4; }

// ------------------------------------------------------------ substep path

// One substep: the pending force of the previous substep (if Pend) is added
// and all modes are propagated, w <- C (w + sum_s b_s F_s); with In the
// sympathetic input wIn u is added after the rotation. Returns
// y_s = sum Im(proj_s w) for every string and (if wantOut) sum wOut Im w.
// Same dr form and operation order as the bank kernel.
template <int S, bool Pend, bool In>
inline void propagate(const HammerKeyModes& m, const double* force, float u, bool wantOut, float* ySum,
                      float& oSum) noexcept {
    using namespace simd;
    vf accY[S], accO = zero();
    [[maybe_unused]] vf f[S];
    for (int s = 0; s < S; ++s) {
        accY[s] = zero();
        if constexpr (Pend) f[s] = set1(static_cast<float>(force[s]));
    }
    [[maybe_unused]] const vf uu = set1(u);
    for (int i = 0; i < m.count; i += kWidth) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        if constexpr (Pend) {
            for (int s = 0; s < S; ++s) {
                zr = fmadd(load(m.bRe[s] + i), f[s], zr);
                zi = fmadd(load(m.bIm[s] + i), f[s], zi);
            }
        }
        const vf d = load(m.dSub + i), c = load(m.sSub + i);
        vf nr = fnmadd(c, zi, fmadd(d, zr, zr));
        vf ni = fmadd(c, zr, fmadd(d, zi, zi));
        if constexpr (In) {
            nr = fmadd(load(m.wInRe + i), uu, nr);
            ni = fmadd(load(m.wInIm + i), uu, ni);
        }
        store(m.zr + i, nr);
        store(m.zi + i, ni);
        for (int s = 0; s < S; ++s)
            accY[s] = fmadd(load(m.pRe[s] + i), ni, fmadd(load(m.pIm[s] + i), nr, accY[s]));
        if (wantOut) accO = fmadd(load(m.wOut + i), ni, accO);
    }
    for (int s = 0; s < S; ++s) ySum[s] = hsum(accY[s]);
    oSum = wantOut ? hsum(accO) : 0.0f;
}

template <int S>
inline void applyForce(const HammerKeyModes& m, const double* force) noexcept {
    using namespace simd;
    vf f[S];
    for (int s = 0; s < S; ++s) f[s] = set1(static_cast<float>(force[s]));
    for (int i = 0; i < m.count; i += kWidth) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        for (int s = 0; s < S; ++s) {
            zr = fmadd(load(m.bRe[s] + i), f[s], zr);
            zi = fmadd(load(m.bIm[s] + i), f[s], zi);
        }
        store(m.zr + i, zr);
        store(m.zi + i, zi);
    }
}

template <int S>
inline void propagateAny(const HammerKeyModes& m, bool pend, const double* force, bool in, float u,
                         bool wantOut, float* ySum, float& oSum) noexcept {
    if (pend) {
        if (in) propagate<S, true, true>(m, force, u, wantOut, ySum, oSum);
        else propagate<S, true, false>(m, force, u, wantOut, ySum, oSum);
    } else {
        if (in) propagate<S, false, true>(m, force, u, wantOut, ySum, oSum);
        else propagate<S, false, false>(m, force, u, wantOut, ySum, oSum);
    }
}

// Damper loss for one base sample (c_eff = c a commutes with the rotation),
// after adding a pending force; returns the new strike-point displacements.
template <int S>
inline void applyDamper(const HammerKeyModes& m, bool pend, const double* force, double* y) noexcept {
    using namespace simd;
    vf accY[S], f[S];
    for (int s = 0; s < S; ++s) {
        accY[s] = zero();
        f[s] = set1(pend ? static_cast<float>(force[s]) : 0.0f);
    }
    for (int i = 0; i < m.count; i += kWidth) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        if (pend) {
            for (int s = 0; s < S; ++s) {
                zr = fmadd(load(m.bRe[s] + i), f[s], zr);
                zi = fmadd(load(m.bIm[s] + i), f[s], zi);
            }
        }
        // z + (a - 1) z, as the bank applies the damper (dr form): a = 1 + am1
        // rounded to float would misstate a damper rate of 1e-4 per sample
        // by about 1e-3 of itself.
        const vf am1 = load(m.dampAm1 + i);
        zr = fmadd(am1, zr, zr);
        zi = fmadd(am1, zi, zi);
        store(m.zr + i, zr);
        store(m.zi + i, zi);
        for (int s = 0; s < S; ++s)
            accY[s] = fmadd(load(m.pRe[s] + i), zi, fmadd(load(m.pIm[s] + i), zr, accY[s]));
    }
    for (int s = 0; s < S; ++s) y[s] = hsum(accY[s]);
}

// ------------------------------------------------------------- sample path

// One pass per base sample (see hammer.h). Pend: 0 nothing pending, 1 the
// input only, 2 forces and input. fp[t * R + q]: pending forces; Y[s * R +
// r]: free strike-point displacements at the end of substep r; oFree: the
// output of the free state at the end of the sample.
template <int S, int R, int Pend>
inline void samplePass(const HammerKeyModes& m, const float* fp, float u, float* Y, float& oFree) noexcept {
    using namespace simd;
    constexpr int SR = S * R, W = kWidth, stride = recVectors(S, R) * W;
    vf acc[SR], accO = zero();
    [[maybe_unused]] vf f[SR];
    for (int k = 0; k < SR; ++k) {
        acc[k] = zero();
        if constexpr (Pend == 2) f[k] = set1(fp[k]);
    }
    [[maybe_unused]] const vf uu = set1(u);
    const float* rec = m.rec;
    for (int i = 0; i < m.count; i += W, rec += stride) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        if constexpr (Pend == 2) {
            for (int k = 0; k < SR; ++k) {
                zr = fmadd(load(rec + recB(k) * W), f[k], zr);
                zi = fmadd(load(rec + (recB(k) + 1) * W), f[k], zi);
            }
        }
        if constexpr (Pend >= 1) {
            zr = fmadd(load(rec + recIn(S, R) * W), uu, zr);
            zi = fmadd(load(rec + (recIn(S, R) + 1) * W), uu, zi);
        }
        for (int k = 0; k < SR; ++k)
            acc[k] = fmadd(load(rec + recP(S, R, k) * W), zi, fmadd(load(rec + (recP(S, R, k) + 1) * W), zr, acc[k]));
        const vf d = load(rec + recC(S, R) * W), c = load(rec + (recC(S, R) + 1) * W);
        const vf nr = fnmadd(c, zi, fmadd(d, zr, zr));
        const vf ni = fmadd(c, zr, fmadd(d, zi, zi));
        store(m.zr + i, nr);
        store(m.zi + i, ni);
        accO = fmadd(load(rec + recOut(S, R) * W), ni, accO);
    }
    for (int k = 0; k < SR; ++k) Y[k] = hsum(acc[k]);
    oFree = hsum(accO);
}

// Adds the pending forces (if forces) and input to the state.
template <int S, int R>
inline void flushSample(const HammerKeyModes& m, const float* fp, float u, bool forces) noexcept {
    using namespace simd;
    constexpr int SR = S * R, W = kWidth, stride = recVectors(S, R) * W;
    vf f[SR];
    for (int k = 0; k < SR; ++k) f[k] = set1(forces ? fp[k] : 0.0f);
    const vf uu = set1(u);
    const float* rec = m.rec;
    for (int i = 0; i < m.count; i += W, rec += stride) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        if (forces) {
            for (int k = 0; k < SR; ++k) {
                zr = fmadd(load(rec + recB(k) * W), f[k], zr);
                zi = fmadd(load(rec + (recB(k) + 1) * W), f[k], zi);
            }
        }
        zr = fmadd(load(rec + recIn(S, R) * W), uu, zr);
        zi = fmadd(load(rec + (recIn(S, R) + 1) * W), uu, zi);
        store(m.zr + i, zr);
        store(m.zi + i, zi);
    }
}

// The hammer has cleared the strings: the modes advance by one base sample
// without projections (Pend: 0 nothing pending, 1 the input).
template <int S, int R, int Pend>
inline void freePass(const HammerKeyModes& m, float u, float& oFree) noexcept {
    using namespace simd;
    constexpr int W = kWidth, stride = recVectors(S, R) * W;
    vf accO = zero();
    [[maybe_unused]] const vf uu = set1(u);
    const float* rec = m.rec;
    for (int i = 0; i < m.count; i += W, rec += stride) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        if constexpr (Pend >= 1) {
            zr = fmadd(load(rec + recIn(S, R) * W), uu, zr);
            zi = fmadd(load(rec + (recIn(S, R) + 1) * W), uu, zi);
        }
        const vf d = load(rec + recC(S, R) * W), c = load(rec + (recC(S, R) + 1) * W);
        const vf nr = fnmadd(c, zi, fmadd(d, zr, zr));
        const vf ni = fmadd(c, zr, fmadd(d, zi, zi));
        store(m.zr + i, nr);
        store(m.zi + i, ni);
        accO = fmadd(load(rec + recOut(S, R) * W), ni, accO);
    }
    oFree = hsum(accO);
}

// Bound of each string's strike-point displacement from the current state,
// sum (|Re p| + |Im p|)(|Re z| + |Im z|) (scalar: a rare check).
template <int S>
inline void strikeBound(const HammerKeyModes& m, double* A) noexcept {
    for (int s = 0; s < S; ++s) {
        double a = 0.0;
        for (int i = 0; i < m.count; ++i)
            a += (std::fabs(double(m.pRe[s][i])) + std::fabs(double(m.pIm[s][i]))) *
                 (std::fabs(double(m.zr[i])) + std::fabs(double(m.zi[i])));
        A[s] = a;
    }
}

// --------------------------------------------------------------- the felt

// Felt in series with the residual string compliance. With e the felt
// compression, the total compression (hammer to modal string displacement)
// is d(e) = e + cRes F(e), and the contact potential is
// W = Phi(e) + cRes F^2 / 2, so that dW/dd = F. Everything is explicit in e,
// which is therefore the unknown of the solve (one pow per evaluation, and
// no division: F = (K e^(p-1)) e, dF/de = p K e^(p-1)).
struct Felt {
    double K, p, mu, invH, cRes;
    double pm1, invP1;  // p - 1, 1 / (p + 1)

    Felt(double K_, double p_, double mu_, double invH_, double cRes_) noexcept
        : K(K_), p(p_), mu(mu_), invH(invH_), cRes(cRes_), pm1(p_ - 1.0), invP1(1.0 / (p_ + 1.0)) {}
    Felt(double K_, double p_, double mu_, double invH_, double cRes_, double pm1_, double invP1_) noexcept
        : K(K_), p(p_), mu(mu_), invH(invH_), cRes(cRes_), pm1(pm1_), invP1(invP1_) {}

    struct Point {
        double e, d, W, F;
        double dD, dF;  // dd/de, dF/de
    };

    Point at(double e) const noexcept {
        if (e <= 0.0) return {e, e, 0.0, 0.0, 1.0, 0.0};
        const double g = K * fastPow(e, pm1);
        const double F = g * e, dF = p * g;
        return {e, e + cRes * F, F * e * invP1 + 0.5 * cRes * F * F, F, 1.0 + cRes * dF, dF};
    }
    // Inverse of d(e), for the rare case where the total compression changes
    // outside the solve (damper scaling during contact). Convex and
    // increasing: Newton from e = d converges monotonically.
    Point fromTotal(double d) const noexcept {
        if (d <= 0.0 || cRes == 0.0) return at(d);
        double e = d;
        for (int it = 0; it < 60; ++it) {
            const Point q = at(e);
            const double r = q.d - d;
            e = std::max(0.0, e - r / q.dD);
            if (std::fabs(r) <= 1e-14 * d) break;
        }
        return at(e);
    }
};

// Force over one substep from the end point q1 and start point q0: discrete
// gradient of W between the total compressions, times the Hunt-Crossley
// factor; dFdE is its derivative with respect to the felt compression q1.e.
inline double substepForce(const Felt& f, const Felt::Point& q1, const Felt::Point& q0,
                           double& dFdE) noexcept {
    const double dd = q1.d - q0.d;
    const double scale = std::max(std::fabs(q1.d), std::fabs(q0.d));
    double G, dG;  // dG: with respect to q1.d
    if (std::fabs(dd) <= 1e-7 * scale) {
        G = 0.5 * (q1.F + q0.F);  // midpoint, O(dd^2)
        dG = 0.5 * q1.dF / q1.dD;
    } else {
        const double inv = 1.0 / dd;
        G = (q1.W - q0.W) * inv;
        dG = (q1.F - G) * inv;
    }
    const double hc = 1.0 + f.mu * dd * f.invH;
    const double F = G * hc;
    if (F <= 0.0) {
        dFdE = 0.0;
        return 0.0;
    }
    dFdE = (dG * hc + G * f.mu * f.invH) * q1.dD;
    return F;
}

// One string, scalar, bracketed: solves r(e) = d(e) - dFree + A F(e) = 0 for
// the end-of-substep felt compression e. r is increasing in e (W is convex in
// d, d increasing in e), so the root is unique; it lies in
// [min(dFree - A F(dFree), 0), dFree]. The fallback of the Newton solve.
inline Felt::Point solveContact(const Felt& felt, double dFree, const Felt::Point& q0, double A,
                                double guess, double& Fout) noexcept {
    double dF;
    const Felt::Point qHi = felt.at(dFree);
    const double Fhi = substepForce(felt, qHi, q0, dF);
    if (Fhi <= 0.0) {
        Fout = 0.0;
        return qHi;
    }
    double lo = std::min(dFree - A * Fhi, 0.0), hi = dFree;
    const double tol = kSolveTol * std::max(std::fabs(dFree), std::fabs(q0.d));
    double e = std::clamp(guess, lo, hi);
    Felt::Point q = felt.at(e);
    for (int it = 0; it < 60; ++it) {
        const double F = substepForce(felt, q, q0, dF);
        const double r = q.d - dFree + A * F;
        if (std::fabs(r) <= tol) break;
        if (r > 0.0) hi = e;
        else lo = e;
        double next = e - r / (q.dD + A * dF);
        if (!(next >= lo && next <= hi)) next = 0.5 * (lo + hi);  // safeguard
        const bool done = std::fabs(next - e) <= 1e-3 * kSolveTol * std::fabs(e);
        e = next;
        q = felt.at(e);
        if (done) break;
    }
    Fout = substepForce(felt, q, q0, dF);
    return q;
}

// Solves J x = r in place for S <= 3 (Cramer's rule: one division). The
// contact Jacobian has a positive diagonal that dominates its columns
// (G_ss >> |G_st|, h^2/2M), so no pivoting is needed. Returns false if J is
// singular or not finite.
template <int S>
inline bool solveSmall(const double (&J)[S][S], double (&r)[S]) noexcept {
    if constexpr (S == 1) {
        if (!(std::fabs(J[0][0]) > 0.0)) return false;
        r[0] /= J[0][0];
        return std::isfinite(r[0]);
    } else if constexpr (S == 2) {
        const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        if (!(std::fabs(det) > 0.0)) return false;
        const double inv = 1.0 / det;
        const double x0 = (r[0] * J[1][1] - r[1] * J[0][1]) * inv;
        const double x1 = (J[0][0] * r[1] - J[1][0] * r[0]) * inv;
        r[0] = x0;
        r[1] = x1;
        return std::isfinite(x0) && std::isfinite(x1);
    } else {
        const double c00 = J[1][1] * J[2][2] - J[1][2] * J[2][1];
        const double c01 = J[1][2] * J[2][0] - J[1][0] * J[2][2];
        const double c02 = J[1][0] * J[2][1] - J[1][1] * J[2][0];
        const double det = J[0][0] * c00 + J[0][1] * c01 + J[0][2] * c02;
        if (!(std::fabs(det) > 0.0)) return false;
        const double inv = 1.0 / det;
        const double c10 = J[0][2] * J[2][1] - J[0][1] * J[2][2];
        const double c11 = J[0][0] * J[2][2] - J[0][2] * J[2][0];
        const double c12 = J[0][1] * J[2][0] - J[0][0] * J[2][1];
        const double c20 = J[0][1] * J[1][2] - J[0][2] * J[1][1];
        const double c21 = J[0][2] * J[1][0] - J[0][0] * J[1][2];
        const double c22 = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        const double x0 = (c00 * r[0] + c10 * r[1] + c20 * r[2]) * inv;
        const double x1 = (c01 * r[0] + c11 * r[1] + c21 * r[2]) * inv;
        const double x2 = (c02 * r[0] + c12 * r[1] + c22 * r[2]) * inv;
        r[0] = x0;
        r[1] = x1;
        r[2] = x2;
        return std::isfinite(x0) && std::isfinite(x1) && std::isfinite(x2);
    }
}

// x^q from x0^q (g0 = x0^q, accurate) for x close to x0: the binomial series
// of (1 + t)^q, t = x / x0 - 1, five terms (relative error below 1e-16 for
// |t| <= 1e-3; larger steps use fastPow). The Newton iterates of one substep
// differ by far less than that, so only the first evaluation of a substep
// needs the table-driven power.
inline bool powNear(double x, double x0, double invX0, double g0, double q, double& out) noexcept {
    const double t = (x - x0) * invX0;
    if (!(std::fabs(t) <= 1e-3)) return false;
    const double c2 = q * (q - 1.0) * 0.5, c3 = c2 * (q - 2.0) * (1.0 / 3.0), c4 = c3 * (q - 3.0) * 0.25,
                 c5 = c4 * (q - 4.0) * 0.2;
    out = g0 * (1.0 + t * (q + t * (c2 + t * (c3 + t * (c4 + t * c5)))));
    return true;
}

// S strings: r_s(e) = d_s(e_s) - dFree_s + Ah sum_t F_t + sum_t G_st F_t = 0,
// Newton's method; the bracketed nested solve if it has not converged after
// 12 steps. Evaluations work on plain per-string arrays (no Point copies) and
// reuse the previous iterate's power (powNear).
template <int S>
inline void solveContacts(const Felt& felt, const double* dFree, const Felt::Point* q0, double Ah,
                          const double (&G)[kMaxStrings][kMaxStrings], const double* guess,
                          Felt::Point* q1, double* F, double* jDiag, int& fallbacks,
                          std::int64_t& evals) noexcept {
    double scale = 0.0;
    for (int s = 0; s < S; ++s) scale = std::max(scale, std::max(std::fabs(dFree[s]), std::fabs(q0[s].d)));
    const double tol = kSolveTol * scale;
    const double muInvH = felt.mu * felt.invH;
    double e[S], dF[S], r[S];
    double gPrev[S] = {}, ePrev[S] = {}, invEPrev[S] = {};  // last evaluated power K e^(p-1) and its point
    for (int s = 0; s < S; ++s) {
        e[s] = std::min(guess[s], dFree[s]);
        ePrev[s] = 0.0;
    }
    for (int it = 0; it < 12; ++it) {
        ++evals;
        double P = 0.0;
        for (int s = 0; s < S; ++s) {
            const double es = e[s];
            Felt::Point& q = q1[s];
            q.e = es;
            if (es <= 0.0) {
                q.d = es;
                q.W = q.F = q.dF = 0.0;
                q.dD = 1.0;
            } else {
                double g;
                if (!(ePrev[s] > 0.0 && powNear(es, ePrev[s], invEPrev[s], gPrev[s], felt.pm1, g))) {
                    g = felt.K * fastPow(es, felt.pm1);
                    ePrev[s] = es;
                    invEPrev[s] = 1.0 / es;
                    gPrev[s] = g;
                }
                const double f = g * es, cf = felt.cRes * f;
                q.F = f;
                q.dF = felt.p * g;
                q.d = es + cf;
                q.W = f * es * felt.invP1 + 0.5 * cf * f;
                q.dD = 1.0 + felt.cRes * q.dF;
            }
            // Discrete gradient of W between the total compressions, times
            // the Hunt-Crossley factor (substepForce, inlined).
            const Felt::Point& o = q0[s];
            const double dd = q.d - o.d;
            double Gd, dG;
            if (std::fabs(dd) <= 1e-7 * std::max(std::fabs(q.d), std::fabs(o.d))) {
                Gd = 0.5 * (q.F + o.F);
                dG = 0.5 * q.dF / q.dD;
            } else {
                const double inv = 1.0 / dd;
                Gd = (q.W - o.W) * inv;
                dG = (q.F - Gd) * inv;
            }
            const double hc = 1.0 + muInvH * dd;
            const double fs = Gd * hc;
            if (fs > 0.0) {
                F[s] = fs;
                dF[s] = (dG * hc + Gd * muInvH) * q.dD;
            } else {
                F[s] = 0.0;
                dF[s] = 0.0;
            }
            P += F[s];
        }
        double worst = 0.0;
        for (int s = 0; s < S; ++s) {
            double g = Ah * P;
            for (int t = 0; t < S; ++t) g += G[s][t] * F[t];
            r[s] = q1[s].d - dFree[s] + g;
            worst = std::max(worst, std::fabs(r[s]));
        }
        if (worst <= tol) return;
        double J[S][S];
        for (int s = 0; s < S; ++s)
            for (int t = 0; t < S; ++t) J[s][t] = (Ah + G[s][t]) * dF[t];
        for (int s = 0; s < S; ++s) {
            J[s][s] += q1[s].dD;
            jDiag[s] = J[s][s];
        }
        if (!solveSmall<S>(J, r)) break;
        for (int s = 0; s < S; ++s) e[s] -= r[s];
    }

    // Fallback: bracketed solve for the total force P, each string solved
    // by the scalar method for a given P.
    ++fallbacks;
    auto forcesFor = [&](double P) {
        double sum = 0.0;
        for (int s = 0; s < S; ++s) {
            q1[s] = solveContact(felt, dFree[s] - Ah * P, q0[s], G[s][s], q1[s].e, F[s]);
            sum += F[s];
        }
        return sum;
    };
    double lo = 0.0, hi = forcesFor(0.0);
    if (hi <= 0.0) return;
    for (int it = 0; it < 200; ++it) {
        const double mid = 0.5 * (lo + hi);
        const double phi = mid - forcesFor(mid);
        if (phi > 0.0) hi = mid;
        else lo = mid;
        if (hi - lo <= 1e-14 * hi) break;
    }
    forcesFor(0.5 * (lo + hi));
}

// The hammer and felt side of a substep, shared by both paths: given the
// strings' free strike-point displacements at the end of the substep, solve
// the contact, move the hammer, keep the statistics.
template <int S>
struct ContactStep {
    Felt felt;
    double invM, Ah, h;
    Felt::Point q0[S];

    ContactStep(const HammerState& st, const HammerParams& p, const HammerKeyModes& m) noexcept
        : felt(p.K, p.p, p.mu, m.invH, m.cRes, st.feltPm1, st.feltInvP1), invM(st.invMass),
          Ah(st.halfH2InvMass), h(m.h) {
        for (int s = 0; s < S; ++s) q0[s] = {st.feltPrev[s], st.xiPrev[s], st.wPrev[s], st.fPrev[s], 1.0, 0.0};
    }
    void save(HammerState& st) const noexcept {
        for (int s = 0; s < S; ++s) {
            st.xiPrev[s] = q0[s].d;
            st.feltPrev[s] = q0[s].e;
            st.wPrev[s] = q0[s].W;
            st.fPrev[s] = q0[s].F;
        }
    }
    // Damper scaling moved the strings by y - yPrev outside the solve.
    void damped(HammerState& st, const double* y) noexcept {
        for (int s = 0; s < S; ++s) {
            q0[s] = felt.fromTotal(q0[s].d + st.yPrev[s] - y[s]);
            st.yPrev[s] = y[s];
        }
    }

    // Returns the total force; F[s] the force on string s.
    double run(HammerState& st, const HammerParams& p, const double (&G)[kMaxStrings][kMaxStrings],
               const double* yfree, double* F) noexcept {
        const double xt = st.x + st.v * h;
        double dFree[S];
        bool touching = false;
        for (int s = 0; s < S; ++s) {
            dFree[s] = xt - p.offset[s] - yfree[s];
            touching = touching || dFree[s] > 0.0 || q0[s].d > 0.0;
        }
        Felt::Point q1[S];
        double P = 0.0;
        if (touching) {
            // Predictor: quadratic extrapolation of the felt compression,
            // corrected by the part of the free compression that its own
            // quadratic extrapolation misses (the strings' reflected waves)
            // times de/d(free compression) of the last solve. With it the
            // first Newton step nearly always meets the tolerance (about 2.1
            // evaluations per substep instead of 2.6).
            double guess[S], jDiag[S];
            for (int s = 0; s < S; ++s) {
                guess[s] = q0[s].e + 2.0 * st.dxiPrev[s] - st.dxiPrev2[s];
                jDiag[s] = 0.0;
                if (st.history >= 3 && st.sensPrev[s] > 0.0)
                    guess[s] += (dFree[s] - (st.dFreePrev[s] + 2.0 * st.dDFreePrev[s] - st.dDFreePrev2[s])) *
                                st.sensPrev[s];
            }
            solveContacts<S>(felt, dFree, q0, Ah, G, guess, q1, F, jDiag, st.fallbackSolves, st.newtonEvals);
            for (int s = 0; s < S; ++s) {
                P += F[s];
                st.sensPrev[s] = jDiag[s] > 0.0 ? 1.0 / jDiag[s] : 0.0;
            }
            ++st.history;
        } else {
            st.history = 0;
            for (int s = 0; s < S; ++s) {
                F[s] = 0.0;
                q1[s] = felt.at(dFree[s]);
            }
        }
        for (int s = 0; s < S; ++s) {
            st.dDFreePrev2[s] = st.dDFreePrev[s];
            st.dDFreePrev[s] = dFree[s] - st.dFreePrev[s];
            st.dFreePrev[s] = dFree[s];
        }
        st.x = xt - Ah * P;
        st.v -= h * P * invM;
        for (int s = 0; s < S; ++s) {
            double y = yfree[s];
            for (int t = 0; t < S; ++t) y += G[s][t] * F[t];
            if (F[s] > 0.0) {
                st.workOnString += F[s] * (y - st.yPrev[s]);
                st.feltLoss += F[s] * (q1[s].d - q0[s].d) - (q1[s].W - q0[s].W);
            }
            st.yPrev[s] = y;
            st.dxiPrev2[s] = st.dxiPrev[s];
            st.dxiPrev[s] = q1[s].e - q0[s].e;
            q0[s] = q1[s];
        }
        if (P > 0.0) {
            st.peakForce = std::max(st.peakForce, P);
            if (!st.inContact) {
                ++st.contacts;
                if (st.firstContact < 0) st.firstContact = st.substeps;
            }
            st.inContact = true;
            st.lastContact = st.substeps;
            ++st.contactSubsteps;
        } else {
            st.inContact = false;
        }
        ++st.substeps;
        return P;
    }
};

template <int S>
void runSubstepT(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in, float* out,
                 int n) noexcept {
    ContactStep<S> cs(h, p, m);
    const int R = m.oversampling;
    const bool hasIn = in != nullptr && m.wInRe != nullptr;
    for (int k = 0; k < n; ++k) {
        if (m.damped) {
            double y[S];
            applyDamper<S>(m, h.pending, h.fPend, y);
            h.pending = false;
            cs.damped(h, y);
        }
        float ofree = 0.0f;
        double F[S] = {};
        for (int r = 0; r < R; ++r) {
            const bool last = r == R - 1;
            float yf[S], of;
            const bool inNow = last && hasIn && in[k] != 0.0f;
            propagateAny<S>(m, h.pending, h.fPend, inNow, inNow ? in[k] : 0.0f, last, yf, of);
            h.pending = false;
            if (last) ofree = of;
            double yfree[S];
            for (int s = 0; s < S; ++s) yfree[s] = yf[s];
            if (cs.run(h, p, m.G, yfree, F) > 0.0) {
                for (int s = 0; s < S; ++s) h.fPend[s] = F[s];
                h.pending = true;
            }
        }
        double o = ofree;
        for (int s = 0; s < S; ++s) o += m.gOut[s] * F[s];
        out[k] = static_cast<float>(o);
    }
    cs.save(h);
}

template <int S, int R>
void flushSampleAny(HammerState& h, const HammerKeyModes& m) noexcept {
    flushSample<S, R>(m, h.fPendS, h.uPend, h.pendForces);
    h.pending = false;
}

template <int S, int R>
void runSampleT(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in, float* out,
                int n) noexcept {
    ContactStep<S> cs(h, p, m);
    const SampleResponse& sr = *m.resp;
    double G[kMaxStrings][kMaxStrings] = {};
    for (int s = 0; s < S; ++s)
        for (int t = 0; t < S; ++t) G[s][t] = sr.h[s][t][0];
    for (int k = 0; k < n; ++k) {
        if (m.damped) {
            if (h.pending) flushSampleAny<S, R>(h, m);
            double y[S];
            applyDamper<S>(m, false, nullptr, y);
            cs.damped(h, y);
        }
        if (!h.cleared && h.history == 0 && !h.inContact && !(h.pending && h.pendForces) && h.v <= 0.0 && h.lastContact >= 0 &&
            h.substeps >= h.clearCheckAt) {
            h.clearCheckAt = h.substeps + 16 * R;
            double A[S];
            strikeBound<S>(m, A);
            bool clear = true;
            for (int s = 0; s < S; ++s) {
                const double a = A[s] + (h.pending ? sr.pAbsIn[s] * std::fabs(double(h.uPend)) : 0.0);
                clear = clear && h.x - p.offset[s] + 2.0 * a + 1e-6 < 0.0;
            }
            h.cleared = clear;
        }
        if (h.cleared) {
            float oFree;
            if (h.pending) freePass<S, R, 1>(m, h.uPend, oFree);
            else freePass<S, R, 0>(m, 0.0f, oFree);
            h.pending = false;
            const float u = in != nullptr ? in[k] : 0.0f;
            h.x += h.v * (R * m.h);
            h.substeps += R;
            out[k] = static_cast<float>(static_cast<double>(oFree) + sr.gIn * u);
            if (u != 0.0f) {
                h.pending = true;
                h.pendForces = false;
                h.uPend = u;
            }
            continue;
        }
        float Y[S * R], oFree;
        if (!h.pending) samplePass<S, R, 0>(m, h.fPendS, 0.0f, Y, oFree);
        else if (!h.pendForces) samplePass<S, R, 1>(m, h.fPendS, h.uPend, Y, oFree);
        else samplePass<S, R, 2>(m, h.fPendS, h.uPend, Y, oFree);
        h.pending = false;
        const float u = in != nullptr ? in[k] : 0.0f;
        double F[S][R] = {};
        bool any = false;
        for (int r = 0; r < R; ++r) {
            double yfree[S];
            for (int s = 0; s < S; ++s) {
                double y = Y[s * R + r];
                for (int q = 0; q < r; ++q)
                    for (int t = 0; t < S; ++t) y += sr.h[s][t][r - q] * F[t][q];
                if (r == R - 1) y += sr.gP[s] * u;
                yfree[s] = y;
            }
            double Fr[S];
            if (cs.run(h, p, G, yfree, Fr) > 0.0) any = true;
            for (int s = 0; s < S; ++s) F[s][r] = Fr[s];
        }
        double o = static_cast<double>(oFree) + sr.gIn * u;
        if (any)
            for (int s = 0; s < S; ++s)
                for (int r = 0; r < R; ++r) o += sr.gOut[s][r] * F[s][r];
        out[k] = static_cast<float>(o);
        if (any || u != 0.0f) {
            h.pending = true;
            h.pendForces = any;
            for (int s = 0; s < S; ++s)
                for (int r = 0; r < R; ++r) h.fPendS[s * R + r] = static_cast<float>(F[s][r]);
            h.uPend = u;
        }
    }
    cs.save(h);
}

template <int S>
void runT(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in, float* out,
          int n) noexcept {
    if (m.rec && m.resp) {
        switch (m.oversampling) {
            case 1: runSampleT<S, 1>(h, p, m, in, out, n); return;
            case 2: runSampleT<S, 2>(h, p, m, in, out, n); return;
            case 3: runSampleT<S, 3>(h, p, m, in, out, n); return;
            case 4: runSampleT<S, 4>(h, p, m, in, out, n); return;
            default: break;
        }
    }
    runSubstepT<S>(h, p, m, in, out, n);
}

template <int S>
void flushT(HammerState& h, const HammerKeyModes& m) noexcept {
    if (m.rec && m.resp) {
        switch (m.oversampling) {
            case 1: flushSampleAny<S, 1>(h, m); return;
            case 2: flushSampleAny<S, 2>(h, m); return;
            case 3: flushSampleAny<S, 3>(h, m); return;
            case 4: flushSampleAny<S, 4>(h, m); return;
            default: break;
        }
    }
    applyForce<S>(m, h.fPend);
    h.pending = false;
}

// C = exp(lambda t) as (Re(C) - 1, Im(C)), without cancellation.
cd expMinusOne(cd lambda, double t) noexcept {
    const double sigma = -lambda.real(), theta = lambda.imag() * t;
    const double rm1 = std::expm1(-sigma * t);
    const double half = std::sin(0.5 * theta);
    return {rm1 * std::cos(theta) - 2.0 * half * half, (1.0 + rm1) * std::sin(theta)};
}

}  // namespace

bool ContactTables::allocate(int modeCapacity, int segments) noexcept {
    const std::size_t n = static_cast<std::size_t>(std::max(modeCapacity, 0));
    bool ok = dSub_.allocate(n) && sSub_.allocate(n) &&
              seg_.allocate(static_cast<std::size_t>(std::max(segments, 1)));
    for (int s = 0; s < kMaxStrings; ++s)
        ok = ok && pRe_[s].allocate(n) && pIm_[s].allocate(n) && bRe_[s].allocate(n) &&
             bIm_[s].allocate(n);
    rec_.clear();
    rec_.resize(static_cast<std::size_t>(std::max(segments, 1)));
    cur_ = -1;
    return ok;
}

bool ContactTables::beginSegment(const ModeBank& bank, int s, int strings, int oversampling) noexcept {
    cur_ = -1;
    if (s < 0 || static_cast<std::size_t>(s) >= rec_.size()) return false;
    rec_[s].release();
    seg_[s].hasRec = false;
    seg_[s].resp = SampleResponse{};
    const int S = std::clamp(strings, 1, kMaxStrings);
    if (oversampling < 1 || oversampling > kMaxSampleOversampling) return true;  // substep path
    const int count = bank.segmentModeCount(s);
    const int blocks = (count + simd::kWidth - 1) / simd::kWidth;
    const std::size_t floats = static_cast<std::size_t>(blocks) * recVectors(S, oversampling) * simd::kWidth;
    if (!rec_[s].allocate(floats)) return false;
    cur_ = s;
    curFirst_ = bank.segmentFirstMode(s);
    curCount_ = count;
    curStrings_ = S;
    curR_ = oversampling;
    return true;
}

void ContactTables::setMode(int i, std::complex<double> lambda, int strings,
                            const std::complex<double>* proj, const std::complex<double>* input,
                            double fs, int oversampling) noexcept {
    const double h = 1.0 / (fs * oversampling);
    const cd cm1 = expMinusOne(lambda, h);  // C - 1
    dSub_[i] = static_cast<float>(cm1.real());
    sSub_[i] = static_cast<float>(cm1.imag());
    cd b[kMaxStrings];
    for (int s = 0; s < kMaxStrings; ++s) {
        const bool used = s < strings;
        b[s] = used ? input[s] * cm1 / lambda : 0.0;
        bRe_[s][i] = static_cast<float>(b[s].real());
        bIm_[s][i] = static_cast<float>(b[s].imag());
        pRe_[s][i] = used ? static_cast<float>(proj[s].real()) : 0.0f;
        pIm_[s][i] = used ? static_cast<float>(proj[s].imag()) : 0.0f;
    }

    // Sample path: this mode's lane of its block's record, and its share of
    // the key's strike-point responses.
    if (cur_ < 0 || i < curFirst_ || i >= curFirst_ + curCount_) return;
    const int S = curStrings_, R = curR_;
    if (oversampling != R || strings != S) return;
    const int W = simd::kWidth, j = i - curFirst_;
    float* r = rec_[cur_].data() + static_cast<std::size_t>(j / W) * recVectors(S, R) * W + j % W;
    cd Cpow[kMaxSampleOversampling + 1];  // C^k
    Cpow[0] = 1.0;
    for (int k = 1; k <= R; ++k) Cpow[k] = 1.0 + expMinusOne(lambda, k * h);
    for (int s = 0; s < S; ++s)
        for (int q = 0; q < R; ++q) {
            const int k = s * R + q;
            const cd B = Cpow[R - 1 - q] * b[s];
            const cd P = proj[s] * Cpow[q + 1];
            r[recB(k) * W] = static_cast<float>(B.real());
            r[(recB(k) + 1) * W] = static_cast<float>(B.imag());
            r[recP(S, R, k) * W] = static_cast<float>(P.real());
            r[(recP(S, R, k) + 1) * W] = static_cast<float>(P.imag());
        }
    const cd cr = expMinusOne(lambda, R * h);
    r[recC(S, R) * W] = static_cast<float>(cr.real());
    r[(recC(S, R) + 1) * W] = static_cast<float>(cr.imag());
    SampleResponse& resp = seg_[cur_].resp;
    for (int s = 0; s < S; ++s)
        for (int t = 0; t < S; ++t)
            for (int d = 0; d < R; ++d) resp.h[s][t][d] += (proj[s] * Cpow[d] * b[t]).imag();
}

void ContactTables::setMode(int i, double freqHz, double sigma, double phi, double modalMass,
                            double fs, int oversampling) noexcept {
    const double omega = 2.0 * kPi * freqHz;
    const std::complex<double> proj = phi, input = phi / (modalMass * omega);
    setMode(i, {-sigma, omega}, 1, &proj, &input, fs, oversampling);
}

void ContactTables::finishSegment(const ModeBank& bank, int s, int strings, double fs,
                                  int oversampling, double residualCompliance,
                                  double releaseSeconds, double maxSeconds) noexcept {
    Seg& sg = seg_[s];
    sg.strings = std::clamp(strings, 1, kMaxStrings);
    sg.cRes = residualCompliance;
    sg.oversampling = oversampling;
    sg.h = 1.0 / (fs * oversampling);
    sg.release = static_cast<std::int64_t>(std::ceil(releaseSeconds / sg.h));
    sg.maxSteps = static_cast<std::int64_t>(std::ceil(maxSeconds / sg.h));
    const int m0 = bank.segmentFirstMode(s), m1 = m0 + bank.segmentModeCount(s);
    for (int a = 0; a < kMaxStrings; ++a) {
        sg.gOut[a] = 0.0;
        for (int b = 0; b < kMaxStrings; ++b) sg.G[a][b] = 0.0;
    }
    // From the float coefficients the solver really applies, so that the
    // predicted end-of-substep displacement matches the state exactly.
    for (int i = m0; i < m1; ++i) {
        for (int t = 0; t < sg.strings; ++t) {
            sg.gOut[t] += double(bank.weightOut(i)) * bIm_[t][i];
            for (int a = 0; a < sg.strings; ++a)
                sg.G[a][t] += double(pRe_[a][i]) * bIm_[t][i] + double(pIm_[a][i]) * bRe_[t][i];
        }
    }

    // Sample path: the bank's weights into the record, and the output and
    // input responses from the float coefficients the pass applies.
    const bool rec = cur_ == s && curStrings_ == sg.strings && curR_ == oversampling;
    cur_ = -1;
    if (!rec) {
        rec_[s].release();
        sg.hasRec = false;
        return;
    }
    const int S = sg.strings, R = oversampling, W = simd::kWidth;
    SampleResponse& resp = sg.resp;
    for (int i = m0; i < m1; ++i) {
        const int j = i - m0;
        float* r = rec_[s].data() + static_cast<std::size_t>(j / W) * recVectors(S, R) * W + j % W;
        const float wo = bank.weightOut(i), wr = bank.weightIn(i), wi = bank.weightInIm(i);
        r[recIn(S, R) * W] = wr;
        r[(recIn(S, R) + 1) * W] = wi;
        r[recOut(S, R) * W] = wo;
        for (int k = 0; k < S * R; ++k) resp.gOut[k / R][k % R] += double(wo) * r[(recB(k) + 1) * W];
        resp.gIn += double(wo) * wi;
        for (int a = 0; a < S; ++a) {
            resp.gP[a] += double(pRe_[a][i]) * wi + double(pIm_[a][i]) * wr;
            resp.pAbsIn[a] += (std::fabs(double(pRe_[a][i])) + std::fabs(double(pIm_[a][i]))) *
                              (std::fabs(double(wr)) + std::fabs(double(wi)));
        }
    }
    sg.hasRec = true;
}

HammerKeyModes ContactTables::view(ModeBank& bank, int s) const noexcept {
    HammerKeyModes m = substepView(bank, s);
    const Seg& sg = seg_[s];
    if (sg.hasRec) {
        m.rec = rec_[s].data();
        m.resp = &sg.resp;
    }
    return m;
}

HammerKeyModes ContactTables::substepView(ModeBank& bank, int s) const noexcept {
    const Seg& sg = seg_[s];
    const int m0 = bank.segmentFirstMode(s);
    HammerKeyModes m;
    m.count = bank.segmentModeCount(s);
    m.strings = sg.strings;
    m.zr = bank.stateReData() + m0;
    m.zi = bank.stateImData() + m0;
    m.wOut = bank.weightOutData() + m0;
    m.dampAm1 = bank.damperFactorMinusOneData() + m0;
    m.damped = bank.segmentDamper(s) > 0.0f;
    m.dSub = dSub_.data() + m0;
    m.sSub = sSub_.data() + m0;
    for (int a = 0; a < kMaxStrings; ++a) {
        m.pRe[a] = pRe_[a].data() + m0;
        m.pIm[a] = pIm_[a].data() + m0;
        m.bRe[a] = bRe_[a].data() + m0;
        m.bIm[a] = bIm_[a].data() + m0;
        m.gOut[a] = sg.gOut[a];
        for (int b = 0; b < kMaxStrings; ++b) m.G[a][b] = sg.G[a][b];
    }
    m.wInRe = bank.weightInReData() + m0;
    m.wInIm = bank.weightInImData() + m0;
    m.cRes = sg.cRes;
    m.oversampling = sg.oversampling;
    m.h = sg.h;
    m.invH = sg.h > 0.0 ? 1.0 / sg.h : 0.0;
    m.releaseSubsteps = sg.release;
    m.maxSubsteps = sg.maxSteps;
    return m;
}

double strikeDisplacement(const HammerKeyModes& m, int s) noexcept {
    double y = 0.0;
    for (int i = 0; i < m.count; ++i) y += double(m.pRe[s][i]) * m.zi[i] + double(m.pIm[s][i]) * m.zr[i];
    return y;
}

double contactPotential(const HammerParams& p, const HammerKeyModes& m, double d) noexcept {
    const Felt felt(p.K, p.p, p.mu, 1.0 / m.h, m.cRes);
    return felt.fromTotal(d).W;
}

double modalEnergy(double zr, double zi, double freqHz, double sigma, double modalMass) noexcept {
    const double w = 2.0 * kPi * freqHz;
    const double qd = w * zr - sigma * zi;
    return 0.5 * modalMass * (qd * qd + (w * w + sigma * sigma) * zi * zi);
}

void launchHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m,
                  double velocity) noexcept {
    flushHammer(h, m);
    const int S = std::clamp(m.strings, 1, kMaxStrings);
    double y[kMaxStrings];
    double first = 0.0;
    for (int s = 0; s < S; ++s) {
        y[s] = strikeDisplacement(m, s);
        const double reach = y[s] + p.offset[s];
        first = s == 0 ? reach : std::min(first, reach);
    }
    h.v = velocity;
    h.x = first - 0.5 * velocity * m.h;
    for (int s = 0; s < kMaxStrings; ++s) {
        const bool used = s < S;
        h.xiPrev[s] = used ? h.x - p.offset[s] - y[s] : 0.0;  // negative: not in contact
        h.feltPrev[s] = h.xiPrev[s];
        h.wPrev[s] = 0.0;
        h.fPrev[s] = 0.0;
        h.dxiPrev[s] = velocity * m.h;
        h.dxiPrev2[s] = velocity * m.h;
        h.yPrev[s] = used ? y[s] : 0.0;
    }
    h.history = 0;
    h.cleared = false;
    h.clearCheckAt = 0;
    for (int s = 0; s < kMaxStrings; ++s) h.sensPrev[s] = 0.0;
    h.invMass = 1.0 / p.mass;
    h.halfH2InvMass = 0.5 * m.h * m.h / p.mass;
    h.feltPm1 = p.p - 1.0;
    h.feltInvP1 = 1.0 / (p.p + 1.0);
    h.launchVelocity = velocity;
    h.kineticEnergy0 = 0.5 * p.mass * velocity * velocity;
    h.feltLoss = 0.0;
    h.workOnString = 0.0;
    h.peakForce = 0.0;
    h.substeps = 0;
    h.firstContact = h.lastContact = -1;
    h.contactSubsteps = 0;
    h.contacts = 0;
    h.inContact = false;
    h.fallbackSolves = 0;
    h.newtonEvals = 0;
}

void stepHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in,
                float* out, int n) noexcept {
    switch (m.strings) {
        case 3: runT<3>(h, p, m, in, out, n); break;
        case 2: runT<2>(h, p, m, in, out, n); break;
        default: runT<1>(h, p, m, in, out, n); break;
    }
}

void flushHammer(HammerState& h, const HammerKeyModes& m) noexcept {
    if (!h.pending) return;
    switch (m.strings) {
        case 3: flushT<3>(h, m); break;
        case 2: flushT<2>(h, m); break;
        default: flushT<1>(h, m); break;
    }
}

void runHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, float* out, int n,
               const float* in) noexcept {
    stepHammer(h, p, m, in, out, n);
    flushHammer(h, m);
}

bool hammerFinished(const HammerState& h, const HammerKeyModes& m) noexcept {
    if (h.substeps >= m.maxSubsteps || h.cleared) return true;
    return h.lastContact >= 0 && !h.inContact && h.substeps - h.lastContact > m.releaseSubsteps;
}

}  // namespace pm
