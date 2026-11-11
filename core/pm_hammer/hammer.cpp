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

// Relative tolerance of the contact solves (on the compression). The energy
// identity is off by about F times the residual per substep, i.e. by about
// 1e-10 of the contact energy: far below what the tests resolve (1e-6),
// while each extra Newton evaluation would cost S pow calls per substep.
constexpr double kSolveTol = 1e-10;

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
    const vf one = set1(1.0f);
    for (int i = 0; i < m.count; i += kWidth) {
        vf zr = load(m.zr + i), zi = load(m.zi + i);
        if (pend) {
            for (int s = 0; s < S; ++s) {
                zr = fmadd(load(m.bRe[s] + i), f[s], zr);
                zi = fmadd(load(m.bIm[s] + i), f[s], zi);
            }
        }
        const vf a = add(one, load(m.dampAm1 + i));
        zr = mul(zr, a);
        zi = mul(zi, a);
        store(m.zr + i, zr);
        store(m.zi + i, zi);
        for (int s = 0; s < S; ++s)
            accY[s] = fmadd(load(m.pRe[s] + i), zi, fmadd(load(m.pIm[s] + i), zr, accY[s]));
    }
    for (int s = 0; s < S; ++s) y[s] = hsum(accY[s]);
}

// Felt in series with the residual string compliance. With e the felt
// compression, the total compression (hammer to modal string displacement)
// is d(e) = e + cRes F(e), and the contact potential is
// W = Phi(e) + cRes F^2 / 2, so that dW/dd = F. Everything is explicit in e,
// which is therefore the unknown of the solve (one pow per evaluation).
struct Felt {
    double K, p, mu, invH, cRes;

    struct Point {
        double e, d, W, F;
        double dD, dF;  // dd/de, dF/de
    };

    Point at(double e) const noexcept {
        if (e <= 0.0) return {e, e, 0.0, 0.0, 1.0, 0.0};
        const double F = K * fastPow(e, p);
        const double dF = p * F / e;
        return {e, e + cRes * F, F * e / (p + 1.0) + 0.5 * cRes * F * F, F, 1.0 + cRes * dF, dF};
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
        G = (q1.W - q0.W) / dd;
        dG = (q1.F - G) / dd;
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

// One string, scalar: solves r(e) = d(e) - dFree + A F(e) = 0 for the
// end-of-substep felt compression e. r is increasing in e (W is convex in d,
// d increasing in e), so the root is unique; it lies in
// [min(dFree - A F(dFree), 0), dFree].
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

// Solves J x = r in place for S <= 3 by Gaussian elimination with partial
// pivoting. Returns false if J is singular.
template <int S>
inline bool solveSmall(double (&J)[S][S], double (&r)[S]) noexcept {
    for (int c = 0; c < S; ++c) {
        int p = c;
        for (int i = c + 1; i < S; ++i)
            if (std::fabs(J[i][c]) > std::fabs(J[p][c])) p = i;
        if (!(std::fabs(J[p][c]) > 0.0)) return false;
        if (p != c) {
            for (int j = 0; j < S; ++j) std::swap(J[p][j], J[c][j]);
            std::swap(r[p], r[c]);
        }
        for (int i = c + 1; i < S; ++i) {
            const double f = J[i][c] / J[c][c];
            for (int j = c; j < S; ++j) J[i][j] -= f * J[c][j];
            r[i] -= f * r[c];
        }
    }
    for (int c = S - 1; c >= 0; --c) {
        double s = r[c];
        for (int j = c + 1; j < S; ++j) s -= J[c][j] * r[j];
        r[c] = s / J[c][c];
    }
    return true;
}

// S strings: r_s(e) = d_s(e_s) - dFree_s + Ah sum_t F_t + sum_t G_st F_t = 0.
template <int S>
inline void solveContacts(const Felt& felt, const double* dFree, const Felt::Point* q0, double Ah,
                          const double (&G)[kMaxStrings][kMaxStrings], const double* guess,
                          Felt::Point* q1, double* F, int& fallbacks) noexcept {
    double scale = 0.0;
    for (int s = 0; s < S; ++s) scale = std::max({scale, std::fabs(dFree[s]), std::fabs(q0[s].d)});
    const double tol = kSolveTol * scale;
    double e[S], dF[S], r[S];
    for (int s = 0; s < S; ++s) e[s] = std::min(guess[s], dFree[s]);
    for (int it = 0; it < 12; ++it) {
        double P = 0.0;
        for (int s = 0; s < S; ++s) {
            q1[s] = felt.at(e[s]);
            F[s] = substepForce(felt, q1[s], q0[s], dF[s]);
            P += F[s];
        }
        double worst = 0.0;
        for (int s = 0; s < S; ++s) {
            double g = 0.0;
            for (int t = 0; t < S; ++t) g += G[s][t] * F[t];
            r[s] = q1[s].d - dFree[s] + Ah * P + g;
            worst = std::max(worst, std::fabs(r[s]));
        }
        if (worst <= tol) return;
        double J[S][S];
        for (int s = 0; s < S; ++s)
            for (int t = 0; t < S; ++t) J[s][t] = (s == t ? q1[s].dD : 0.0) + (Ah + G[s][t]) * dF[t];
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

template <int S>
void runHammerT(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in, float* out,
                int n) noexcept {
    const Felt felt{p.K, p.p, p.mu, 1.0 / m.h, m.cRes};
    const double invM = 1.0 / p.mass;
    const double Ah = 0.5 * m.h * m.h * invM;
    const int R = m.oversampling;
    const bool hasIn = in != nullptr && m.wInRe != nullptr;
    Felt::Point q0[S];
    for (int s = 0; s < S; ++s) q0[s] = {h.feltPrev[s], h.xiPrev[s], h.wPrev[s], h.fPrev[s], 1.0, 0.0};
    for (int k = 0; k < n; ++k) {
        if (m.damped) {
            double y[S];
            applyDamper<S>(m, h.pending, h.fPend, y);
            h.pending = false;
            for (int s = 0; s < S; ++s) {
                q0[s] = felt.fromTotal(q0[s].d + h.yPrev[s] - y[s]);
                h.yPrev[s] = y[s];
            }
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
            const double xt = h.x + h.v * m.h;
            double dFree[S];
            bool touching = false;
            for (int s = 0; s < S; ++s) {
                dFree[s] = xt - p.offset[s] - yf[s];
                touching = touching || dFree[s] > 0.0 || q0[s].d > 0.0;
            }
            Felt::Point q1[S];
            double P = 0.0;
            if (touching) {
                // Quadratic extrapolation of the felt compression: with it one
                // Newton step usually meets the tolerance.
                double guess[S];
                for (int s = 0; s < S; ++s) guess[s] = q0[s].e + 2.0 * h.dxiPrev[s] - h.dxiPrev2[s];
                if constexpr (S == 1) {
                    q1[0] = solveContact(felt, dFree[0], q0[0], Ah + m.G[0][0], guess[0], F[0]);
                } else {
                    solveContacts<S>(felt, dFree, q0, Ah, m.G, guess, q1, F, h.fallbackSolves);
                }
                for (int s = 0; s < S; ++s) P += F[s];
                if (P > 0.0) {
                    for (int s = 0; s < S; ++s) h.fPend[s] = F[s];
                    h.pending = true;
                }
            } else {
                for (int s = 0; s < S; ++s) {
                    F[s] = 0.0;
                    q1[s] = felt.at(dFree[s]);
                }
            }
            h.x = xt - Ah * P;
            h.v -= m.h * P * invM;
            for (int s = 0; s < S; ++s) {
                double y = yf[s];
                for (int t = 0; t < S; ++t) y += m.G[s][t] * F[t];
                if (F[s] > 0.0) {
                    h.workOnString += F[s] * (y - h.yPrev[s]);
                    h.feltLoss += F[s] * (q1[s].d - q0[s].d) - (q1[s].W - q0[s].W);
                }
                h.yPrev[s] = y;
                h.dxiPrev2[s] = h.dxiPrev[s];
                h.dxiPrev[s] = q1[s].e - q0[s].e;
                q0[s] = q1[s];
            }
            if (P > 0.0) {
                h.peakForce = std::max(h.peakForce, P);
                if (!h.inContact) {
                    ++h.contacts;
                    if (h.firstContact < 0) h.firstContact = h.substeps;
                }
                h.inContact = true;
                h.lastContact = h.substeps;
                ++h.contactSubsteps;
            } else {
                h.inContact = false;
            }
            ++h.substeps;
        }
        double o = ofree;
        for (int s = 0; s < S; ++s) o += m.gOut[s] * F[s];
        out[k] = static_cast<float>(o);
    }
    for (int s = 0; s < S; ++s) {
        h.xiPrev[s] = q0[s].d;
        h.feltPrev[s] = q0[s].e;
        h.wPrev[s] = q0[s].W;
        h.fPrev[s] = q0[s].F;
    }
}

}  // namespace

bool ContactTables::allocate(int modeCapacity, int segments) noexcept {
    const std::size_t n = static_cast<std::size_t>(std::max(modeCapacity, 0));
    bool ok = dSub_.allocate(n) && sSub_.allocate(n) &&
              seg_.allocate(static_cast<std::size_t>(std::max(segments, 1)));
    for (int s = 0; s < kMaxStrings; ++s)
        ok = ok && pRe_[s].allocate(n) && pIm_[s].allocate(n) && bRe_[s].allocate(n) &&
             bIm_[s].allocate(n);
    return ok;
}

void ContactTables::setMode(int i, std::complex<double> lambda, int strings,
                            const std::complex<double>* proj, const std::complex<double>* input,
                            double fs, int oversampling) noexcept {
    const double h = 1.0 / (fs * oversampling);
    const double sigma = -lambda.real(), omega = lambda.imag();
    const double theta = omega * h;
    const double rm1 = std::expm1(-sigma * h);
    const double half = std::sin(0.5 * theta);
    const double cRe1 = rm1 * std::cos(theta) - 2.0 * half * half;  // Re(C) - 1
    const double cIm = (1.0 + rm1) * std::sin(theta);
    const std::complex<double> cm1(cRe1, cIm);
    dSub_[i] = static_cast<float>(cRe1);
    sSub_[i] = static_cast<float>(cIm);
    for (int s = 0; s < kMaxStrings; ++s) {
        const bool used = s < strings;
        const std::complex<double> b = used ? input[s] * cm1 / lambda : 0.0;
        bRe_[s][i] = static_cast<float>(b.real());
        bIm_[s][i] = static_cast<float>(b.imag());
        pRe_[s][i] = used ? static_cast<float>(proj[s].real()) : 0.0f;
        pIm_[s][i] = used ? static_cast<float>(proj[s].imag()) : 0.0f;
    }
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
}

HammerKeyModes ContactTables::view(ModeBank& bank, int s) const noexcept {
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
    const Felt felt{p.K, p.p, p.mu, 1.0 / m.h, m.cRes};
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
}

void stepHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, const float* in,
                float* out, int n) noexcept {
    switch (m.strings) {
        case 3: runHammerT<3>(h, p, m, in, out, n); break;
        case 2: runHammerT<2>(h, p, m, in, out, n); break;
        default: runHammerT<1>(h, p, m, in, out, n); break;
    }
}

void flushHammer(HammerState& h, const HammerKeyModes& m) noexcept {
    if (!h.pending) return;
    switch (m.strings) {
        case 3: applyForce<3>(m, h.fPend); break;
        case 2: applyForce<2>(m, h.fPend); break;
        default: applyForce<1>(m, h.fPend); break;
    }
    h.pending = false;
}

void runHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, float* out, int n,
               const float* in) noexcept {
    stepHammer(h, p, m, in, out, n);
    flushHammer(h, m);
}

bool hammerFinished(const HammerState& h, const HammerKeyModes& m) noexcept {
    if (h.substeps >= m.maxSubsteps) return true;
    return h.lastContact >= 0 && !h.inContact && h.substeps - h.lastContact > m.releaseSubsteps;
}

}  // namespace pm
