#include "pm_hammer/hammer.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

#include "pm_base/simd.h"

namespace pm {

namespace {

constexpr double kPi = std::numbers::pi;

// One substep of free propagation of all modes, z <- C z, returning
// sum phi Im z and (if wantOut) sum wOut Im z afterwards. Same dr form and
// operation order as the bank kernel.
inline void propagate(const HammerKeyModes& m, bool wantOut, float& ySum, float& oSum) noexcept {
    using namespace simd;
    vf accY = zero(), accO = zero();
    for (int i = 0; i < m.count; i += kWidth) {
        const vf zr = load(m.zr + i), zi = load(m.zi + i);
        const vf d = load(m.dSub + i), s = load(m.sSub + i);
        const vf nr = fnmadd(s, zi, fmadd(d, zr, zr));
        const vf ni = fmadd(s, zr, fmadd(d, zi, zi));
        store(m.zr + i, nr);
        store(m.zi + i, ni);
        accY = fmadd(load(m.phi + i), ni, accY);
        if (wantOut) accO = fmadd(load(m.wOut + i), ni, accO);
    }
    ySum = hsum(accY);
    oSum = wantOut ? hsum(accO) : 0.0f;
}

inline void applyForce(const HammerKeyModes& m, float force) noexcept {
    using namespace simd;
    const vf f = set1(force);
    for (int i = 0; i < m.count; i += kWidth) {
        store(m.zr + i, fmadd(load(m.bRe + i), f, load(m.zr + i)));
        store(m.zi + i, fmadd(load(m.bIm + i), f, load(m.zi + i)));
    }
}

// Damper loss for one base sample (c_eff = c a commutes with the rotation);
// returns the new strike-point displacement.
inline double applyDamper(const HammerKeyModes& m) noexcept {
    using namespace simd;
    vf accY = zero();
    const vf one = set1(1.0f);
    for (int i = 0; i < m.count; i += kWidth) {
        const vf a = add(one, load(m.dampAm1 + i));
        const vf zi = mul(load(m.zi + i), a);
        store(m.zr + i, mul(load(m.zr + i), a));
        store(m.zi + i, zi);
        accY = fmadd(load(m.phi + i), zi, accY);
    }
    return hsum(accY);
}

// Felt in series with the residual string compliance. With e the felt
// compression, the total compression (hammer to modal string displacement)
// is d(e) = e + cRes F(e), and the contact potential is
// W = Phi(e) + cRes F^2 / 2, so that dW/dd = F. Everything is explicit in e,
// which is therefore the unknown of the solve (one pow per Newton step).
struct Felt {
    double K, p, mu, invH, cRes;

    struct Point {
        double e, d, W, F;
        double dD, dF;  // dd/de, dF/de
    };

    Point at(double e) const noexcept {
        if (e <= 0.0) return {e, e, 0.0, 0.0, 1.0, 0.0};
        const double F = K * std::pow(e, p);
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

// Solves r(e) = d(e) - dFree + A F(e) = 0 for the end-of-substep felt
// compression e. r is increasing in e (W is convex in d, d increasing in e),
// so the root is unique; it lies in [min(dFree - A F(dFree), 0), dFree].
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
    const double tol = 1e-14 * std::max(std::fabs(dFree), std::fabs(q0.d));
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
        const bool done = std::fabs(next - e) <= 1e-13 * std::fabs(e);
        e = next;
        q = felt.at(e);
        if (done) break;
    }
    Fout = substepForce(felt, q, q0, dF);
    return q;
}

}  // namespace

bool ContactTables::allocate(int modeCapacity, int segments) noexcept {
    const std::size_t n = static_cast<std::size_t>(std::max(modeCapacity, 0));
    return dSub_.allocate(n) && sSub_.allocate(n) && bRe_.allocate(n) && bIm_.allocate(n) &&
           phi_.allocate(n) && seg_.allocate(static_cast<std::size_t>(std::max(segments, 1)));
}

void ContactTables::setMode(int i, double freqHz, double sigma, double phi, double modalMass,
                            double fs, int oversampling) noexcept {
    const double h = 1.0 / (fs * oversampling);
    const double omega = 2.0 * kPi * freqHz;
    const double theta = omega * h;
    const double rm1 = std::expm1(-sigma * h);
    const double half = std::sin(0.5 * theta);
    const double cRe1 = rm1 * std::cos(theta) - 2.0 * half * half;  // Re(C) - 1
    const double cIm = (1.0 + rm1) * std::sin(theta);
    const std::complex<double> s(-sigma, omega);
    const std::complex<double> b = (phi / (modalMass * omega)) * std::complex<double>(cRe1, cIm) / s;
    dSub_[i] = static_cast<float>(cRe1);
    sSub_[i] = static_cast<float>(cIm);
    bRe_[i] = static_cast<float>(b.real());
    bIm_[i] = static_cast<float>(b.imag());
    phi_[i] = static_cast<float>(phi);
}

void ContactTables::finishSegment(const ModeBank& bank, int s, double fs, int oversampling,
                                  double residualCompliance, double releaseSeconds,
                                  double maxSeconds) noexcept {
    Seg& sg = seg_[s];
    sg.cRes = residualCompliance;
    sg.oversampling = oversampling;
    sg.h = 1.0 / (fs * oversampling);
    sg.release = static_cast<std::int64_t>(std::ceil(releaseSeconds / sg.h));
    sg.maxSteps = static_cast<std::int64_t>(std::ceil(maxSeconds / sg.h));
    double g = 0.0, gOut = 0.0;
    const int m0 = bank.segmentFirstMode(s), m1 = m0 + bank.segmentModeCount(s);
    for (int i = m0; i < m1; ++i) {
        g += double(phi_[i]) * bIm_[i];
        gOut += double(bank.weightOut(i)) * bIm_[i];
    }
    sg.g = g;
    sg.gOut = gOut;
}

HammerKeyModes ContactTables::view(ModeBank& bank, int s) const noexcept {
    const Seg& sg = seg_[s];
    const int m0 = bank.segmentFirstMode(s);
    HammerKeyModes m;
    m.count = bank.segmentModeCount(s);
    m.zr = bank.stateReData() + m0;
    m.zi = bank.stateImData() + m0;
    m.wOut = bank.weightOutData() + m0;
    m.dampAm1 = bank.damperFactorMinusOneData() + m0;
    m.damped = bank.segmentDamper(s) > 0.0f;
    m.dSub = dSub_.data() + m0;
    m.sSub = sSub_.data() + m0;
    m.bRe = bRe_.data() + m0;
    m.bIm = bIm_.data() + m0;
    m.phi = phi_.data() + m0;
    m.g = sg.g;
    m.cRes = sg.cRes;
    m.gOut = sg.gOut;
    m.oversampling = sg.oversampling;
    m.h = sg.h;
    m.releaseSubsteps = sg.release;
    m.maxSubsteps = sg.maxSteps;
    return m;
}

double strikeDisplacement(const HammerKeyModes& m) noexcept {
    double y = 0.0;
    for (int i = 0; i < m.count; ++i) y += double(m.phi[i]) * m.zi[i];
    return y;
}

double contactPotential(const HammerParams& p, const HammerKeyModes& m, double xi) noexcept {
    const Felt felt{p.K, p.p, p.mu, 1.0 / m.h, m.cRes};
    return felt.fromTotal(xi).W;
}

double modalEnergy(double zr, double zi, double freqHz, double sigma, double modalMass) noexcept {
    const double w = 2.0 * kPi * freqHz;
    const double qd = w * zr - sigma * zi;
    return 0.5 * modalMass * (qd * qd + (w * w + sigma * sigma) * zi * zi);
}

void launchHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m,
                  double velocity) noexcept {
    const double y = strikeDisplacement(m);
    h.v = velocity;
    h.x = y - 0.5 * velocity * m.h;
    h.xiPrev = h.x - y;  // negative: not yet in contact
    h.feltPrev = h.xiPrev;
    h.wPrev = 0.0;
    h.fPrev = 0.0;
    h.dxiPrev = velocity * m.h;
    h.yPrev = y;
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
}

void runHammer(HammerState& h, const HammerParams& p, const HammerKeyModes& m, float* out,
               int n) noexcept {
    const Felt felt{p.K, p.p, p.mu, 1.0 / m.h, m.cRes};
    const double invM = 1.0 / p.mass;
    const double Ah = 0.5 * m.h * m.h * invM;
    const double A = Ah + m.g;
    const int R = m.oversampling;
    Felt::Point q0{h.feltPrev, h.xiPrev, h.wPrev, h.fPrev, 1.0, 0.0};
    for (int k = 0; k < n; ++k) {
        if (m.damped) {
            const double y = applyDamper(m);
            h.xiPrev += h.yPrev - y;
            h.yPrev = y;
            q0 = felt.fromTotal(h.xiPrev);
        }
        float ofree = 0.0f;
        double F = 0.0;
        for (int r = 0; r < R; ++r) {
            const bool last = r == R - 1;
            float yf, of;
            propagate(m, last, yf, of);
            if (last) ofree = of;
            const double yFree = yf;
            const double xt = h.x + h.v * m.h;
            const double dFree = xt - yFree;
            F = 0.0;
            Felt::Point q1;
            if (dFree > 0.0 || q0.d > 0.0) {
                q1 = solveContact(felt, dFree, q0, A, q0.e + h.dxiPrev, F);
                if (F > 0.0) applyForce(m, static_cast<float>(F));
            } else {
                q1 = felt.at(dFree);
            }
            h.x = xt - Ah * F;
            h.v -= m.h * F * invM;
            const double y = yFree + m.g * F;
            if (F > 0.0) {
                h.workOnString += F * (y - h.yPrev);
                h.feltLoss += F * (q1.d - q0.d) - (q1.W - q0.W);
                h.peakForce = std::max(h.peakForce, F);
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
            h.yPrev = y;
            h.dxiPrev = q1.e - q0.e;
            q0 = q1;
            ++h.substeps;
        }
        out[k] = static_cast<float>(ofree + m.gOut * F);
    }
    h.xiPrev = q0.d;
    h.feltPrev = q0.e;
    h.wPrev = q0.W;
    h.fPrev = q0.F;
}

bool hammerFinished(const HammerState& h, const HammerKeyModes& m) noexcept {
    if (h.substeps >= m.maxSubsteps) return true;
    return h.lastContact >= 0 && !h.inContact && h.substeps - h.lastContact > m.releaseSubsteps;
}

}  // namespace pm
