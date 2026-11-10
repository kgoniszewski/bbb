#include "fdtd_string.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pm_fdtd {

namespace {
constexpr double kPi = std::numbers::pi;

double minH(const StringSpec& s, double k) {
    const double c2 = s.tension / s.linearDensity;
    const double b3p = s.b3 * c2;
    const double ei = s.B * s.tension * s.length * s.length / (kPi * kPi);
    const double kappa2 = ei / s.linearDensity;
    const double a = c2 * k * k + 4.0 * b3p * k;
    return std::sqrt(0.5 * (a + std::sqrt(a * a + 16.0 * kappa2 * k * k)));
}
}  // namespace

Grid gridFor(const StringSpec& s, double k) {
    Grid g;
    g.k = k;
    g.points = std::max(4, static_cast<int>(std::floor(s.length / minH(s, k))));
    g.h = s.length / g.points;
    return g;
}

double stepForPoints(const StringSpec& s, int minPoints) {
    // minH grows with k: bisect for the largest k whose grid has minPoints.
    double lo = 1e-12, hi = 1e-3;
    for (int it = 0; it < 200; ++it) {
        const double mid = std::sqrt(lo * hi);
        if (s.length / minH(s, mid) >= minPoints) lo = mid;
        else hi = mid;
    }
    return lo;
}

FdtdString::FdtdString(const StringSpec& s, double k) : spec_(s), grid_(gridFor(s, k)) {
    const int N = grid_.points;
    const double h = grid_.h;
    c2_ = s.tension / s.linearDensity;
    b3p_ = s.b3 * c2_;
    ei_ = s.B * s.tension * s.length * s.length / (kPi * kPi);
    lambda2_ = c2_ * k * k / (h * h);
    nu2_ = ei_ / s.linearDensity * k * k / (h * h * h * h);
    lossMix_ = 2.0 * b3p_ * k / (h * h);
    b1k_ = s.b1 * k;
    y_.assign(static_cast<std::size_t>(N + 1), 0.0);
    yPrev_ = y_;
    yNext_ = y_;
    const double xs = s.strikePos * N;
    is_ = std::clamp(static_cast<int>(std::floor(xs)), 1, N - 2);
    ws_ = xs - is_;
    // Hammer touching the string, moving up at v0.
    eta_ = 0.0;
    etaPrev_ = -s.velocity * k;
    xiPrev_ = etaPrev_;
}

double FdtdString::strikeDisplacement() const { return (1.0 - ws_) * y_[is_] + ws_ * y_[is_ + 1]; }

double FdtdString::hammerKineticEnergy() const {
    const double v = hammerVelocity();
    return 0.5 * spec_.hammerMass * v * v;
}

void FdtdString::setDisplacement(const std::vector<double>& y) {
    y_ = y;
    yPrev_ = y;
    hammer_ = false;
    force_ = 0.0;
}

void FdtdString::step() {
    const int N = grid_.points;
    const double k = grid_.k, h = grid_.h;
    // Ghost points of the simply supported ends: y_-1 = -y_1, y_N+1 = -y_N-1.
    auto at = [&](const std::vector<double>& v, int i) {
        if (i < 0) return -v[static_cast<std::size_t>(-i)];
        if (i > N) return -v[static_cast<std::size_t>(2 * N - i)];
        return v[static_cast<std::size_t>(i)];
    };
    const double inv = 1.0 / (1.0 + b1k_);
    // Free update (no hammer force).
    for (int i = 1; i < N; ++i) {
        const double d2 = at(y_, i + 1) - 2.0 * y_[i] + at(y_, i - 1);
        const double d2p = at(yPrev_, i + 1) - 2.0 * yPrev_[i] + at(yPrev_, i - 1);
        const double d4 = at(y_, i + 2) - 4.0 * at(y_, i + 1) + 6.0 * y_[i] - 4.0 * at(y_, i - 1) + at(y_, i - 2);
        yNext_[i] = (2.0 * y_[i] - (1.0 - b1k_) * yPrev_[i] + lambda2_ * d2 - nu2_ * d4 + lossMix_ * (d2 - d2p)) * inv;
    }
    yNext_[0] = yNext_[N] = 0.0;
    // Hammer force F^n: the stiffness term from the current compression
    // (explicit, Chaigne-Askenfelt), the Hunt-Crossley rate as the centred
    // difference (xi^{n+1} - xi^{n-1}) / 2k. xi^{n+1} is linear in F^n, so
    // the damping term is solved implicitly: an explicit rate taken from a
    // single grid point picks up grid-frequency motion and is unstable at
    // high forces.
    force_ = 0.0;
    if (hammer_) {
        const double xi = eta_ - strikeDisplacement();
        const double etaFree = 2.0 * eta_ - etaPrev_;
        const double xiFree = etaFree - ((1.0 - ws_) * yNext_[is_] + ws_ * yNext_[is_ + 1]);
        const double gS = k * k / (spec_.linearDensity * h) * inv * ((1.0 - ws_) * (1.0 - ws_) + ws_ * ws_);
        const double a = k * k / spec_.hammerMass + gS;  // xi^{n+1} = xiFree - a F
        if (xi > 0.0) {
            const double stiff = spec_.feltK * std::pow(xi, spec_.feltP);
            const double c = spec_.feltMu / (2.0 * k);
            force_ = std::max(0.0, stiff * (1.0 + c * (xiFree - xiPrev_)) / (1.0 + stiff * c * a));
        }
        xiPrev_ = xi;
        etaPrev_ = eta_;
        eta_ = etaFree - k * k * force_ / spec_.hammerMass;
        if (force_ > 0.0) {
            const double f = k * k / (spec_.linearDensity * h) * force_ * inv;
            yNext_[is_] += f * (1.0 - ws_);
            yNext_[is_ + 1] += f * ws_;
        }
    }
    std::swap(yPrev_, y_);
    std::swap(y_, yNext_);
    ++n_;
}

double FdtdString::bridgeForce() const {
    const int N = grid_.points;
    const double h = grid_.h;
    // Centred differences at x = L with y_N = 0 and the odd ghosts
    // y_N+1 = -y_N-1, y_N+2 = -y_N-2.
    const double yx = -y_[N - 1] / h;
    const double yxxx = (2.0 * y_[N - 1] - y_[N - 2]) / (h * h * h);
    return -(spec_.tension * yx - ei_ * yxxx);
}

double FdtdString::stringEnergy() const {
    const int N = grid_.points;
    const double k = grid_.k, h = grid_.h, mu = spec_.linearDensity;
    double kin = 0.0, ten = 0.0, stf = 0.0;
    auto d2 = [&](const std::vector<double>& v, int i) {
        const double l = i == 0 ? -v[1] : v[static_cast<std::size_t>(i - 1)];
        const double r = i == N ? -v[static_cast<std::size_t>(N - 1)] : v[static_cast<std::size_t>(i + 1)];
        return r - 2.0 * v[static_cast<std::size_t>(i)] + l;
    };
    for (int i = 0; i <= N; ++i) {
        const double v = (y_[i] - yPrev_[i]) / k;
        kin += 0.5 * mu * h * v * v;
        if (i < N) ten += 0.5 * spec_.tension / h * (y_[i + 1] - y_[i]) * (yPrev_[i + 1] - yPrev_[i]);
        stf += 0.5 * ei_ / (h * h * h) * d2(y_, i) * d2(yPrev_, i);
    }
    return kin + ten + stf;
}

double FdtdString::modeDecayRate(int n) const {
    const double kn = n * kPi / spec_.length;
    return spec_.b1 + b3p_ * kn * kn;
}

ModalState FdtdString::modes(int count) const {
    const int N = grid_.points;
    const double k = grid_.k;
    const double m = 0.5 * spec_.linearDensity * spec_.length;
    ModalState s;
    count = std::min(count, N - 1);
    for (int n = 1; n <= count; ++n) {
        double q = 0.0, qp = 0.0;
        for (int i = 1; i < N; ++i) {
            const double ph = std::sin(n * kPi * i / N);
            q += y_[i] * ph;
            qp += yPrev_[i] * ph;
        }
        q *= 2.0 / N;
        qp *= 2.0 / N;
        // Mid-step values: displacement and velocity both at t - k/2.
        const double qm = 0.5 * (q + qp), qd = (q - qp) / k;
        const double sn = std::sin(n * kPi / (2.0 * N));
        const double s2 = lambda2_ * sn * sn + 4.0 * nu2_ * sn * sn * sn * sn;
        const double omega = 2.0 / k * std::asin(std::min(1.0, std::sqrt(s2)));
        s.q.push_back(qm);
        s.qdot.push_back(qd);
        s.omega.push_back(omega);
        // The mid-step displacement is cos(omega k / 2) times the true one.
        const double qc = qm / std::cos(0.5 * omega * k);
        s.energy.push_back(0.5 * m * (qd * qd + omega * omega * qc * qc));
    }
    return s;
}

}  // namespace pm_fdtd
