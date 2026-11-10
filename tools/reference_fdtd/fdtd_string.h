// Offline finite-difference reference: one stiff, lossy string struck by a
// Hunt-Crossley hammer (plan §0, §5.4). Not real-time; double precision.
//
// Equation (Bensa et al. 2003 form of plan §1.2):
//
//   mu y_tt = T y_xx - EI y_xxxx - 2 mu b1 y_t + 2 mu b3' y_txx + f_hammer(x, t)
//
// on 0 < x < L, simply supported (y = y_xx = 0 at both ends). The plan's
// Chaigne-Askenfelt loss term +2 mu b3 y_ttt is not used: an explicit scheme
// for a third time derivative is only conditionally (and delicately)
// stable, whereas the mixed derivative y_txx gives a well-posed equation and
// a scheme with a simple stability bound. Conversion: mode n (wavenumber
// k_n = n pi / L) decays at
//   Bensa:              sigma_n = b1 + b3' k_n^2
//   Chaigne-Askenfelt:  sigma_n = b1 + b3 omega_n^2 = b1 + b3 c^2 k_n^2 (1 + B n^2)
// so b3' = b3 c^2 (c^2 = T / mu) matches the flexible-string part exactly;
// for the stiff string the CA law decays partial n faster by the factor
// (1 + B n^2) in its b3 term (34 % at n = 30 for C4). The cross-check
// compares partial energies extrapolated back to the strike with each
// model's own decay rates, so this difference does not enter it.
//
// Stiffness: EI = B T L^2 / pi^2 (B = pi^2 EI / (T L^2)).
//
// Scheme (explicit, time step k, grid spacing h = L / N):
//   y^{n+1} (1 + b1 k) = 2 y^n - (1 - b1 k) y^{n-1} + lambda^2 D2 y^n
//                        - nu^2 D4 y^n + (2 b3' k / h^2) D2 (y^n - y^{n-1})
//                        + k^2 / (mu h) F^n w
// with D2, D4 the unscaled second and fourth differences, lambda = c k / h,
// nu = kappa k / h^2, kappa^2 = EI / mu, and w the linear-interpolation
// weights of the strike point on its two neighbouring grid points. Stable
// for h^2 >= (c^2 k^2 + 4 b3' k + sqrt((c^2 k^2 + 4 b3' k)^2 + 16 kappa^2 k^2)) / 2
// (Bensa et al. 2003); N is the largest grid count that satisfies it.
//
// Hammer (Chaigne-Askenfelt): mass M at position eta,
//   M eta'' = -F,   F = K xi^p (1 + mu_h dxi/dt) for xi = eta - y(x_s) > 0,
//   eta^{n+1} = 2 eta^n - eta^{n-1} - k^2 F^n / M,
// with K (xi^n)^p explicit and dxi/dt = (xi^{n+1} - xi^{n-1}) / 2k, where
// xi^{n+1} is linear in F^n (hammer and the two grid points under it), so
// F^n follows from one division (a fully explicit rate is unstable at ff).
// The hammer starts touching the string (xi = 0) at velocity v0.
//
// Outputs: bridge force F_b = -(T y_x - EI y_xxx) at x = L (the reaction on
// the bridge, the real-time model's sign), hammer force and the string
// displacement at the strike point, all sampled at the output rate (mean
// over each output period); and the modal state (discrete sine transform
// of displacement and velocity) at any time.
#pragma once

#include <vector>

namespace pm_fdtd {

struct StringSpec {
    double length = 0.62;        // m
    double tension = 670.0;      // N
    double linearDensity = 6.4e-3;  // kg/m
    double B = 3.8e-4;
    double b1 = 0.5;             // 1/s
    double b3 = 6.25e-9;         // s, Chaigne-Askenfelt; converted to b3' = b3 c^2
    double strikePos = 0.12;     // fraction of L
    double hammerMass = 2.97e-3; // kg
    double feltK = 4.5e9;        // N / m^p
    double feltP = 2.5;
    double feltMu = 0.0;         // s/m
    double velocity = 2.0;       // m/s
};

struct Grid {
    int points = 0;      // N (intervals)
    double h = 0.0;      // m
    double k = 0.0;      // s
};

// Largest N (finest grid) that is stable for time step k.
Grid gridFor(const StringSpec& s, double k);
// Time step for which the stable grid has at least minPoints intervals.
double stepForPoints(const StringSpec& s, int minPoints);

struct ModalState {
    std::vector<double> q, qdot;   // per mode n = 1..count (index n - 1)
    std::vector<double> omega;     // numerical (grid) angular frequency, lossless
    std::vector<double> energy;    // (m/2)(qdot^2 + omega^2 q^2), m = mu L / 2
};

class FdtdString {
public:
    FdtdString(const StringSpec& s, double k);
    const Grid& grid() const { return grid_; }
    const StringSpec& spec() const { return spec_; }

    // Advances one time step.
    void step();
    double time() const { return n_ * grid_.k; }
    double hammerForce() const { return force_; }
    bool inContact() const { return force_ > 0.0; }
    double strikeDisplacement() const;
    double bridgeForce() const;
    double hammerVelocity() const { return (eta_ - etaPrev_) / grid_.k; }
    double hammerKineticEnergy() const;
    // Energy of the scheme (conserved exactly without loss and hammer).
    double stringEnergy() const;
    ModalState modes(int count) const;
    // Decay rate of mode n of the continuous Bensa-form equation.
    double modeDecayRate(int n) const;

    // For tests: set an initial displacement (zero velocity), no hammer.
    void setDisplacement(const std::vector<double>& y);
    std::vector<double>& displacement() { return y_; }

private:
    StringSpec spec_;
    Grid grid_;
    double c2_, b3p_, ei_, lambda2_, nu2_, lossMix_, b1k_;
    std::vector<double> y_, yPrev_, yNext_;
    int is_ = 0;              // left neighbour of the strike point
    double ws_ = 0.0;         // weight of is_ + 1
    double eta_ = 0.0, etaPrev_ = 0.0, xiPrev_ = 0.0, force_ = 0.0;
    bool hammer_ = true;
    long long n_ = 0;
};

}  // namespace pm_fdtd
