// Complex eigen-solver for the unison normal modes (pm_voicing/complex_eigen.h).
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>

#include "pm_voicing/complex_eigen.h"
#include "test_harness.h"

using pm::cplx;

namespace {

double residual(int n, const cplx* A, const cplx* lam, const cplx* V) {
    double worst = 0.0;
    const double an = pm::norm1(n, A);
    for (int j = 0; j < n; ++j) {
        double e = 0.0;
        for (int i = 0; i < n; ++i) {
            cplx s = -lam[j] * V[i * n + j];
            for (int k = 0; k < n; ++k) s += A[i * n + k] * V[k * n + j];
            e += std::norm(s);
        }
        worst = std::max(worst, std::sqrt(e) / an);
    }
    return worst;
}

double inverseError(int n, const cplx* V, const cplx* Vi) {
    double worst = 0.0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            cplx s = 0.0;
            for (int k = 0; k < n; ++k) s += V[i * n + k] * Vi[k * n + j];
            worst = std::max(worst, std::abs(s - (i == j ? 1.0 : 0.0)));
        }
    return worst;
}

}  // namespace

// Random dense complex matrices, 1x1 to 6x6.
TEST(random_matrices) {
    std::mt19937 rng(7);
    std::normal_distribution<double> g;
    for (int n = 1; n <= 6; ++n) {
        double worstRes = 0.0, worstInv = 0.0;
        for (int trial = 0; trial < 500; ++trial) {
            cplx A[36], lam[6], V[36], Vi[36];
            for (int i = 0; i < n * n; ++i) A[i] = cplx(g(rng), g(rng));
            const pm::EigenResult r = pm::eigenDecompose(n, A, lam, V, Vi);
            CHECK(r.ok);
            worstRes = std::max(worstRes, residual(n, A, lam, V));
            worstInv = std::max(worstInv, inverseError(n, V, Vi) / r.condition);
        }
        std::printf("  %dx%d, 500 random matrices: worst ||Av - lv|| / ||A|| %.2e, ||V V^-1 - I|| / cond %.2e\n",
                    n, n, worstRes, worstInv);
        CHECK_LE(worstRes, 1e-13);
        CHECK_LE(worstInv, 1e-12);
    }
}

// Matrices shaped like the unison problem: a large imaginary diagonal
// (oscillator frequencies) plus a small complex coupling. Eigenvalues must
// be accurate relative to the coupling, not just to the frequency.
TEST(unison_shaped_matrices) {
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    double worst = 0.0;
    for (int trial = 0; trial < 300; ++trial) {
        const int n = 2 + trial % 5;
        cplx A[36], lam[6], V[36], Vi[36];
        const double w = 2.0 * 3.14159 * 261.6;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                A[i * n + j] = cplx(u(rng), u(rng)) * (i == j ? 0.5 : 1.0);
        for (int i = 0; i < n; ++i) A[i * n + i] += cplx(-0.5, w * (1.0 + 1e-4 * u(rng)));
        const pm::EigenResult r = pm::eigenDecompose(n, A, lam, V, Vi);
        CHECK(r.ok);
        // Residual relative to the coupling scale (about 1), not ||A|| ~ w.
        worst = std::max(worst, residual(n, A, lam, V) * pm::norm1(n, A));
    }
    std::printf("  300 unison-shaped matrices: worst ||Av - lv|| %.2e (coupling ~ 1, frequency ~ 1644)\n", worst);
    CHECK_LE(worst, 1e-9);
}

// Repeated eigenvalues of a diagonalizable matrix: identity, a diagonal
// with pairs, and identical oscillators coupled through one bridge
// (a I - c 11^T: one symmetric mode, a degenerate antisymmetric pair).
TEST(degenerate_cases) {
    {
        cplx A[36] = {}, lam[6], V[36], Vi[36];
        for (int i = 0; i < 6; ++i) A[i * 6 + i] = 2.0;
        const pm::EigenResult r = pm::eigenDecompose(6, A, lam, V, Vi);
        std::printf("  6x6 identity x 2: ok %d, condition %.3g, residual %.2e\n", r.ok, r.condition, r.residual);
        CHECK(r.ok && r.condition < 10.0 && r.residual < 1e-15);
    }
    {
        cplx A[36] = {}, lam[6], V[36], Vi[36];
        const cplx d[6] = {cplx(0, 5), cplx(0, 5), cplx(-1, 2), cplx(-1, 2), cplx(0, 5), cplx(3, 0)};
        for (int i = 0; i < 6; ++i) A[i * 6 + i] = d[i];
        const pm::EigenResult r = pm::eigenDecompose(6, A, lam, V, Vi);
        CHECK(r.ok && r.condition < 10.0 && r.residual < 1e-15);
    }
    for (int n = 2; n <= 6; ++n) {
        cplx A[36], lam[6], V[36], Vi[36];
        const cplx c(0.7, 0.2), a(-0.3, 1644.0);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) A[i * n + j] = -c + (i == j ? a : cplx(0.0));
        const pm::EigenResult r = pm::eigenDecompose(n, A, lam, V, Vi);
        // Expected: a - n c once, a (n - 1) times.
        int sym = 0, anti = 0;
        for (int j = 0; j < n; ++j) {
            if (std::abs(lam[j] - (a - double(n) * c)) < 1e-9) ++sym;
            if (std::abs(lam[j] - a) < 1e-9) ++anti;
        }
        std::printf("  %d identical coupled oscillators: ok %d, condition %.3g, residual %.2e, "
                    "symmetric %d, degenerate %d\n",
                    n, r.ok, r.condition, r.residual, sym, anti);
        CHECK(r.ok && r.condition < 100.0 && r.residual < 1e-13);
        CHECK(sym == 1 && anti == n - 1);
    }
}

// Near-defective (exceptional point) and defective matrices: two oscillators
// with resistive coupling c and detuning d have eigenvalues
// -c +- sqrt(c^2 - d^2 / 4); at d = 2c they coalesce and the matrix is a
// Jordan block. The solver must not fail silently: V is reported as
// ill-conditioned (and the unison solver then perturbs, see test_unison).
TEST(near_defective_reported) {
    const double c = 1.0;
    for (double eps : {1e-2, 1e-4, 1e-8, 0.0}) {
        const double d = 2.0 * c * (1.0 + eps);
        cplx A[4] = {cplx(-c, 0.5 * d), cplx(-c, 0.0), cplx(-c, 0.0), cplx(-c, -0.5 * d)}, lam[2], V[4], Vi[4];
        const pm::EigenResult r = pm::eigenDecompose(2, A, lam, V, Vi);
        std::printf("  detuning 2c (1 + %.0e): ok %d, condition %.3g, residual %.2e\n", eps, r.ok,
                    r.condition, r.residual);
        CHECK(r.residual < 1e-7);
        if (eps >= 1e-4) CHECK(r.ok);
        if (eps <= 1e-8) CHECK(!r.ok || r.condition > 1e3);
    }
    cplx J[4] = {cplx(1, 0), cplx(1, 0), cplx(0, 0), cplx(1, 0)}, lam[2], V[4], Vi[4];
    const pm::EigenResult r = pm::eigenDecompose(2, J, lam, V, Vi);
    std::printf("  Jordan block: ok %d, condition %.3g\n", r.ok, r.condition);
    CHECK(!r.ok || r.condition > 1e6);
}
