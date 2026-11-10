#include "pm_voicing/complex_eigen.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pm {

namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

inline double abs1(cplx z) noexcept { return std::fabs(z.real()) + std::fabs(z.imag()); }

// Givens rotation G = [c s; -conj(s) c] (c real) with G [a; b] = [r; 0].
inline void givens(cplx a, cplx b, double& c, cplx& s) noexcept {
    const double na = std::abs(a), nb = std::abs(b);
    if (nb == 0.0) {
        c = 1.0;
        s = 0.0;
        return;
    }
    if (na == 0.0) {
        c = 0.0;
        s = std::conj(b) / nb;
        return;
    }
    const double r = std::hypot(na, nb);
    c = na / r;
    s = (a / na) * std::conj(b) / r;
}

}  // namespace

double norm1(int n, const cplx* A) noexcept {
    double m = 0.0;
    for (int j = 0; j < n; ++j) {
        double s = 0.0;
        for (int i = 0; i < n; ++i) s += std::abs(A[i * n + j]);
        m = std::max(m, s);
    }
    return m;
}

bool eigenComplex(int n, const cplx* A, cplx* lambda, cplx* V) noexcept {
    if (n < 1 || n > kMaxEigenN) return false;
    cplx H[kMaxEigenN * kMaxEigenN], Q[kMaxEigenN * kMaxEigenN];
    for (int i = 0; i < n * n; ++i) H[i] = A[i];
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) Q[i * n + j] = i == j ? 1.0 : 0.0;

    // Householder reduction to upper Hessenberg form, H = Q^H A Q.
    for (int k = 0; k + 2 < n; ++k) {
        cplx v[kMaxEigenN];
        double xn = 0.0;
        for (int i = k + 1; i < n; ++i) xn += std::norm(H[i * n + k]);
        xn = std::sqrt(xn);
        if (xn == 0.0) continue;
        const cplx x0 = H[(k + 1) * n + k];
        const cplx phase = std::abs(x0) > 0.0 ? x0 / std::abs(x0) : cplx(1.0);
        const cplx alpha = -phase * xn;
        for (int i = k + 1; i < n; ++i) v[i] = H[i * n + k];
        v[k + 1] -= alpha;
        double vn = 0.0;
        for (int i = k + 1; i < n; ++i) vn += std::norm(v[i]);
        vn = std::sqrt(vn);
        if (vn == 0.0) continue;
        for (int i = k + 1; i < n; ++i) v[i] /= vn;
        // H <- (I - 2 v v^H) H
        for (int j = 0; j < n; ++j) {
            cplx s = 0.0;
            for (int i = k + 1; i < n; ++i) s += std::conj(v[i]) * H[i * n + j];
            for (int i = k + 1; i < n; ++i) H[i * n + j] -= 2.0 * v[i] * s;
        }
        // H <- H (I - 2 v v^H), Q <- Q (I - 2 v v^H)
        for (cplx* M : {H, Q}) {
            for (int i = 0; i < n; ++i) {
                cplx s = 0.0;
                for (int j = k + 1; j < n; ++j) s += M[i * n + j] * v[j];
                for (int j = k + 1; j < n; ++j) M[i * n + j] -= 2.0 * s * std::conj(v[j]);
            }
        }
        for (int i = k + 2; i < n; ++i) H[i * n + k] = 0.0;
    }

    // Shifted QR iteration to upper triangular (Schur) form.
    int hi = n - 1, iter = 0;
    const int maxIter = 60 * n;
    int total = 0;
    while (hi > 0) {
        int l = hi;
        for (; l > 0; --l) {
            const double s = abs1(H[l * n + l]) + abs1(H[(l - 1) * n + l - 1]);
            if (abs1(H[l * n + l - 1]) <= kEps * (s > 0.0 ? s : 1e-300)) {
                H[l * n + l - 1] = 0.0;
                break;
            }
        }
        if (l == hi) {
            --hi;
            iter = 0;
            continue;
        }
        if (++total > maxIter * n) return false;
        ++iter;
        // Wilkinson shift: eigenvalue of the trailing 2x2 closer to its last
        // diagonal entry; an exceptional shift every 10 stagnating steps.
        cplx mu;
        const cplx a = H[(hi - 1) * n + hi - 1], b = H[(hi - 1) * n + hi];
        const cplx c = H[hi * n + hi - 1], d = H[hi * n + hi];
        if (iter % 10 == 0) {
            mu = d + std::abs(H[hi * n + hi - 1]) * cplx(0.75, 0.43);
        } else {
            const cplx tr2 = 0.5 * (a + d);
            const cplx disc = std::sqrt(0.25 * (a - d) * (a - d) + b * c);
            const cplx m1 = tr2 + disc, m2 = tr2 - disc;
            mu = std::abs(m1 - d) < std::abs(m2 - d) ? m1 : m2;
        }
        for (int k = l; k <= hi; ++k) H[k * n + k] -= mu;
        double cs[kMaxEigenN];
        cplx sn[kMaxEigenN];
        for (int k = l; k < hi; ++k) {
            givens(H[k * n + k], H[(k + 1) * n + k], cs[k], sn[k]);
            for (int j = k; j < n; ++j) {
                const cplx x = H[k * n + j], y = H[(k + 1) * n + j];
                H[k * n + j] = cs[k] * x + sn[k] * y;
                H[(k + 1) * n + j] = -std::conj(sn[k]) * x + cs[k] * y;
            }
        }
        for (int k = l; k < hi; ++k) {
            // Right multiplication by G_k^H on columns k, k+1.
            const int rows = std::min(k + 2, hi + 1);
            for (int i = 0; i < rows; ++i) {
                const cplx x = H[i * n + k], y = H[i * n + k + 1];
                H[i * n + k] = cs[k] * x + std::conj(sn[k]) * y;
                H[i * n + k + 1] = -sn[k] * x + cs[k] * y;
            }
            for (int i = 0; i < n; ++i) {
                const cplx x = Q[i * n + k], y = Q[i * n + k + 1];
                Q[i * n + k] = cs[k] * x + std::conj(sn[k]) * y;
                Q[i * n + k + 1] = -sn[k] * x + cs[k] * y;
            }
        }
        for (int k = l; k <= hi; ++k) H[k * n + k] += mu;
    }

    for (int i = 0; i < n; ++i) lambda[i] = H[i * n + i];

    // Eigenvectors of T by back-substitution, then V = Q X.
    double tnorm = 0.0;
    for (int i = 0; i < n; ++i)
        for (int j = i; j < n; ++j) tnorm = std::max(tnorm, std::abs(H[i * n + j]));
    const double small = std::max(kEps * tnorm, 1e-300);
    for (int k = 0; k < n; ++k) {
        cplx x[kMaxEigenN];
        for (int i = 0; i < n; ++i) x[i] = 0.0;
        x[k] = 1.0;
        for (int i = k - 1; i >= 0; --i) {
            cplx s = 0.0;
            for (int j = i + 1; j <= k; ++j) s += H[i * n + j] * x[j];
            cplx den = H[i * n + i] - H[k * n + k];
            if (std::abs(den) < small) den = small;
            x[i] = -s / den;
            // Keep the vector bounded (repeated eigenvalues can make it grow).
            double mx = 0.0;
            for (int j = i; j <= k; ++j) mx = std::max(mx, std::abs(x[j]));
            if (mx > 1e100)
                for (int j = i; j <= k; ++j) x[j] /= mx;
        }
        double nrm = 0.0;
        for (int i = 0; i < n; ++i) {
            cplx s = 0.0;
            for (int j = 0; j <= k; ++j) s += Q[i * n + j] * x[j];
            V[i * n + k] = s;
            nrm += std::norm(s);
        }
        nrm = std::sqrt(nrm);
        if (nrm > 0.0)
            for (int i = 0; i < n; ++i) V[i * n + k] /= nrm;
    }
    return true;
}

bool invertComplex(int n, const cplx* A, cplx* Ainv) noexcept {
    if (n < 1 || n > kMaxEigenN) return false;
    cplx M[kMaxEigenN * 2 * kMaxEigenN];
    const int w = 2 * n;
    double amax = 0.0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            M[i * w + j] = A[i * n + j];
            M[i * w + n + j] = i == j ? 1.0 : 0.0;
            amax = std::max(amax, std::abs(A[i * n + j]));
        }
    if (amax == 0.0) return false;
    for (int c = 0; c < n; ++c) {
        int p = c;
        for (int r = c + 1; r < n; ++r)
            if (std::abs(M[r * w + c]) > std::abs(M[p * w + c])) p = r;
        if (std::abs(M[p * w + c]) <= 1e-14 * amax) return false;
        if (p != c)
            for (int j = 0; j < w; ++j) std::swap(M[p * w + j], M[c * w + j]);
        const cplx inv = 1.0 / M[c * w + c];
        for (int j = 0; j < w; ++j) M[c * w + j] *= inv;
        for (int r = 0; r < n; ++r) {
            if (r == c) continue;
            const cplx f = M[r * w + c];
            if (f == 0.0) continue;
            for (int j = 0; j < w; ++j) M[r * w + j] -= f * M[c * w + j];
        }
    }
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) Ainv[i * n + j] = M[i * w + n + j];
    return true;
}

EigenResult eigenDecompose(int n, const cplx* A, cplx* lambda, cplx* V, cplx* Vinv) noexcept {
    EigenResult r;
    if (!eigenComplex(n, A, lambda, V)) return r;
    const double an = std::max(norm1(n, A), 1e-300);
    for (int j = 0; j < n; ++j) {
        double e = 0.0;
        for (int i = 0; i < n; ++i) {
            cplx s = -lambda[j] * V[i * n + j];
            for (int k = 0; k < n; ++k) s += A[i * n + k] * V[k * n + j];
            e += std::norm(s);
        }
        r.residual = std::max(r.residual, std::sqrt(e) / an);
    }
    if (!invertComplex(n, V, Vinv)) {
        r.condition = std::numeric_limits<double>::infinity();
        return r;
    }
    r.condition = norm1(n, V) * norm1(n, Vinv);
    r.ok = std::isfinite(r.condition);
    return r;
}

}  // namespace pm
