// Small dense complex eigen-solver for the unison normal modes (plan §1.3).
// Non-real-time, no dependencies, matrices up to kMaxEigenN x kMaxEigenN.
//
// eigenComplex(): Householder reduction to upper Hessenberg form, then
// explicitly shifted QR iterations (Wilkinson shift, exceptional shifts on
// stagnation) with Givens rotations to the complex Schur form A = Q T Q^H.
// Eigenvectors come from back-substitution on T (denominators T_ii - T_kk
// below eps * |T| are replaced by eps * |T|, as in LAPACK's ztrevc, so
// repeated eigenvalues give finite vectors) and are normalised to unit
// 2-norm.
//
// A defective (or nearly defective) matrix yields (nearly) parallel
// eigenvectors: eigenDecompose() reports this through the condition number
// of V so the caller can perturb the problem or fall back.
//
// Matrices are row-major: A[i * n + j]. Eigenvector j is column j of V:
// V[i * n + j].
#pragma once

#include <complex>

namespace pm {

inline constexpr int kMaxEigenN = 16;

using cplx = std::complex<double>;

// Returns false if the QR iteration did not converge (not seen in practice
// for n <= 8; the iteration limit is 60 n per eigenvalue).
bool eigenComplex(int n, const cplx* A, cplx* lambda, cplx* V) noexcept;

// Gauss-Jordan inverse with partial pivoting. Returns false if a pivot is
// zero or below 1e-14 times the largest entry of A.
bool invertComplex(int n, const cplx* A, cplx* Ainv) noexcept;

// 1-norm of a square matrix.
double norm1(int n, const cplx* A) noexcept;

struct EigenResult {
    bool ok = false;          // decomposed and inverted
    double condition = 0.0;   // ||V||_1 ||V^-1||_1
    double residual = 0.0;    // max_j ||A v_j - lambda_j v_j|| / ||A||_1
};

// eigenComplex + inverse of V + quality measures.
EigenResult eigenDecompose(int n, const cplx* A, cplx* lambda, cplx* V, cplx* Vinv) noexcept;

}  // namespace pm
