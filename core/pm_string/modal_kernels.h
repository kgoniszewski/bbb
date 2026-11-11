// Internal kernel interface for ModeBank. Not part of the public API.
#pragma once

namespace pm::detail {

struct BankView {
    float* zr;
    float* zi;
    const float* dr;  // Re(c) - 1
    const float* ci;
    const float* wInRe;  // complex input weight: u enters Re z with wInRe, Im z with wInIm
    const float* wInIm;
    const float* wOut;   // real output weight (normal modes are phase-normalised)
};

// Both kernels advance the listed groups by n samples (n <= ModeBank::kCullInterval)
// and add the summed output to out[0..n). 'in' may be null: no input, and
// the input multiply-adds are skipped (bit-identical to a zero input).

// Reference: one mode at a time, plain scalar code.
void runModesScalar(const BankView& b, const int* groups, int nGroups, const float* in,
                    float* out, int n) noexcept;

// Vectorised: several groups interleaved per time step. 'scratch' holds
// n * simd::kWidth floats, aligned to the SIMD width.
void runModesSimd(const BankView& b, const int* groups, int nGroups, const float* in,
                  float* out, int n, float* scratch) noexcept;

// Block-coupled processing (soundboard loop, mode_bank.h). Both advance the
// listed groups in registers over n <= ModeBank::kCullInterval samples.
//
// Impulse: for each listed group q, out[q * stride + k] = sum over its modes
// of wOut Im(c^(k+1) wIn), k < n (its response to a unit input at sample 0,
// from the second tap on); the state is neither read nor written.
void impulseScalar(const BankView& b, const int* groups, int nGroups, float* out, int stride, int n) noexcept;
void impulseSimd(const BankView& b, const int* groups, int nGroups, float* out, int stride, int n) noexcept;
// Forced update: z += wIn * sum_{m<n} c^(n-1-m) u[m] (Horner in c), the part
// of the state the input u[0..n) adds on top of the free evolution. A free
// pass over the same n samples followed by this pass gives the state of
// runModes* with input u, up to rounding.
void forcedScalar(const BankView& b, const int* groups, int nGroups, const float* u, int n) noexcept;
void forcedSimd(const BankView& b, const int* groups, int nGroups, const float* u, int n) noexcept;

}  // namespace pm::detail
