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

}  // namespace pm::detail
