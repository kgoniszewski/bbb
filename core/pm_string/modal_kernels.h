// Internal kernel interface for ModeBank. Not part of the public API.
#pragma once

namespace pm::detail {

struct BankView {
    float* zr;
    float* zi;
    const float* dr;  // Re(c) - 1
    const float* ci;
    const float* wIn;
    const float* wOut;
};

// Both kernels advance the listed groups by n samples (n <= ModeBank::kCullInterval)
// and add the summed output to out[0..n).

// Reference: one mode at a time, plain scalar code.
void runModesScalar(const BankView& b, const int* groups, int nGroups, const float* in,
                    float* out, int n) noexcept;

// Vectorised: several groups interleaved per time step. 'scratch' holds
// n * simd::kWidth floats, aligned to the SIMD width.
void runModesSimd(const BankView& b, const int* groups, int nGroups, const float* in,
                  float* out, int n, float* scratch) noexcept;

}  // namespace pm::detail
