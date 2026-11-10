// Fast, accurate x^p for x > 0 (the hammer's felt law, F = K xi^p), as
// exp2(p log2 x) with table-driven log2 and exp2 in double precision:
//
//   log2(x): x = 2^e m, m in [1, 2); with c the centre of m's 1/128 cell,
//            log2 x = e + log2 c + log2(1 + r),  r = m / c - 1, |r| < 1/256,
//            log2(1 + r) from a 7-term series (error < 1e-17).
//   exp2(y): y = k / 128 + f, |f| <= 1/256; 2^y = 2^(k div 128) 2^((k mod 128) / 128) 2^f,
//            2^f from a 6-term series (error < 1e-18).
//
// Relative error of fastPow: below 2e-15 * max(1, |p log2 x|), i.e. about
// 1e-13 for the felt (measured in test_hammer). The tables are built at
// compile time, so nothing is initialised on first use (no guard, no lock).
// No division, no libm call; about 4x faster than std::pow on x86-64.
#pragma once

#include <cstdint>
#include <cstring>

namespace pm {

namespace fastmath_detail {

inline constexpr int kBits = 7;
inline constexpr int kSize = 1 << kBits;
inline constexpr double kLn2 = 0.69314718055994530942;
inline constexpr double kInvLn2 = 1.44269504088896340736;

constexpr double lnSeries(double c) {  // ln c for c in [1, 2]
    const double t = (c - 1.0) / (c + 1.0), t2 = t * t;
    double s = 0.0, p = t;
    for (int k = 0; k < 40; ++k) {
        s += p / (2 * k + 1);
        p *= t2;
    }
    return 2.0 * s;
}

constexpr double expSeries(double x) {  // e^x for |x| <= ln 2
    double s = 1.0, term = 1.0;
    for (int k = 1; k < 30; ++k) {
        term *= x / k;
        s += term;
    }
    return s;
}

struct Tables {
    double invC[kSize];   // 1 / c_i, c_i = 1 + (i + 0.5) / 128
    double log2C[kSize];  // log2(c_i) computed from the rounded 1 / invC
    double exp2T[kSize];  // 2^(i / 128)
};

constexpr Tables makeTables() {
    Tables t{};
    for (int i = 0; i < kSize; ++i) {
        const double c = 1.0 + (i + 0.5) / kSize;
        t.invC[i] = 1.0 / c;
        // log2 of the c that the rounded invC really inverts.
        t.log2C[i] = lnSeries(1.0 / t.invC[i]) * kInvLn2;
        t.exp2T[i] = expSeries(kLn2 * i / kSize);
    }
    return t;
}

inline constexpr Tables kTables = makeTables();

}  // namespace fastmath_detail

// log2(x), x > 0 and finite (normal).
inline double fastLog2(double x) noexcept {
    using namespace fastmath_detail;
    std::uint64_t b;
    std::memcpy(&b, &x, sizeof b);
    const int e = static_cast<int>((b >> 52) & 0x7FF) - 1023;
    const int i = static_cast<int>((b >> (52 - kBits)) & (kSize - 1));
    b = (b & 0x000FFFFFFFFFFFFFull) | 0x3FF0000000000000ull;
    double m;
    std::memcpy(&m, &b, sizeof m);
    const double r = m * kTables.invC[i] - 1.0;
    // ln(1 + r), Estrin-like grouping to shorten the dependency chain.
    const double r2 = r * r;
    const double a = r - 0.5 * r2;
    const double b3 = (1.0 / 3.0) - 0.25 * r;
    const double b5 = 0.2 - (1.0 / 6.0) * r + (1.0 / 7.0) * r2;
    const double ln1p = a + r2 * r * (b3 + r2 * b5);
    return static_cast<double>(e) + kTables.log2C[i] + ln1p * kInvLn2;
}

// 2^y for |y| < 1000.
inline double fastExp2(double y) noexcept {
    using namespace fastmath_detail;
    // Round y * 128 to nearest with the 1.5 * 2^52 trick (valid for |y| < 2^43).
    const double shifter = 6755399441055744.0;
    const double kd = y * kSize + shifter;
    std::uint64_t kb;
    std::memcpy(&kb, &kd, sizeof kb);
    const std::int64_t k = static_cast<std::int64_t>(static_cast<std::int32_t>(kb & 0xFFFFFFFFull));
    const double f = (y - (kd - shifter) / kSize) * kLn2;  // |f| <= ln2 / 256
    const double f2 = f * f;
    const double p = 1.0 + f + f2 * (0.5 + f * (1.0 / 6.0) + f2 * ((1.0 / 24.0) + f * (1.0 / 120.0) + f2 * (1.0 / 720.0)));
    const std::uint64_t sb = static_cast<std::uint64_t>((k >> kBits) + 1023) << 52;
    double scale;
    std::memcpy(&scale, &sb, sizeof scale);
    return p * kTables.exp2T[k & (kSize - 1)] * scale;
}

// x^p for x > 0 and p >= 1 (x below 1e-200 gives 0).
inline double fastPow(double x, double p) noexcept {
    return x < 1e-200 ? 0.0 : fastExp2(p * fastLog2(x));
}

}  // namespace pm
