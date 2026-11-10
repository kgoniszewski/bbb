// Minimal float SIMD abstraction, selected at compile time:
//   NEON (AArch64 / ARMv7 with NEON)      width 4
//   AVX2 + FMA (x86, when enabled)        width 8
//   SSE2 (x86-64 baseline)                width 4
//   portable scalar fallback              width 4 (plain struct)
// Kernels are written once against these functions; only this header differs
// per instruction set. All loads/stores require kWidth*4-byte alignment.
// Define PM_SIMD_FORCE_SCALAR to test the portable fallback on any machine.
#pragma once

#if defined(PM_SIMD_FORCE_SCALAR)
#define PM_SIMD_SCALAR 1
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define PM_SIMD_NEON 1
#elif defined(__AVX2__) && defined(__FMA__)
#include <immintrin.h>
#define PM_SIMD_AVX2 1
#elif defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#define PM_SIMD_SSE2 1
#else
#define PM_SIMD_SCALAR 1
#endif

namespace pm::simd {

#if defined(PM_SIMD_NEON)

using vf = float32x4_t;
inline constexpr int kWidth = 4;
inline constexpr const char* kIsaName = "neon";

inline vf load(const float* p) noexcept { return vld1q_f32(p); }
inline void store(float* p, vf v) noexcept { vst1q_f32(p, v); }
inline vf set1(float x) noexcept { return vdupq_n_f32(x); }
inline vf zero() noexcept { return vdupq_n_f32(0.0f); }
inline vf add(vf a, vf b) noexcept { return vaddq_f32(a, b); }
inline vf sub(vf a, vf b) noexcept { return vsubq_f32(a, b); }
inline vf mul(vf a, vf b) noexcept { return vmulq_f32(a, b); }
#if defined(__aarch64__)
// a*b + c and c - a*b, fused.
inline vf fmadd(vf a, vf b, vf c) noexcept { return vfmaq_f32(c, a, b); }
inline vf fnmadd(vf a, vf b, vf c) noexcept { return vfmsq_f32(c, a, b); }
inline float hsum(vf v) noexcept { return vaddvq_f32(v); }
#else
inline vf fmadd(vf a, vf b, vf c) noexcept { return vmlaq_f32(c, a, b); }
inline vf fnmadd(vf a, vf b, vf c) noexcept { return vmlsq_f32(c, a, b); }
inline float hsum(vf v) noexcept {
    const float32x2_t s = vadd_f32(vget_low_f32(v), vget_high_f32(v));
    return vget_lane_f32(vpadd_f32(s, s), 0);
}
#endif

#elif defined(PM_SIMD_AVX2)

using vf = __m256;
inline constexpr int kWidth = 8;
inline constexpr const char* kIsaName = "avx2+fma";

inline vf load(const float* p) noexcept { return _mm256_load_ps(p); }
inline void store(float* p, vf v) noexcept { _mm256_store_ps(p, v); }
inline vf set1(float x) noexcept { return _mm256_set1_ps(x); }
inline vf zero() noexcept { return _mm256_setzero_ps(); }
inline vf add(vf a, vf b) noexcept { return _mm256_add_ps(a, b); }
inline vf sub(vf a, vf b) noexcept { return _mm256_sub_ps(a, b); }
inline vf mul(vf a, vf b) noexcept { return _mm256_mul_ps(a, b); }
inline vf fmadd(vf a, vf b, vf c) noexcept { return _mm256_fmadd_ps(a, b, c); }
inline vf fnmadd(vf a, vf b, vf c) noexcept { return _mm256_fnmadd_ps(a, b, c); }
inline float hsum(vf v) noexcept {
    __m128 s = _mm_add_ps(_mm256_castps256_ps128(v), _mm256_extractf128_ps(v, 1));
    s = _mm_add_ps(s, _mm_movehl_ps(s, s));
    s = _mm_add_ss(s, _mm_shuffle_ps(s, s, 0x55));
    return _mm_cvtss_f32(s);
}

#elif defined(PM_SIMD_SSE2)

using vf = __m128;
inline constexpr int kWidth = 4;
inline constexpr const char* kIsaName = "sse2";

inline vf load(const float* p) noexcept { return _mm_load_ps(p); }
inline void store(float* p, vf v) noexcept { _mm_store_ps(p, v); }
inline vf set1(float x) noexcept { return _mm_set1_ps(x); }
inline vf zero() noexcept { return _mm_setzero_ps(); }
inline vf add(vf a, vf b) noexcept { return _mm_add_ps(a, b); }
inline vf sub(vf a, vf b) noexcept { return _mm_sub_ps(a, b); }
inline vf mul(vf a, vf b) noexcept { return _mm_mul_ps(a, b); }
// SSE2 has no FMA: these round twice, unlike the NEON and AVX2 versions.
inline vf fmadd(vf a, vf b, vf c) noexcept { return _mm_add_ps(_mm_mul_ps(a, b), c); }
inline vf fnmadd(vf a, vf b, vf c) noexcept { return _mm_sub_ps(c, _mm_mul_ps(a, b)); }
inline float hsum(vf v) noexcept {
    __m128 s = _mm_add_ps(v, _mm_movehl_ps(v, v));
    s = _mm_add_ss(s, _mm_shuffle_ps(s, s, 0x55));
    return _mm_cvtss_f32(s);
}

#else  // PM_SIMD_SCALAR

struct vf {
    float v[4];
};
inline constexpr int kWidth = 4;
inline constexpr const char* kIsaName = "scalar-fallback";

inline vf load(const float* p) noexcept { return {{p[0], p[1], p[2], p[3]}}; }
inline void store(float* p, vf a) noexcept {
    for (int i = 0; i < 4; ++i) p[i] = a.v[i];
}
inline vf set1(float x) noexcept { return {{x, x, x, x}}; }
inline vf zero() noexcept { return set1(0.0f); }
inline vf add(vf a, vf b) noexcept {
    for (int i = 0; i < 4; ++i) a.v[i] += b.v[i];
    return a;
}
inline vf sub(vf a, vf b) noexcept {
    for (int i = 0; i < 4; ++i) a.v[i] -= b.v[i];
    return a;
}
inline vf mul(vf a, vf b) noexcept {
    for (int i = 0; i < 4; ++i) a.v[i] *= b.v[i];
    return a;
}
inline vf fmadd(vf a, vf b, vf c) noexcept { return add(mul(a, b), c); }
inline vf fnmadd(vf a, vf b, vf c) noexcept { return sub(c, mul(a, b)); }
inline float hsum(vf a) noexcept { return (a.v[0] + a.v[1]) + (a.v[2] + a.v[3]); }

#endif

}  // namespace pm::simd
