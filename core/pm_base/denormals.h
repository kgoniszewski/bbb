// RAII guard that enables flush-to-zero (and denormals-are-zero on x86) for
// the current thread and restores the previous mode on exit. Place one at the
// top of every audio render callback.
#pragma once

#include <cstdint>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <xmmintrin.h>
#define PM_FTZ_X86 1
#elif defined(__aarch64__) && (defined(__GNUC__) || defined(__clang__))
#define PM_FTZ_AARCH64 1
#elif defined(__arm__) && defined(__ARM_FP) && (defined(__GNUC__) || defined(__clang__))
#define PM_FTZ_ARM32 1
#endif

namespace pm {

inline constexpr bool kFlushDenormalsSupported =
#if defined(PM_FTZ_X86) || defined(PM_FTZ_AARCH64) || defined(PM_FTZ_ARM32)
    true;
#else
    false;
#endif

class ScopedFlushDenormals {
public:
    ScopedFlushDenormals() noexcept {
#if defined(PM_FTZ_X86)
        saved_ = _mm_getcsr();
        _mm_setcsr(static_cast<unsigned>(saved_) | 0x8040u);  // FTZ (bit 15) | DAZ (bit 6)
#elif defined(PM_FTZ_AARCH64)
        std::uint64_t fpcr;
        __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
        saved_ = fpcr;
        fpcr |= (std::uint64_t{1} << 24);  // FPCR.FZ
        __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#elif defined(PM_FTZ_ARM32)
        std::uint32_t fpscr;
        __asm__ __volatile__("vmrs %0, fpscr" : "=r"(fpscr));
        saved_ = fpscr;
        fpscr |= (std::uint32_t{1} << 24);  // FPSCR.FZ
        __asm__ __volatile__("vmsr fpscr, %0" : : "r"(fpscr));
#endif
    }

    ~ScopedFlushDenormals() {
#if defined(PM_FTZ_X86)
        _mm_setcsr(static_cast<unsigned>(saved_));
#elif defined(PM_FTZ_AARCH64)
        const std::uint64_t fpcr = saved_;
        __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#elif defined(PM_FTZ_ARM32)
        const std::uint32_t fpscr = static_cast<std::uint32_t>(saved_);
        __asm__ __volatile__("vmsr fpscr, %0" : : "r"(fpscr));
#endif
    }

    ScopedFlushDenormals(const ScopedFlushDenormals&) = delete;
    ScopedFlushDenormals& operator=(const ScopedFlushDenormals&) = delete;

private:
    [[maybe_unused]] std::uint64_t saved_ = 0;
};

}  // namespace pm
