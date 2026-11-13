// Cache-line aligned, move-only array for structure-of-arrays data.
// Allocation happens only in allocate(), which must not be called on the
// audio thread.
//
// Cache-set staggering. The hot kernels stream many arrays of the same
// length in lockstep at the same index (the hammer solver: 17 to 19 arrays
// per substep; the bank kernels 6 to 8). If the allocator places those
// arrays at the same offset modulo the L1 way size (4 KiB on x86, 16 KiB on
// Apple silicon), every one of those streams maps to the same cache set and
// an 8-way L1 thrashes: a page- or 32 KiB-granular allocator does exactly
// that for arrays of tens of KiB, and with a 512-byte quantum the outcome
// depends on the heap's history. Simulated on a 128 KiB 8-way L1, a 12-note
// onset buffer then takes 715 000 L1 read misses instead of 6 600 for the
// same instructions. Arrays of kStaggerMinBytes or more therefore start at
// an address chosen modulo 16 KiB (over-allocating by up to 16 KiB), each
// one 37 cache lines on from the previous, so arrays allocated together (a
// bank's and its contact tables' arrays) spread over the sets at equal
// indices whatever the allocator does.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>
#include <utility>

namespace pm {

inline constexpr std::size_t kCacheLine = 64;

namespace detail {
// Offsets modulo 16 KiB (the L1 way size on Apple silicon; a multiple of
// x86's 4 KiB), in steps of 37 lines.
inline constexpr std::size_t kStaggerSpan = 16384;
inline constexpr std::size_t kStaggerStep = 37;
inline constexpr std::size_t kStaggerMinBytes = 8192;  // smaller arrays: plain cache-line alignment
inline std::atomic<unsigned> staggerCounter{0};
// Bytes held by all live AlignedArrays (process-wide, including the stagger
// over-allocation), for memory reports; changed only in allocate / release.
inline std::atomic<std::size_t> liveBytes{0};
}  // namespace detail

inline std::size_t alignedLiveBytes() noexcept { return detail::liveBytes.load(std::memory_order_relaxed); }

template <typename T>
class AlignedArray {
    static_assert(std::is_trivially_copyable_v<T>, "AlignedArray holds plain data only");

public:
    AlignedArray() noexcept = default;
    ~AlignedArray() { release(); }

    AlignedArray(const AlignedArray&) = delete;
    AlignedArray& operator=(const AlignedArray&) = delete;

    AlignedArray(AlignedArray&& o) noexcept
        : data_(std::exchange(o.data_, nullptr)), raw_(std::exchange(o.raw_, nullptr)),
          size_(std::exchange(o.size_, 0)) {}
    AlignedArray& operator=(AlignedArray&& o) noexcept {
        if (this != &o) {
            release();
            data_ = std::exchange(o.data_, nullptr);
            raw_ = std::exchange(o.raw_, nullptr);
            size_ = std::exchange(o.size_, 0);
        }
        return *this;
    }

    // Zero-initialised. The byte size is rounded up to a whole number of cache
    // lines so SIMD loops may read a full line past the last element.
    // Returns false on allocation failure (no exceptions).
    bool allocate(std::size_t count) noexcept {
        release();
        if (count == 0) return true;
        const std::size_t bytes = ((count * sizeof(T) + kCacheLine - 1) / kCacheLine) * kCacheLine;
        const bool stagger = bytes >= detail::kStaggerMinBytes;
        const std::size_t extra = stagger ? detail::kStaggerSpan - kCacheLine : 0;
        void* p = ::operator new(bytes + extra, std::align_val_t{kCacheLine}, std::nothrow);
        if (!p) return false;
        detail::liveBytes.fetch_add(bytes + extra, std::memory_order_relaxed);
        std::size_t offset = 0;
        if (stagger) {
            // Successive large arrays start kStaggerStep lines apart modulo
            // the span (an odd step: all 256 offsets are used before one
            // repeats, and any 64 successive ones differ modulo 4 KiB too).
            const unsigned slot = detail::staggerCounter.fetch_add(1, std::memory_order_relaxed);
            const std::size_t lines = detail::kStaggerSpan / kCacheLine;
            const std::size_t want = (slot * detail::kStaggerStep % lines) * kCacheLine;
            const std::size_t have = reinterpret_cast<std::uintptr_t>(p) % detail::kStaggerSpan;
            offset = (want + detail::kStaggerSpan - have) % detail::kStaggerSpan;
        }
        std::memset(static_cast<unsigned char*>(p) + offset, 0, bytes);
        raw_ = p;
        data_ = reinterpret_cast<T*>(static_cast<unsigned char*>(p) + offset);
        size_ = count;
        return true;
    }

    void release() noexcept {
        if (raw_) {
            const std::size_t bytes = ((size_ * sizeof(T) + kCacheLine - 1) / kCacheLine) * kCacheLine;
            const std::size_t extra = bytes >= detail::kStaggerMinBytes ? detail::kStaggerSpan - kCacheLine : 0;
            detail::liveBytes.fetch_sub(bytes + extra, std::memory_order_relaxed);
            ::operator delete(raw_, std::align_val_t{kCacheLine}, std::nothrow);
        }
        data_ = nullptr;
        raw_ = nullptr;
        size_ = 0;
    }

    T* data() noexcept { return data_; }
    const T* data() const noexcept { return data_; }
    std::size_t size() const noexcept { return size_; }
    T& operator[](std::size_t i) noexcept { return data_[i]; }
    const T& operator[](std::size_t i) const noexcept { return data_[i]; }

private:
    T* data_ = nullptr;
    void* raw_ = nullptr;
    std::size_t size_ = 0;
};

}  // namespace pm
