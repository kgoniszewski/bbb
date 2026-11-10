// Cache-line aligned, move-only array for structure-of-arrays data.
// Allocation happens only in allocate(), which must not be called on the
// audio thread.
#pragma once

#include <cstddef>
#include <cstring>
#include <new>
#include <type_traits>
#include <utility>

namespace pm {

inline constexpr std::size_t kCacheLine = 64;

template <typename T>
class AlignedArray {
    static_assert(std::is_trivially_copyable_v<T>, "AlignedArray holds plain data only");

public:
    AlignedArray() noexcept = default;
    ~AlignedArray() { release(); }

    AlignedArray(const AlignedArray&) = delete;
    AlignedArray& operator=(const AlignedArray&) = delete;

    AlignedArray(AlignedArray&& o) noexcept
        : data_(std::exchange(o.data_, nullptr)), size_(std::exchange(o.size_, 0)) {}
    AlignedArray& operator=(AlignedArray&& o) noexcept {
        if (this != &o) {
            release();
            data_ = std::exchange(o.data_, nullptr);
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
        void* p = ::operator new(bytes, std::align_val_t{kCacheLine}, std::nothrow);
        if (!p) return false;
        std::memset(p, 0, bytes);
        data_ = static_cast<T*>(p);
        size_ = count;
        return true;
    }

    void release() noexcept {
        if (data_) ::operator delete(data_, std::align_val_t{kCacheLine}, std::nothrow);
        data_ = nullptr;
        size_ = 0;
    }

    T* data() noexcept { return data_; }
    const T* data() const noexcept { return data_; }
    std::size_t size() const noexcept { return size_; }
    T& operator[](std::size_t i) noexcept { return data_[i]; }
    const T& operator[](std::size_t i) const noexcept { return data_[i]; }

private:
    T* data_ = nullptr;
    std::size_t size_ = 0;
};

}  // namespace pm
