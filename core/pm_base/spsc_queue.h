// Wait-free single-producer / single-consumer ring buffer (plan §3.1).
// Exactly one thread may call push() and exactly one other thread may call
// pop(). Indices are free-running 32-bit counters; because N divides 2^32,
// unsigned wrap-around keeps (tail - head) correct.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace pm {

template <typename T, std::size_t N>
class SpscQueue {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");
    static_assert(N <= (std::size_t{1} << 31), "N too large for 32-bit indices");
    static_assert(std::is_trivially_copyable_v<T>, "T is copied with plain assignment");

public:
    static constexpr std::size_t capacity() noexcept { return N; }

    // Producer thread. Returns false (and drops the element) when full.
    bool push(const T& e) noexcept {
        const std::uint32_t t = tail_.load(std::memory_order_relaxed);
        if (t - head_.load(std::memory_order_acquire) == N) return false;
        buf_[t & (N - 1)] = e;
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    // Consumer thread. Returns false when empty.
    bool pop(T& e) noexcept {
        const std::uint32_t h = head_.load(std::memory_order_relaxed);
        if (h == tail_.load(std::memory_order_acquire)) return false;
        e = buf_[h & (N - 1)];
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    // Approximate when called concurrently; exact from either side when the
    // other side is idle.
    std::size_t size() const noexcept {
        return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
    }
    bool empty() const noexcept { return size() == 0; }

private:
    std::array<T, N> buf_{};
    alignas(64) std::atomic<std::uint32_t> head_{0};  // written by consumer
    alignas(64) std::atomic<std::uint32_t> tail_{0};  // written by producer
    // alignas(64) also rounds sizeof up, so nothing shares tail_'s line.
};

}  // namespace pm
