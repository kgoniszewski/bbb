// SPSC queue: single-thread semantics and a two-thread stress test.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "pm_base/spsc_queue.h"
#include "test_harness.h"

namespace {

struct Item {
    std::uint64_t seq;
    std::uint64_t check;
};

constexpr std::uint64_t kMix = 0x9E3779B97F4A7C15ull;

}  // namespace

TEST(empty_full_and_fifo_order) {
    pm::SpscQueue<int, 8> q;
    int v = -1;
    CHECK(q.empty());
    CHECK(!q.pop(v));
    for (int i = 0; i < 8; ++i) CHECK(q.push(i));
    CHECK(q.size() == 8);
    CHECK(!q.push(99));  // full: rejected, not overwritten
    for (int i = 0; i < 8; ++i) {
        CHECK(q.pop(v));
        CHECK(v == i);
    }
    CHECK(!q.pop(v));
    CHECK(q.empty());
}

TEST(wraps_around_many_times) {
    pm::SpscQueue<int, 4> q;
    int next = 0, expect = 0, v = 0;
    for (int round = 0; round < 10000; ++round) {
        const int k = 1 + round % 4;
        for (int i = 0; i < k; ++i) CHECK(q.push(next++));
        for (int i = 0; i < k; ++i) {
            CHECK(q.pop(v));
            CHECK(v == expect++);
        }
    }
    CHECK(q.empty());
}

TEST(two_thread_stress) {
    constexpr std::uint64_t kCount = 2000000;
    static pm::SpscQueue<Item, 256> q;
    std::atomic<bool> start{false};
    std::uint64_t fullRetries = 0;

    std::thread producer([&] {
        while (!start.load(std::memory_order_acquire)) {
        }
        for (std::uint64_t i = 0; i < kCount; ++i) {
            const Item it{i, i * kMix};
            while (!q.push(it)) {
                ++fullRetries;
                std::this_thread::yield();
            }
        }
    });

    std::uint64_t received = 0, errors = 0;
    start.store(true, std::memory_order_release);
    while (received < kCount) {
        Item it;
        if (q.pop(it)) {
            if (it.seq != received || it.check != received * kMix) ++errors;
            ++received;
        }
    }
    producer.join();
    Item extra;
    CHECK(!q.pop(extra));
    std::printf("  %llu items, %llu errors, producer saw full %llu times\n",
                static_cast<unsigned long long>(received), static_cast<unsigned long long>(errors),
                static_cast<unsigned long long>(fullRetries));
    CHECK(errors == 0);
    CHECK(received == kCount);
}
