// render() must not allocate (plan §4.2): every global operator new/delete
// is replaced by a counting version, armed only around render() calls.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "piano_utils.h"
#include "test_harness.h"

namespace {
std::atomic<bool> gArmed{false};
std::atomic<long> gAllocs{0}, gFrees{0};

void* countedAlloc(std::size_t n, std::size_t align) {
    if (gArmed.load(std::memory_order_relaxed)) gAllocs.fetch_add(1, std::memory_order_relaxed);
    if (n == 0) n = 1;
    void* p = nullptr;
    if (align <= alignof(std::max_align_t)) p = std::malloc(n);
    else if (posix_memalign(&p, align, n) != 0) p = nullptr;
    return p;
}
void countedFree(void* p) {
    if (gArmed.load(std::memory_order_relaxed) && p) gFrees.fetch_add(1, std::memory_order_relaxed);
    std::free(p);
}
}  // namespace

void* operator new(std::size_t n) {
    void* p = countedAlloc(n, 0);
    if (!p) std::abort();
    return p;
}
void* operator new[](std::size_t n) { return operator new(n); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept { return countedAlloc(n, 0); }
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept { return countedAlloc(n, 0); }
void* operator new(std::size_t n, std::align_val_t a) {
    void* p = countedAlloc(n, static_cast<std::size_t>(a));
    if (!p) std::abort();
    return p;
}
void* operator new[](std::size_t n, std::align_val_t a) { return operator new(n, a); }
void* operator new(std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return countedAlloc(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return countedAlloc(n, static_cast<std::size_t>(a));
}
void operator delete(void* p) noexcept { countedFree(p); }
void operator delete[](void* p) noexcept { countedFree(p); }
void operator delete(void* p, std::size_t) noexcept { countedFree(p); }
void operator delete[](void* p, std::size_t) noexcept { countedFree(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { countedFree(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { countedFree(p); }
void operator delete(void* p, std::align_val_t) noexcept { countedFree(p); }
void operator delete[](void* p, std::align_val_t) noexcept { countedFree(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { countedFree(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { countedFree(p); }
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { countedFree(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { countedFree(p); }

using namespace pm_test;

TEST(counter_works) {
    gAllocs = 0;
    gArmed = true;
    auto* v = new std::vector<int>(10);
    gArmed = false;
    delete v;
    CHECK(gAllocs.load() >= 1);
}

TEST(render_does_not_allocate) {
    constexpr double kFs = 48000.0;
    pm::Engine eng;
    CHECK(eng.prepare(kFs, 64));
    std::vector<pm::Event> ev;
    for (int n = 21; n <= 108; ++n) ev.push_back(noteOn(100 + n, n, 30 + n));  // pool exhaustion
    ev.push_back(sustain(3000, true));
    for (int n = 21; n <= 108; n += 3) ev.push_back(noteOff(5000 + n, n));
    ev.push_back(noteOn(6000, 60, 127));  // re-strike of a sounding string
    pm::Event una = simple(6500, pm::EventType::UnaCorda);
    una.value = 40000;
    ev.push_back(una);
    ev.push_back(noteOn(6600, 62, 100));  // struck with una corda
    pm::Event sos = simple(7000, pm::EventType::Sostenuto);
    sos.value = 65535;
    ev.push_back(sos);
    ev.push_back(noteOff(7500, 62));
    sos.time = 8500;
    sos.value = 0;
    ev.push_back(sos);
    ev.push_back(sustain(9000, false));
    ev.push_back(simple(12000, pm::EventType::AllNotesOff));
    ev.push_back(noteOn(14000, 72, 90));
    ev.push_back(simple(16000, pm::EventType::Panic));
    ev.push_back(noteOn(40000, 48, 60));
    pm::Event host = noteOn(0, 50, 64);
    host.timeBase = pm::TimeBase::HostTime;
    ev.push_back(host);

    std::vector<float> l(256), r(256);
    pm::ClockMapping clk{0, 1.0};
    std::size_t next = 0;
    gAllocs = 0;
    gFrees = 0;
    for (int pos = 0; pos < 60000; pos += 256) {
        while (next < ev.size() && ev[next].time < static_cast<std::uint64_t>(pos + 256))
            eng.pushEvent(ev[next++]);
        clk.hostTimeAtBufferStart = static_cast<std::uint64_t>(pos);
        eng.setStereoWidth(pos % 1024 ? 0.35f : 0.5f);  // parameter changes too
        gArmed = true;
        eng.render(l.data(), r.data(), 256, &clk);
        gArmed = false;
    }
    pm::StrikeInfo info;
    int strikes = 0;
    while (eng.popStrikeInfo(info)) ++strikes;
    std::printf("  allocations during render: %ld, frees: %ld (%d strikes, %llu deferred)\n",
                gAllocs.load(), gFrees.load(), strikes,
                static_cast<unsigned long long>(eng.stats().deferredStrikes));
    CHECK(gAllocs.load() == 0);
    CHECK(gFrees.load() == 0);
    CHECK(strikes > 32);
    CHECK(eng.stats().deferredStrikes > 0);
}
