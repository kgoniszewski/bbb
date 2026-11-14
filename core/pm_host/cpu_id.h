// Which CPU the calling thread is running on, and a per-CPU tally of render
// calls (plan §4.5). Used by the load meter (audio thread), the benchmark's
// scenarios and the speed probe, to tell whether a slow buffer ran on an
// efficiency core.
//
// currentCpu() is real-time safe where it is implemented:
//  - Apple platforms: pthread_cpu_number_np (macOS 11, iOS 14.2; <pthread.h>).
//    libpthread implements it with _os_cpu_number() from libsyscall's
//    os/tsd.h, an inline read of a per-CPU value the kernel keeps in a
//    user-readable register (a thread-ID register on arm64, a segment / IDT
//    descriptor on x86): no system call, no lock, no allocation. This is from
//    the open-source libpthread/xnu headers of earlier releases; the arm64
//    branch of the current header was not available to check here.
//  - Linux: sched_getcpu (glibc: the rseq area or the vDSO, no system call
//    on current kernels and glibc >= 2.35).
//  - elsewhere: -1 (unknown).
// The value can be stale by the time it is used (the thread may migrate at
// any instant), which is why a render call reads it at its start and at its
// end.
#pragma once

#include <cstdint>

#if defined(__APPLE__)
#include <pthread.h>
#elif defined(__linux__)
#include <sched.h>
#endif

namespace pm::host {

inline int currentCpu() noexcept {
#if defined(__APPLE__)
    std::size_t n = 0;
    return pthread_cpu_number_np(&n) == 0 ? static_cast<int>(n) : -1;
#elif defined(__linux__)
    return sched_getcpu();  // -1 on failure
#else
    return -1;
#endif
}

inline constexpr int kMaxCpus = 64;

// Per-CPU totals of one thread's render calls and speed-probe runs. Plain
// data (one writer); the load meter keeps an atomic mirror of it.
struct CpuUsage {
    std::uint64_t tracked = 0;     // render calls whose start CPU was read and < kMaxCpus
    std::uint64_t unknown = 0;     // render calls without a usable CPU number
    std::uint64_t migrations = 0;  // render calls that ended on another CPU than they started on
    int worstStart = -1, worstEnd = -1;  // the most expensive buffer's CPUs (-1: unknown)
    std::uint32_t buffers[kMaxCpus] = {};  // by the CPU a render call started on
    double renderUs[kMaxCpus] = {};        // summed render time of those calls
    std::uint32_t probeRuns[kMaxCpus] = {};
    double probeLatencyUs[kMaxCpus] = {};     // summed
    double probeThroughputUs[kMaxCpus] = {};  // summed

    void addBuffer(int cpuStart, int cpuEnd, double us) noexcept {
        if (cpuStart >= 0 && cpuStart < kMaxCpus) {
            ++tracked;
            ++buffers[cpuStart];
            renderUs[cpuStart] += us;
        } else {
            ++unknown;
        }
        if (cpuStart >= 0 && cpuEnd >= 0 && cpuStart != cpuEnd) ++migrations;
    }
    void addProbe(int cpu, double latencyUs, double throughputUs) noexcept {
        if (cpu < 0 || cpu >= kMaxCpus) return;
        ++probeRuns[cpu];
        probeLatencyUs[cpu] += latencyUs;
        probeThroughputUs[cpu] += throughputUs;
    }
};

}  // namespace pm::host
