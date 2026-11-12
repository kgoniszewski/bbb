// App host through the C API (pm_api.h, "App host"): host-time scheduling of
// MIDI events into render buffers, the two producer queues, the MIDI
// monitor ring, the prepare gate under a running audio thread, and no
// allocation on the MIDI and audio paths.
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <thread>
#include <vector>

#include "pm_api/pm_api.h"
#include "pm_engine/events.h"
#include "test_harness.h"

// ---- counting allocator, armed only around the calls under test
namespace {
std::atomic<bool> gArmed{false};
std::atomic<long> gAllocs{0};
void* countedAlloc(std::size_t n, std::size_t align) {
    if (gArmed.load(std::memory_order_relaxed)) gAllocs.fetch_add(1, std::memory_order_relaxed);
    if (n == 0) n = 1;
    void* p = nullptr;
    if (align <= alignof(std::max_align_t)) p = std::malloc(n);
    else if (posix_memalign(&p, align, n) != 0) p = nullptr;
    return p;
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
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }

namespace {

constexpr double kFs = 48000.0;
constexpr int kFrames = 256;
// Apple silicon's timebase: 125/3 ns per tick (24 MHz), 500 ticks per sample at 48 kHz.
constexpr std::uint32_t kNumer = 125, kDenom = 3;
constexpr std::uint64_t kTicks = 500;
constexpr std::uint64_t kT0 = 1000000000000ull;

std::uint64_t bufferStart(int b) { return kT0 + static_cast<std::uint64_t>(b) * kFrames * kTicks; }

pm_host* makeHost(bool prepare) {
    pm_host* h = pm_host_create();
    if (!h) return nullptr;
    pm_host_set_timebase(h, kNumer, kDenom);
    // The board does not matter here and the plain engine prepares faster.
    pm_engine_set_param(pm_host_engine(h), PM_PARAM_SOUNDBOARD_ENABLED, 0.0);
    if (prepare && !pm_host_prepare(h, kFs, kFrames)) {
        pm_host_destroy(h);
        return nullptr;
    }
    return h;
}

pm_event sampleEvent(std::uint8_t type, std::uint64_t sample, std::uint8_t note, std::uint16_t value) {
    pm_event e;
    std::memset(&e, 0, sizeof e);
    e.type = type;
    e.time_base = PM_TIME_SAMPLE;
    e.time = sample;
    e.note = note;
    e.value = value;
    return e;
}

// A Core MIDI MIDIEventList with one packet (packed layout, see midi.h).
std::vector<unsigned char> eventList(std::uint64_t stamp, const std::vector<std::uint32_t>& words) {
    std::vector<unsigned char> b(8 + 12 + 4 * words.size());
    const std::int32_t protocol = 1;
    const std::uint32_t packets = 1, n = static_cast<std::uint32_t>(words.size());
    std::memcpy(&b[0], &protocol, 4);
    std::memcpy(&b[4], &packets, 4);
    std::memcpy(&b[8], &stamp, 8);
    std::memcpy(&b[16], &n, 4);
    std::memcpy(&b[20], words.data(), 4 * words.size());
    return b;
}

}  // namespace

TEST(time_conversion) {
    CHECK_NEAR(pm_host_ticks_per_sample(48000.0, 125, 3), 500.0, 1e-9);
    CHECK_NEAR(pm_host_ticks_per_sample(44100.0, 1, 1), 1e9 / 44100.0, 1e-6);
    CHECK(pm_host_ticks_per_sample(0.0, 1, 1) == 0.0 && pm_host_ticks_per_sample(48000.0, 0, 1) == 0.0);
    const pm::ClockMapping c{1000, 500.0};
    CHECK(pm::hostTimeToFrameDelta(1000, c) == 0);
    CHECK(pm::hostTimeToFrameDelta(1499, c) == 0);
    CHECK(pm::hostTimeToFrameDelta(1500, c) == 1);
    CHECK(pm::hostTimeToFrameDelta(999, c) == -1);  // floor, not truncation
    CHECK(pm::hostTimeToFrameDelta(0, c) == -2);
    // Across the counter's wrap.
    const pm::ClockMapping w{~0ull - 999, 500.0};
    CHECK(pm::hostTimeToFrameDelta(500, w) == 3);
    CHECK(pm::hostTimeToFrameDelta(~0ull - 2000, w) == -3);
    // Absurd stamps stay defined.
    const pm::ClockMapping tiny{0, 1e-9};
    CHECK(pm::hostTimeToFrameDelta(~0ull >> 1, tiny) == static_cast<std::int64_t>(1e15));
}

TEST(midi_host_time_lands_on_exact_sample) {
    // Host A gets MIDI with host timestamps (byte stream, UMP and event-list
    // paths) as the buffers go by; host B gets the expected absolute sample
    // times through the UI queue. The outputs must be identical.
    pm_host* a = makeHost(true);
    pm_host* b = makeHost(true);
    CHECK(a && b);
    if (!a || !b) return;
    const std::uint16_t v100 = pm::value16FromMidi7(100);
    auto pushTo = [](pm_host* h, pm_event e) { CHECK(pm_host_push_event(h, &e) == 1); };
    // C4 at frame 100 of buffer 1 (stamp half a sample later: floor).
    pushTo(b, sampleEvent(PM_EVENT_NOTE_ON, 256 + 100, 60, v100));
    // E4 stamped 14 samples before buffer 3 started: late, frame 0 of buffer 3.
    pushTo(b, sampleEvent(PM_EVENT_NOTE_ON, 3 * 256, 64, v100));
    // G4 sent during buffer 5 for frame 300 after its start: frame 44 of buffer 6.
    pushTo(b, sampleEvent(PM_EVENT_NOTE_ON, 5 * 256 + 300, 67, v100));
    // Sustain with timestamp 0 ("now") before buffer 8.
    pushTo(b, sampleEvent(PM_EVENT_SUSTAIN, 8 * 256, 0, 65535));
    // C4 off, release velocity 64, frame 10 of buffer 9.
    pushTo(b, sampleEvent(PM_EVENT_NOTE_OFF, 9 * 256 + 10, 60,
                                                         pm::value16FromMidi7(64)));

    std::vector<float> la(kFrames), ra(kFrames), lb(kFrames), rb(kFrames), outA, outB;
    for (int buf = 0; buf < 24; ++buf) {
        // MIDI arriving on the MIDI thread before this buffer is rendered.
        if (buf == 1) {
            const std::uint8_t on[] = {0x90, 60, 100};
            pm_host_midi_bytes(a, on, 3, bufferStart(1) + 100 * kTicks + kTicks / 2, 1);
        } else if (buf == 3) {
            const std::uint32_t w = 0x20904064u;  // UMP note on E4 100
            pm_host_midi_ump(a, &w, 1, bufferStart(3) - 14 * kTicks, 2);
        } else if (buf == 5) {
            const auto list = eventList(bufferStart(5) + 300 * kTicks, {0x20904364u});  // G4 100
            CHECK(pm_host_midi_event_list(a, list.data(), 3) == 1);
        } else if (buf == 8) {
            const std::uint8_t cc[] = {0xB0, 64, 127};
            pm_host_midi_bytes(a, cc, 3, 0, 1);
        } else if (buf == 9) {
            const std::uint8_t off[] = {0x80, 60, 64};
            pm_host_midi_bytes(a, off, 3, bufferStart(9) + 10 * kTicks, 1);
        }
        pm_host_render(a, la.data(), ra.data(), kFrames, bufferStart(buf));
        pm_host_render(b, lb.data(), rb.data(), kFrames, bufferStart(buf));
        outA.insert(outA.end(), la.begin(), la.end());
        outA.insert(outA.end(), ra.begin(), ra.end());
        outB.insert(outB.end(), lb.begin(), lb.end());
        outB.insert(outB.end(), rb.begin(), rb.end());
    }
    double peak = 0.0, diff = 0.0;
    for (std::size_t i = 0; i < outA.size(); ++i) {
        peak = std::max(peak, static_cast<double>(std::fabs(outB[i])));
        diff = std::max(diff, static_cast<double>(std::fabs(outA[i] - outB[i])));
    }
    std::printf("  peak %.3g, max |A - B| %.3g\n", peak, diff);
    CHECK(peak > 1e-4);
    CHECK_LE(diff, 1e-7 * peak);
    // Nothing before the first note's sample (buffer 0 and buffer 1 up to frame 100).
    double early = 0.0;
    for (int i = 0; i < kFrames; ++i) early = std::max(early, static_cast<double>(std::fabs(outA[i])));
    for (int i = 0; i < 100; ++i) early = std::max(early, static_cast<double>(std::fabs(outA[2 * kFrames + i])));
    CHECK(early == 0.0);
    pm_stats st;
    pm_engine_get_stats(pm_host_engine(a), &st);
    CHECK(st.dropped_events == 0);
    pm_host_counters c;
    pm_host_get_counters(a, &c);
    CHECK(c.midi_messages == 5 && c.midi_events == 5 && c.dropped_events == 0 && c.midi_malformed == 0);
    pm_host_load load;
    pm_host_get_load(a, &load);
    CHECK(load.buffers == 24 && load.max_load > 0.0 && load.mean_load > 0.0 && load.last_frames == kFrames);
    pm_host_reset_load(a);
    pm_host_render(a, la.data(), ra.data(), kFrames, 0);
    pm_host_get_load(a, &load);
    CHECK(load.buffers == 1);
    pm_host_destroy(a);
    pm_host_destroy(b);
}

TEST(monitor_ring) {
    pm_host* h = makeHost(false);
    CHECK(h != nullptr);
    if (!h) return;
    pm_midi_monitor_entry e;
    CHECK(pm_host_monitor_pop(h, &e) == 0);
    const std::uint8_t knob[] = {0xB0, 14, 64, 0xF8};
    pm_host_midi_bytes(h, knob, 4, 555, 7);
    CHECK(pm_host_monitor_pop(h, &e) == 1);
    CHECK(e.kind == PM_MIDI_CONTROL_CHANGE && e.data1 == 14 && e.data2 == 64 && e.source_id == 7 &&
          e.host_time == 555 && e.channel == 0 && e.status == 0xB0 && e.protocol == 1 && e.word_count == 0);
    CHECK(pm_host_monitor_pop(h, &e) == 1 && e.kind == PM_MIDI_REALTIME && e.status == 0xF8);
    const std::uint32_t pb[] = {0x20E37F7Fu};  // pitch bend ch 4, maximum
    pm_host_midi_ump(h, pb, 1, 9, 2);
    CHECK(pm_host_monitor_pop(h, &e) == 1 && e.kind == PM_MIDI_PITCH_BEND && e.value == 16383 &&
          e.channel == 3 && e.word_count == 1 && e.words[0] == pb[0]);
    pm_host_counters c;
    pm_host_get_counters(h, &c);
    CHECK(c.midi_messages == 3 && c.midi_events == 0);

    pm_host_set_monitor_enabled(h, 0);
    pm_host_midi_bytes(h, knob, 3, 1, 1);
    CHECK(pm_host_monitor_pop(h, &e) == 0);
    pm_host_set_monitor_enabled(h, 1);
    // Overflow: counted, never blocking. The event queue (not drained on an
    // unprepared host) also overflows and counts.
    for (int i = 0; i < 1100; ++i) {
        const std::uint8_t on[] = {0x90, 60, 100};
        pm_host_midi_bytes(h, on, 3, 1, 1);
    }
    int popped = 0;
    while (pm_host_monitor_pop(h, &e)) ++popped;
    pm_host_get_counters(h, &c);
    CHECK(popped == 1024 && c.monitor_dropped == 76);
    CHECK(c.midi_events == 1024 && c.dropped_events == 76);
    pm_host_destroy(h);
}

TEST(prepare_gate_with_running_audio_thread) {
    pm_host* h = makeHost(false);
    CHECK(h != nullptr);
    if (!h) return;
    std::vector<float> l(64), r(64);
    // Unprepared: silence, nothing recorded.
    l[0] = 1.0f;
    pm_host_render(h, l.data(), r.data(), 64, 0);
    CHECK(l[0] == 0.0f && pm_host_is_prepared(h) == 0);
    pm_host_load load;
    pm_host_get_load(h, &load);
    CHECK(load.buffers == 0);

    std::atomic<bool> stop{false};
    std::atomic<long> renders{0};
    std::thread audio([&] {
        std::vector<float> al(64), ar(64);
        while (!stop.load()) {
            pm_host_render(h, al.data(), ar.data(), 64, 0);
            renders.fetch_add(1);
        }
    });
    CHECK(pm_host_prepare(h, kFs, 64) == 1);
    CHECK(pm_host_is_prepared(h) == 1 && pm_host_sample_rate(h) == kFs);
    // Play while the audio thread runs, then re-prepare at another rate.
    pm_event on = sampleEvent(PM_EVENT_NOTE_ON, 0, 60, 40000);
    CHECK(pm_host_push_event(h, &on) == 1);
    const long before = renders.load();
    while (renders.load() < before + 200) std::this_thread::yield();
    CHECK(pm_host_prepare(h, 44100.0, 64) == 1);
    CHECK(pm_host_sample_rate(h) == 44100.0);
    CHECK(pm_host_prepare(h, -1.0, 64) == 0);
    const long after = renders.load();
    while (renders.load() < after + 200) std::this_thread::yield();
    stop.store(true);
    audio.join();
    pm_host_get_load(h, &load);
    CHECK(load.buffers > 0);

    // Events queued before a prepare are discarded.
    CHECK(pm_host_push_event(h, &on) == 1);
    CHECK(pm_host_prepare(h, kFs, 64) == 1);
    double peak = 0.0;
    for (int i = 0; i < 50; ++i) {
        pm_host_render(h, l.data(), r.data(), 64, 0);
        for (int j = 0; j < 64; ++j) peak = std::max(peak, static_cast<double>(std::fabs(l[j])));
    }
    CHECK(peak == 0.0);
    pm_host_destroy(h);
}

TEST(no_allocation_on_midi_and_audio_paths) {
    pm_host* h = makeHost(true);
    CHECK(h != nullptr);
    if (!h) return;
    std::vector<float> l(kFrames), r(kFrames);
    const auto list = eventList(bufferStart(0) + 10 * kTicks, {0x20903C64u, 0x20B04040u, 0x40904000u, 0xFFFF0000u});
    pm_midi_monitor_entry e;
    pm_host_load load;
    pm_host_counters c;
    gAllocs.store(0);
    gArmed.store(true);
    for (int buf = 0; buf < 40; ++buf) {
        const std::uint8_t bytes[] = {0x90, static_cast<std::uint8_t>(40 + buf), 90, 0xF8, 0xB0, 67, 30};
        pm_host_midi_bytes(h, bytes, 7, bufferStart(buf) + 5 * kTicks, 1);
        const std::uint32_t w[] = {0x20804000u, 0x20B04000u};
        pm_host_midi_ump(h, w, 2, bufferStart(buf), 2);
        pm_host_midi_event_list(h, list.data(), 3);
        pm_event ui = sampleEvent(PM_EVENT_NOTE_ON, 0, 80, 30000);
        pm_host_push_event(h, &ui);
        pm_host_render(h, l.data(), r.data(), kFrames, bufferStart(buf));
        while (pm_host_monitor_pop(h, &e)) {}
        pm_host_get_load(h, &load);
        pm_host_get_counters(h, &c);
    }
    gArmed.store(false);
    std::printf("  allocations on the MIDI/audio/monitor paths: %ld\n", gAllocs.load());
    CHECK(gAllocs.load() == 0);
    pm_host_destroy(h);
}

// Phase timing through the C API (0.8): off by default, then the split of
// real render calls adds up to their render time, and the worst buffer
// records the events it applied.
TEST(phase_timing) {
    pm_host* h = makeHost(true);
    CHECK(h != nullptr);
    if (!h) return;
    std::vector<float> l(kFrames), r(kFrames);
    pm_host_phase_diagnostics pd;
    pm_host_get_phase_diagnostics(h, &pd);
    CHECK(pd.phase_timing == 0 && pd.timed_buffers == 0);
    for (int buf = 0; buf < 20; ++buf) pm_host_render(h, l.data(), r.data(), kFrames, bufferStart(buf));
    pm_host_get_phase_diagnostics(h, &pd);
    CHECK(pd.timed_buffers == 0);
    pm_host_set_phase_timing(h, 1);
    pm_host_reset_load(h);
    gAllocs.store(0);
    gArmed.store(true);
    for (int buf = 20; buf < 200; ++buf) {
        if (buf % 40 == 25) {
            const std::uint8_t on[] = {0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100};
            pm_host_midi_bytes(h, on, 9, 0, 1);
        }
        if (buf % 40 == 35) {
            const std::uint8_t off[] = {0x80, 60, 64, 0x90, 64, 0, 0x80, 67, 64, 0xB0, 64, 127};
            pm_host_midi_bytes(h, off, 12, 0, 1);
        }
        pm_host_render(h, l.data(), r.data(), kFrames, bufferStart(buf));
        pm_host_get_phase_diagnostics(h, &pd);
    }
    gArmed.store(false);
    pm_host_diagnostics d;
    pm_host_get_diagnostics(h, &d);
    pm_host_get_phase_diagnostics(h, &pd);
    double sumMean = 0.0, sumWorst = 0.0;
    for (int p = 0; p < PM_PHASE_COUNT; ++p) {
        sumMean += pd.mean_phase_us[p];
        sumWorst += pd.worst_phase_us[p];
    }
    std::printf("  timed %llu buffers; mean split %.1f us (render mean %.1f us); worst %.1f us of %.1f us, "
                "%d on / %d off / %d pedal, %d damper updates, %d responses, %d kernels; allocations %ld\n",
                static_cast<unsigned long long>(pd.timed_buffers), sumMean, d.mean_us, sumWorst, d.worst_render_us,
                pd.worst_note_ons, pd.worst_note_offs, pd.worst_pedal_events, pd.worst_damper_updates,
                pd.worst_response_groups, pd.worst_kernel_rebuilds, gAllocs.load());
    CHECK(pd.phase_timing == 1 && pd.timed_buffers == 180 && pd.worst_timed == 1);
    CHECK(pd.top_count >= 2);
    // The engine's phases cover its render; pm_host_render adds the event
    // forwarding and the meter around it.
    CHECK(sumMean > 0.0 && sumMean <= d.mean_us * 1.001);
    CHECK(sumWorst > 0.0 && sumWorst <= d.worst_render_us * 1.001);
    CHECK(gAllocs.load() == 0);
    pm_host_destroy(h);
}

// Speed probe (0.9): off by default; on, it runs on the render thread every
// Nth call without allocating, its statistics are ordered and plausible
// against the same workload on this (the same) thread, and it is not part
// of the render time.
TEST(speed_probe) {
    pm_host* h = makeHost(true);
    CHECK(h != nullptr);
    if (!h) return;
    std::vector<float> l(kFrames), r(kFrames);
    pm_speed_probe_stats ps;
    for (int buf = 0; buf < 8; ++buf) pm_host_render(h, l.data(), r.data(), kFrames, bufferStart(buf));
    pm_host_get_speed_probe(h, &ps);
    CHECK(ps.interval == 0 && ps.runs == 0);
    pm_host_set_speed_probe(h, 4);
    pm_host_reset_load(h);
    gAllocs.store(0);
    gArmed.store(true);
    for (int buf = 8; buf < 8 + 400; ++buf) pm_host_render(h, l.data(), r.data(), kFrames, bufferStart(buf));
    gArmed.store(false);
    pm_host_get_speed_probe(h, &ps);
    pm_speed_probe_stats ref;
    const double refUs = pm_speed_probe_run_us(101, &ref);
    pm_host_diagnostics d;
    pm_host_get_diagnostics(h, &d);
    std::printf("  audio-thread probe: %llu runs, min %.2f / median %.2f / mean %.2f / max %.2f us; "
                "this thread %.2f us; render mean %.1f us; allocations %ld\n",
                static_cast<unsigned long long>(ps.runs), ps.min_us, ps.median_us, ps.mean_us, ps.max_us, refUs,
                d.mean_us, gAllocs.load());
    CHECK(ps.interval == 4 && ps.runs == 100);
    CHECK(ps.min_us > 0.0 && ps.min_us <= ps.median_us && ps.median_us <= ps.max_us);
    CHECK(ps.min_us <= ps.mean_us && ps.mean_us <= ps.max_us);
    // Same thread, same work: the medians agree to well within a factor 3
    // (scheduling noise only).
    CHECK(ps.median_us < 3.0 * refUs && refUs < 3.0 * ps.median_us);
    CHECK(gAllocs.load() == 0);
    pm_host_set_speed_probe(h, 0);
    pm_host_reset_load(h);
    for (int buf = 0; buf < 8; ++buf) pm_host_render(h, l.data(), r.data(), kFrames, bufferStart(buf));
    pm_host_get_speed_probe(h, &ps);
    CHECK(ps.interval == 0 && ps.runs == 0);
    pm_host_destroy(h);
}

// Worst-buffer hammer detail (0.9): ten hard notes arriving in one 512-frame
// buffer (board on). Most hammers have left their strings again before the
// buffer ends, which the end-of-buffer count used to miss.
TEST(worst_buffer_hammer_detail) {
    pm_host* h = pm_host_create();
    CHECK(h != nullptr);
    if (!h) return;
    constexpr int kBig = 512;
    CHECK(pm_host_prepare(h, 44100.0, kBig) == 1);
    std::vector<float> l(kBig), r(kBig);
    for (int buf = 0; buf < 4; ++buf) pm_host_render(h, l.data(), r.data(), kBig, 0);
    pm_host_reset_load(h);
    const std::uint8_t notes[10] = {24, 29, 33, 36, 40, 43, 48, 52, 55, 60};
    for (std::uint8_t n : notes) {
        const std::uint8_t on[] = {0x90, n, 127};
        pm_host_midi_bytes(h, on, 3, 0, 1);
    }
    // Only the burst buffer is rendered after the reset, so it is the worst
    // buffer by construction: the meter ranks buffers by measured wall time,
    // and on a loaded machine a preempted later buffer could outrank it.
    pm_host_render(h, l.data(), r.data(), kBig, 0);
    pm_host_diagnostics d;
    pm_host_worst_detail w;
    pm_host_phase_diagnostics pd;
    pm_host_get_diagnostics(h, &d);
    pm_host_get_worst_detail(h, &w);
    pm_host_get_phase_diagnostics(h, &pd);
    std::printf("  worst buffer #%llu: %d note-ons; %d hammers touched strings, at most %d at once, %d contact "
                "samples, %d active at the end; %d Newton evaluations, %d fallbacks; kernel %d rebuilds + %d "
                "key updates\n",
                static_cast<unsigned long long>(d.worst_index), pd.worst_note_ons, w.hammers_in_contact,
                w.max_hammers_in_contact, w.hammer_contact_samples, d.worst_hammers_active, w.newton_evals,
                w.fallback_solves, w.kernel_rebuilds, w.kernel_key_updates);
    CHECK(d.buffers == 1 && d.worst_index == 1);
    CHECK(pd.worst_note_ons == 10);
    CHECK(w.hammers_in_contact == 10 && d.worst_hammers_in_contact == 10);
    CHECK(w.max_hammers_in_contact >= 5 && w.max_hammers_in_contact <= 10);
    CHECK(d.worst_hammers_active < w.hammers_in_contact);  // released within the buffer
    // Contact lasts 1-5 ms (44-220 samples) per hammer.
    CHECK(w.hammer_contact_samples >= 10 * 40 && w.hammer_contact_samples <= 10 * kBig);
    CHECK(w.newton_evals >= w.hammer_contact_samples && w.fallback_solves == 0);
    pm_host_destroy(h);
}

TEST(null_tolerance) {
    pm_event e = sampleEvent(PM_EVENT_NOTE_ON, 0, 60, 1);
    pm_midi_monitor_entry m;
    pm_host_load load;
    pm_host_counters c;
    float buf[4];
    pm_host_destroy(nullptr);
    CHECK(pm_host_engine(nullptr) == nullptr);
    pm_host_set_phase_timing(nullptr, 1);
    pm_host_get_phase_diagnostics(nullptr, nullptr);
    pm_host_set_timebase(nullptr, 1, 1);
    CHECK(pm_host_prepare(nullptr, kFs, 64) == 0);
    CHECK(pm_host_is_prepared(nullptr) == 0 && pm_host_sample_rate(nullptr) == 0.0);
    pm_host_render(nullptr, buf, buf, 4, 0);
    CHECK(pm_host_midi_event_list(nullptr, nullptr, 0) == 0);
    pm_host_midi_ump(nullptr, nullptr, 1, 0, 0);
    pm_host_midi_bytes(nullptr, nullptr, 1, 0, 0);
    CHECK(pm_host_push_event(nullptr, &e) == 0);
    CHECK(pm_host_monitor_pop(nullptr, &m) == 0);
    pm_host_set_monitor_enabled(nullptr, 1);
    pm_host_get_load(nullptr, &load);
    pm_host_reset_load(nullptr);
    pm_host_set_load_budget(nullptr, 0.5);
    pm_host_get_counters(nullptr, &c);
    pm_host* h = makeHost(false);
    CHECK(h != nullptr);
    if (!h) return;
    CHECK(pm_host_midi_event_list(h, nullptr, 0) == 0);
    pm_host_midi_bytes(h, nullptr, 3, 0, 0);
    pm_host_midi_ump(h, nullptr, 3, 0, 0);
    CHECK(pm_host_push_event(h, nullptr) == 0);
    pm_host_render(h, nullptr, buf, 4, 0);
    pm_host_destroy(h);
}
