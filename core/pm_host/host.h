// App-side glue between the platform threads and the engine (plan §3.1,
// §5.2), portable so that it is tested on every platform.
//
// Threads and queues (each queue has exactly one producer and one consumer):
//   MIDI thread  --receive*()-->  midi queue    --forwardEvents()--> engine (audio thread)
//                             \-> monitor ring  --popMonitor()-----> UI thread
//   UI thread    --pushUi()---->  ui queue      --forwardEvents()--> engine
// The audio thread is then the engine queue's only producer and consumer.
// receive*() must be called from one thread at a time (Core MIDI calls an
// input port's receive block on one thread); pushUi() and popMonitor() from
// one other thread (the main thread).
//
// Prepare gate: runExclusive() (non-real-time) closes the gate, waits until
// the audio thread is outside its render section, runs the given function
// (engine prepare) and reopens the gate. The audio thread never waits:
// enterRender() returns false while the gate is closed and the caller then
// outputs silence. Events queued while the gate is closed are discarded.
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

#include "pm_base/spsc_queue.h"
#include "pm_engine/events.h"
#include "pm_host/load_meter.h"
#include "pm_host/midi.h"

namespace pm::host {

struct MonitorEntry {
    std::uint64_t hostTime = 0;  // Core MIDI timestamp (0 = "now")
    std::uint32_t sourceId = 0;
    MidiMessage message;
};

struct HostCounters {
    std::uint64_t midiMessages = 0;    // decoded messages, all kinds
    std::uint64_t midiMalformed = 0;
    std::uint64_t midiEvents = 0;      // engine events produced from MIDI
    std::uint64_t droppedEvents = 0;   // midi or ui queue full
    std::uint64_t monitorDropped = 0;  // monitor ring full (UI not draining)
};

class Host {
public:
    static constexpr std::size_t kEventQueue = 1024;
    static constexpr std::size_t kMonitorQueue = 1024;
    static constexpr int kByteParsers = 16;  // byte-stream state per (source id % 16)

    // ---- MIDI thread
    // A Core MIDI MIDIEventList (layout in midi.h). Returns the packet count.
    std::uint32_t receiveEventList(const void* eventList, std::uint32_t sourceId) noexcept;
    // UMP words with one timestamp (host ticks; 0 = now).
    void receiveUmp(const std::uint32_t* words, std::size_t n, std::uint64_t hostTime,
                    std::uint32_t sourceId) noexcept;
    // MIDI 1.0 bytes (running status kept per source id % kByteParsers).
    void receiveBytes(const std::uint8_t* bytes, std::size_t n, std::uint64_t hostTime,
                      std::uint32_t sourceId) noexcept;

    // ---- UI thread
    bool pushUi(const Event& e) noexcept;
    bool popMonitor(MonitorEntry& out) noexcept { return monitor_.pop(out); }
    void setMonitorEnabled(bool on) noexcept { monitorOn_.store(on, std::memory_order_relaxed); }
    bool monitorEnabled() const noexcept { return monitorOn_.load(std::memory_order_relaxed); }

    // ---- audio thread
    bool enterRender() noexcept {
        inRender_.store(true, std::memory_order_seq_cst);
        if (ready_.load(std::memory_order_seq_cst)) return true;
        inRender_.store(false, std::memory_order_release);
        return false;
    }
    void exitRender() noexcept { inRender_.store(false, std::memory_order_release); }
    // Pops every queued event (MIDI first, then UI) into push(const Event&).
    // Returns the number forwarded.
    template <typename Push>
    int forwardEvents(Push&& push) noexcept {
        int n = 0;
        Event e;
        while (midi_.pop(e)) {
            push(static_cast<const Event&>(e));
            ++n;
        }
        while (ui_.pop(e)) {
            push(static_cast<const Event&>(e));
            ++n;
        }
        return n;
    }
    LoadMeter& meter() noexcept { return meter_; }
    const LoadMeter& meter() const noexcept { return meter_; }

    // ---- non-real-time
    template <typename Fn>
    bool runExclusive(Fn&& fn) {
        std::lock_guard<std::mutex> lock(prepareMutex_);
        ready_.store(false, std::memory_order_seq_cst);
        while (inRender_.load(std::memory_order_seq_cst)) std::this_thread::yield();
        // The audio thread is out and stays out: this thread may consume the
        // event queues (ownership returns through ready_).
        Event e;
        while (midi_.pop(e)) {}
        while (ui_.pop(e)) {}
        const bool ok = fn();
        ready_.store(ok, std::memory_order_seq_cst);
        return ok;
    }
    bool ready() const noexcept { return ready_.load(std::memory_order_acquire); }

    HostCounters counters() const noexcept;

private:
    void handleMessage(const MidiMessage& m, std::uint64_t hostTime, std::uint32_t sourceId) noexcept;

    SpscQueue<Event, kEventQueue> midi_;
    SpscQueue<Event, kEventQueue> ui_;
    SpscQueue<MonitorEntry, kMonitorQueue> monitor_;
    MidiByteParser byteParsers_[kByteParsers];
    LoadMeter meter_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> inRender_{false};
    std::atomic<bool> monitorOn_{true};
    std::atomic<std::uint64_t> messages_{0}, malformed_{0}, events_{0}, dropped_{0}, monitorDropped_{0};
    std::mutex prepareMutex_;
};

}  // namespace pm::host
