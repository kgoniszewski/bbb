#include "pm_host/host.h"

namespace pm::host {

void Host::handleMessage(const MidiMessage& m, std::uint64_t hostTime, std::uint32_t sourceId) noexcept {
    messages_.fetch_add(1, std::memory_order_relaxed);
    if (monitorOn_.load(std::memory_order_relaxed)) {
        MonitorEntry entry;
        entry.hostTime = hostTime;
        entry.sourceId = sourceId;
        entry.message = m;
        if (!monitor_.push(entry)) monitorDropped_.fetch_add(1, std::memory_order_relaxed);
    }
    Event ev[kMaxEventsPerMessage];
    // Core MIDI's timestamp 0 means "now": an absolute sample time of 0 is
    // always in the past, so the engine applies it at the start of the next
    // buffer (and does not count it late in constant-latency mode).
    const int n = hostTime == 0 ? toEngineEvents(m, 0, TimeBase::SampleTime, ev)
                                : toEngineEvents(m, hostTime, TimeBase::HostTime, ev);
    for (int i = 0; i < n; ++i) {
        if (midi_.push(ev[i])) events_.fetch_add(1, std::memory_order_relaxed);
        else dropped_.fetch_add(1, std::memory_order_relaxed);
    }
}

std::uint32_t Host::receiveEventList(const void* eventList, std::uint32_t sourceId) noexcept {
    return forEachEventPacket(eventList, [&](std::uint64_t stamp, const std::uint32_t* words, std::uint32_t count) {
        receiveUmp(words, count, stamp, sourceId);
    });
}

void Host::receiveUmp(const std::uint32_t* words, std::size_t n, std::uint64_t hostTime,
                      std::uint32_t sourceId) noexcept {
    if (!words) return;
    ParseCounters c;
    parseUmp(words, n, c, [&](const MidiMessage& m) { handleMessage(m, hostTime, sourceId); });
    if (c.malformed) malformed_.fetch_add(c.malformed, std::memory_order_relaxed);
}

void Host::receiveBytes(const std::uint8_t* bytes, std::size_t n, std::uint64_t hostTime,
                        std::uint32_t sourceId) noexcept {
    if (!bytes) return;
    MidiByteParser& p = byteParsers_[sourceId % kByteParsers];
    const std::uint64_t before = p.counters().malformed;
    p.feed(bytes, n, [&](const MidiMessage& m) { handleMessage(m, hostTime, sourceId); });
    const std::uint64_t bad = p.counters().malformed - before;
    if (bad) malformed_.fetch_add(bad, std::memory_order_relaxed);
}

bool Host::pushUi(const Event& e) noexcept {
    if (ui_.push(e)) return true;
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
}

HostCounters Host::counters() const noexcept {
    HostCounters c;
    c.midiMessages = messages_.load(std::memory_order_relaxed);
    c.midiMalformed = malformed_.load(std::memory_order_relaxed);
    c.midiEvents = events_.load(std::memory_order_relaxed);
    c.droppedEvents = dropped_.load(std::memory_order_relaxed);
    c.monitorDropped = monitorDropped_.load(std::memory_order_relaxed);
    return c;
}

}  // namespace pm::host
