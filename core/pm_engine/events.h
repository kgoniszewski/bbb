// Engine input events (plan §3.1). Plain data, copied through the SPSC queue.
#pragma once

#include <cmath>
#include <cstdint>

namespace pm {

enum class EventType : std::uint8_t {
    None = 0,
    NoteOn = 1,       // note, value = velocity (0 = note-off)
    NoteOff = 2,      // note, value = release velocity (0 = default)
    Sustain = 3,      // value = pedal position 0..65535 (CC64)
    AllNotesOff = 4,  // every key released; dampers fall normally
    Panic = 5,        // keys and pedals up, hammers stopped, all strings damped hard for 300 ms
    Sostenuto = 6,    // value = pedal position 0..65535 (CC66), on/off at half travel
    UnaCorda = 7,     // value = pedal position 0..65535 (CC67), continuous
    KeyDownSilent = 8, // note: key pressed down without sounding (damper lifted, no hammer); NoteOff releases it
};

enum class TimeBase : std::uint8_t {
    HostTime = 0,    // caller's clock, mapped per render call by ClockMapping
    SampleTime = 1,  // absolute engine sample index (frames rendered since prepare)
};

struct Event {
    std::uint64_t time = 0;
    EventType type = EventType::None;
    TimeBase timeBase = TimeBase::SampleTime;
    std::uint8_t note = 0;      // MIDI note number, 21 (A0) .. 108 (C8)
    std::uint8_t reserved = 0;
    std::uint16_t value = 0;    // 16-bit velocity or controller value
    std::uint16_t reserved2 = 0;
};

// Filled by the caller for every render call: host time of the first frame
// of the buffer, and host ticks per sample (on Apple: mach_absolute_time
// ticks, i.e. 1e9 / fs * timebase.denom / timebase.numer).
struct ClockMapping {
    std::uint64_t hostTimeAtBufferStart = 0;
    double hostTicksPerSample = 0.0;
};

// Frames from the start of the buffer to host time t: floor((t - start) /
// ticksPerSample), signed (negative = already past; correct across counter
// wrap). The engine clamps negative results to 0 (due now) and keeps
// results >= the buffer length pending for a later buffer.
inline std::int64_t hostTimeToFrameDelta(std::uint64_t t, const ClockMapping& c) noexcept {
    const auto dt = static_cast<std::int64_t>(t - c.hostTimeAtBufferStart);
    double f = std::floor(static_cast<double>(dt) / c.hostTicksPerSample);
    // Keep the conversion defined for absurd stamps (about 650 years at 48 kHz).
    if (f > 1e15) f = 1e15;
    if (f < -1e15) f = -1e15;
    return static_cast<std::int64_t>(f);
}

// Host ticks per sample from a tick-to-nanosecond ratio (Apple:
// mach_timebase_info, ns = ticks * numer / denom; numer = denom = 1 for a
// nanosecond clock): ticks/sample = 1e9 / fs * denom / numer.
inline double hostTicksPerSample(double sampleRate, std::uint32_t numer, std::uint32_t denom) noexcept {
    if (!(sampleRate > 0.0) || numer == 0 || denom == 0) return 0.0;
    return 1e9 / sampleRate * static_cast<double>(denom) / static_cast<double>(numer);
}

// MIDI 1.0 7-bit value -> 16-bit (0 stays 0, 127 becomes 65535).
inline std::uint16_t value16FromMidi7(int v7) noexcept {
    if (v7 <= 0) return 0;
    if (v7 >= 127) return 65535;
    return static_cast<std::uint16_t>((v7 * 65535 + 63) / 127);
}

}  // namespace pm
