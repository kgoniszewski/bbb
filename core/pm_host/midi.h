// MIDI input for the app host (plan §3.1): MIDI 1.0 byte streams, Universal
// MIDI Packets (MIDI 1.0 and 2.0 protocol) and Core MIDI's MIDIEventList
// layout, decoded into MidiMessage and mapped to engine events.
//
// Real-time safe: no allocation, locks, exceptions or I/O. Malformed input
// is counted and skipped; nothing is ever read past the given length.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "pm_engine/events.h"

namespace pm::host {

enum class MidiKind : std::uint8_t {
    NoteOff = 1,
    NoteOn = 2,
    PolyPressure = 3,
    ControlChange = 4,
    ProgramChange = 5,
    ChannelPressure = 6,
    PitchBend = 7,
    SysEx = 8,          // byte stream: one message per complete (or cut) F0..F7; UMP: one per packet
    SystemCommon = 9,   // F1 F2 F3 F6
    Realtime = 10,      // F8..FF
    Utility = 11,       // UMP type 0 (NOOP, jitter-reduction timestamps)
    Other = 12,         // decoded but not one of the above (e.g. MIDI 2.0 per-note controllers, UMP stream)
};

// One decoded message. For MIDI 1.0 (bytes or UMP type 2), status/data1/data2
// are the original bytes and value is data2 (pitch bend: the 14-bit value).
// For MIDI 2.0 channel voice (UMP type 4) status/data1/data2 hold the MIDI
// 1.0 equivalent (values reduced to 7 bits) and value the full-resolution
// one (velocity 16 bits, controllers and pitch bend 32 bits).
struct MidiMessage {
    MidiKind kind = MidiKind::Other;
    std::uint8_t channel = 0;    // 0..15 for channel messages
    std::uint8_t status = 0;     // full status byte, F0 for SysEx
    std::uint8_t data1 = 0;
    std::uint8_t data2 = 0;
    std::uint8_t group = 0;      // UMP group (0 for byte streams)
    std::uint8_t protocol = 1;   // 1 = MIDI 1.0 values, 2 = MIDI 2.0 values
    std::uint8_t wordCount = 0;  // UMP words in words[] (0 for byte streams)
    std::uint32_t value = 0;
    std::uint32_t length = 0;    // SysEx: payload bytes (without F0/F7)
    std::uint32_t words[4] = {0, 0, 0, 0};
};

// UMP message length in 32-bit words for a message type (the top nibble of
// its first word).
inline int umpWordCount(std::uint32_t firstWord) noexcept {
    static constexpr std::uint8_t kWords[16] = {1, 1, 1, 2, 2, 4, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4};
    return kWords[firstWord >> 28];
}

struct ParseCounters {
    std::uint64_t messages = 0;
    std::uint64_t malformed = 0;  // stray data bytes, bad UMP status, truncated packets
};

// MIDI 1.0 byte-stream parser with running status. Real-time bytes (F8..FF)
// may appear anywhere, even inside another message, and leave the parser's
// state untouched. A status byte inside a SysEx ends it (reported as SysEx
// with the bytes so far). System common messages cancel running status.
class MidiByteParser {
public:
    // Calls sink(const MidiMessage&) for every complete message.
    template <typename Sink>
    void feed(const std::uint8_t* bytes, std::size_t n, Sink&& sink) noexcept {
        MidiMessage m[2];
        for (std::size_t i = 0; i < n; ++i) {
            const int k = feedByte(bytes[i], m);
            for (int j = 0; j < k; ++j) sink(static_cast<const MidiMessage&>(m[j]));
        }
    }
    // Writes the messages completed by b to out[0..1] and returns how many
    // (two when a status byte cuts a SysEx and completes a message itself).
    int feedByte(std::uint8_t b, MidiMessage* out) noexcept;
    void reset() noexcept;
    const ParseCounters& counters() const noexcept { return counters_; }

private:
    void emitSysEx(MidiMessage& out) noexcept;
    std::uint8_t running_ = 0;   // channel status for running status (0 = none)
    std::uint8_t pending_ = 0;   // status of the message being assembled (0 = none)
    std::uint8_t data_[2] = {0, 0};
    std::uint8_t have_ = 0, need_ = 0;
    bool inSysEx_ = false;
    std::uint32_t sysExLength_ = 0;
    ParseCounters counters_;
};

// Decodes the message at words[0..available). Returns the number of words
// consumed (>= 1 when available >= 1: a packet longer than 'available' is
// counted malformed and the rest is consumed), and sets ok = false for a
// message that cannot be decoded.
int decodeUmp(const std::uint32_t* words, std::size_t available, MidiMessage& out, bool& ok) noexcept;

// Decodes every message in words[0..n).
template <typename Sink>
void parseUmp(const std::uint32_t* words, std::size_t n, ParseCounters& c, Sink&& sink) noexcept {
    std::size_t i = 0;
    while (i < n) {
        MidiMessage m;
        bool ok = true;
        i += static_cast<std::size_t>(decodeUmp(words + i, n - i, m, ok));
        if (ok) {
            ++c.messages;
            sink(m);
        } else {
            ++c.malformed;
        }
    }
}

// Core MIDI's MIDIEventList, as laid out by <CoreMIDI/MIDIServices.h>
// (#pragma pack(4)): { int32 protocol; uint32 numPackets; packets... },
// each packet { uint64 timeStamp; uint32 wordCount; uint32 words[wordCount] },
// the next packet following the last word. Packets are read with memcpy
// (8-byte stamps sit on 4-byte boundaries). wordCount is clamped to 64, the
// size of MIDIEventPacket::words.
constexpr std::size_t kEventListHeaderBytes = 8;
constexpr std::size_t kEventPacketHeaderBytes = 12;
constexpr std::uint32_t kEventPacketMaxWords = 64;

// Calls packet(timeStamp, const uint32_t* words, wordCount) for each packet.
// Returns the number of packets visited.
template <typename PacketFn>
std::uint32_t forEachEventPacket(const void* eventList, PacketFn&& packet) noexcept {
    if (!eventList) return 0;
    const auto* p = static_cast<const unsigned char*>(eventList);
    std::uint32_t numPackets = 0;
    std::memcpy(&numPackets, p + 4, 4);
    p += kEventListHeaderBytes;
    alignas(4) std::uint32_t words[kEventPacketMaxWords];
    for (std::uint32_t k = 0; k < numPackets; ++k) {
        std::uint64_t stamp = 0;
        std::uint32_t count = 0;
        std::memcpy(&stamp, p, 8);
        std::memcpy(&count, p + 8, 4);
        if (count > kEventPacketMaxWords) count = kEventPacketMaxWords;
        std::memcpy(words, p + kEventPacketHeaderBytes, count * 4u);
        packet(stamp, static_cast<const std::uint32_t*>(words), count);
        p += kEventPacketHeaderBytes + count * 4u;
    }
    return numPackets;
}

// Engine events for one message, stamped with (time, base). Returns how
// many were written to out (at most kMaxEventsPerMessage). Mapping (omni,
// every channel): note-on (velocity 0 = note-off in MIDI 1.0), note-off with
// release velocity, CC64 sustain / CC66 sostenuto / CC67 una corda
// (continuous, 7-bit values scaled to 16 bits; MIDI 2.0 32-bit values
// reduced to 16), CC120 all sound off -> panic, CC123 all notes off,
// CC121 reset all controllers -> the three pedals up. Notes outside the
// piano's range (21..108) and everything else give no event.
constexpr int kMaxEventsPerMessage = 3;
int toEngineEvents(const MidiMessage& m, std::uint64_t time, TimeBase base, Event* out) noexcept;

}  // namespace pm::host
