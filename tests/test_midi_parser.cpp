// MIDI input parsing (plan §3.8 "simulated devices"): MIDI 1.0 byte streams
// with running status, UMP in MIDI 1.0 and 2.0 protocol, Core MIDI event
// lists, malformed input, mapping to engine events, and fuzzing.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "pm_host/midi.h"
#include "test_harness.h"

using pm::Event;
using pm::EventType;
using pm::host::MidiByteParser;
using pm::host::MidiKind;
using pm::host::MidiMessage;

namespace {

std::vector<MidiMessage> parseBytes(MidiByteParser& p, const std::vector<std::uint8_t>& b) {
    std::vector<MidiMessage> out;
    p.feed(b.data(), b.size(), [&](const MidiMessage& m) { out.push_back(m); });
    return out;
}

std::vector<MidiMessage> parseWords(const std::vector<std::uint32_t>& w, pm::host::ParseCounters& c) {
    std::vector<MidiMessage> out;
    pm::host::parseUmp(w.data(), w.size(), c, [&](const MidiMessage& m) { out.push_back(m); });
    return out;
}

int events(const MidiMessage& m, Event* ev) {
    return pm::host::toEngineEvents(m, 1234, pm::TimeBase::HostTime, ev);
}

// Appends a Core MIDI event packet (12-byte header, then the words).
void appendPacket(std::vector<unsigned char>& buf, std::uint64_t stamp, const std::vector<std::uint32_t>& words) {
    const std::size_t at = buf.size();
    buf.resize(at + 12 + 4 * words.size());
    const auto n = static_cast<std::uint32_t>(words.size());
    std::memcpy(&buf[at], &stamp, 8);
    std::memcpy(&buf[at + 8], &n, 4);
    if (n) std::memcpy(&buf[at + 12], words.data(), 4 * words.size());
}

std::vector<unsigned char> eventListHeader(std::int32_t protocol, std::uint32_t packets) {
    std::vector<unsigned char> buf(8);
    std::memcpy(&buf[0], &protocol, 4);
    std::memcpy(&buf[4], &packets, 4);
    return buf;
}

}  // namespace

TEST(note_on_off_and_running_status) {
    MidiByteParser p;
    // Note-on C4 100, then by running status D4 80 and C4 velocity 0 (= off).
    const auto m = parseBytes(p, {0x90, 0x3C, 0x64, 0x3E, 0x50, 0x3C, 0x00});
    CHECK(m.size() == 3);
    if (m.size() != 3) return;
    CHECK(m[0].kind == MidiKind::NoteOn && m[0].data1 == 60 && m[0].data2 == 100 && m[0].channel == 0);
    CHECK(m[1].kind == MidiKind::NoteOn && m[1].data1 == 62 && m[1].data2 == 80);
    CHECK(m[2].kind == MidiKind::NoteOn && m[2].data2 == 0);
    Event ev[3];
    CHECK(events(m[0], ev) == 1 && ev[0].type == EventType::NoteOn && ev[0].note == 60 &&
          ev[0].value == pm::value16FromMidi7(100) && ev[0].time == 1234 &&
          ev[0].timeBase == pm::TimeBase::HostTime);
    CHECK(events(m[2], ev) == 1 && ev[0].type == EventType::NoteOff && ev[0].note == 60 && ev[0].value == 0);
    CHECK(p.counters().messages == 3 && p.counters().malformed == 0);

    // Note-off with release velocity, channel 10.
    const auto off = parseBytes(p, {0x89, 0x30, 0x7F});
    CHECK(off.size() == 1 && off[0].kind == MidiKind::NoteOff && off[0].channel == 9);
    CHECK(events(off[0], ev) == 1 && ev[0].type == EventType::NoteOff && ev[0].value == 65535);

    // One-data-byte messages with running status.
    const auto pc = parseBytes(p, {0xC2, 0x05, 0x06, 0xD2, 0x40, 0x41});
    CHECK(pc.size() == 4);
    if (pc.size() == 4) {
        CHECK(pc[0].kind == MidiKind::ProgramChange && pc[1].data1 == 6 && pc[1].channel == 2);
        CHECK(pc[2].kind == MidiKind::ChannelPressure && pc[3].data1 == 0x41);
    }
    // Pitch bend: 14-bit value.
    const auto pb = parseBytes(p, {0xE0, 0x00, 0x40, 0x7F, 0x7F});
    CHECK(pb.size() == 2 && pb[0].kind == MidiKind::PitchBend && pb[0].value == 8192 && pb[1].value == 16383);
    CHECK(events(pb[0], ev) == 0);
}

TEST(realtime_and_system_common) {
    MidiByteParser p;
    // Real-time bytes inside a message leave it intact.
    auto m = parseBytes(p, {0x90, 0xF8, 0x3C, 0xFE, 0x64, 0x3D, 0xFA, 0x65});
    CHECK(m.size() == 5);
    if (m.size() == 5) {
        CHECK(m[0].kind == MidiKind::Realtime && m[0].status == 0xF8);
        CHECK(m[1].kind == MidiKind::Realtime && m[1].status == 0xFE);
        CHECK(m[2].kind == MidiKind::NoteOn && m[2].data1 == 0x3C && m[2].data2 == 0x64);
        CHECK(m[3].status == 0xFA);
        CHECK(m[4].kind == MidiKind::NoteOn && m[4].data1 == 0x3D);  // running status survived
    }
    // System common cancels running status: the trailing data bytes are stray.
    m = parseBytes(p, {0x90, 0x3C, 0x64, 0xF3, 0x01, 0x3C, 0x64});
    CHECK(m.size() == 2);
    if (m.size() == 2) CHECK(m[1].kind == MidiKind::SystemCommon && m[1].status == 0xF3 && m[1].data1 == 1);
    CHECK(p.counters().malformed == 2);
    // Song position (two bytes), tune request (none), undefined F4/F5.
    m = parseBytes(p, {0xF2, 0x10, 0x20, 0xF6, 0xF4, 0xF5});
    CHECK(m.size() == 2 && m[0].data1 == 0x10 && m[0].data2 == 0x20 && m[1].status == 0xF6);
}

TEST(sysex_handling) {
    MidiByteParser p;
    auto m = parseBytes(p, {0xF0, 0x7E, 0x7F, 0x06, 0x01, 0xF7, 0x90, 0x3C, 0x64});
    CHECK(m.size() == 2 && m[0].kind == MidiKind::SysEx && m[0].length == 4 && m[1].kind == MidiKind::NoteOn);
    // Real-time inside SysEx.
    m = parseBytes(p, {0xF0, 0x01, 0xF8, 0x02, 0xF7});
    CHECK(m.size() == 2 && m[0].kind == MidiKind::Realtime && m[1].kind == MidiKind::SysEx && m[1].length == 2);
    // SysEx cut by a status byte: reported, and the new message is parsed.
    const auto bad0 = p.counters().malformed;
    m = parseBytes(p, {0xF0, 0x01, 0x02, 0x90, 0x3C, 0x64});
    CHECK(m.size() == 2 && m[0].kind == MidiKind::SysEx && m[0].length == 2 && m[1].kind == MidiKind::NoteOn);
    CHECK(p.counters().malformed == bad0 + 1);
    // F0 inside SysEx starts a new one; a tune request cutting one is still reported.
    m = parseBytes(p, {0xF0, 0x01, 0xF0, 0x02, 0x03, 0xF6});
    CHECK(m.size() == 3 && m[0].length == 1 && m[1].kind == MidiKind::SysEx && m[1].length == 2 &&
          m[2].status == 0xF6);
    // Running status does not survive SysEx.
    m = parseBytes(p, {0x90, 0x3C, 0x64, 0xF0, 0xF7, 0x3C, 0x00});
    CHECK(m.size() == 2);
}

TEST(malformed_bytes) {
    MidiByteParser p;
    auto m = parseBytes(p, {0x3C, 0x64, 0xF7});  // stray data and EOX
    CHECK(m.empty() && p.counters().malformed == 3);
    // A message cut by another status byte is lost and counted.
    m = parseBytes(p, {0x90, 0x3C, 0x80, 0x3C, 0x00});
    CHECK(m.size() == 1 && m[0].kind == MidiKind::NoteOff);
    CHECK(p.counters().malformed == 4);
    p.reset();
    m = parseBytes(p, {0x3C, 0x00});
    CHECK(m.empty());
}

TEST(controller_mapping) {
    MidiByteParser p;
    Event ev[pm::host::kMaxEventsPerMessage];
    auto one = [&](std::uint8_t cc, std::uint8_t v) {
        const auto m = parseBytes(p, {0xB3, cc, v});
        return m.size() == 1 ? events(m[0], ev) : -1;
    };
    CHECK(one(64, 127) == 1 && ev[0].type == EventType::Sustain && ev[0].value == 65535);
    CHECK(one(64, 64) == 1 && ev[0].value == pm::value16FromMidi7(64));  // continuous: half pedal
    CHECK(one(64, 0) == 1 && ev[0].value == 0);
    CHECK(one(66, 127) == 1 && ev[0].type == EventType::Sostenuto);
    CHECK(one(67, 90) == 1 && ev[0].type == EventType::UnaCorda && ev[0].value == pm::value16FromMidi7(90));
    CHECK(one(120, 0) == 1 && ev[0].type == EventType::Panic);
    CHECK(one(123, 0) == 1 && ev[0].type == EventType::AllNotesOff);
    CHECK(one(124, 0) == 1 && ev[0].type == EventType::AllNotesOff);
    CHECK(one(121, 0) == 3 && ev[0].type == EventType::Sustain && ev[1].type == EventType::Sostenuto &&
          ev[2].type == EventType::UnaCorda && ev[2].value == 0);
    for (int cc : {1, 7, 11, 14, 15, 21, 74, 88}) CHECK(one(static_cast<std::uint8_t>(cc), 100) == 0);
    // Notes outside the piano give no event.
    auto m = parseBytes(p, {0x90, 20, 100, 109, 100, 21, 1, 108, 1});
    CHECK(m.size() == 4 && events(m[0], ev) == 0 && events(m[1], ev) == 0);
    CHECK(events(m[2], ev) == 1 && ev[0].note == 21 && ev[0].value == pm::value16FromMidi7(1));
    CHECK(events(m[3], ev) == 1 && ev[0].note == 108);
    // Others ignored.
    m = parseBytes(p, {0xA0, 60, 10, 0xF8, 0xF2, 1, 2});
    for (const auto& x : m) CHECK(events(x, ev) == 0);
}

TEST(ump_midi1_protocol) {
    pm::host::ParseCounters c;
    // Type 2 channel voice (what Core MIDI delivers with protocol 1.0), type 1
    // system, type 0 utility, mixed with longer packets.
    const std::vector<std::uint32_t> w = {
        0x20903C64u,               // note on, group 0, ch 0
        0x2389407Fu,               // group 3, note off ch 9
        0x22B34040u,               // CC64 = 64, ch 3
        0x10F80000u,               // clock
        0x10F31100u,               // song select 0x11
        0x00000000u,               // NOOP
        0x30160102u, 0x03040506u,  // SysEx7 start packet, 6 bytes
        0x50000000u, 0, 0, 0,      // 128-bit data
        0x20E00040u,               // pitch bend centre
    };
    const auto m = parseWords(w, c);
    CHECK(c.malformed == 0 && m.size() == 9 && c.messages == 9);
    if (m.size() != 9) return;
    CHECK(m[0].kind == MidiKind::NoteOn && m[0].data1 == 60 && m[0].data2 == 100 && m[0].wordCount == 1);
    CHECK(m[1].kind == MidiKind::NoteOff && m[1].group == 3 && m[1].channel == 9 && m[1].data2 == 127);
    CHECK(m[2].kind == MidiKind::ControlChange && m[2].channel == 3 && m[2].data1 == 64 && m[2].data2 == 64);
    CHECK(m[3].kind == MidiKind::Realtime && m[3].status == 0xF8);
    CHECK(m[4].kind == MidiKind::SystemCommon && m[4].data1 == 0x11);
    CHECK(m[5].kind == MidiKind::Utility);
    CHECK(m[6].kind == MidiKind::SysEx && m[6].length == 6 && m[6].wordCount == 2 && m[6].words[1] == 0x03040506u);
    CHECK(m[7].kind == MidiKind::Other && m[7].wordCount == 4);
    CHECK(m[8].kind == MidiKind::PitchBend && m[8].value == 8192);
    Event ev[3];
    CHECK(events(m[0], ev) == 1 && ev[0].type == EventType::NoteOn && ev[0].value == pm::value16FromMidi7(100));
    CHECK(events(m[2], ev) == 1 && ev[0].type == EventType::Sustain);

    // Byte stream and UMP decode identically.
    MidiByteParser p;
    const auto b = parseBytes(p, {0x90, 0x3C, 0x64, 0x89, 0x40, 0x7F, 0xB3, 0x40, 0x40});
    CHECK(b.size() == 3);
    for (std::size_t i = 0; i < b.size() && i < 3; ++i)
        CHECK(b[i].kind == m[i].kind && b[i].status == m[i].status && b[i].data1 == m[i].data1 &&
              b[i].data2 == m[i].data2 && b[i].value == m[i].value);
}

TEST(ump_midi2_protocol) {
    pm::host::ParseCounters c;
    const std::vector<std::uint32_t> w = {
        0x40913C00u, 0xFFFF0000u,  // note on ch 1, velocity 65535
        0x40903C00u, 0x00000000u,  // note on, velocity 0 (still a note on in 2.0)
        0x40803C00u, 0x80000000u,  // note off, release velocity 0x8000
        0x40B04000u, 0x80000000u,  // CC64 = half
        0x40B04300u, 0xFFFFFFFFu,  // CC67 = full
        0x40E00000u, 0x80000000u,  // pitch bend centre
        0x40C00000u, 0x05000000u,  // program 5
        0x40003C00u, 0x12345678u,  // registered per-note controller: Other
    };
    const auto m = parseWords(w, c);
    CHECK(c.malformed == 0 && m.size() == 8);
    if (m.size() != 8) return;
    Event ev[3];
    CHECK(m[0].protocol == 2 && m[0].kind == MidiKind::NoteOn && m[0].channel == 1 && m[0].value == 65535 &&
          m[0].data2 == 127);
    CHECK(events(m[0], ev) == 1 && ev[0].type == EventType::NoteOn && ev[0].value == 65535);
    CHECK(events(m[1], ev) == 1 && ev[0].type == EventType::NoteOn && ev[0].value == 1);
    CHECK(events(m[2], ev) == 1 && ev[0].type == EventType::NoteOff && ev[0].value == 0x8000);
    CHECK(events(m[3], ev) == 1 && ev[0].type == EventType::Sustain && ev[0].value == 0x8000);
    CHECK(events(m[4], ev) == 1 && ev[0].type == EventType::UnaCorda && ev[0].value == 0xFFFF);
    CHECK(m[5].kind == MidiKind::PitchBend && m[5].data2 == 64 && m[5].data1 == 0);
    CHECK(m[6].kind == MidiKind::ProgramChange && m[6].data1 == 5);
    CHECK(m[7].kind == MidiKind::Other && events(m[7], ev) == 0);
}

TEST(ump_malformed) {
    pm::host::ParseCounters c;
    // Bad status in type 1 and 2, oversized SysEx7 count, then a truncated
    // 64-bit packet at the end (consumed, counted).
    const std::vector<std::uint32_t> w = {0x10900000u, 0x20123456u, 0x10F70000u, 0x30070000u, 0, 0x20903C64u,
                                          0x40903C00u};
    const auto m = parseWords(w, c);
    CHECK(m.size() == 1 && m[0].kind == MidiKind::NoteOn);
    CHECK(c.malformed == 5);
    // Empty input.
    pm::host::ParseCounters c2;
    CHECK(parseWords({}, c2).empty() && c2.malformed == 0);
}

TEST(core_midi_event_list_layout) {
    auto buf = eventListHeader(1, 3);
    appendPacket(buf, 1000, {0x20903C64u, 0x20803C00u});
    appendPacket(buf, 0x0123456789ABCDEFull, {0x40903C00u, 0xFFFF0000u});  // stamp on a 4-byte boundary
    appendPacket(buf, 7, {});
    // Copy to an offset of 4 so that the list itself is not 8-byte aligned.
    std::vector<unsigned char> mis(buf.size() + 4);
    std::copy(buf.begin(), buf.end(), mis.begin() + 4);
    std::vector<std::uint64_t> stamps;
    std::vector<std::uint32_t> counts, words;
    const auto n = pm::host::forEachEventPacket(mis.data() + 4, [&](std::uint64_t s, const std::uint32_t* w, std::uint32_t k) {
        stamps.push_back(s);
        counts.push_back(k);
        for (std::uint32_t i = 0; i < k; ++i) words.push_back(w[i]);
    });
    CHECK(n == 3 && stamps.size() == 3);
    if (stamps.size() != 3) return;
    CHECK(stamps[0] == 1000 && stamps[1] == 0x0123456789ABCDEFull && stamps[2] == 7);
    CHECK(counts[0] == 2 && counts[1] == 2 && counts[2] == 0);
    CHECK(words.size() == 4 && words[0] == 0x20903C64u && words[3] == 0xFFFF0000u);
    CHECK(pm::host::forEachEventPacket(nullptr, [](std::uint64_t, const std::uint32_t*, std::uint32_t) {}) == 0);
    // The header sizes match Core MIDI's packed layout.
    CHECK(pm::host::kEventListHeaderBytes == 8 && pm::host::kEventPacketHeaderBytes == 12);
}

TEST(fuzz_byte_stream) {
    std::mt19937 rng(12345);
    MidiByteParser p;
    std::uint64_t sunk = 0;
    bool ok = true;
    std::vector<std::uint8_t> buf(4096);
    for (int round = 0; round < 500; ++round) {
        // Mix of uniformly random bytes and biased "plausible" streams.
        for (auto& b : buf) {
            const unsigned r = rng();
            b = (round & 1) ? static_cast<std::uint8_t>(r) : static_cast<std::uint8_t>((r & 7) == 0 ? 0x80 | (r >> 8) : (r >> 8) & 0x7F);
        }
        p.feed(buf.data(), buf.size(), [&](const MidiMessage& m) {
            ++sunk;
            if (m.channel > 15 || m.data1 > 127 || m.data2 > 127) ok = false;
            if (m.kind == MidiKind::Realtime && m.status < 0xF8) ok = false;
            Event ev[pm::host::kMaxEventsPerMessage];
            const int n = pm::host::toEngineEvents(m, 0, pm::TimeBase::SampleTime, ev);
            if (n < 0 || n > pm::host::kMaxEventsPerMessage) ok = false;
            for (int i = 0; i < n; ++i)
                if ((ev[i].type == EventType::NoteOn || ev[i].type == EventType::NoteOff) &&
                    (ev[i].note < 21 || ev[i].note > 108))
                    ok = false;
        });
    }
    CHECK(ok);
    CHECK(sunk == p.counters().messages);
    CHECK(sunk > 100000);
}

TEST(fuzz_ump_and_event_lists) {
    std::mt19937 rng(777);
    bool ok = true;
    std::uint64_t total = 0, bad = 0;
    std::vector<std::uint32_t> w(512);
    for (int round = 0; round < 2000; ++round) {
        const std::size_t n = rng() % w.size();
        for (std::size_t i = 0; i < n; ++i) w[i] = static_cast<std::uint32_t>(rng());
        pm::host::ParseCounters c;
        std::size_t consumed = 0;
        std::size_t i = 0;
        while (i < n) {  // decodeUmp always advances and never past the end
            MidiMessage m;
            bool good = true;
            const int k = pm::host::decodeUmp(w.data() + i, n - i, m, good);
            if (k < 1 || i + static_cast<std::size_t>(k) > n || m.wordCount > 4) ok = false;
            if (k < 1) break;
            i += static_cast<std::size_t>(k);
            consumed += static_cast<std::size_t>(k);
        }
        if (consumed != n) ok = false;
        pm::host::parseUmp(w.data(), n, c, [&](const MidiMessage& m) {
            if (m.channel > 15 || m.data1 > 127 || m.data2 > 127) ok = false;
            Event ev[pm::host::kMaxEventsPerMessage];
            const int e = pm::host::toEngineEvents(m, 1, pm::TimeBase::HostTime, ev);
            if (e < 0 || e > pm::host::kMaxEventsPerMessage) ok = false;
        });
        total += c.messages;
        bad += c.malformed;

        // A well-formed event list with random packets (word counts up to
        // 64, the last one exceeding it to test the clamp).
        const std::uint32_t packets = 1 + rng() % 8;
        auto buf = eventListHeader(1, packets);
        std::uint32_t expectWords = 0;
        for (std::uint32_t k = 0; k < packets; ++k) {
            std::vector<std::uint32_t> pw(rng() % 65);
            for (auto& x : pw) x = static_cast<std::uint32_t>(rng());
            expectWords += static_cast<std::uint32_t>(pw.size());
            appendPacket(buf, rng(), pw);
        }
        std::uint32_t seen = 0;
        pm::host::forEachEventPacket(buf.data(), [&](std::uint64_t, const std::uint32_t*, std::uint32_t k) { seen += k; });
        if (seen != expectWords) ok = false;
    }
    CHECK(ok);
    CHECK(total > 10000 && bad > 0);
}

TEST(event_list_word_count_clamped) {
    // A packet header claiming more than 64 words is read as 64 (the size of
    // MIDIEventPacket::words); nothing beyond is touched.
    auto buf = eventListHeader(1, 1);
    std::vector<std::uint32_t> words(64, 0x20903C64u);
    appendPacket(buf, 5, words);
    const std::uint32_t bogus = 1000;
    std::memcpy(&buf[8 + 8], &bogus, 4);
    std::uint32_t seen = 0;
    pm::host::forEachEventPacket(buf.data(), [&](std::uint64_t, const std::uint32_t*, std::uint32_t k) { seen = k; });
    CHECK(seen == 64);
}
