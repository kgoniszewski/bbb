#include "pm_host/midi.h"

namespace pm::host {
namespace {

MidiKind channelKind(std::uint8_t status) noexcept {
    switch (status & 0xF0) {
        case 0x80: return MidiKind::NoteOff;
        case 0x90: return MidiKind::NoteOn;
        case 0xA0: return MidiKind::PolyPressure;
        case 0xB0: return MidiKind::ControlChange;
        case 0xC0: return MidiKind::ProgramChange;
        case 0xD0: return MidiKind::ChannelPressure;
        default: return MidiKind::PitchBend;
    }
}

int channelDataBytes(std::uint8_t status) noexcept {
    const std::uint8_t hi = status & 0xF0;
    return (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
}

// MIDI 1.0 message from its bytes (channel voice or system common).
void fromBytes(std::uint8_t status, std::uint8_t d1, std::uint8_t d2, MidiMessage& m) noexcept {
    m.status = status;
    m.data1 = d1;
    m.data2 = d2;
    m.protocol = 1;
    if (status < 0xF0) {
        m.kind = channelKind(status);
        m.channel = status & 0x0F;
        if (m.kind == MidiKind::PitchBend) m.value = static_cast<std::uint32_t>(d1) | (static_cast<std::uint32_t>(d2) << 7);
        else if (channelDataBytes(status) == 1) m.value = d1;
        else m.value = d2;
    } else {
        m.kind = status >= 0xF8 ? MidiKind::Realtime : MidiKind::SystemCommon;
        m.value = d1;
    }
}

}  // namespace

void MidiByteParser::reset() noexcept {
    running_ = pending_ = have_ = need_ = 0;
    inSysEx_ = false;
    sysExLength_ = 0;
}

void MidiByteParser::emitSysEx(MidiMessage& out) noexcept {
    out = MidiMessage{};
    out.kind = MidiKind::SysEx;
    out.status = 0xF0;
    out.length = sysExLength_;
    inSysEx_ = false;
    sysExLength_ = 0;
    ++counters_.messages;
}

int MidiByteParser::feedByte(std::uint8_t b, MidiMessage* out) noexcept {
    if (b >= 0xF8) {  // real-time: anywhere, state untouched
        out[0] = MidiMessage{};
        fromBytes(b, 0, 0, out[0]);
        ++counters_.messages;
        return 1;
    }
    if (b >= 0x80) {
        int n = 0;
        if (inSysEx_) {
            emitSysEx(out[n++]);
            if (b == 0xF7) return n;
            ++counters_.malformed;  // SysEx cut by a status byte: reported with the bytes so far
        } else if (b == 0xF7) {
            ++counters_.malformed;  // EOX without SysEx
            return 0;
        }
        // A partly assembled message is lost when a new status arrives.
        if (pending_ != 0 && have_ > 0) ++counters_.malformed;
        pending_ = have_ = need_ = 0;
        if (b >= 0xF0) {
            running_ = 0;  // system common and SysEx cancel running status
            switch (b) {
                case 0xF0: inSysEx_ = true; sysExLength_ = 0; break;
                case 0xF1: case 0xF3: pending_ = b; need_ = 1; break;
                case 0xF2: pending_ = b; need_ = 2; break;
                case 0xF6:
                    out[n] = MidiMessage{};
                    fromBytes(b, 0, 0, out[n++]);
                    ++counters_.messages;
                    break;
                default: break;  // F4, F5: undefined, ignored
            }
        } else {
            running_ = pending_ = b;
            need_ = static_cast<std::uint8_t>(channelDataBytes(b));
        }
        return n;
    }
    // Data byte.
    if (inSysEx_) {
        ++sysExLength_;
        return 0;
    }
    if (pending_ == 0) {
        if (running_ == 0) {
            ++counters_.malformed;  // stray data byte
            return 0;
        }
        pending_ = running_;
        need_ = static_cast<std::uint8_t>(channelDataBytes(running_));
        have_ = 0;
    }
    data_[have_++] = b;
    if (have_ < need_) return 0;
    out[0] = MidiMessage{};
    fromBytes(pending_, data_[0], need_ == 2 ? data_[1] : 0, out[0]);
    pending_ = have_ = 0;
    ++counters_.messages;
    return 1;
}

int decodeUmp(const std::uint32_t* words, std::size_t available, MidiMessage& out, bool& ok) noexcept {
    ok = true;
    out = MidiMessage{};
    if (available == 0) {
        ok = false;
        return 0;
    }
    const std::uint32_t w0 = words[0];
    const int n = umpWordCount(w0);
    if (static_cast<std::size_t>(n) > available) {
        ok = false;
        return static_cast<int>(available);
    }
    out.wordCount = static_cast<std::uint8_t>(n);
    for (int i = 0; i < n; ++i) out.words[i] = words[i];
    out.group = static_cast<std::uint8_t>((w0 >> 24) & 0x0F);
    const std::uint32_t mt = w0 >> 28;
    const auto status = static_cast<std::uint8_t>((w0 >> 16) & 0xFF);
    const auto b1 = static_cast<std::uint8_t>((w0 >> 8) & 0x7F);
    const auto b2 = static_cast<std::uint8_t>(w0 & 0x7F);
    switch (mt) {
        case 0x0:
            out.kind = MidiKind::Utility;
            out.status = status;
            return n;
        case 0x1:
            if (status < 0xF1 || status == 0xF7) {
                ok = false;
                return n;
            }
            fromBytes(status, b1, b2, out);
            return n;
        case 0x2:
            if (status < 0x80 || status >= 0xF0) {
                ok = false;
                return n;
            }
            fromBytes(status, b1, b2, out);
            return n;
        case 0x3: {
            const std::uint32_t count = (w0 >> 16) & 0x0F;
            if (count > 6) {
                ok = false;
                return n;
            }
            out.kind = MidiKind::SysEx;
            out.status = 0xF0;
            out.data1 = static_cast<std::uint8_t>((w0 >> 20) & 0x0F);  // 0 complete, 1 start, 2 continue, 3 end
            out.length = count;
            return n;
        }
        case 0x4: {
            const std::uint32_t w1 = words[1];
            const auto opcode = static_cast<std::uint8_t>((w0 >> 20) & 0x0F);
            out.protocol = 2;
            out.channel = static_cast<std::uint8_t>((w0 >> 16) & 0x0F);
            if (opcode < 0x8 || opcode > 0xE) {
                out.kind = MidiKind::Other;  // per-note and registered controllers, per-note management
                out.status = status;
                out.data1 = b1;
                out.value = w1;
                return n;
            }
            out.status = static_cast<std::uint8_t>((opcode << 4) | out.channel);
            out.kind = channelKind(out.status);
            switch (opcode) {
                case 0x8:
                case 0x9:
                    out.data1 = b1;
                    out.value = w1 >> 16;
                    out.data2 = static_cast<std::uint8_t>(out.value >> 9);
                    if (opcode == 0x9 && out.data2 == 0) out.data2 = 1;  // 2.0 velocity 0 is still a note-on
                    break;
                case 0xA:
                case 0xB:
                    out.data1 = b1;
                    out.value = w1;
                    out.data2 = static_cast<std::uint8_t>(w1 >> 25);
                    break;
                case 0xC:
                    out.data1 = static_cast<std::uint8_t>((w1 >> 24) & 0x7F);
                    out.value = out.data1;
                    break;
                case 0xD:
                    out.value = w1;
                    out.data1 = static_cast<std::uint8_t>(w1 >> 25);
                    break;
                default:  // 0xE pitch bend
                    out.value = w1;
                    out.data1 = static_cast<std::uint8_t>((w1 >> 18) & 0x7F);
                    out.data2 = static_cast<std::uint8_t>(w1 >> 25);
                    break;
            }
            return n;
        }
        default:
            out.kind = MidiKind::Other;
            out.status = status;
            return n;
    }
}

int toEngineEvents(const MidiMessage& m, std::uint64_t time, TimeBase base, Event* out) noexcept {
    auto make = [&](EventType t, std::uint8_t note, std::uint16_t value) {
        Event e;
        e.time = time;
        e.timeBase = base;
        e.type = t;
        e.note = note;
        e.value = value;
        return e;
    };
    const bool v2 = m.protocol == 2;
    switch (m.kind) {
        case MidiKind::NoteOn:
        case MidiKind::NoteOff: {
            if (m.data1 < 21 || m.data1 > 108) return 0;
            const std::uint16_t vel = v2 ? static_cast<std::uint16_t>(m.value) : value16FromMidi7(m.data2);
            if (m.kind == MidiKind::NoteOff) {
                out[0] = make(EventType::NoteOff, m.data1, vel);
            } else if (vel == 0) {
                if (v2) out[0] = make(EventType::NoteOn, m.data1, 1);  // see decodeUmp
                else out[0] = make(EventType::NoteOff, m.data1, 0);  // MIDI 1.0 running-status note-off
            } else {
                out[0] = make(EventType::NoteOn, m.data1, vel);
            }
            return 1;
        }
        case MidiKind::ControlChange: {
            const std::uint16_t v = v2 ? static_cast<std::uint16_t>(m.value >> 16) : value16FromMidi7(m.data2);
            switch (m.data1) {
                case 64: out[0] = make(EventType::Sustain, 0, v); return 1;
                case 66: out[0] = make(EventType::Sostenuto, 0, v); return 1;
                case 67: out[0] = make(EventType::UnaCorda, 0, v); return 1;
                case 120: out[0] = make(EventType::Panic, 0, 0); return 1;
                case 121:
                    out[0] = make(EventType::Sustain, 0, 0);
                    out[1] = make(EventType::Sostenuto, 0, 0);
                    out[2] = make(EventType::UnaCorda, 0, 0);
                    return 3;
                case 123:
                case 124:
                case 125:
                case 126:
                case 127:  // mode messages also end all notes (MIDI 1.0 spec)
                    out[0] = make(EventType::AllNotesOff, 0, 0);
                    return 1;
                default: return 0;
            }
        }
        default: return 0;
    }
}

}  // namespace pm::host
