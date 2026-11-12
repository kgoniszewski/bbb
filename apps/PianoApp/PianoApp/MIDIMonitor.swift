// MIDI monitor log. Entries come from the C++ host's lock-free ring (filled
// on the MIDI thread); this class drains it on the main thread only.
import Foundation
import PianoCore

struct MonitorLine: Identifiable {
    let id: Int
    let text: String
}

final class MIDIMonitor {
    static let maxLines = 2000
    static let trimBatch = 500

    private(set) var lines: [MonitorLine] = []
    private var nextId = 0
    private var lastHostTime: UInt64 = 0
    private var markers = 0
    var paused = false

    /// Drains the ring. Returns true if lines were added. Main thread.
    func drain(host: EngineHost, sourceName: (UInt32) -> String) -> Bool {
        var entry = pm_midi_monitor_entry()
        var added = false
        var budget = 4096  // keep one drain short; the rest comes next tick
        while budget > 0 && pm_host_monitor_pop(host.raw, &entry) == 1 {
            budget -= 1
            if paused { continue }
            let text = MIDIMonitor.format(entry, delta: deltaMs(entry.host_time, host: host), source: sourceName(entry.source_id))
            lines.append(MonitorLine(id: nextId, text: text))
            nextId += 1
            added = true
        }
        // Trimmed in batches: removeFirst moves the whole array.
        if lines.count > MIDIMonitor.maxLines + MIDIMonitor.trimBatch {
            lines.removeFirst(lines.count - MIDIMonitor.maxLines)
        }
        return added
    }

    func clear() {
        lines.removeAll()
        lastHostTime = 0
        markers = 0
    }

    /// Inserts a numbered marker line (for capture sessions: press it before
    /// each control you test and note what the number stands for).
    func addMarker() -> Int {
        markers += 1
        lines.append(MonitorLine(id: nextId, text: "----- marker \(markers) -----"))
        nextId += 1
        return markers
    }

    /// The whole log as text (for Copy / Share), with a header.
    func exportText(header: String) -> String {
        var out = header + "\n"
        out += "delta_ms\tsource\tgroup\tchannel\ttype\tdata\traw\n"
        for line in lines {
            out += line.text + "\n"
        }
        return out
    }

    private func deltaMs(_ t: UInt64, host: EngineHost) -> Double? {
        defer { if t != 0 { lastHostTime = t } }
        if t == 0 || lastHostTime == 0 { return nil }
        return host.milliseconds(ticks: Int64(bitPattern: t &- lastHostTime))
    }

    // MARK: - Formatting (tab-separated so the export pastes into a spreadsheet)

    static func format(_ e: pm_midi_monitor_entry, delta: Double?, source: String) -> String {
        let deltaText = delta.map { String(format: "+%.3f", $0) } ?? "-"
        let kind = Int(e.kind)
        let channel = isChannelKind(kind) ? String(Int(e.channel) + 1) : "-"
        let (type, data) = describe(e)
        return "\(deltaText)\t\(source)\t\(Int(e.group) + 1)\t\(channel)\t\(type)\t\(data)\t\(raw(e))"
    }

    private static func isChannelKind(_ kind: Int) -> Bool {
        return kind >= Int(PM_MIDI_NOTE_OFF.rawValue) && kind <= Int(PM_MIDI_PITCH_BEND.rawValue)
    }

    private static func describe(_ e: pm_midi_monitor_entry) -> (String, String) {
        let d1 = Int(e.data1), d2 = Int(e.data2)
        let v2 = e.protocol == 2
        switch Int(e.kind) {
        case Int(PM_MIDI_NOTE_ON.rawValue):
            return ("NoteOn", v2 ? "\(noteName(d1)) (\(d1)) vel16 \(e.value)" : "\(noteName(d1)) (\(d1)) vel \(d2)")
        case Int(PM_MIDI_NOTE_OFF.rawValue):
            return ("NoteOff", v2 ? "\(noteName(d1)) (\(d1)) vel16 \(e.value)" : "\(noteName(d1)) (\(d1)) vel \(d2)")
        case Int(PM_MIDI_CONTROL_CHANGE.rawValue):
            return ("CC", v2 ? "cc \(d1) value32 \(e.value)" : "cc \(d1) value \(d2)")
        case Int(PM_MIDI_PITCH_BEND.rawValue):
            return ("PitchBend", v2 ? "value32 \(e.value)" : "value \(e.value) (\(Int(e.value) - 8192))")
        case Int(PM_MIDI_PROGRAM_CHANGE.rawValue):
            return ("Program", "\(d1)")
        case Int(PM_MIDI_CHANNEL_PRESSURE.rawValue):
            return ("ChanPressure", v2 ? "value32 \(e.value)" : "\(d1)")
        case Int(PM_MIDI_POLY_PRESSURE.rawValue):
            return ("PolyPressure", "\(noteName(d1)) \(d2)")
        case Int(PM_MIDI_SYSEX.rawValue):
            return ("SysEx", "\(e.length) bytes")
        case Int(PM_MIDI_SYSTEM_COMMON.rawValue):
            return ("SysCommon", String(format: "%02X %d %d", Int(e.status), d1, d2))
        case Int(PM_MIDI_REALTIME.rawValue):
            return ("Realtime", realtimeName(Int(e.status)))
        case Int(PM_MIDI_UTILITY.rawValue):
            return ("Utility", "")
        default:
            return ("Other", String(format: "status %02X", Int(e.status)))
        }
    }

    private static func raw(_ e: pm_midi_monitor_entry) -> String {
        let count = Int(e.word_count)
        if count == 0 {
            // Byte stream: the MIDI 1.0 bytes.
            return String(format: "%02X %02X %02X", Int(e.status), Int(e.data1), Int(e.data2))
        }
        let words = [e.words.0, e.words.1, e.words.2, e.words.3]
        var parts: [String] = []
        for i in 0..<min(count, 4) {
            parts.append(String(format: "%08X", words[i]))
        }
        return parts.joined(separator: " ")
    }

    private static func realtimeName(_ status: Int) -> String {
        switch status {
        case 0xF8: return "Clock"
        case 0xFA: return "Start"
        case 0xFB: return "Continue"
        case 0xFC: return "Stop"
        case 0xFE: return "ActiveSensing"
        case 0xFF: return "Reset"
        default: return String(format: "%02X", status)
        }
    }

    static func noteName(_ note: Int) -> String {
        let names = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
        guard note >= 0 && note < 128 else { return "?" }
        return "\(names[note % 12])\(note / 12 - 1)"
    }
}
