// Owner of the C++ engine host (pm_host in core/pm_api/pm_api.h).
//
// Threads:
//  - render: AudioController's source-node block calls pm_host_render with the
//    raw pointer only (no Swift objects, no allocation).
//  - MIDI: MIDIController's receive block calls pm_host_midi_event_list.
//  - main thread: on-screen keyboard and pedal events (pushNow), meters,
//    monitor draining.
//  - prepareQueue: every call that changes prepare-time parameters
//    (quality tier, soundboard) and every pm_host_prepare, serialised.
import Foundation
import PianoCore

final class EngineHost {
    /// pm_host*; valid for the lifetime of this object.
    let raw: OpaquePointer
    /// The engine inside the host (parameters and statistics only).
    let engine: OpaquePointer
    /// Host-clock ticks to nanoseconds: ns = ticks * numer / denom.
    let timebaseNumer: UInt32
    let timebaseDenom: UInt32

    private let prepareQueue = DispatchQueue(label: "piano.engine.prepare", qos: .userInitiated)

    init?() {
        guard let h = pm_host_create() else { return nil }
        guard let e = pm_host_engine(h) else {
            pm_host_destroy(h)
            return nil
        }
        raw = h
        engine = e
        var tb = mach_timebase_info_data_t()
        mach_timebase_info(&tb)
        timebaseNumer = tb.numer == 0 ? 1 : tb.numer
        timebaseDenom = tb.denom == 0 ? 1 : tb.denom
        pm_host_set_timebase(h, timebaseNumer, timebaseDenom)
    }

    deinit {
        // AudioController and MIDIController must be torn down first.
        pm_host_destroy(raw)
    }

    // MARK: - Preparing (background)

    /// Prepares the engine for a sample rate on the background queue, then
    /// calls completion on the main thread with success. The audio thread may
    /// keep running: it outputs silence until the prepare is done.
    func prepare(sampleRate: Double, maxFrames: Int, completion: @escaping (Bool) -> Void) {
        // Pointers cross to the queue as integers (OpaquePointer is not Sendable).
        let hostBits = Int(bitPattern: raw)
        let frames = Int32(max(16, min(maxFrames, 8192)))
        // The closure retains self, so deinit (pm_host_destroy) cannot run
        // while a prepare is in flight.
        prepareQueue.async { [self] in
            let ok = withExtendedLifetime(self) {
                pm_host_prepare(OpaquePointer(bitPattern: hostBits), sampleRate, frames) == 1
            }
            DispatchQueue.main.async {
                completion(ok)
            }
        }
    }

    /// Sets a prepare-time parameter (tuning, soundboard, quality tier) on the
    /// prepare queue, so it never races a running prepare.
    func setPrepareTimeParam(_ id: pm_param, _ value: Double) {
        let engineBits = Int(bitPattern: engine)
        let idValue = Int32(id.rawValue)
        prepareQueue.async { [self] in
            withExtendedLifetime(self) {
                _ = pm_engine_set_param(OpaquePointer(bitPattern: engineBits), idValue, value)
            }
        }
    }

    /// Reads any parameter (the value may lag a pending setPrepareTimeParam).
    func param(_ id: pm_param) -> Double {
        var value = 0.0
        _ = pm_engine_get_param(engine, Int32(id.rawValue), &value)
        return value
    }

    /// Real-time parameters may be set from the main thread directly.
    func setRealtimeParam(_ id: pm_param, _ value: Double) {
        _ = pm_engine_set_param(engine, Int32(id.rawValue), value)
    }

    var isPrepared: Bool { pm_host_is_prepared(raw) == 1 }
    var preparedSampleRate: Double { pm_host_sample_rate(raw) }

    // MARK: - Events from the main thread

    /// Queues an event for "as soon as possible" (the start of the next buffer).
    @discardableResult
    func pushNow(_ type: pm_event_type, note: Int = 0, value: Int = 0) -> Bool {
        var e = pm_event()
        e.time = 0
        e.type = UInt8(truncatingIfNeeded: type.rawValue)
        e.time_base = UInt8(truncatingIfNeeded: PM_TIME_SAMPLE.rawValue)
        e.note = UInt8(clamping: note)
        e.value = UInt16(clamping: value)
        return pm_host_push_event(raw, &e) == 1
    }

    func noteOn(_ note: Int, velocity16: Int) { pushNow(PM_EVENT_NOTE_ON, note: note, value: max(1, velocity16)) }
    func noteOff(_ note: Int) { pushNow(PM_EVENT_NOTE_OFF, note: note, value: 0) }
    func sustain(_ down: Bool) { pushNow(PM_EVENT_SUSTAIN, value: down ? 65535 : 0) }
    func allNotesOff() { pushNow(PM_EVENT_ALL_NOTES_OFF) }
    func panic() { pushNow(PM_EVENT_PANIC) }

    // MARK: - Meters (main thread)

    func load() -> pm_host_load {
        var l = pm_host_load()
        pm_host_get_load(raw, &l)
        return l
    }

    func resetLoad() { pm_host_reset_load(raw) }

    func counters() -> pm_host_counters {
        var c = pm_host_counters()
        pm_host_get_counters(raw, &c)
        return c
    }

    func stats() -> pm_stats {
        var s = pm_stats()
        pm_engine_get_stats(engine, &s)
        return s
    }

    /// Load diagnostics: percentiles, frame counts and the worst buffer.
    func diagnostics() -> pm_host_diagnostics {
        var d = pm_host_diagnostics()
        pm_host_get_diagnostics(raw, &d)
        return d
    }

    /// The MIDI monitor ring is filled only while enabled.
    func setMonitorEnabled(_ on: Bool) { pm_host_set_monitor_enabled(raw, on ? 1 : 0) }

    /// Thread CPU time per buffer (two kernel clock reads per buffer; off by default).
    func setCpuTiming(_ on: Bool) { pm_host_set_cpu_timing(raw, on ? 1 : 0) }

    /// Per-phase render time (a few user-space clock reads per sub-block; off by default).
    func setPhaseTiming(_ on: Bool) { pm_host_set_phase_timing(raw, on ? 1 : 0) }

    /// Phase split (worst buffer, top 1 %, mean) and what the worst buffer did.
    func phaseDiagnostics() -> pm_host_phase_diagnostics {
        var d = pm_host_phase_diagnostics()
        pm_host_get_phase_diagnostics(raw, &d)
        return d
    }

    /// How the engine was compiled (SIMD kernel, optimisation, NDEBUG, compiler).
    static var buildInfo: String { String(cString: pm_build_info()) }

    /// Host ticks to milliseconds.
    func milliseconds(ticks: Int64) -> Double {
        return Double(ticks) * Double(timebaseNumer) / Double(timebaseDenom) / 1_000_000.0
    }
}
