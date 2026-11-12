// App state for the SwiftUI views. Main thread only. Owns the engine host,
// the audio and MIDI controllers, and polls meters and the MIDI monitor ring
// on timers.
//
// UI cost. An ObservableObject invalidates every view observing it on each
// assignment to a @Published property, even of an unchanged value, so:
//  - What changes while playing lives in small stores of its own: the
//    meters (MeterStore: one Equatable snapshot of values rounded to what
//    the screen shows), the process-CPU line (CpuStore, one Text), the
//    keyboard position (KeyboardStore). Nothing is assigned unless it
//    differs from what is shown, so an idle engine publishes nothing (the
//    process-CPU line at most once a second).
//  - Meters are polled at 4 Hz, and only while the Play tab is visible and
//    the app is active; the MIDI monitor ring is enabled, drained and
//    formatted only while the monitor tab is visible and the app is active.
//  - Live counters that change without playing (MIDI active sensing or
//    clock, buffer counts, mean render time) are not on screen; the
//    diagnostics text with all of them is built only when Copy is pressed.
//  - The keyboard observes only KeyboardStore; the Play screen itself
//    observes nothing.
//  - What is left on the main thread cannot be profiled from here, so the
//    Play screen has a "UI cost test" switch (UICostMode) that hides the
//    keyboard, stops the meter poll, or both, while the once-a-second
//    process-CPU line keeps running: the change in "main thread" CPU shows
//    what the hidden part cost.
import Combine
import Foundation
import PianoCore
#if canImport(UIKit)
import UIKit
#elseif canImport(AppKit)
import AppKit
#endif

/// What the Play screen shows of the meters, rounded to the displayed
/// precision, so that an unchanged display compares equal.
struct MeterSnapshot: Equatable {
    var recentPct = 0.0  // render load, % of the buffer period, 0.1 % steps
    var meanPct = 0.0
    var p99Pct = 0.0
    var maxPct = 0.0
    var budgetPct = 70
    var overBudget: UInt64 = 0
    var renderLine = ""      // frame counts seen (changes only with the I/O configuration)
    var worstLine = "worst buffer: -"
}

/// The engine and MIDI status rows, kept apart from the load meters so that
/// a changing load (which it does at every poll while sound plays) does not
/// re-render the status section, and vice versa.
struct EngineStatusSnapshot: Equatable {
    var renderFrames = 0
    var awakeModes = 0
    var activeHammers = 0
    var midiEvents: UInt64 = 0   // engine events from MIDI (not active sensing or clock)
    var droppedEvents: UInt64 = 0
}

/// Meter values, separate from AppModel so that only the views showing them
/// re-render, and only when what they show changed.
final class MeterStore: ObservableObject {
    @Published private(set) var snapshot = MeterSnapshot()

    func update(_ s: MeterSnapshot) {
        if s != snapshot { snapshot = s }
    }
}

final class EngineStatusStore: ObservableObject {
    @Published private(set) var snapshot = EngineStatusSnapshot()

    func update(_ s: EngineStatusSnapshot) {
        if s != snapshot { snapshot = s }
    }
}

/// The process-CPU line (updated once a second while the Play tab is shown),
/// observed by one Text only.
final class CpuStore: ObservableObject {
    @Published private(set) var line = "process CPU: measuring..."

    func update(_ l: String) {
        if l != line { line = l }
    }
}

/// The on-screen keyboard's position, the only state the keyboard observes.
final class KeyboardStore: ObservableObject {
    @Published var lowestNote = 48  // first C shown
}

/// The MIDI monitor's published lines, separate from AppModel so that the
/// monitor screen re-renders only when lines arrive.
final class MonitorStore: ObservableObject {
    @Published var lines: [MonitorLine] = []
    @Published var paused = false
}

/// Play-screen diagnostics: what to switch off to see what it costs on the
/// main thread (the process-CPU line stays on in every mode).
enum UICostMode: Int, CaseIterable, Identifiable {
    case normal = 0, noKeyboard = 1, noMeters = 2, minimal = 3
    var id: Int { rawValue }
    var label: String {
        switch self {
        case .normal: return "All"
        case .noKeyboard: return "No keyboard"
        case .noMeters: return "No meters"
        case .minimal: return "CPU line only"
        }
    }
    var showsKeyboard: Bool { self == .normal || self == .noMeters }
    var pollsMeters: Bool { self == .normal || self == .noKeyboard }
}

/// Small store for the UI cost mode (observed by the Play screen only).
final class UICostStore: ObservableObject {
    @Published var mode: UICostMode = .normal
}

enum QualityTier: Int, CaseIterable, Identifiable {
    case full = 0, balanced = 1, eco = 2
    var id: Int { rawValue }
    var label: String {
        switch self {
        case .full: return "Full"
        case .balanced: return "Balanced"
        case .eco: return "Eco"
        }
    }
}

final class AppModel: ObservableObject {
    // Audio status
    @Published var audio = AudioController.Status()
    // MIDI
    @Published var sources: [MIDIController.Source] = []
    @Published var midiMessage = ""
    // Meters (separately observed)
    let meters = MeterStore()
    let engineStatus = EngineStatusStore()
    let cpuStore = CpuStore()
    let keyboard = KeyboardStore()
    @Published var cpuTiming = false
    @Published var phaseTiming = false
    @Published var speedProbe = false
    let uiCost = UICostStore()
    /// Speed probe on the audio thread: every this many render calls.
    static let speedProbeInterval = 8
    /// The probe on the benchmark thread at the last benchmark run (reference).
    private var benchProbe: pm_speed_probe_stats?
    // Offline benchmark (Benchmark tab)
    @Published var benchRunning = false
    @Published var benchProgress = ""
    @Published var benchLines: [String] = []
    @Published var benchAllTiers = false
    @Published var benchLiveIO = false  // 512 frames at 44.1 kHz (what an iPad app on a Mac is granted)
    private var benchHeader = ""
    private let benchQueue = DispatchQueue(label: "piano.benchmark", qos: .userInteractive)
    // Controls
    @Published var sustainDown = false
    @Published var tier: QualityTier = .full
    @Published var boardOn = true
    // MIDI monitor
    let monitorStore = MonitorStore()

    let host: EngineHost?
    private var audioController: AudioController?
    private var midiController: MIDIController?
    private let monitor = MIDIMonitor()
    private var meterTimer: Timer?    // 4 Hz, while the Play tab is visible and the app active
    private var monitorTimer: Timer?  // 10 Hz, while the monitor tab is visible and the app active
    private var meterTicks = 0
    private var monitorVisible = false
    private var playVisible = false
    private var appActive = true
    // Previous 1 Hz CPU sample: process CPU, main-thread CPU and audio render
    // time (seconds), and the wall time (seconds).
    private var cpuPrev: (process: Double, main: Double, render: Double, wall: Double)?
    // Last process-CPU reading, fractions of one core (< 0: unknown).
    private var processCpu = -1.0, mainThreadCpu = -1.0, audioRenderCpu = -1.0
    private var started = false
    private var terminateObserver: NSObjectProtocol?

    init() {
        host = EngineHost()
        guard let host = host else {
            audio.message = "Could not create the engine"
            return
        }
        let ac = AudioController(host: host)
        ac.onStatus = { [weak self] status in
            self?.audio = status
        }
        audioController = ac
        let mc = MIDIController(host: host)
        mc.onSourcesChanged = { [weak self] list in
            self?.sources = list
        }
        mc.onSourceRemoved = { [weak self] _ in
            // Release whatever the device was holding (plan §3.1). All notes
            // and the pedals, since sources are not tracked per note yet.
            guard let host = self?.host else { return }
            host.allNotesOff()
            host.pushNow(PM_EVENT_SUSTAIN, value: 0)
            host.pushNow(PM_EVENT_SOSTENUTO, value: 0)
            host.pushNow(PM_EVENT_UNA_CORDA, value: 0)
        }
        midiController = mc
        host.setMonitorEnabled(false)  // until the monitor screen is shown
        tier = QualityTier(rawValue: Int(host.param(PM_PARAM_QUALITY_TIER))) ?? .full
        boardOn = host.param(PM_PARAM_SOUNDBOARD_ENABLED) >= 0.5
    }

    // MARK: - Lifecycle

    func start() {
        guard !started, host != nil else { return }
        started = true
        audioController?.start()
        if let mc = midiController {
            mc.start()
            midiMessage = mc.statusMessage
            sources = mc.sources
        }
        updateTimers()
        #if canImport(UIKit)
        // Playing from a hardware keyboard does not touch the screen.
        UIApplication.shared.isIdleTimerDisabled = true
        let terminate = UIApplication.willTerminateNotification
        #else
        let terminate = NSApplication.willTerminateNotification
        #endif
        guard terminateObserver == nil else { return }
        terminateObserver = NotificationCenter.default.addObserver(forName: terminate, object: nil, queue: .main) { [weak self] _ in
            self?.terminate()
        }
    }

    /// willTerminate: stop the render callback (bounded wait) and nothing
    /// more. The host is never destroyed here, so a late callback is safe.
    private func terminate() {
        meterTimer?.invalidate()
        meterTimer = nil
        monitorTimer?.invalidate()
        monitorTimer = nil
        audioController?.shutdownForTermination()
    }

    func stop() {
        audioController?.stop()
        started = false
        updateTimers()
    }

    // MARK: - Polling (main thread)

    /// Starts or stops the polling timers for what is visible: meters only
    /// while the Play tab is shown and the app is active, the MIDI monitor
    /// only while its tab is shown and the app is active. With neither,
    /// the main thread is not woken at all.
    private func updateTimers() {
        let wantMeters = started && playVisible && appActive
        if wantMeters, meterTimer == nil {
            cpuPrev = nil  // the first CPU reading covers a full second of this period
            meterTicks = 0
            let t = Timer(timeInterval: 0.25, repeats: true) { [weak self] _ in
                self?.meterTick()
            }
            t.tolerance = 0.05
            RunLoop.main.add(t, forMode: .common)
            meterTimer = t
            meterTick()
        } else if !wantMeters, let t = meterTimer {
            t.invalidate()
            meterTimer = nil
        }
        let wantMonitor = started && monitorVisible && appActive
        host?.setMonitorEnabled(wantMonitor)
        if wantMonitor, monitorTimer == nil {
            let t = Timer(timeInterval: 0.1, repeats: true) { [weak self] _ in
                self?.drainMonitor()
            }
            t.tolerance = 0.02
            RunLoop.main.add(t, forMode: .common)
            monitorTimer = t
        } else if !wantMonitor, let t = monitorTimer {
            t.invalidate()
            monitorTimer = nil
        }
    }

    /// The Play tab calls this from onAppear / onDisappear.
    func setPlayVisible(_ visible: Bool) {
        playVisible = visible
        updateTimers()
    }

    /// Scene phase: polling stops while the app is not active (audio and
    /// MIDI keep running).
    func setAppActive(_ active: Bool) {
        appActive = active
        updateTimers()
    }

    /// The monitor screen calls this from onAppear / onDisappear: the MIDI
    /// thread fills the monitor ring, and this model drains and formats it,
    /// only while the screen is visible (and the app active).
    func setMonitorVisible(_ visible: Bool) {
        monitorVisible = visible
        updateTimers()
    }

    private func drainMonitor() {
        guard let host = host else { return }
        let mc = midiController
        monitor.paused = monitorStore.paused
        if monitor.drain(host: host, sourceName: { id in mc?.name(forSource: id) ?? "?" }) {
            monitorStore.lines = monitor.lines
        }
    }

    private func meterTick() {
        guard let host = host else { return }
        meterTicks += 1
        if uiCost.mode.pollsMeters { updateMeters(host) }
        if meterTicks % 4 == 1 { updateProcessCpu(host) }  // once a second
    }

    private static func roundedPct(_ fraction: Double) -> Double { (fraction * 1000.0).rounded() / 10.0 }

    /// Builds the displayed snapshot; MeterStore publishes it only if it
    /// differs from what is shown. A handful of C calls, no allocation on
    /// the audio side.
    private func updateMeters(_ host: EngineHost) {
        let d = host.diagnostics()
        let counters = host.counters()
        let stats = host.stats()
        var s = MeterSnapshot()
        s.recentPct = AppModel.roundedPct(d.recent_load)
        s.meanPct = AppModel.roundedPct(d.mean_load)
        s.p99Pct = AppModel.roundedPct(d.p99_load)
        s.maxPct = AppModel.roundedPct(d.max_load)
        s.budgetPct = Int((d.budget * 100).rounded())
        s.overBudget = d.over_budget
        if d.buffers > 0 {
            s.renderLine = "Render buffer: \(d.last_frames) frames = \(AppModel.us(d.last_period_us)); seen \(d.min_frames)...\(d.max_frames) frames, \(d.distinct_frame_counts) distinct size(s)"
        }
        s.worstLine = AppModel.worstBufferText(d)
        meters.update(s)
        var e = EngineStatusSnapshot()
        e.renderFrames = Int(d.last_frames)
        e.awakeModes = Int(stats.awake_modes)
        e.activeHammers = Int(stats.active_hammers)
        e.midiEvents = counters.midi_events
        e.droppedEvents = counters.dropped_events + stats.dropped_events
        engineStatus.update(e)
    }

    /// Process CPU (getrusage: all threads), main-thread CPU (this thread's
    /// CPU clock) and audio render time (from the load meter) over the last
    /// second, as fractions of one core. Once a second on the main thread,
    /// while the Play tab is visible: two system calls and one meter
    /// snapshot (a few microseconds), nothing on the audio thread.
    private func updateProcessCpu(_ host: EngineHost) {
        var ru = rusage()
        guard getrusage(RUSAGE_SELF, &ru) == 0 else { return }
        let process = Double(ru.ru_utime.tv_sec) + Double(ru.ru_utime.tv_usec) * 1e-6
            + Double(ru.ru_stime.tv_sec) + Double(ru.ru_stime.tv_usec) * 1e-6
        let main = Double(clock_gettime_nsec_np(CLOCK_THREAD_CPUTIME_ID)) * 1e-9
        let d = host.diagnostics()
        let render = d.mean_us * Double(d.buffers) * 1e-6  // since the last meter reset
        let wall = ProcessInfo.processInfo.systemUptime
        if let p = cpuPrev, wall > p.wall + 0.5 {
            let dt = wall - p.wall
            processCpu = (process - p.process) / dt
            mainThreadCpu = (main - p.main) / dt
            // A meter reset in between makes the render total go backwards.
            audioRenderCpu = render >= p.render ? (render - p.render) / dt : -1.0
            cpuStore.update(processCpuText())
        }
        cpuPrev = (process, main, render, wall)
    }

    // MARK: - Actions

    func noteOn(_ note: Int, velocity: Double) {
        // velocity 0...1 -> 16-bit, with a floor so a touch is never silent.
        let v = max(0.05, min(1.0, velocity))
        host?.noteOn(note, velocity16: Int(v * 65535.0))
    }

    func noteOff(_ note: Int) { host?.noteOff(note) }

    func setSustain(_ down: Bool) {
        sustainDown = down
        host?.sustain(down)
    }

    func panic() {
        sustainDown = false
        host?.panic()
    }

    func resetLoad() { host?.resetLoad() }

    func setCpuTiming(_ on: Bool) {
        cpuTiming = on
        host?.setCpuTiming(on)
        host?.resetLoad()  // keep the worst buffer comparable with its CPU time
    }

    func setPhaseTiming(_ on: Bool) {
        phaseTiming = on
        host?.setPhaseTiming(on)
        host?.resetLoad()  // so that the worst buffer and the top 1 % are all timed
    }

    func setSpeedProbe(_ on: Bool) {
        speedProbe = on
        host?.setSpeedProbe(everyBuffers: on ? AppModel.speedProbeInterval : 0)
        host?.resetLoad()
    }

    func setUICostMode(_ mode: UICostMode) {
        guard mode != uiCost.mode else { return }
        uiCost.mode = mode
        cpuPrev = nil  // the next CPU reading covers a full second in the new mode
    }

    /// The speed-probe line of the diagnostics: the audio thread's probe
    /// times against the same workload on the benchmark thread (the last
    /// benchmark run) or, without one, on the main thread now.
    func speedProbeText() -> String {
        guard let host = host else { return "" }
        let p = host.speedProbe()
        if p.interval == 0 && p.runs == 0 { return "audio-thread speed probe: off" }
        var out = "audio-thread speed probe (every \(p.interval) buffers, \(p.runs) runs): "
        guard p.runs > 0 else { return out + "no runs yet" }
        out += String(format: "min %.1f / median %.1f / mean %.1f / max %.1f us", p.min_us, p.median_us, p.mean_us, p.max_us)
        let ref: pm_speed_probe_stats
        let refName: String
        if let b = benchProbe {
            ref = b
            refName = "benchmark thread"
        } else {
            ref = EngineHost.runSpeedProbe(runs: 101)
            refName = "main thread now (no benchmark run yet)"
        }
        out += String(format: "; %@ %.1f us (median, min %.1f)", refName, ref.median_us, ref.min_us)
        if p.mean_us > 0 && p.median_us > 0 && p.min_us > 0 {
            out += String(format: " -> audio thread ran at %.2fx (mean), %.2fx (median), %.2fx (fastest run)",
                          ref.median_us / p.mean_us, ref.median_us / p.median_us, ref.median_us / p.min_us)
        }
        return out
    }

    // MARK: - Diagnostics text (Copy diagnostics)

    /// Device, build and audio facts shared by the diagnostics and the benchmark export.
    func environmentText() -> String {
        var out = "build: \(EngineHost.buildInfo); API \(String(cString: pm_version_string()))\n"
        let info = ProcessInfo.processInfo
        var machine = "?"
        var u = utsname()
        if uname(&u) == 0 {
            machine = withUnsafeBytes(of: &u.machine) { raw -> String in
                guard let base = raw.baseAddress else { return "?" }
                return String(cString: base.assumingMemoryBound(to: CChar.self))
            }
        }
        out += "device: \(machine), \(info.operatingSystemVersionString), \(info.activeProcessorCount) cores"
        if info.isiOSAppOnMac { out += ", iPad app on Mac" }
        if info.isMacCatalystApp { out += ", Mac Catalyst" }
        out += ", thermal state \(AppModel.thermalName(info.thermalState))"
        if info.isLowPowerModeEnabled { out += ", low power mode" }
        out += "\n"
        return out
    }

    /// The I/O configuration actually granted against the one requested
    /// (an iPad app on a Mac, for one, gets the Mac's device settings,
    /// typically 512 frames at 44.1 kHz, whatever it asks for). Empty when
    /// they agree or nothing is known yet.
    func ioMismatchText() -> String {
        let a = audio
        guard a.sampleRate > 0 else { return "" }
        var parts: [String] = []
        if abs(a.sampleRate - AudioController.preferredSampleRate) > 0.5 {
            parts.append(String(format: "%.0f Hz", a.sampleRate) + " (requested " + String(format: "%.0f", AudioController.preferredSampleRate) + ")")
        }
        if a.ioBufferFrames > 0 && a.ioBufferFrames != AudioController.preferredFrames {
            parts.append("\(a.ioBufferFrames) frames (requested \(AudioController.preferredFrames))")
        }
        guard !parts.isEmpty else { return "" }
        var out = "granted " + parts.joined(separator: ", ")
        if ProcessInfo.processInfo.isiOSAppOnMac { out += "; an iPad app on a Mac runs at the Mac's output settings" }
        return out
    }

    /// Built only when Copy is pressed, from fresh reads.
    func diagnosticsText() -> String {
        var out = "Piano diagnostics, \(Date())\n"
        out += environmentText()
        guard let host = host else { return out + "no engine\n" }
        let d = host.diagnostics()
        let pd = host.phaseDiagnostics()
        let stats = host.stats()
        let counters = host.counters()
        out += "audio: \(audio.message), \(String(format: "%.0f", audio.sampleRate)) Hz, I/O buffer \(audio.ioBufferFrames) frames"
        out += String(format: ", output latency %.2f ms, route %@\n", audio.outputLatencyMs, audio.route)
        if !audio.ioRequest.isEmpty { out += "I/O request: " + audio.ioRequest + "\n" }
        let mismatch = ioMismatchText()
        if !mismatch.isEmpty { out += "I/O: " + mismatch + "\n" }
        out += "engine: tier \(tier.label), soundboard \(boardOn ? "on" : "off"), \(stats.awake_modes) modes awake, \(stats.active_hammers) hammers active\n"
        out += "MIDI: \(counters.midi_messages) messages, \(counters.midi_events) engine events, \(counters.dropped_events + stats.dropped_events) dropped\n"
        out += "render calls: last \(d.last_frames) frames = \(AppModel.us(d.last_period_us)); "
        out += "frames seen \(d.min_frames)...\(d.max_frames), \(d.distinct_frame_counts) distinct size(s)\n"
        out += "since reset: \(d.buffers) buffers, over \(Int((d.budget * 100).rounded())) %: \(d.over_budget)\n"
        out += "render time: mean \(AppModel.us(d.mean_us)) (\(AppModel.pct(d.mean_load))), "
        out += "p99 \(AppModel.us(d.p99_us)) (\(AppModel.pct(d.p99_load))), "
        out += "max \(AppModel.us(d.max_us)) (\(AppModel.pct(d.max_load)))\n"
        out += AppModel.worstBufferText(d) + "\n"
        if d.worst_index > 0 {
            let w = host.worstDetail()
            out += "worst buffer events: \(pd.worst_note_ons) note-on, \(pd.worst_note_offs) note-off, \(pd.worst_pedal_events) pedal; "
            out += "\(pd.worst_damper_updates) damper updates, \(pd.worst_response_groups) impulse responses recomputed, "
            out += "zone kernel \(w.kernel_rebuilds) full rebuilds + \(w.kernel_key_updates) key updates in place, "
            out += "\(pd.worst_idle_sub_blocks) idle sub-blocks\n"
            out += "worst buffer hammers: \(w.hammers_in_contact) touched strings, at most \(w.max_hammers_in_contact) at once, "
            out += "\(w.hammer_contact_samples) hammer-samples in contact, \(d.worst_hammers_active) still active at its end; "
            out += "\(w.newton_evals) Newton evaluations, \(w.fallback_solves) fallback solves\n"
        }
        if pd.phase_timing == 1 || pd.timed_buffers > 0 {
            out += "phase timing: \(pd.phase_timing == 1 ? "on" : "off"), \(pd.timed_buffers) buffers timed; us per buffer, "
            out += "events / hammers / free / forced / board / other:\n"
            if pd.worst_timed == 1 { out += "  worst buffer:  " + AppModel.phaseText(pd.worst_phase_us) + "\n" }
            out += "  top 1 % (\(pd.top_count)): " + AppModel.phaseText(pd.top_phase_us) + "\n"
            out += "  mean:          " + AppModel.phaseText(pd.mean_phase_us) + "\n"
        } else {
            out += "phase timing: off\n"
        }
        if d.cpu_timing == 1 {
            out += "thread CPU time: on; largest wall minus CPU time of a buffer \(AppModel.us(d.max_stall_us))\n"
        } else {
            out += "thread CPU time: off\n"
        }
        out += speedProbeText() + "\n"
        out += processCpuText() + "\n"
        out += "UI cost test: \(uiCost.mode.label)\n"
        out += "(loads are render time / each buffer's own period; p99 is the upper edge of a 0.25 % bin)\n"
        return out
    }

    typealias Phases6 = (Double, Double, Double, Double, Double, Double)

    static func phaseText(_ p: Phases6) -> String {
        let v = [p.0, p.1, p.2, p.3, p.4, p.5]
        return v.map { String(format: "%.1f", $0) }.joined(separator: " / ")
    }

    static func worstBufferText(_ d: pm_host_diagnostics) -> String {
        guard d.worst_index > 0 else { return "worst buffer: -" }
        var out = "worst buffer #\(d.worst_index): \(d.worst_frames) frames, \(AppModel.us(d.worst_render_us)) of "
        out += "\(AppModel.us(d.worst_period_us)) (\(AppModel.pct(d.worst_load)))"
        if d.worst_cpu_us >= 0 { out += ", CPU \(AppModel.us(d.worst_cpu_us))" }
        out += "; \(d.worst_hammers_in_contact) hammers touched strings (\(d.worst_hammers_active) active at its end), "
        out += "\(d.worst_awake_modes) modes awake, \(d.worst_events) events, \(d.worst_sub_blocks) sub-blocks"
        return out
    }

    /// Whole-process CPU over the last second, split into the audio render
    /// (from the load meter), the main thread (UI) and the rest.
    func processCpuText() -> String {
        guard processCpu >= 0 else { return "process CPU: measuring..." }
        var out = "process CPU (last second, % of one core): total \(AppModel.pct(processCpu))"
        if audioRenderCpu >= 0 { out += ", audio render \(AppModel.pct(audioRenderCpu))" }
        if mainThreadCpu >= 0 { out += ", main thread \(AppModel.pct(mainThreadCpu))" }
        if audioRenderCpu >= 0 && mainThreadCpu >= 0 {
            out += ", other threads \(AppModel.pct(max(0, processCpu - audioRenderCpu - mainThreadCpu)))"
        }
        return out
    }

    static func us(_ v: Double) -> String { String(format: "%.0f us", v) }
    static func pct(_ v: Double) -> String { String(format: "%.1f %%", v * 100.0) }

    static func thermalName(_ s: ProcessInfo.ThermalState) -> String {
        switch s {
        case .nominal: return "nominal"
        case .fair: return "fair"
        case .serious: return "serious"
        case .critical: return "critical"
        @unknown default: return "unknown"
        }
    }

    // MARK: - Offline benchmark

    /// Runs the bench_engine scenarios on a background thread (QoS
    /// userInteractive) with audio output paused, so that the numbers are the
    /// engine's alone. Results arrive line by line.
    func runBenchmark() {
        guard !benchRunning, host != nil else { return }
        benchRunning = true
        benchLines = []
        benchProgress = "Pausing audio"
        var config = pm_bench_config()
        let rate = benchLiveIO ? 44100.0 : 48000.0
        let frames = benchLiveIO ? 512 : 64
        config.sample_rate = rate
        config.seconds = 3
        config.block_frames = Int32(frames)
        config.tier = Int32(tier.rawValue)
        config.hammer_strikes = 200
        config.flags = benchAllTiers ? Int32(PM_BENCH_ALL_TIERS.rawValue) : 0
        benchHeader = "Piano benchmark, \(Date())\n" + environmentText()
        benchHeader += String(format: "fs %.0f Hz, %d-frame buffers (%.0f us), ", rate, frames, Double(frames) / rate * 1e6)
        benchHeader += "board-on cases at tier \(tier.label), 3 s per case; audio paused during the run\n"
        if let ac = audioController {
            ac.suspendOutput { [weak self] in
                self?.startBenchmark(config)
            }
        } else {
            startBenchmark(config)
        }
    }

    private func startBenchmark(_ config: pm_bench_config) {
        var cfg = config
        let count = Int(pm_bench_item_count(&cfg))
        benchProgress = "0/\(count)"
        benchQueue.async { [weak self] in
            // The speed probe's reference: the same workload as on the audio
            // thread, here on the benchmark thread.
            let probe = EngineHost.runSpeedProbe(runs: 501)
            let probeLine = String(format: "speed probe on the benchmark thread: min %.2f / median %.2f us", probe.min_us, probe.median_us)
            DispatchQueue.main.async {
                self?.benchProbe = probe
                self?.benchLines.append(probeLine)
            }
            var c = config
            for i in 0..<count {
                let name = String(cString: pm_bench_item_name(&c, Int32(i)))
                DispatchQueue.main.async {
                    self?.benchProgress = "\(i + 1)/\(count): \(name)"
                }
                var r = pm_bench_result()
                let ok = pm_bench_run_item(&c, Int32(i), &r) == 1
                let line = AppModel.benchLine(name: name, result: r, ok: ok)
                DispatchQueue.main.async {
                    self?.benchLines.append(line)
                }
            }
            DispatchQueue.main.async {
                guard let self = self else { return }
                self.benchRunning = false
                self.benchProgress = "Done (\(count) items); audio resumed"
                self.audioController?.resumeOutput()
            }
        }
    }

    static let benchColumns = "case".padding(toLength: 38, withPad: " ", startingAt: 0)
        + "  mean us   p99 us   max us   mean %   p99 %   max %"

    static func benchLine(name: String, result r: pm_bench_result, ok: Bool) -> String {
        guard ok else { return name + ": failed" }
        if r.kind == Int32(PM_BENCH_HAMMER.rawValue) {
            return String(format: "%@ (%d strings, %d modes, R=%d): %.1f us in contact, %.1f us averaged over its life",
                          name, r.strings, r.modes, r.oversampling, r.contact_us, r.life_us)
        }
        return name.padding(toLength: 38, withPad: " ", startingAt: 0)
            + String(format: " %8.1f %8.1f %8.1f %8.2f %7.2f %7.2f", r.mean_us, r.p99_us, r.max_us,
                     r.mean_load * 100, r.p99_load * 100, r.max_load * 100)
    }

    func benchmarkExport() -> String {
        var out = benchHeader + AppModel.benchColumns + "\n"
        for line in benchLines { out += line + "\n" }
        return out
    }

    /// Moves the on-screen keyboard (which starts on a C) by octaves, keeping
    /// `shown` octaves plus the top C inside the piano (C1 ... C8).
    func shiftKeyboard(octaves: Int, shown: Int) {
        let highest = 108 - 12 * shown
        let next = max(24, min(highest, keyboard.lowestNote + 12 * octaves))
        if next != keyboard.lowestNote { keyboard.lowestNote = next }
    }

    /// Quality tier and soundboard are prepare-time parameters: set them and
    /// re-prepare (about a second of silence).
    func setTier(_ t: QualityTier) {
        guard let host = host, t != tier else { return }
        tier = t
        host.setPrepareTimeParam(PM_PARAM_QUALITY_TIER, Double(t.rawValue))
        audioController?.reprepare { _ in }
    }

    func setBoard(_ on: Bool) {
        guard let host = host, on != boardOn else { return }
        boardOn = on
        host.setPrepareTimeParam(PM_PARAM_SOUNDBOARD_ENABLED, on ? 1.0 : 0.0)
        audioController?.reprepare { _ in }
    }

    // MARK: - Monitor

    func addMonitorMarker() {
        _ = monitor.addMarker()
        monitorStore.lines = monitor.lines
    }

    func clearMonitor() {
        monitor.clear()
        monitorStore.lines = []
    }

    func monitorExport() -> String {
        var header = "Piano MIDI monitor, \(Date())"
        let names = sources.map { $0.name }.joined(separator: ", ")
        header += "\nsources: \(names.isEmpty ? "none" : names)"
        if let host = host {
            let c = host.counters()
            header += "\ntimebase \(host.timebaseNumer)/\(host.timebaseDenom), messages \(c.midi_messages)"
            header += ", malformed \(c.midi_malformed), monitor dropped \(c.monitor_dropped)"
        }
        return monitor.exportText(header: header)
    }
}
