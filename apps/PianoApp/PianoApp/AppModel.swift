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
    /// Its two parts (latency-bound, throughput-bound), the reference for the ratios.
    private var benchProbeSplit: pm_speed_probe_split?
    /// Performance levels and the assumed CPU-number-to-core-type mapping.
    let coreLayout = CoreLayout.read()
    // Offline benchmark (Benchmark tab)
    @Published var benchRunning = false
    @Published var benchProgress = ""
    @Published var benchLines: [String] = []
    @Published var benchAllTiers = false
    @Published var benchLiveIO = false  // 512 frames at 44.1 kHz (what an iPad app on a Mac is granted)
    /// Sleep to each buffer's real-time deadline between render calls, as
    /// an audio thread does (PM_BENCH_PACED).
    @Published var benchPaced = false
    /// Paced, and walk a 32 MiB buffer between render calls (other work on the core).
    @Published var benchEvict = false
    /// Phase timing during the scenarios (the live meter's phase split uses it too).
    @Published var benchPhases = true
    /// Paced runs on the Mach time-constraint (real-time) thread policy
    /// (PacedRealtime), without a workgroup deadline (not available from
    /// Swift). Off: the benchmark thread's normal QoS (userInteractive).
    @Published var benchRealtime = true
    /// Scenario run on the audio thread ("Run on audio thread"), an index into sceneNames.
    @Published var sceneItem = 0
    /// Run the scene on a private engine prepared as the benchmark's (not the app's instance).
    @Published var sceneFreshEngine = false
    @Published var sceneRunning = false
    /// Scenes still to run after the current one ("Run all on audio thread").
    private var sceneQueue: [Int] = []
    private var sceneSeries = 0  // scenes in the running series (0: a single scene)
    /// The scenario names (benchmark items without the hammer micro-benchmark).
    let sceneNames: [String] = {
        var c = pm_bench_config()
        c.flags = Int32(PM_BENCH_NO_HAMMER.rawValue)
        let n = Int(pm_bench_item_count(&c))
        return (0..<n).map { String(cString: pm_bench_item_name(&c, Int32($0))) }
    }()
    private var sceneTimer: Timer?
    private var sceneName = ""
    private var benchConfigText = ""   // the benchmark engine's configuration (last scenario run)
    private var sceneConfigText = ""   // the audio-thread run's engine configuration
    private var benchHeader = ""
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
        let split = host.speedProbeSplit()
        if split.latency.runs > 0 {
            let refSplit = benchProbeSplit ?? EngineHost.runSpeedProbeSplit(runs: 101)
            out += "\n  parts on the audio thread: " + probeSplitText(split, reference: refSplit)
            if benchProbeSplit == nil { out += " (reference: main thread now, no benchmark run yet)" }
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
        out += "cores: " + coreLayout.description + "\n"
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
            out += "worst buffer work: " + AppModel.workDetailText(host.worstWork()) + "\n"
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
        out += "cores: " + coreLayout.description + "\n"
        for line in coreLayout.usageLines(host.cpuUsage(), indent: "") { out += line + "\n" }
        out += processCpuText() + "\n"
        out += "UI cost test: \(uiCost.mode.label)\n"
        out += "(loads are render time / each buffer's own period; p99 is the upper edge of a 0.25 % bin)\n"
        out += "engine configuration:\n" + host.describeEngine()
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
    /// The benchmark configuration of the Benchmark tab's switches.
    private func benchConfig() -> pm_bench_config {
        var config = pm_bench_config()
        let rate = benchLiveIO ? 44100.0 : 48000.0
        let frames = benchLiveIO ? 512 : 64
        config.sample_rate = rate
        config.seconds = 3
        config.block_frames = Int32(frames)
        config.tier = Int32(tier.rawValue)
        config.hammer_strikes = 200
        var flags: UInt32 = 0
        if benchAllTiers { flags |= PM_BENCH_ALL_TIERS.rawValue }
        if benchPhases { flags |= PM_BENCH_PHASE_TIMING.rawValue }
        if benchPaced { flags |= PM_BENCH_PACED.rawValue }
        if benchEvict { flags |= PM_BENCH_PACED_EVICT.rawValue }
        if speedProbe { flags |= PM_BENCH_SPEED_PROBE.rawValue }  // the Diagnostics switch
        config.flags = Int32(flags)
        return config
    }

    func runBenchmark() {
        guard !benchRunning, !sceneRunning, host != nil else { return }
        benchRunning = true
        benchLines = []
        benchProgress = "Pausing audio"
        let config = benchConfig()
        let rate = config.sample_rate
        let frames = Int(config.block_frames)
        benchHeader = "Piano benchmark, \(Date())\n" + environmentText()
        benchHeader += String(format: "fs %.0f Hz, %d-frame buffers (%.0f us), ", rate, frames, Double(frames) / rate * 1e6)
        benchHeader += "board-on cases at tier \(tier.label), 3 s per case; audio paused during the run; "
        benchHeader += benchEvict ? "paced to real time with a 32 MiB eviction walk between buffers" : (benchPaced ? "paced to real time" : "buffers back to back")
        benchHeader += benchPhases ? ", phase timing on" : ", phase timing off"
        benchHeader += speedProbe ? ", speed probe before every 8th buffer\n" : "\n"
        benchHeader += "benchmark thread: a dedicated thread at QoS userInteractive"
        if benchPaced || benchEvict {
            benchHeader += benchRealtime
                ? "; paced loop on a time-constraint (real-time) thread policy, but without a workgroup deadline (NOT the audio I/O thread's scheduling: no per-buffer deadlines reach the performance controller)"
                : "; paced loop at that QoS (NOT an audio thread's scheduling class: no real-time policy, no workgroup deadlines)"
        }
        benchHeader += ". Audio-thread rows: the real Core Audio I/O thread (real-time, in the output device's workgroup).\n"
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
        let layout = coreLayout
        let paced = (config.flags & Int32(PM_BENCH_PACED.rawValue | PM_BENCH_PACED_EVICT.rawValue)) != 0
        let realtime = paced && benchRealtime
        // A dedicated thread rather than a dispatch queue: the paced loop may
        // change its scheduling policy (PacedRealtime), which must not leak
        // into a pooled worker thread.
        let thread = Thread { [weak self] in
            let rt = realtime ? PacedRealtime() : nil
            rt?.install()
            // The speed probe's reference: the same workload as on the audio
            // thread, here on the benchmark thread, back to back.
            let probe = EngineHost.runSpeedProbe(runs: 501)
            let split = EngineHost.runSpeedProbeSplit(runs: 501)
            var probeLine = String(format: "speed probe on the benchmark thread: min %.2f / median %.2f us", probe.min_us, probe.median_us)
            probeLine += String(format: " (latency-bound part %.2f, throughput-bound part %.2f us, medians); on %@ now",
                                split.latency.median_us, split.throughput.median_us, layout.label(EngineHost.currentCpu))
            DispatchQueue.main.async {
                self?.benchProbe = probe
                self?.benchProbeSplit = split
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
                var lines = [AppModel.benchLine(name: name, result: r, ok: ok)]
                if ok && r.kind == Int32(PM_BENCH_SCENARIO.rawValue) {
                    lines += AppModel.benchDetailLines(r)
                    lines += layout.usageLines(EngineHost.benchLastCpuUsage())
                    let ps = EngineHost.benchLastProbeSplit()
                    if ps.latency.runs > 0 {
                        lines.append("    speed probe before every 8th buffer (\(ps.latency.runs) runs): " + probeSplitText(ps, reference: split))
                    }
                }
                let conf = ok && r.kind == Int32(PM_BENCH_SCENARIO.rawValue) ? AppModel.benchLastConfig() : nil
                DispatchQueue.main.async {
                    self?.benchLines.append(contentsOf: lines)
                    if let conf = conf { self?.benchConfigText = conf }
                }
            }
            rt?.uninstall()
            let notes = rt?.notes ?? []
            DispatchQueue.main.async {
                guard let self = self else { return }
                if !notes.isEmpty { self.benchLines.append("paced thread: " + notes.joined(separator: "; ")) }
                self.benchRunning = false
                self.benchProgress = "Done (\(count) items); audio resumed"
                self.audioController?.resumeOutput()
            }
        }
        thread.name = "piano.benchmark"
        thread.qualityOfService = .userInteractive
        thread.start()
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

    static func benchLastConfig() -> String {
        var buf = [CChar](repeating: 0, count: 4096)
        _ = pm_bench_last_config(&buf, Int32(buf.count))
        return String(cString: buf)
    }

    /// Phase split and work counts of a benchmark row (the same lines as an
    /// audio-thread run's, so the two can be compared line by line).
    static func benchDetailLines(_ r: pm_bench_result) -> [String] {
        var out: [String] = []
        if r.has_phases == 1 {
            out.append("    phases us (events / hammers / free / forced / board / other): mean " + phaseText(r.mean_phase_us)
                       + "; top 1 % (\(r.top_count)) " + phaseText(r.top_phase_us) + "; worst " + phaseText(r.worst_phase_us))
        }
        out.append(workText(meanAwake: r.mean_awake_modes, free: r.free_group_blocks, forced: r.forced_group_blocks,
                            members: r.loop_members, listeners: r.loop_listeners, rebuilds: r.kernel_rebuilds,
                            keyUpdates: r.kernel_key_updates, usPerSample: r.hammer_us_per_sample,
                            usPerContact: r.hammer_us_per_contact_sample, overruns: r.overruns, worst: r.worst))
        return out
    }

    static func workText(meanAwake: Double, free: Double, forced: Double, members: Double, listeners: Double,
                         rebuilds: Double, keyUpdates: Double, usPerSample: Double, usPerContact: Double,
                         overruns: Int32, worst w: pm_work_detail) -> String {
        var out = String(format: "    per buffer: awake modes %.0f, group-blocks free %.0f / forced %.0f, loop members %.1f / listeners %.1f, ",
                         meanAwake, free, forced, members, listeners)
        out += String(format: "kernel %.2f rebuilds / %.2f key updates; hammers phase %.3f us per hammer-sample (%.3f per contact sample); %d overruns\n",
                      rebuilds, keyUpdates, usPerSample, usPerContact, overruns)
        out += "    worst buffer: " + workDetailText(w)
        return out
    }

    static func workDetailText(_ w: pm_work_detail) -> String {
        var out = "\(w.note_ons) on / \(w.note_offs) off / \(w.pedal_events) pedal, \(w.awake_modes) modes awake; "
        out += "hammers \(w.hammers_in_contact) touched (\(w.max_hammers_in_contact) at once), \(w.hammer_samples) hammer-samples stepped "
        out += "(\(w.hammer_damped_samples) damped), \(w.hammer_contact_samples) in contact, \(w.newton_evals) Newton / \(w.fallback_solves) fallbacks; "
        out += "\(w.damper_updates) damper updates, \(w.response_groups) responses, kernel \(w.kernel_rebuilds) + \(w.kernel_key_updates); "
        out += "group-blocks free \(w.free_group_blocks) / forced \(w.forced_group_blocks); loop \(w.loop_members) members, "
        out += "\(w.loop_listeners) listeners, \(w.loop_entries) entries, \(w.out_zones) zones"
        return out
    }

    func benchmarkExport() -> String {
        var out = benchHeader + AppModel.benchColumns + "\n"
        for line in benchLines { out += line + "\n" }
        if !benchConfigText.isEmpty {
            out += "benchmark engine configuration (last scenario run):\n" + benchConfigText
        }
        if !sceneConfigText.isEmpty {
            out += "audio-thread run engine configuration (\(sceneName)):\n" + sceneConfigText
        }
        if let host = host {
            out += "app engine configuration now:\n" + host.describeEngine()
        }
        return out
    }

    // MARK: - Benchmark scene on the audio thread

    /// Plays the selected scenario's events through the real host and audio
    /// render path in real time (audible), on the app's engine (reset to its
    /// just-prepared state first) or, with sceneFreshEngine, on a private
    /// engine prepared as the benchmark's; then appends a row with the same
    /// statistics, phase split and counts as a benchmark row, and the speed
    /// probe over the run. Uses the Benchmark tab's seconds, tier and phase
    /// timing; the frame count and sample rate are the device's.
    func runOnAudioThread() {
        guard !benchRunning, !sceneRunning else { return }
        guard sceneItem >= 0 && sceneItem < sceneNames.count else { return }
        sceneQueue = []
        sceneSeries = 0
        appendSceneHeader(count: 1)
        startScene(item: sceneItem)
    }

    /// Every scene in the list, one after the other, on the audio thread
    /// (one row each, the same format), so that a full paced-on-the-audio-
    /// thread table needs one tap. Cancel stops the series.
    func runAllOnAudioThread() {
        guard !benchRunning, !sceneRunning, !sceneNames.isEmpty else { return }
        sceneQueue = Array(1..<sceneNames.count)
        sceneSeries = sceneNames.count
        appendSceneHeader(count: sceneNames.count)
        startScene(item: 0)
    }

    private func appendSceneHeader(count: Int) {
        var line = "audio-thread runs (\(count) scene\(count == 1 ? "" : "s")): the real Core Audio I/O thread, "
        line += String(format: "%.0f Hz, I/O buffer %d frames; ", audio.sampleRate, audio.ioBufferFrames)
        line += sceneFreshEngine ? "benchmark-prepared engine" : "app engine (reset first)"
        line += speedProbe ? "; speed probe every \(AppModel.speedProbeInterval) buffers" : "; speed probe off (Diagnostics)"
        line += benchPhases ? "; phase timing on (CPU numbers recorded)" : "; phase timing off"
        benchLines.append(line)
    }

    private func startScene(item: Int) {
        guard let host = host else { return }
        var config = benchConfig()
        config.flags &= ~Int32(PM_BENCH_ALL_TIERS.rawValue)  // sceneNames are the default item list
        if sceneFreshEngine { config.flags |= Int32(PM_BENCH_SCENE_FRESH_ENGINE.rawValue) }
        sceneName = sceneNames[item]
        sceneRunning = true
        let position = sceneSeries > 0 ? " (\(item + 1)/\(sceneSeries))" : ""
        benchProgress = "Starting \(sceneName) on the audio thread" + position
        if benchHeader.isEmpty { benchHeader = "Piano benchmark, \(Date())\n" + environmentText() }
        host.startScene(config: config, item: item) { [weak self] ok in
            guard let self = self else { return }
            guard ok else {
                self.sceneRunning = false
                self.sceneQueue = []
                self.benchProgress = "Could not start the scene (audio not running or engine not prepared)"
                return
            }
            self.sceneConfigText = host.sceneConfig()
            self.benchProgress = "Playing \(self.sceneName) on the audio thread (\(self.sceneFreshEngine ? "benchmark-prepared engine" : "app engine")); leave the keyboard alone"
            let t = Timer(timeInterval: 0.25, repeats: true) { [weak self] _ in self?.pollScene() }
            RunLoop.main.add(t, forMode: .common)
            self.sceneTimer = t
        }
    }

    func cancelScene() {
        sceneQueue = []
        host?.cancelScene()
    }

    private func pollScene() {
        guard let host = host else { return }
        let r = host.sceneResult()
        if r.state == Int32(PM_SCENE_RUNNING.rawValue) { return }
        sceneTimer?.invalidate()
        sceneTimer = nil
        sceneRunning = false
        guard r.state == Int32(PM_SCENE_DONE.rawValue) else {
            sceneQueue = []
            benchProgress = "Scene cancelled"
            return
        }
        benchProgress = "Done: \(sceneName) on the audio thread"
        benchLines.append(contentsOf: AppModel.sceneLines(name: sceneName, result: r, benchProbe: benchProbe))
        benchLines.append(contentsOf: coreLayout.usageLines(host.sceneCpuUsage()))
        let ps = host.sceneProbeSplit()
        if ps.latency.runs > 0 {
            benchLines.append("    speed probe parts over the run: " + probeSplitText(ps, reference: benchProbeSplit))
        }
        // The engine keeps playing from the scene's end state: silence it.
        host.allNotesOff()
        host.pushNow(PM_EVENT_SUSTAIN, value: 0)
        if !sceneQueue.isEmpty {
            let next = sceneQueue.removeFirst()
            // A moment for the all-notes-off to land; the next scene resets the engine anyway.
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) { [weak self] in
                guard let self = self, !self.sceneRunning, !self.benchRunning else { return }
                self.startScene(item: next)
            }
        }
    }

    static func sceneLines(name: String, result r: pm_host_scene_result, benchProbe: pm_speed_probe_stats?) -> [String] {
        let label = (r.fresh_engine == 1 ? "audio thread, bench engine: " : "audio thread, app engine: ") + name
        var head = String(label.prefix(38)).padding(toLength: 38, withPad: " ", startingAt: 0)
        head += String(format: " %8.1f %8.1f %8.1f %8.2f %7.2f %7.2f", r.mean_us, r.p99_us, r.max_us,
                       r.mean_load * 100, r.p99_load * 100, r.max_load * 100)
        head += String(format: "   (%d buffers of %d-%d frames at %.0f Hz; p99 to the meter's 1.5 %% bins)",
                       r.buffers, r.min_frames, r.max_frames, r.sample_rate)
        var out = [head]
        if r.has_phases == 1 {
            out.append("    phases us (events / hammers / free / forced / board / other): mean " + phaseText(r.mean_phase_us)
                       + "; top 1 % (\(r.top_count)) " + phaseText(r.top_phase_us)
                       + (r.worst_timed == 1 ? "; worst " + phaseText(r.worst_phase_us) : ""))
        }
        out.append(workText(meanAwake: r.mean_awake_modes, free: r.free_group_blocks, forced: r.forced_group_blocks,
                            members: r.loop_members, listeners: r.loop_listeners, rebuilds: r.kernel_rebuilds,
                            keyUpdates: r.kernel_key_updates, usPerSample: r.hammer_us_per_sample,
                            usPerContact: r.hammer_us_per_contact_sample, overruns: r.overruns, worst: r.worst))
        let p = r.probe
        if p.runs > 0 {
            var line = String(format: "    speed probe over the run (%llu runs): min %.1f / median %.1f / mean %.1f / max %.1f us",
                              p.runs, p.min_us, p.median_us, p.mean_us, p.max_us)
            if let b = benchProbe, p.mean_us > 0, p.median_us > 0 {
                line += String(format: "; benchmark thread %.1f us -> %.2fx (mean), %.2fx (median)", b.median_us,
                               b.median_us / p.mean_us, b.median_us / p.median_us)
            }
            out.append(line)
        } else {
            out.append("    speed probe: off (switch it on in Diagnostics to compare clocks)")
        }
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
