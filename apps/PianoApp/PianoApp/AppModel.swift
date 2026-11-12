// App state for the SwiftUI views. Main thread only. Owns the engine host,
// the audio and MIDI controllers, and polls meters and the MIDI monitor ring
// on a timer (20 Hz).
//
// UI cost. The meters live in their own ObservableObject (MeterStore), so a
// meter update re-renders only the views that show meters, never the whole
// Play screen or the keyboard; they are published at 10 Hz and only when a
// value changed. The MIDI monitor ring is filled (on the MIDI thread) and
// drained and formatted (here) only while the monitor screen is visible.
import Combine
import Foundation
import PianoCore
#if canImport(UIKit)
import UIKit
#elseif canImport(AppKit)
import AppKit
#endif

/// Meter values, separate from AppModel so that only the views showing them
/// re-render when they change (10 Hz while audio runs).
final class MeterStore: ObservableObject {
    @Published var meanLoad = 0.0
    @Published var recentLoad = 0.0
    @Published var maxLoad = 0.0
    @Published var overBudget: UInt64 = 0
    @Published var loadBudget = 0.7
    @Published var buffers: UInt64 = 0
    @Published var renderFrames = 0
    @Published var activeHammers = 0
    @Published var awakeModes = 0
    @Published var midiMessages: UInt64 = 0
    @Published var droppedEvents: UInt64 = 0
    // Load diagnostics (percentiles, frame counts, the worst buffer)
    @Published var diag = pm_host_diagnostics()
    // Process CPU over the last second, fractions of one core (< 0: unknown).
    @Published var processCpu = -1.0
    @Published var mainThreadCpu = -1.0
    @Published var audioRenderCpu = -1.0
}

/// The MIDI monitor's published lines, separate from AppModel so that the
/// monitor screen re-renders only when lines arrive.
final class MonitorStore: ObservableObject {
    @Published var lines: [MonitorLine] = []
    @Published var paused = false
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
    @Published var cpuTiming = false
    // Offline benchmark (Benchmark tab)
    @Published var benchRunning = false
    @Published var benchProgress = ""
    @Published var benchLines: [String] = []
    @Published var benchAllTiers = false
    private var benchHeader = ""
    private let benchQueue = DispatchQueue(label: "piano.benchmark", qos: .userInteractive)
    // Controls
    @Published var sustainDown = false
    @Published var tier: QualityTier = .full
    @Published var boardOn = true
    @Published var lowestNote = 48  // on-screen keyboard: first C shown
    // MIDI monitor
    let monitorStore = MonitorStore()

    let host: EngineHost?
    private var audioController: AudioController?
    private var midiController: MIDIController?
    private let monitor = MIDIMonitor()
    private var timer: Timer?
    private var ticks = 0
    private var monitorVisible = false
    // Previous 1 Hz CPU sample: process CPU, main-thread CPU and audio render
    // time (seconds), and the wall time (seconds).
    private var cpuPrev: (process: Double, main: Double, render: Double, wall: Double)?
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
        let t = Timer(timeInterval: 0.05, repeats: true) { [weak self] _ in
            self?.tick()
        }
        RunLoop.main.add(t, forMode: .common)
        timer = t
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
        timer?.invalidate()
        timer = nil
        audioController?.shutdownForTermination()
    }

    func stop() {
        timer?.invalidate()
        timer = nil
        audioController?.stop()
        started = false
    }

    private func tick() {
        guard let host = host else { return }
        ticks += 1
        if monitorVisible {
            let mc = midiController
            monitor.paused = monitorStore.paused
            if monitor.drain(host: host, sourceName: { id in mc?.name(forSource: id) ?? "?" }) {
                monitorStore.lines = monitor.lines
            }
        }
        // Meters at 10 Hz, process CPU at 1 Hz.
        if ticks % 2 == 0 {
            updateMeters(host)
        }
        if ticks % 20 == 0 {
            updateProcessCpu(host)
        }
    }

    /// Assigns only changed values, so unchanged meters cause no re-render.
    private func updateMeters(_ host: EngineHost) {
        let m = meters
        let load = host.load()
        if m.meanLoad != load.mean_load { m.meanLoad = load.mean_load }
        if m.recentLoad != load.recent_load { m.recentLoad = load.recent_load }
        if m.maxLoad != load.max_load { m.maxLoad = load.max_load }
        if m.overBudget != load.over_budget { m.overBudget = load.over_budget }
        if m.loadBudget != load.budget { m.loadBudget = load.budget }
        if m.buffers != load.buffers { m.buffers = load.buffers }
        if m.renderFrames != Int(load.last_frames) { m.renderFrames = Int(load.last_frames) }
        let counters = host.counters()
        let stats = host.stats()
        let dropped = counters.dropped_events + stats.dropped_events
        if m.midiMessages != counters.midi_messages { m.midiMessages = counters.midi_messages }
        if m.droppedEvents != dropped { m.droppedEvents = dropped }
        if m.activeHammers != Int(stats.active_hammers) { m.activeHammers = Int(stats.active_hammers) }
        if m.awakeModes != Int(stats.awake_modes) { m.awakeModes = Int(stats.awake_modes) }
        let d = host.diagnostics()
        if d.buffers != m.diag.buffers || d.worst_index != m.diag.worst_index || d.cpu_timing != m.diag.cpu_timing {
            m.diag = d
        }
    }

    /// Process CPU (getrusage: all threads), main-thread CPU (this thread's
    /// CPU clock) and audio render time (from the load meter) over the last
    /// second, as fractions of one core. Called once a second on the main
    /// thread; three cheap reads, nothing on the audio thread.
    private func updateProcessCpu(_ host: EngineHost) {
        var ru = rusage()
        guard getrusage(RUSAGE_SELF, &ru) == 0 else { return }
        let process = Double(ru.ru_utime.tv_sec) + Double(ru.ru_utime.tv_usec) * 1e-6
            + Double(ru.ru_stime.tv_sec) + Double(ru.ru_stime.tv_usec) * 1e-6
        let main = Double(clock_gettime_nsec_np(CLOCK_THREAD_CPUTIME_ID)) * 1e-9
        let d = host.diagnostics()
        let render = d.mean_us * Double(d.buffers) * 1e-6  // since the last meter reset
        let wall = ProcessInfo.processInfo.systemUptime
        if let p = cpuPrev, wall > p.wall {
            let dt = wall - p.wall
            meters.processCpu = (process - p.process) / dt
            meters.mainThreadCpu = (main - p.main) / dt
            // A meter reset in between makes the render total go backwards.
            meters.audioRenderCpu = render >= p.render ? (render - p.render) / dt : -1.0
        }
        cpuPrev = (process, main, render, wall)
    }

    /// The monitor screen calls this from onAppear / onDisappear: the MIDI
    /// thread fills the monitor ring, and this model drains and formats it,
    /// only while the screen is visible.
    func setMonitorVisible(_ visible: Bool) {
        monitorVisible = visible
        host?.setMonitorEnabled(visible)
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

    func diagnosticsText() -> String {
        let d = meters.diag
        var out = "Piano diagnostics, \(Date())\n"
        out += environmentText()
        out += "audio: \(audio.message), \(String(format: "%.0f", audio.sampleRate)) Hz, I/O buffer \(audio.ioBufferFrames) frames"
        out += String(format: ", output latency %.2f ms, route %@\n", audio.outputLatencyMs, audio.route)
        out += "engine: tier \(tier.label), soundboard \(boardOn ? "on" : "off"), \(meters.awakeModes) modes awake, \(meters.activeHammers) hammers active\n"
        out += "render calls: last \(d.last_frames) frames = \(AppModel.us(d.last_period_us)); "
        out += "frames seen \(d.min_frames)...\(d.max_frames), \(d.distinct_frame_counts) distinct size(s)\n"
        out += "since reset: \(d.buffers) buffers, over \(Int((d.budget * 100).rounded())) %: \(d.over_budget)\n"
        out += "render time: mean \(AppModel.us(d.mean_us)) (\(AppModel.pct(d.mean_load))), "
        out += "p99 \(AppModel.us(d.p99_us)) (\(AppModel.pct(d.p99_load))), "
        out += "max \(AppModel.us(d.max_us)) (\(AppModel.pct(d.max_load)))\n"
        out += worstBufferText() + "\n"
        if d.cpu_timing == 1 {
            out += "thread CPU time: on; largest wall minus CPU time of a buffer \(AppModel.us(d.max_stall_us))\n"
        } else {
            out += "thread CPU time: off\n"
        }
        out += processCpuText() + "\n"
        out += "(loads are render time / each buffer's own period; p99 is the upper edge of a 0.25 % bin)\n"
        return out
    }

    func worstBufferText() -> String {
        let d = meters.diag
        guard d.worst_index > 0 else { return "worst buffer: -" }
        var out = "worst buffer #\(d.worst_index): \(d.worst_frames) frames, \(AppModel.us(d.worst_render_us)) of "
        out += "\(AppModel.us(d.worst_period_us)) (\(AppModel.pct(d.worst_load)))"
        if d.worst_cpu_us >= 0 { out += ", CPU \(AppModel.us(d.worst_cpu_us))" }
        out += "; \(d.worst_hammers_in_contact) hammers in contact (\(d.worst_hammers_active) active), "
        out += "\(d.worst_awake_modes) modes awake, \(d.worst_events) events, \(d.worst_sub_blocks) sub-blocks"
        return out
    }

    /// Whole-process CPU over the last second, split into the audio render
    /// (from the load meter), the main thread (UI) and the rest.
    func processCpuText() -> String {
        let m = meters
        guard m.processCpu >= 0 else { return "process CPU: measuring..." }
        var out = "process CPU (last second, % of one core): total \(AppModel.pct(m.processCpu))"
        if m.audioRenderCpu >= 0 { out += ", audio render \(AppModel.pct(m.audioRenderCpu))" }
        if m.mainThreadCpu >= 0 { out += ", main thread \(AppModel.pct(m.mainThreadCpu))" }
        if m.audioRenderCpu >= 0 && m.mainThreadCpu >= 0 {
            out += ", other threads \(AppModel.pct(max(0, m.processCpu - m.audioRenderCpu - m.mainThreadCpu)))"
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
        config.sample_rate = 48000
        config.seconds = 3
        config.block_frames = 64
        config.tier = Int32(tier.rawValue)
        config.hammer_strikes = 200
        config.flags = benchAllTiers ? Int32(PM_BENCH_ALL_TIERS.rawValue) : 0
        benchHeader = "Piano benchmark, \(Date())\n" + environmentText()
        benchHeader += "fs 48000 Hz, 64-frame buffers (1333 us), board-on cases at tier \(tier.label), 3 s per case; audio paused during the run\n"
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
        lowestNote = max(24, min(highest, lowestNote + 12 * octaves))
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
            header += "\ntimebase \(host.timebaseNumer)/\(host.timebaseDenom), messages \(meters.midiMessages)"
            let c = host.counters()
            header += ", malformed \(c.midi_malformed), monitor dropped \(c.monitor_dropped)"
        }
        return monitor.exportText(header: header)
    }
}
