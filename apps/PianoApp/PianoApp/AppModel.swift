// App state for the SwiftUI views. Main thread only. Owns the engine host,
// the audio and MIDI controllers, and polls meters and the MIDI monitor ring
// on a timer (20 Hz).
import Combine
import Foundation
import PianoCore
#if canImport(UIKit)
import UIKit
#elseif canImport(AppKit)
import AppKit
#endif

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
    @Published var midiMessages: UInt64 = 0
    @Published var droppedEvents: UInt64 = 0
    // CPU meter: fractions of the buffer period
    @Published var meanLoad = 0.0
    @Published var recentLoad = 0.0
    @Published var maxLoad = 0.0
    @Published var overBudget: UInt64 = 0
    @Published var loadBudget = 0.7
    @Published var buffers: UInt64 = 0
    @Published var renderFrames = 0
    @Published var activeHammers = 0
    @Published var awakeModes = 0
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
        terminateObserver = NotificationCenter.default.addObserver(forName: terminate, object: nil, queue: .main) { [weak self] _ in
            self?.stop()
        }
    }

    func stop() {
        timer?.invalidate()
        timer = nil
        audioController?.stop()
        started = false
    }

    private func tick() {
        guard let host = host else { return }
        let load = host.load()
        meanLoad = load.mean_load
        recentLoad = load.recent_load
        maxLoad = load.max_load
        overBudget = load.over_budget
        loadBudget = load.budget
        buffers = load.buffers
        renderFrames = Int(load.last_frames)
        let counters = host.counters()
        midiMessages = counters.midi_messages
        let stats = host.stats()
        droppedEvents = counters.dropped_events + stats.dropped_events
        activeHammers = Int(stats.active_hammers)
        awakeModes = Int(stats.awake_modes)
        let mc = midiController
        monitor.paused = monitorStore.paused
        if monitor.drain(host: host, sourceName: { id in mc?.name(forSource: id) ?? "?" }) {
            monitorStore.lines = monitor.lines
        }
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
            header += "\ntimebase \(host.timebaseNumer)/\(host.timebaseDenom), messages \(midiMessages)"
            let c = host.counters()
            header += ", malformed \(c.midi_malformed), monitor dropped \(c.monitor_dropped)"
        }
        return monitor.exportText(header: header)
    }
}
