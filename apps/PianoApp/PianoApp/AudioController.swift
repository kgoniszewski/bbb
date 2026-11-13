// Audio output: AVAudioEngine with one AVAudioSourceNode whose render block
// calls the C++ engine. On iOS also the AVAudioSession (category, preferred
// 48 kHz / 64 frames, interruptions, route changes, media-services reset).
//
// The render block captures only the pm_host pointer (an OpaquePointer, a
// plain value): no Swift object, no allocation, no lock, no I/O.
//
// Everything else (session category, preferences and activation, engine
// start/stop, graph rebuilds, notification handling) runs on one serial
// queue, `audioQueue`: AVAudioSession calls can block for a long time and
// must stay off the main thread, and serialising them means a notification
// can never interleave with the launch sequence or another reconfigure.
// Status snapshots are handed to `onStatus` on the main queue.
import AVFoundation
import Foundation
import PianoCore
#if os(macOS)
import AudioToolbox
import CoreAudio
#endif

final class AudioController {
    struct Status {
        var sampleRate: Double = 0
        var ioBufferFrames: Int = 0      // granted I/O buffer (iOS session / macOS device)
        var outputLatencyMs: Double = 0  // reported output latency (iOS)
        var route: String = "-"
        var running = false
        var preparing = false
        var message = "Starting"
        /// macOS: what was asked of the output device and its answer
        /// (supported range, errors); empty where the system decides (iOS).
        var ioRequest = ""
    }

    static let preferredSampleRate: Double = 48000
    static let preferredFrames: Int = 64

    private let host: EngineHost
    private var engine = AVAudioEngine()
    private var sourceNode: AVAudioSourceNode?
    private var nodeSampleRate: Double = 0
    private var observers: [NSObjectProtocol] = []
    private var generation = 0  // invalidates completions of superseded prepares
    private var terminated = false  // set by shutdownForTermination(); drops all later queue work
    private var suspended = false   // output stopped for the offline benchmark (suspendOutput)

    /// Owns every session and engine lifecycle call; all state above (and
    /// `status`) is touched only on this queue after init.
    private let audioQueue = DispatchQueue(label: "piano.audio.session", qos: .userInitiated)
    /// Route-change and configuration-change notifications often arrive in
    /// bursts during a device switch; they are coalesced into one pass that
    /// runs once the burst has been quiet for this long.
    private static let coalesceDelay: DispatchTimeInterval = .milliseconds(250)
    private var pendingRouteWork: DispatchWorkItem?
    private var pendingReconfigureReason: String?  // non-nil: the coalesced pass must reconfigure

    /// Called on the main thread whenever the status changes. Set it on the
    /// main thread before `start()`.
    var onStatus: ((Status) -> Void)?
    private var status = Status() {
        didSet {
            let snapshot = status
            DispatchQueue.main.async { [weak self] in
                self?.onStatus?(snapshot)
            }
        }
    }

    init(host: EngineHost) {
        self.host = host
    }

    deinit {
        // `host` is released after this body: the render block must be stopped
        // before the pm_host it points to can be destroyed.
        engine.stop()
        for o in observers { NotificationCenter.default.removeObserver(o) }
    }

    // MARK: - Start / stop

    /// Runs `work` on `audioQueue` unless the controller is gone by then.
    private func onAudioQueue(_ work: @escaping (AudioController) -> Void) {
        audioQueue.async { [weak self] in
            guard let self = self, !self.terminated else { return }
            work(self)
        }
    }

    func start() {
        onAudioQueue { me in
            // Observers first: anything they post is queued behind this block.
            me.installObservers()
            me.configureSession()
            me.reconfigure(reason: "Starting")
        }
    }

    /// Full asynchronous teardown (engine, source node, observers, session).
    /// `start()` afterwards rebuilds everything.
    func stop() {
        onAudioQueue { me in
            me.tearDown()
            me.status.message = "Stopped"
            #if os(iOS)
            try? AVAudioSession.sharedInstance().setActive(false, options: [])
            #endif
        }
    }

    /// Stops audio output for the offline benchmark, so that it measures the
    /// engine alone, and calls `completion` on the main thread once the render
    /// callback has stopped. The session, the graph and the prepared engine
    /// stay; notifications that would restart the output are held off until
    /// `resumeOutput()`.
    func suspendOutput(completion: @escaping () -> Void) {
        onAudioQueue { me in
            me.suspended = true
            me.cancelPendingRouteWork()
            me.generation += 1  // a prepare still in flight must not restart the engine
            me.engine.stop()
            me.status.running = false
            me.status.message = "Paused for benchmark"
            DispatchQueue.main.async { completion() }
        }
    }

    /// Restarts the output after `suspendOutput` (re-preparing only if the
    /// hardware sample rate changed meanwhile).
    func resumeOutput() {
        onAudioQueue { me in
            guard me.suspended else { return }
            me.suspended = false
            me.reconfigure(reason: "Resuming after benchmark")
        }
    }

    /// Process exit (willTerminate), main thread. Stops the render callback
    /// with a bounded wait and disables the controller for good. The session
    /// is not deactivated (that can block while I/O winds down, and the OS
    /// reclaims it) and the host is not destroyed. If audioQueue is busy past
    /// the timeout, the teardown still runs when it gets there; the host stays
    /// alive regardless, so a late render callback never sees freed memory.
    func shutdownForTermination(timeout: DispatchTimeInterval = .milliseconds(300)) {
        let done = DispatchSemaphore(value: 0)
        audioQueue.async { [self] in
            if !terminated {
                terminated = true
                tearDown()
            }
            done.signal()
        }
        _ = done.wait(timeout: .now() + timeout)
    }

    /// audioQueue only.
    private func tearDown() {
        suspended = false  // an explicit stop is not undone by resumeOutput()
        cancelPendingRouteWork()
        generation += 1  // a prepare still in flight must not restart the engine
        engine.stop()
        if let node = sourceNode {
            engine.detach(node)
            sourceNode = nil
        }
        nodeSampleRate = 0
        removeObservers()
        status.running = false
    }

    // MARK: - Session (iOS)

    private func configureSession() {
        #if os(iOS)
        let session = AVAudioSession.sharedInstance()
        do {
            try session.setCategory(.playback, mode: .default, options: [])
        } catch {
            status.message = "Audio session category failed: \(error.localizedDescription)"
        }
        // Preferences may be refused or rounded; the granted values are read back.
        try? session.setPreferredSampleRate(AudioController.preferredSampleRate)
        try? session.setPreferredIOBufferDuration(Double(AudioController.preferredFrames) / AudioController.preferredSampleRate)
        do {
            try session.setActive(true, options: [])
        } catch {
            status.message = "Audio session activation failed: \(error.localizedDescription)"
        }
        #endif
    }

    private func readHardwareStatus() {
        #if os(iOS)
        let session = AVAudioSession.sharedInstance()
        status.sampleRate = session.sampleRate
        status.ioBufferFrames = Int((session.ioBufferDuration * session.sampleRate).rounded())
        status.outputLatencyMs = session.outputLatency * 1000.0
        let outputs = session.currentRoute.outputs.map { $0.portName }
        status.route = outputs.isEmpty ? "none" : outputs.joined(separator: ", ")
        #else
        let format = engine.outputNode.outputFormat(forBus: 0)
        status.sampleRate = format.sampleRate
        status.ioBufferFrames = macBufferFrames() ?? 0
        status.outputLatencyMs = 0
        status.route = "Default output device"
        #endif
    }

    // MARK: - Graph

    /// Stops the engine, re-prepares the C++ engine if the hardware sample
    /// rate differs from the prepared one, rebuilds the source node and starts.
    /// audioQueue only.
    private func reconfigure(reason: String) {
        // This pass supersedes any coalesced route/configuration pass.
        cancelPendingRouteWork()
        generation += 1
        let myGeneration = generation
        engine.stop()
        status.running = false
        if suspended {
            // Benchmark running: stay stopped; resumeOutput() reconfigures.
            readHardwareStatus()
            status.message = "Paused for benchmark"
            return
        }
        status.message = reason
        #if os(macOS)
        // Native macOS app: ask the output device itself (HAL) for 48 kHz
        // and 64 frames, as an audio app would. (An iPad app on a Mac runs
        // the iOS path and gets the device's current settings.)
        status.ioRequest = requestMacDeviceIO()
        #endif
        readHardwareStatus()
        var rate = status.sampleRate
        if !(rate > 0) {
            // The device can report 0 Hz mid-switch; keep the prepared rate
            // rather than forcing a re-prepare to the preferred one.
            rate = host.isPrepared && host.preparedSampleRate > 0 ? host.preparedSampleRate : AudioController.preferredSampleRate
        }
        // Same rate (e.g. a route change between outputs at 48 kHz): no
        // re-prepare, which would cost about a second of silence.
        if host.isPrepared && abs(host.preparedSampleRate - rate) < 0.5 {
            buildGraphAndStart(sampleRate: rate)
            return
        }
        status.preparing = true
        status.message = "Preparing engine at \(Int(rate)) Hz"
        host.prepare(sampleRate: rate, maxFrames: max(AudioController.preferredFrames, status.ioBufferFrames)) { [weak self] ok in
            // EngineHost calls back on the main thread; hop to audioQueue.
            self?.onAudioQueue { me in
                guard myGeneration == me.generation else { return }
                me.status.preparing = false
                if ok {
                    me.buildGraphAndStart(sampleRate: rate)
                } else {
                    me.status.message = "Engine prepare failed"
                }
            }
        }
    }

    /// Re-prepares at the current rate (after prepare-time parameter changes).
    /// Audio keeps running and is silent for the duration of the prepare.
    /// Call from the main thread; completion runs on the main thread.
    func reprepare(completion: @escaping (Bool) -> Void) {
        onAudioQueue { me in
            let rate = me.host.preparedSampleRate > 0 ? me.host.preparedSampleRate : me.status.sampleRate
            guard rate > 0 else {
                DispatchQueue.main.async { completion(false) }
                return
            }
            me.status.preparing = true
            me.host.prepare(sampleRate: rate, maxFrames: max(AudioController.preferredFrames, me.status.ioBufferFrames)) { [weak me] ok in
                // Main thread here: clear the flag on audioQueue, report on main.
                me?.onAudioQueue { inner in
                    inner.status.preparing = false
                    DispatchQueue.main.async { completion(ok) }
                }
            }
        }
    }

    private func buildGraphAndStart(sampleRate: Double) {
        if sourceNode == nil || nodeSampleRate != sampleRate {
            if let old = sourceNode {
                engine.detach(old)
                sourceNode = nil
            }
            guard let format = AVAudioFormat(standardFormatWithSampleRate: sampleRate, channels: 2) else {
                status.message = "No audio format for \(sampleRate) Hz"
                return
            }
            let node = AudioController.makeSourceNode(host: host.raw, format: format)
            engine.attach(node)
            engine.connect(node, to: engine.mainMixerNode, format: format)
            sourceNode = node
            nodeSampleRate = sampleRate
        }
        engine.prepare()
        do {
            try engine.start()
            status.running = true
            status.message = "Running"
            #if os(macOS)
            // Again once running, in case starting the engine reset the
            // device buffer; readHardwareStatus() below reads the grant.
            status.ioRequest = requestMacDeviceIO()
            #endif
        } catch {
            status.running = false
            status.message = "Audio engine failed to start: \(error.localizedDescription)"
        }
        readHardwareStatus()
    }

    /// The real-time render block. It must stay free of allocation, locks,
    /// Objective-C messaging and I/O: it reads the timestamp, takes the two
    /// channel pointers and calls the C function.
    private static func makeSourceNode(host: OpaquePointer, format: AVAudioFormat) -> AVAudioSourceNode {
        return AVAudioSourceNode(format: format) { _, timestamp, frameCount, audioBufferList -> OSStatus in
            let buffers = UnsafeMutableAudioBufferListPointer(audioBufferList)
            if buffers.count < 1 {
                return noErr
            }
            guard let leftRaw = buffers[0].mData else {
                return noErr
            }
            let left = leftRaw.assumingMemoryBound(to: Float.self)
            var right = left
            if buffers.count >= 2, let rightRaw = buffers[1].mData {
                right = rightRaw.assumingMemoryBound(to: Float.self)
            }
            let ts = timestamp.pointee
            let hostTime: UInt64 = ts.mFlags.contains(.hostTimeValid) ? ts.mHostTime : 0
            pm_host_render(host, left, right, Int32(frameCount), hostTime)
            return noErr
        }
    }

    // MARK: - Notifications

    /// audioQueue only.
    private func removeObservers() {
        for o in observers { NotificationCenter.default.removeObserver(o) }
        observers.removeAll()
    }

    /// audioQueue only; a no-op while observers are installed. Observer blocks
    /// run on the posting thread (queue nil) and only forward to audioQueue.
    private func installObservers() {
        guard observers.isEmpty else { return }
        let center = NotificationCenter.default
        observers.append(center.addObserver(forName: .AVAudioEngineConfigurationChange, object: nil, queue: nil) { [weak self] note in
            let changed = note.object as? AVAudioEngine
            self?.onAudioQueue { me in
                guard let changed = changed, changed === me.engine else { return }
                // The engine has stopped itself (output format or device changed).
                me.scheduleRouteWork(reconfigureReason: "Audio configuration changed")
            }
        })
        #if os(iOS)
        let session = AVAudioSession.sharedInstance()
        observers.append(center.addObserver(forName: AVAudioSession.interruptionNotification, object: session, queue: nil) { [weak self] note in
            guard let info = note.userInfo,
                  let raw = info[AVAudioSessionInterruptionTypeKey] as? UInt,
                  let type = AVAudioSession.InterruptionType(rawValue: raw) else { return }
            self?.onAudioQueue { me in
                me.handleInterruption(type)
            }
        })
        observers.append(center.addObserver(forName: AVAudioSession.routeChangeNotification, object: session, queue: nil) { [weak self] _ in
            self?.onAudioQueue { me in
                me.scheduleRouteWork(reconfigureReason: nil)
            }
        })
        observers.append(center.addObserver(forName: AVAudioSession.mediaServicesWereResetNotification, object: session, queue: nil) { [weak self] _ in
            self?.onAudioQueue { me in
                me.handleMediaServicesReset()
            }
        })
        #endif
    }

    // MARK: - Coalescing route / configuration changes (audioQueue only)

    /// (Re)arms one delayed pass; every call within `coalesceDelay` of the
    /// previous one pushes it back, so a burst ends in a single reconfigure.
    private func scheduleRouteWork(reconfigureReason: String?) {
        if let reason = reconfigureReason {
            pendingReconfigureReason = reason
        }
        pendingRouteWork?.cancel()
        let item = DispatchWorkItem { [weak self] in
            self?.runPendingRouteWork()
        }
        pendingRouteWork = item
        audioQueue.asyncAfter(deadline: .now() + AudioController.coalesceDelay, execute: item)
    }

    private func cancelPendingRouteWork() {
        pendingRouteWork?.cancel()
        pendingRouteWork = nil
        pendingReconfigureReason = nil
    }

    private func runPendingRouteWork() {
        let reason = pendingReconfigureReason
        pendingRouteWork = nil
        pendingReconfigureReason = nil
        if let reason = reason {
            reconfigure(reason: reason)
            return
        }
        // Route change only. A new route can bring another sample rate; the
        // engine then also posts a configuration change, which reconfigures
        // (and re-prepares only if the rate really differs).
        readHardwareStatus()
        if status.running && !engine.isRunning {
            reconfigure(reason: "Output route changed")
        }
    }

    #if os(iOS)
    private func handleInterruption(_ type: AVAudioSession.InterruptionType) {
        switch type {
        case .began:
            // Do not restart into an active interruption.
            cancelPendingRouteWork()
            status.running = false
            status.message = "Interrupted"
        case .ended:
            // A piano should come back after a call or an alarm even when the
            // system does not set .shouldResume.
            configureSession()
            reconfigure(reason: "Resuming after interruption")
        @unknown default:
            break
        }
    }

    private func handleMediaServicesReset() {
        // Every audio object is invalid now: drop them and build a new engine
        // and session.
        generation += 1
        sourceNode = nil
        nodeSampleRate = 0
        removeObservers()
        engine = AVAudioEngine()
        installObservers()
        configureSession()
        reconfigure(reason: "Media services were reset")
    }
    #endif

    // MARK: - macOS buffer size (the device's I/O buffer, set through the output unit)

    #if os(macOS)
    /// The device AVAudioEngine's output unit plays to.
    private func macOutputDevice() -> AudioDeviceID? {
        guard let unit = engine.outputNode.audioUnit else { return nil }
        var device = AudioDeviceID(0)
        var size = UInt32(MemoryLayout<AudioDeviceID>.size)
        let err = AudioUnitGetProperty(unit, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &device, &size)
        return err == noErr && device != 0 ? device : nil
    }

    private static func address(_ selector: AudioObjectPropertySelector) -> AudioObjectPropertyAddress {
        return AudioObjectPropertyAddress(mSelector: selector, mScope: kAudioObjectPropertyScopeGlobal,
                                          mElement: kAudioObjectPropertyElementMain)
    }

    private static func getDeviceProperty<T>(_ device: AudioDeviceID, _ selector: AudioObjectPropertySelector, _ value: inout T) -> OSStatus {
        var addr = address(selector)
        var size = UInt32(MemoryLayout<T>.size)
        return AudioObjectGetPropertyData(device, &addr, 0, nil, &size, &value)
    }

    private static func setDeviceProperty<T>(_ device: AudioDeviceID, _ selector: AudioObjectPropertySelector, _ value: T) -> OSStatus {
        var addr = address(selector)
        var v = value
        return AudioObjectSetPropertyData(device, &addr, 0, nil, UInt32(MemoryLayout<T>.size), &v)
    }

    /// The device's I/O buffer (kAudioDevicePropertyBufferFrameSize), as granted.
    private func macBufferFrames() -> Int? {
        guard let device = macOutputDevice() else { return nil }
        var frames: UInt32 = 0
        return AudioController.getDeviceProperty(device, kAudioDevicePropertyBufferFrameSize, &frames) == noErr ? Int(frames) : nil
    }

    private static func deviceSupportsRate(_ device: AudioDeviceID, _ rate: Double) -> Bool {
        var addr = address(kAudioDevicePropertyAvailableNominalSampleRates)
        var size: UInt32 = 0
        guard AudioObjectGetPropertyDataSize(device, &addr, 0, nil, &size) == noErr, size > 0 else { return false }
        let count = Int(size) / MemoryLayout<AudioValueRange>.size
        var ranges = [AudioValueRange](repeating: AudioValueRange(), count: count)
        guard AudioObjectGetPropertyData(device, &addr, 0, nil, &size, &ranges) == noErr else { return false }
        return ranges.contains { rate >= $0.mMinimum - 0.5 && rate <= $0.mMaximum + 0.5 }
    }

    /// Asks the output device for the preferred rate (only if it supports
    /// it and runs at another one; this changes the device's rate for every
    /// app, as audio apps do) and the preferred buffer, clamped to the
    /// device's range (kAudioDevicePropertyBufferFrameSizeRange). Returns a
    /// line for the status: what was requested and anything refused. A rate
    /// change makes AVAudioEngine post a configuration change, which runs
    /// reconfigure() again; by then the device is at the requested rate and
    /// nothing is set twice.
    private func requestMacDeviceIO() -> String {
        guard let device = macOutputDevice() else { return "no output device found" }
        let wantRate = AudioController.preferredSampleRate
        var notes: [String] = []
        var rate: Float64 = 0
        _ = AudioController.getDeviceProperty(device, kAudioDevicePropertyNominalSampleRate, &rate)
        if abs(rate - wantRate) > 0.5 {
            if AudioController.deviceSupportsRate(device, wantRate) {
                let err = AudioController.setDeviceProperty(device, kAudioDevicePropertyNominalSampleRate, Float64(wantRate))
                if err != noErr { notes.append("sample rate refused (OSStatus \(err))") }
            } else {
                notes.append(String(format: "device has no %.0f Hz", wantRate))
            }
        }
        var range = AudioValueRange()
        var frames = UInt32(AudioController.preferredFrames)
        if AudioController.getDeviceProperty(device, kAudioDevicePropertyBufferFrameSizeRange, &range) == noErr {
            frames = UInt32(max(range.mMinimum, min(range.mMaximum, Double(frames))))
            if Int(frames) != AudioController.preferredFrames {
                notes.append(String(format: "buffer range %.0f...%.0f frames", range.mMinimum, range.mMaximum))
            }
        }
        let err = AudioController.setDeviceProperty(device, kAudioDevicePropertyBufferFrameSize, frames)
        if err != noErr { notes.append("buffer size refused (OSStatus \(err))") }
        var out = String(format: "requested %.0f Hz / %ld frames from the output device", wantRate, Int(frames))
        if !notes.isEmpty { out += "; " + notes.joined(separator: "; ") }
        return out
    }
    #endif
}
