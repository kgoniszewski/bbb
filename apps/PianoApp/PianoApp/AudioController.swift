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
        status.message = reason
        #if os(macOS)
        setMacBufferFrames(AudioController.preferredFrames)
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
    private func macBufferFrames() -> Int? {
        guard let unit = engine.outputNode.audioUnit else { return nil }
        var frames: UInt32 = 0
        var size = UInt32(MemoryLayout<UInt32>.size)
        let err = AudioUnitGetProperty(unit, kAudioDevicePropertyBufferFrameSize, kAudioUnitScope_Global, 0, &frames, &size)
        return err == noErr ? Int(frames) : nil
    }

    private func setMacBufferFrames(_ frames: Int) {
        guard let unit = engine.outputNode.audioUnit else { return }
        var value = UInt32(frames)
        _ = AudioUnitSetProperty(unit, kAudioDevicePropertyBufferFrameSize, kAudioUnitScope_Global, 0, &value,
                                 UInt32(MemoryLayout<UInt32>.size))
    }
    #endif
}
