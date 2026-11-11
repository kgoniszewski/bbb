// Audio output: AVAudioEngine with one AVAudioSourceNode whose render block
// calls the C++ engine. On iOS also the AVAudioSession (category, preferred
// 48 kHz / 64 frames, interruptions, route changes, media-services reset).
//
// The render block captures only the pm_host pointer (an OpaquePointer, a
// plain value): no Swift object, no allocation, no lock, no I/O. Everything
// else here runs on the main thread.
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

    /// Called on the main thread whenever the status changes.
    var onStatus: ((Status) -> Void)?
    private(set) var status = Status() {
        didSet { onStatus?(status) }
    }

    init(host: EngineHost) {
        self.host = host
    }

    deinit {
        for o in observers { NotificationCenter.default.removeObserver(o) }
    }

    // MARK: - Start / stop

    func start() {
        installObservers()
        configureSession()
        reconfigure(reason: "Starting")
    }

    func stop() {
        engine.stop()
        status.running = false
        status.message = "Stopped"
        #if os(iOS)
        try? AVAudioSession.sharedInstance().setActive(false, options: [])
        #endif
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
    func reconfigure(reason: String) {
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
            rate = AudioController.preferredSampleRate
        }
        if host.isPrepared && host.preparedSampleRate == rate {
            buildGraphAndStart(sampleRate: rate)
            return
        }
        status.preparing = true
        status.message = "Preparing engine at \(Int(rate)) Hz"
        host.prepare(sampleRate: rate, maxFrames: max(AudioController.preferredFrames, status.ioBufferFrames)) { [weak self] ok in
            guard let self = self, myGeneration == self.generation else { return }
            self.status.preparing = false
            if ok {
                self.buildGraphAndStart(sampleRate: rate)
            } else {
                self.status.message = "Engine prepare failed"
            }
        }
    }

    /// Re-prepares at the current rate (after prepare-time parameter changes).
    /// Audio keeps running and is silent for the duration of the prepare.
    func reprepare(completion: @escaping (Bool) -> Void) {
        let rate = host.preparedSampleRate > 0 ? host.preparedSampleRate : status.sampleRate
        guard rate > 0 else {
            completion(false)
            return
        }
        status.preparing = true
        host.prepare(sampleRate: rate, maxFrames: max(AudioController.preferredFrames, status.ioBufferFrames)) { [weak self] ok in
            self?.status.preparing = false
            completion(ok)
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

    private func installObservers() {
        guard observers.isEmpty else { return }
        let center = NotificationCenter.default
        observers.append(center.addObserver(forName: .AVAudioEngineConfigurationChange, object: nil, queue: .main) { [weak self] note in
            guard let self = self, let changed = note.object as? AVAudioEngine, changed === self.engine else { return }
            // The engine has stopped itself (output format or device changed).
            self.reconfigure(reason: "Audio configuration changed")
        })
        #if os(iOS)
        let session = AVAudioSession.sharedInstance()
        observers.append(center.addObserver(forName: AVAudioSession.interruptionNotification, object: session, queue: .main) { [weak self] note in
            self?.handleInterruption(note)
        })
        observers.append(center.addObserver(forName: AVAudioSession.routeChangeNotification, object: session, queue: .main) { [weak self] _ in
            guard let self = self else { return }
            self.readHardwareStatus()
            // A new route can bring another sample rate; the engine also
            // posts a configuration change in that case, which reconfigures.
            if self.status.running && !self.engine.isRunning {
                self.reconfigure(reason: "Output route changed")
            }
        })
        observers.append(center.addObserver(forName: AVAudioSession.mediaServicesWereResetNotification, object: session, queue: .main) { [weak self] _ in
            self?.handleMediaServicesReset()
        })
        #endif
    }

    #if os(iOS)
    private func handleInterruption(_ note: Notification) {
        guard let info = note.userInfo,
              let raw = info[AVAudioSessionInterruptionTypeKey] as? UInt,
              let type = AVAudioSession.InterruptionType(rawValue: raw) else { return }
        switch type {
        case .began:
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
        for o in observers { NotificationCenter.default.removeObserver(o) }
        observers.removeAll()
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
