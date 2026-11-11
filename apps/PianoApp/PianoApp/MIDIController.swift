// Core MIDI input: one client, one input port (event-list API, MIDI 1.0
// protocol, so MIDI 1.0 devices arrive unscaled as UMP type 2), connected to
// every source; hot-plug through the client's notify block.
//
// The receive block runs on Core MIDI's thread. It passes the raw
// MIDIEventList and the source id (the connection refCon) to the C++ host:
// no allocation, no Swift objects, no locks. Everything else runs on the
// main thread.
import CoreMIDI
import Foundation
import PianoCore

final class MIDIController {
    struct Source: Identifiable, Equatable {
        let id: UInt32          // our id, passed as the connection refCon
        let uniqueID: MIDIUniqueID
        let endpoint: MIDIEndpointRef
        let name: String
    }

    private let host: EngineHost
    private var client = MIDIClientRef()
    private var port = MIDIPortRef()
    private var connected: [MIDIUniqueID: Source] = [:]
    private var nextId: UInt32 = 1
    /// Every source ever seen, so the monitor can name messages from a source
    /// that has gone away.
    private(set) var names: [UInt32: String] = [:]

    /// Main thread: the connected sources changed.
    var onSourcesChanged: (([Source]) -> Void)?
    /// Main thread: a source disappeared (notes and pedals are released).
    var onSourceRemoved: ((Source) -> Void)?
    private(set) var statusMessage = ""

    init(host: EngineHost) {
        self.host = host
    }

    deinit {
        if port != 0 { MIDIPortDispose(port) }
        if client != 0 { MIDIClientDispose(client) }
    }

    var sources: [Source] {
        return connected.values.sorted { $0.id < $1.id }
    }

    /// Creates the client and port and connects every source. Returns false
    /// (with statusMessage set) on failure.
    @discardableResult
    func start() -> Bool {
        guard client == 0 else { return true }
        let layoutProblem = MIDIController.eventListLayoutProblem()
        var status = MIDIClientCreateWithBlock("Piano" as CFString, &client) { [weak self] notification in
            let id = notification.pointee.messageID
            if id == .msgObjectAdded || id == .msgObjectRemoved || id == .msgSetupChanged || id == .msgPropertyChanged {
                DispatchQueue.main.async {
                    self?.rescan()
                }
            }
        }
        guard status == noErr else {
            statusMessage = "MIDIClientCreate failed (\(status))"
            return false
        }
        let rawHost = host.raw
        status = MIDIInputPortCreateWithProtocol(client, "Piano In" as CFString, ._1_0, &port) { eventList, srcConnRefCon in
            let sourceId = UInt32(truncatingIfNeeded: Int(bitPattern: srcConnRefCon))
            _ = pm_host_midi_event_list(rawHost, UnsafeRawPointer(eventList), sourceId)
        }
        guard status == noErr else {
            statusMessage = "MIDIInputPortCreate failed (\(status))"
            return false
        }
        statusMessage = layoutProblem ?? ""
        rescan()
        return true
    }

    /// Connects new sources and forgets removed ones. Main thread.
    func rescan() {
        guard port != 0 else { return }
        var present = Set<MIDIUniqueID>()
        let count = MIDIGetNumberOfSources()
        for i in 0..<count {
            let endpoint = MIDIGetSource(i)
            if endpoint == 0 { continue }
            var uid: MIDIUniqueID = 0
            if MIDIObjectGetIntegerProperty(endpoint, kMIDIPropertyUniqueID, &uid) != noErr { continue }
            // An unplugged device may stay listed but offline: treat it as gone.
            var offline: Int32 = 0
            if MIDIObjectGetIntegerProperty(endpoint, kMIDIPropertyOffline, &offline) == noErr && offline != 0 { continue }
            present.insert(uid)
            if connected[uid] != nil { continue }
            let id = nextId
            nextId += 1
            let refCon = UnsafeMutableRawPointer(bitPattern: UInt(id))
            if MIDIPortConnectSource(port, endpoint, refCon) == noErr {
                let source = Source(id: id, uniqueID: uid, endpoint: endpoint, name: MIDIController.displayName(endpoint))
                connected[uid] = source
                names[id] = source.name
            }
        }
        for (uid, source) in connected where !present.contains(uid) {
            // Disconnect explicitly, so that a device coming back online is
            // connected exactly once.
            _ = MIDIPortDisconnectSource(port, source.endpoint)
            connected[uid] = nil
            onSourceRemoved?(source)
        }
        onSourcesChanged?(sources)
    }

    func name(forSource id: UInt32) -> String {
        if id == 0 { return "?" }
        return names[id] ?? "source \(id)"
    }

    static func displayName(_ object: MIDIObjectRef) -> String {
        var value: Unmanaged<CFString>?
        if MIDIObjectGetStringProperty(object, kMIDIPropertyDisplayName, &value) == noErr, let v = value {
            return v.takeRetainedValue() as String
        }
        if MIDIObjectGetStringProperty(object, kMIDIPropertyName, &value) == noErr, let v = value {
            return v.takeRetainedValue() as String
        }
        return "Unknown source"
    }

    /// The C++ side reads MIDIEventList with Core MIDI's packed layout
    /// (core/pm_host/midi.h). Returns a description if the SDK disagrees.
    static func eventListLayoutProblem() -> String? {
        let packetSize = MemoryLayout<MIDIEventPacket>.size
        let wordCountOffset = MemoryLayout<MIDIEventPacket>.offset(of: \MIDIEventPacket.wordCount)
        let wordsOffset = MemoryLayout<MIDIEventPacket>.offset(of: \MIDIEventPacket.words)
        let packetOffset = MemoryLayout<MIDIEventList>.offset(of: \MIDIEventList.packet)
        if packetSize == 268 && wordCountOffset == 8 && wordsOffset == 12 && packetOffset == 8 {
            return nil
        }
        return "MIDIEventList layout differs from the parser's (packet \(packetSize) bytes, offsets "
            + "\(String(describing: wordCountOffset)), \(String(describing: wordsOffset)), "
            + "\(String(describing: packetOffset))). Please report."
    }
}
