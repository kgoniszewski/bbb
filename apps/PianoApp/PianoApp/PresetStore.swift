// Presets and saved state (plan §5.3) for the app: factory presets from the
// bundle, user presets and the autosaved state in Application Support.
// Main thread only; the engine work runs through EngineHost.
import Foundation
import PianoCore

struct Preset: Identifiable, Hashable {
    let name: String
    let json: String
    let isFactory: Bool
    var id: String { (isFactory ? "factory:" : "user:") + name }
}

final class PresetStore: ObservableObject {
    @Published private(set) var factory: [Preset] = []
    @Published private(set) var user: [Preset] = []
    /// Name of the preset last applied ("" for none).
    @Published private(set) var selectedName = ""
    /// A parameter changed since the preset was applied.
    @Published private(set) var modified = false
    @Published private(set) var busy = false
    @Published var message = ""

    /// After a preset or the saved state was applied: refresh mirrored values.
    var onApplied: (() -> Void)?

    private let host: EngineHost?
    private var autosaveWork: DispatchWorkItem?
    private static let modifiedKey = "presetModified"
    private static let resetJSON = "{\"format\":\"pm-preset\",\"version\":1,\"params\":{}}"

    init(host: EngineHost?) {
        self.host = host
        factory = PresetStore.loadFactory()
        user = loadUser()
    }

    // MARK: - Files

    private static var supportDirectory: URL? {
        guard let base = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first else { return nil }
        try? FileManager.default.createDirectory(at: base, withIntermediateDirectories: true)
        return base
    }

    private static var userDirectory: URL? {
        guard let dir = supportDirectory?.appendingPathComponent("Presets", isDirectory: true) else { return nil }
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    private static var stateURL: URL? { supportDirectory?.appendingPathComponent("state.json") }

    /// Name and JSON of a preset file, or nil if it is not a pm-preset document.
    private static func readPresetFile(_ url: URL) -> (name: String, json: String)? {
        guard let data = try? Data(contentsOf: url),
              let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              (obj["format"] as? String) == "pm-preset",
              let json = String(data: data, encoding: .utf8) else { return nil }
        let name = (obj["name"] as? String).flatMap { $0.isEmpty ? nil : $0 } ?? url.deletingPathExtension().lastPathComponent
        return (name, json)
    }

    private static func loadFactory() -> [Preset] {
        var urls = Bundle.main.urls(forResourcesWithExtension: "json", subdirectory: "Presets") ?? []
        if urls.isEmpty {
            // The folder may have been flattened into the bundle's root.
            urls = Bundle.main.urls(forResourcesWithExtension: "json", subdirectory: nil) ?? []
        }
        var seen = Set<String>()
        var list: [Preset] = []
        for url in urls {
            guard let p = readPresetFile(url), !seen.contains(p.name) else { continue }
            seen.insert(p.name)
            list.append(Preset(name: p.name, json: p.json, isFactory: true))
        }
        return list.sorted { $0.name.localizedStandardCompare($1.name) == .orderedAscending }
    }

    private func loadUser() -> [Preset] {
        guard let dir = PresetStore.userDirectory,
              let urls = try? FileManager.default.contentsOfDirectory(at: dir, includingPropertiesForKeys: nil) else { return [] }
        var list: [Preset] = []
        for url in urls where url.pathExtension == "json" {
            guard let p = PresetStore.readPresetFile(url) else { continue }
            list.append(Preset(name: p.name, json: p.json, isFactory: false))
        }
        return list.sorted { $0.name.localizedStandardCompare($1.name) == .orderedAscending }
    }

    private static func fileName(for name: String) -> String {
        let bad = CharacterSet(charactersIn: "/\\:?*\"<>|").union(.controlCharacters)
        let cleaned = name.unicodeScalars.map { bad.contains($0) ? "-" : String($0) }.joined()
        return (cleaned.isEmpty ? "Preset" : cleaned) + ".json"
    }

    // MARK: - State (launch and autosave)

    /// Restores state.json. Call before the first prepare: the values are set
    /// synchronously, so prepare-time ones need no second prepare.
    func restoreStateBeforePrepare() {
        guard let host = host else { return }
        guard let url = PresetStore.stateURL, let data = try? Data(contentsOf: url),
              let json = String(data: data, encoding: .utf8) else {
            selectedName = factory.contains { $0.name == "Concert Hall" } ? "Concert Hall" : ""
            return
        }
        let r = host.applyPresetBeforePrepare(json, missingToDefault: false)
        if r.ok == 1 {
            selectedName = stringFromCTuple(r.name)
            modified = UserDefaults.standard.bool(forKey: PresetStore.modifiedKey)
        } else {
            message = "Saved state not restored: " + stringFromCTuple(r.error)
        }
    }

    /// Saves state ~1 s after the last change.
    func scheduleAutosave() {
        autosaveWork?.cancel()
        let work = DispatchWorkItem { [weak self] in self?.saveStateNow() }
        autosaveWork = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0, execute: work)
    }

    /// Writes the full current state (hidden parameters too) to state.json.
    func saveStateNow() {
        autosaveWork?.cancel()
        autosaveWork = nil
        guard let host = host, let url = PresetStore.stateURL else { return }
        let json = host.presetJSON(name: selectedName, fullState: true)
        guard !json.isEmpty else { return }
        try? json.write(to: url, atomically: true, encoding: .utf8)
        UserDefaults.standard.set(modified, forKey: PresetStore.modifiedKey)
    }

    /// A parameter was changed from the UI.
    func noteParamChanged() {
        if !modified { modified = true }
        scheduleAutosave()
    }

    // MARK: - Applying, saving, deleting

    func apply(_ preset: Preset) {
        // Factory presets hold only what they change: the rest goes to the
        // defaults. User presets hold every visible parameter.
        apply(json: preset.json, missingToDefault: preset.isFactory, selecting: preset.name)
    }

    func resetToDefaults() {
        apply(json: PresetStore.resetJSON, missingToDefault: true, selecting: "")
    }

    private func apply(json: String, missingToDefault: Bool, selecting name: String) {
        guard let host = host, !busy else { return }
        busy = true
        message = ""
        host.applyPreset(json, missingToDefault: missingToDefault) { [weak self] r in
            guard let self = self else { return }
            self.busy = false
            if r.ok == 1 {
                self.selectedName = name
                self.modified = false
                let warnings = stringFromCTuple(r.warnings)
                if !warnings.isEmpty { self.message = warnings }
            } else {
                self.message = "Preset not applied: " + stringFromCTuple(r.error)
            }
            self.onApplied?()
            self.scheduleAutosave()
        }
    }

    func isUserName(_ name: String) -> Bool { user.contains { $0.name == name } }

    /// Saves the current visible parameters as a user preset (overwrites one of the same name).
    func saveUser(name rawName: String) {
        let name = rawName.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let host = host, !name.isEmpty, let dir = PresetStore.userDirectory else { return }
        let json = host.presetJSON(name: name, fullState: false)
        guard !json.isEmpty else { return }
        // Overwrite by name: remove a file holding the same preset name under another file name.
        removeUserFiles(named: name)
        let url = dir.appendingPathComponent(PresetStore.fileName(for: name))
        do {
            try json.write(to: url, atomically: true, encoding: .utf8)
            selectedName = name
            modified = false
            message = "Saved \"\(name)\""
        } catch {
            message = "Could not save: \(error.localizedDescription)"
        }
        user = loadUser()
        scheduleAutosave()
    }

    func deleteUser(name: String) {
        removeUserFiles(named: name)
        user = loadUser()
        if selectedName == name {
            modified = true
            scheduleAutosave()
        }
    }

    private func removeUserFiles(named name: String) {
        guard let dir = PresetStore.userDirectory,
              let urls = try? FileManager.default.contentsOfDirectory(at: dir, includingPropertiesForKeys: nil) else { return }
        for url in urls where url.pathExtension == "json" {
            if PresetStore.readPresetFile(url)?.name == name {
                try? FileManager.default.removeItem(at: url)
            }
        }
    }
}
