// Sound tab (M5c): presets, panic, and controls generated from the engine's
// parameter descriptors (ParamDescriptor), visible parameters only.
//
// Observation: only PresetStore and ParamValues, never AppModel, so slider
// drags do not re-render the Play tab and its meters.
import Foundation
import SwiftUI

struct SoundView: View {
    // Not observed: the content observes the stores it shows.
    let model: AppModel

    var body: some View {
        SoundContent(model: model, presets: model.presets, values: model.paramValues)
            .onAppear { model.refreshFromEngine() }
    }
}

struct ParamGroup: Identifiable {
    let title: String
    let params: [ParamDescriptor]
    var id: String { title }
}

struct SoundContent: View {
    let model: AppModel
    @ObservedObject var presets: PresetStore
    @ObservedObject var values: ParamValues
    @State private var showSave = false
    @State private var saveName = ""
    @State private var confirmDelete = false

    /// Groups by key; a visible parameter not listed goes to the last group.
    static let groupKeys: [(title: String, keys: [String])] = [
        ("Microphones", ["stereo_width"]),
        ("Room", ["room_enabled", "room_level_db", "room_decay_s", "room_size", "room_hf_ratio", "room_early_late"]),
        ("Tone", ["treble_db", "master_gain_db"]),
        ("Touch & instrument", []),
    ]

    static let groups: [ParamGroup] = {
        let visible = ParamDescriptor.all.filter { !$0.isHidden }
        let listed = Set(groupKeys.flatMap { $0.keys })
        var out: [ParamGroup] = []
        for (i, g) in groupKeys.enumerated() {
            var params = g.keys.compactMap { k in visible.first { $0.key == k } }
            if i == groupKeys.count - 1 {
                params += visible.filter { !listed.contains($0.key) }
            }
            out.append(ParamGroup(title: g.title, params: params))
        }
        return out
    }()

    private var presetTitle: String {
        let name = presets.selectedName.isEmpty ? "Defaults" : presets.selectedName
        return presets.modified ? name + " (modified)" : name
    }

    var body: some View {
        Form {
            Section("Preset") {
                HStack {
                    Menu {
                        Section("Factory") {
                            ForEach(presets.factory) { p in
                                Button(p.name) { presets.apply(p) }
                            }
                        }
                        if !presets.user.isEmpty {
                            Section("User") {
                                ForEach(presets.user) { p in
                                    Button(p.name) { presets.apply(p) }
                                }
                            }
                        }
                    } label: {
                        Label(presetTitle, systemImage: "slider.horizontal.3")
                    }
                    .disabled(presets.busy)
                    Spacer()
                    if presets.busy { ProgressView() }
                }
                HStack(spacing: 16) {
                    Button("Save As\u{2026}") {
                        saveName = presets.isUserName(presets.selectedName) ? presets.selectedName : ""
                        showSave = true
                    }
                    Button("Delete", role: .destructive) { confirmDelete = true }
                        .disabled(!presets.isUserName(presets.selectedName))
                    Button("Reset to defaults") { presets.resetToDefaults() }
                        .disabled(presets.busy)
                }
                .buttonStyle(.bordered)
                if !presets.message.isEmpty {
                    Text(presets.message).font(.caption).foregroundColor(.secondary)
                }
                Text("Presets that change tuning, quality or resonance re-prepare the engine (about a second of silence).")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            Section {
                Button("Panic (all notes off)", role: .destructive) { model.panic() }
            }
            ForEach(SoundContent.groups) { group in
                if !group.params.isEmpty {
                    Section(group.title) {
                        ForEach(group.params) { d in
                            ParamRow(model: model, d: d, value: values.value(d))
                        }
                    }
                    // A preset being applied writes parameters on the prepare queue.
                    .disabled(presets.busy)
                }
            }
        }
        .alert("Save Preset", isPresented: $showSave) {
            TextField("Name", text: $saveName)
            Button("Save") { presets.saveUser(name: saveName) }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("A user preset with the same name is replaced.")
        }
        .confirmationDialog("Delete \"\(presets.selectedName)\"?", isPresented: $confirmDelete, titleVisibility: .visible) {
            Button("Delete", role: .destructive) { presets.deleteUser(name: presets.selectedName) }
        }
    }
}

struct ParamRow: View {
    let model: AppModel
    let d: ParamDescriptor
    let value: Double

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            control
            if d.isPrepareClass {
                Text(d.isBoolean || d.isInteger
                     ? "Changing this re-prepares the engine (about a second of silence)."
                     : "Applied on release; re-prepares the engine (about a second of silence).")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
    }

    @ViewBuilder private var control: some View {
        if d.isBoolean {
            Toggle(d.name, isOn: Binding(get: { value >= 0.5 },
                                         set: { model.setParam(d, $0 ? 1.0 : 0.0, commit: true) }))
        } else if d.isInteger && !d.enumNames.isEmpty {
            Picker(d.name, selection: Binding(get: { Int(value.rounded()) },
                                              set: { model.setParam(d, Double($0), commit: true) })) {
                ForEach(0..<d.enumNames.count, id: \.self) { i in
                    Text(ParamRow.enumLabel(d.enumNames[i])).tag(Int(d.min.rounded()) + i)
                }
            }
        } else {
            VStack(alignment: .leading, spacing: 2) {
                HStack {
                    Text(d.name)
                    Spacer()
                    Text(d.display(value))
                        .monospacedDigit()
                        .foregroundColor(.secondary)
                }
                slider
            }
        }
    }

    @ViewBuilder private var slider: some View {
        let binding = Binding<Double>(get: { toSlider(value) },
                                      set: { model.setParam(d, fromSlider($0), commit: false) })
        if d.isInteger {
            Slider(value: binding, in: d.min...d.max, step: 1, onEditingChanged: editingChanged)
        } else {
            Slider(value: binding, in: toSlider(d.min)...toSlider(d.max), onEditingChanged: editingChanged)
        }
    }

    private func editingChanged(_ editing: Bool) {
        // Prepare-time parameters take the value on release (one re-prepare).
        guard !editing, d.isPrepareClass else { return }
        model.setParam(d, model.paramValues.value(d), commit: true)
    }

    private func toSlider(_ v: Double) -> Double { d.isLog ? log(max(v, d.min)) : v }
    private func fromSlider(_ s: Double) -> Double { d.isLog ? exp(s) : s }

    static func enumLabel(_ raw: String) -> String {
        raw.replacingOccurrences(of: "_", with: " ").capitalized
    }
}
