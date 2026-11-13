// Play screen (M2 minimal UI): status, MIDI sources, controls, CPU meter
// and an on-screen keyboard strip.
//
// Observation (see AppModel): this view and the keyboard strip observe
// nothing that changes while the engine idles. The status and meter
// sections observe MeterStore (published only when a displayed value
// changes) and AppModel (published only on user actions and audio or MIDI
// configuration changes); the process-CPU line is a leaf of its own.
// PlayContent observes only the UI cost test mode (UICostStore), which hides
// the keyboard and / or stops the meter poll so that the process-CPU line
// shows what they cost on the main thread.
import Foundation
import SwiftUI

struct PlayView: View {
    // Not observed: the sections below observe what they show.
    let model: AppModel
    static let keyboardOctaves = 3

    var body: some View {
        PlayContent(model: model, ui: model.uiCost)
            // Meters are polled only while this tab is shown.
            .onAppear { model.setPlayVisible(true) }
            .onDisappear { model.setPlayVisible(false) }
    }
}

struct PlayContent: View {
    let model: AppModel
    @ObservedObject var ui: UICostStore

    var body: some View {
        let mode = ui.mode
        VStack(spacing: 0) {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    if mode != .minimal {
                        StatusSection(status: model.engineStatus)
                        ControlsSection()
                        CPUMeterView(meters: model.meters, cpu: model.cpuStore)
                    }
                    UICostSection(model: model, mode: mode, cpu: model.cpuStore)
                }
                .padding()
                .frame(maxWidth: .infinity, alignment: .leading)
            }
            if mode.showsKeyboard {
                KeyboardStrip(model: model, keyboard: model.keyboard)
            }
        }
    }
}

/// The UI cost test: hides parts of this screen so that the process-CPU
/// line (kept running once a second in every mode) shows what they cost.
struct UICostSection: View {
    let model: AppModel
    let mode: UICostMode
    let cpu: CpuStore
    @State private var copied = false

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text("UI cost test")
                Picker("UI cost test", selection: Binding(get: { mode }, set: { model.setUICostMode($0) })) {
                    ForEach(UICostMode.allCases) { m in
                        Text(m.label).tag(m)
                    }
                }
                .pickerStyle(.segmented)
                .frame(maxWidth: 420)
            }
            if mode == .minimal {
                ProcessCpuLine(cpu: cpu)
                    .font(.system(.caption, design: .monospaced))
                Button(copied ? "Copied" : "Copy diagnostics") {
                    MonitorView.copyToClipboard(model.diagnosticsText())
                    copied = true
                }
                .buttonStyle(.bordered)
            }
            Text("Hides the keyboard and / or stops the meter updates; the process-CPU line keeps updating once a second, so the change in \"main thread\" shows what the hidden part costs.")
                .font(.caption)
                .foregroundColor(.secondary)
        }
    }
}

struct StatusSection: View {
    @EnvironmentObject var model: AppModel
    @ObservedObject var status: EngineStatusStore

    var body: some View {
        let m = status.snapshot
        let mismatch = model.ioMismatchText()
        VStack(alignment: .leading, spacing: 6) {
            Text("Status").font(.headline)
            row("Audio", model.audio.message + (model.audio.preparing ? " (preparing...)" : ""))
            row("Sample rate", model.audio.sampleRate > 0 ? String(format: "%.0f Hz", model.audio.sampleRate) : "-")
            row("I/O buffer", ioBufferText())
            if !mismatch.isEmpty {
                // The system did not grant the requested 48 kHz / 64 frames.
                HStack(alignment: .firstTextBaseline) {
                    Text(verbatim: "Not as requested")
                        .foregroundColor(.orange)
                        .frame(width: 120, alignment: .leading)
                    Text(verbatim: mismatch)
                        .foregroundColor(.orange)
                }
            }
            if !model.audio.ioRequest.isEmpty {
                row("I/O request", model.audio.ioRequest)
            }
            row("Render buffer", m.renderFrames > 0 ? "\(m.renderFrames) frames" : "-")
            row("Output", model.audio.route)
            row("Engine", "\(m.awakeModes) modes awake, \(m.activeHammers) hammers")
            row("MIDI", "\(m.midiEvents) engine events, \(m.droppedEvents) dropped")
            if !model.midiMessage.isEmpty {
                row("MIDI status", model.midiMessage)
            }
            Text("MIDI sources").font(.subheadline).padding(.top, 4)
            if model.sources.isEmpty {
                Text(verbatim: "none connected").foregroundColor(.secondary)
            } else {
                ForEach(model.sources) { source in
                    Text(verbatim: "\u{2022} " + source.name)
                }
            }
        }
    }

    private func ioBufferText() -> String {
        let frames = model.audio.ioBufferFrames
        guard frames > 0, model.audio.sampleRate > 0 else { return "-" }
        var text = String(format: "%d frames (%.2f ms)", frames, Double(frames) / model.audio.sampleRate * 1000.0)
        if model.audio.outputLatencyMs > 0 {
            text += String(format: ", output latency %.2f ms", model.audio.outputLatencyMs)
        }
        return text
    }

    private func row(_ label: String, _ value: String) -> some View {
        HStack(alignment: .firstTextBaseline) {
            Text(verbatim: label)
                .foregroundColor(.secondary)
                .frame(width: 120, alignment: .leading)
            Text(verbatim: value)
        }
    }
}

struct ControlsSection: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Controls").font(.headline)
            HStack(spacing: 16) {
                Toggle("Sustain", isOn: Binding(get: { model.sustainDown }, set: { model.setSustain($0) }))
                    .toggleStyle(.button)
                Button("Panic") { model.panic() }
                    .buttonStyle(.bordered)
                Toggle("Soundboard", isOn: Binding(get: { model.boardOn }, set: { model.setBoard($0) }))
                    .fixedSize()
            }
            HStack {
                Text("Quality")
                Picker("Quality", selection: Binding(get: { model.tier }, set: { model.setTier($0) })) {
                    ForEach(QualityTier.allCases) { tier in
                        Text(tier.label).tag(tier)
                    }
                }
                .pickerStyle(.segmented)
                .frame(maxWidth: 320)
            }
            Text("Quality and soundboard changes re-prepare the engine (about a second of silence).")
                .font(.caption)
                .foregroundColor(.secondary)
        }
    }
}

struct CPUMeterView: View {
    // Not observed here: LoadRows observes the meters, so a changing load
    // re-renders only the rows, not the toggles and buttons.
    @EnvironmentObject var model: AppModel
    let meters: MeterStore
    let cpu: CpuStore
    @State private var copied = false

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text("CPU: render time / buffer period").font(.headline)
                Spacer()
                Button("Reset") { model.resetLoad() }
                    .buttonStyle(.bordered)
            }
            LoadRows(meters: meters)
            ProcessCpuLine(cpu: cpu)
                .font(.system(.caption, design: .monospaced))
            HStack(spacing: 16) {
                Toggle("Thread CPU time", isOn: Binding(get: { model.cpuTiming }, set: { model.setCpuTiming($0) }))
                    .fixedSize()
                Toggle("Phase timing", isOn: Binding(get: { model.phaseTiming }, set: { model.setPhaseTiming($0) }))
                    .fixedSize()
                Toggle("Speed probe", isOn: Binding(get: { model.speedProbe }, set: { model.setSpeedProbe($0) }))
                    .fixedSize()
                Button(copied ? "Copied" : "Copy diagnostics") {
                    // The full text (render times, phases, worst-buffer
                    // events) is built only here.
                    MonitorView.copyToClipboard(model.diagnosticsText())
                    copied = true
                }
                .buttonStyle(.bordered)
            }
            Text("Copy diagnostics has the render-time percentiles, the worst buffer's events and hammer work and, with Phase timing on, where the time went. Speed probe times a fixed workload on the audio thread every \(AppModel.speedProbeInterval) buffers; against the benchmark thread (run the benchmark once) it shows how fast the audio thread really runs.")
                .font(.caption)
                .foregroundColor(.secondary)
        }
    }
}

/// The load rows: the only part of the CPU section that observes the meters.
struct LoadRows: View {
    @ObservedObject var meters: MeterStore

    var body: some View {
        let m = meters.snapshot
        VStack(alignment: .leading, spacing: 6) {
            meterRow("Now", m.recentPct, budget: m.budgetPct)
            meterRow("Mean", m.meanPct, budget: m.budgetPct)
            meterRow("p99", m.p99Pct, budget: m.budgetPct)
            meterRow("Max", m.maxPct, budget: m.budgetPct)
            Text(verbatim: "Buffers over \(m.budgetPct) %: \(m.overBudget)")
                .font(.caption)
                .monospacedDigit()
            VStack(alignment: .leading, spacing: 2) {
                if !m.renderLine.isEmpty { Text(verbatim: m.renderLine) }
                Text(verbatim: m.worstLine)
            }
            .font(.system(.caption, design: .monospaced))
        }
    }

    private func meterRow(_ label: String, _ pct: Double, budget: Int) -> some View {
        let color: Color = pct > Double(budget) ? .red : (pct > 50 ? .orange : .green)
        return HStack {
            Text(verbatim: label).frame(width: 50, alignment: .leading)
            ProgressView(value: max(0.0, min(pct / 100.0, 1.0)))
                .tint(color)
            Text(verbatim: String(format: "%5.1f %%", pct))
                .monospacedDigit()
                .frame(width: 72, alignment: .trailing)
        }
    }
}

/// The one view that observes the once-a-second process-CPU line.
struct ProcessCpuLine: View {
    @ObservedObject var cpu: CpuStore

    var body: some View {
        Text(verbatim: cpu.line)
    }
}

struct KeyboardStrip: View {
    // Actions only (not observed); the keyboard position is the one thing
    // this strip observes.
    let model: AppModel
    @ObservedObject var keyboard: KeyboardStore

    var body: some View {
        let octaves = PlayView.keyboardOctaves
        let lowest = keyboard.lowestNote
        VStack(spacing: 4) {
            HStack {
                Button("Octave -") { model.shiftKeyboard(octaves: -1, shown: octaves) }
                Text(verbatim: MIDIMonitor.noteName(lowest) + " - " + MIDIMonitor.noteName(lowest + 12 * octaves))
                    .monospacedDigit()
                Button("Octave +") { model.shiftKeyboard(octaves: 1, shown: octaves) }
                Spacer()
                Text("Touch lower on a key to play louder")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            .padding(.horizontal)
            KeyboardView(lowestNote: lowest,
                         octaves: octaves,
                         onNoteOn: { [model] note, velocity in model.noteOn(note, velocity: velocity) },
                         onNoteOff: { [model] note in model.noteOff(note) })
                .frame(height: 180)
        }
        .padding(.bottom, 4)
    }
}
