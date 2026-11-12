// Play screen (M2 minimal UI): status, MIDI sources, controls, CPU meter
// and an on-screen keyboard strip.
//
// Observation (see AppModel): this view and the keyboard strip observe
// nothing that changes while the engine idles. The status and meter
// sections observe MeterStore (published only when a displayed value
// changes) and AppModel (published only on user actions and audio or MIDI
// configuration changes); the process-CPU line is a leaf of its own.
import Foundation
import SwiftUI

struct PlayView: View {
    // Not observed: the sections below observe what they show.
    let model: AppModel
    static let keyboardOctaves = 3

    var body: some View {
        VStack(spacing: 0) {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    StatusSection(meters: model.meters)
                    ControlsSection()
                    CPUMeterView(meters: model.meters, cpu: model.cpuStore)
                }
                .padding()
                .frame(maxWidth: .infinity, alignment: .leading)
            }
            KeyboardStrip(model: model, keyboard: model.keyboard)
        }
        // Meters are polled only while this tab is shown.
        .onAppear { model.setPlayVisible(true) }
        .onDisappear { model.setPlayVisible(false) }
    }
}

struct StatusSection: View {
    @EnvironmentObject var model: AppModel
    @ObservedObject var meters: MeterStore

    var body: some View {
        let m = meters.snapshot
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
    @EnvironmentObject var model: AppModel
    @ObservedObject var meters: MeterStore
    let cpu: CpuStore
    @State private var copied = false

    var body: some View {
        let m = meters.snapshot
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text("CPU: render time / buffer period").font(.headline)
                Spacer()
                Button("Reset") { model.resetLoad() }
                    .buttonStyle(.bordered)
            }
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
                ProcessCpuLine(cpu: cpu)
            }
            .font(.system(.caption, design: .monospaced))
            HStack(spacing: 16) {
                Toggle("Thread CPU time", isOn: Binding(get: { model.cpuTiming }, set: { model.setCpuTiming($0) }))
                    .fixedSize()
                Toggle("Phase timing", isOn: Binding(get: { model.phaseTiming }, set: { model.setPhaseTiming($0) }))
                    .fixedSize()
                Button(copied ? "Copied" : "Copy diagnostics") {
                    // The full text (render times, phases, worst-buffer
                    // events) is built only here.
                    MonitorView.copyToClipboard(model.diagnosticsText())
                    copied = true
                }
                .buttonStyle(.bordered)
            }
            Text("Copy diagnostics has the render-time percentiles, the worst buffer's events and, with Phase timing on, where the time went.")
                .font(.caption)
                .foregroundColor(.secondary)
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
