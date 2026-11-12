// Play screen (M2 minimal UI): status, MIDI sources, controls, CPU meter
// and an on-screen keyboard strip.
import Foundation
import SwiftUI

struct PlayView: View {
    @EnvironmentObject var model: AppModel
    static let keyboardOctaves = 3

    var body: some View {
        VStack(spacing: 0) {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    StatusSection()
                    ControlsSection()
                    CPUMeterView()
                }
                .padding()
                .frame(maxWidth: .infinity, alignment: .leading)
            }
            KeyboardStrip()
        }
    }
}

struct StatusSection: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Status").font(.headline)
            row("Audio", model.audio.message + (model.audio.preparing ? " (preparing...)" : ""))
            row("Sample rate", model.audio.sampleRate > 0 ? String(format: "%.0f Hz", model.audio.sampleRate) : "-")
            row("I/O buffer", ioBufferText())
            row("Render buffer", model.renderFrames > 0 ? "\(model.renderFrames) frames" : "-")
            row("Output", model.audio.route)
            row("Engine", "\(model.awakeModes) modes awake, \(model.activeHammers) hammers")
            row("MIDI", "\(model.midiMessages) messages, \(model.droppedEvents) dropped events")
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
    @State private var copied = false

    var body: some View {
        let d = model.diag
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text("CPU: render time / buffer period").font(.headline)
                Spacer()
                Button("Reset") { model.resetLoad() }
                    .buttonStyle(.bordered)
            }
            meterRow("Now", model.recentLoad)
            meterRow("Mean", model.meanLoad)
            meterRow("p99", d.p99_load)
            meterRow("Max", model.maxLoad)
            Text(verbatim: "Buffers over \(Int((model.loadBudget * 100).rounded())) %: \(model.overBudget) of \(model.buffers)")
                .font(.caption)
                .monospacedDigit()
            VStack(alignment: .leading, spacing: 2) {
                Text(verbatim: "Render buffer: \(d.last_frames) frames = \(AppModel.us(d.last_period_us)); seen \(d.min_frames)...\(d.max_frames) frames, \(d.distinct_frame_counts) distinct size(s)")
                Text(verbatim: "Render time: mean \(AppModel.us(d.mean_us)) (\(AppModel.pct(d.mean_load))), p99 \(AppModel.us(d.p99_us)) (\(AppModel.pct(d.p99_load))), max \(AppModel.us(d.max_us)) (\(AppModel.pct(d.max_load)))")
                Text(verbatim: model.worstBufferText())
                if d.cpu_timing == 1 {
                    Text(verbatim: "Largest wall minus CPU time of a buffer (preemption): \(AppModel.us(d.max_stall_us))")
                }
            }
            .font(.system(.caption, design: .monospaced))
            HStack(spacing: 16) {
                Toggle("Thread CPU time", isOn: Binding(get: { model.cpuTiming }, set: { model.setCpuTiming($0) }))
                    .fixedSize()
                Button(copied ? "Copied" : "Copy diagnostics") {
                    MonitorView.copyToClipboard(model.diagnosticsText())
                    copied = true
                }
                .buttonStyle(.bordered)
            }
        }
    }

    private func meterRow(_ label: String, _ value: Double) -> some View {
        let color: Color = value > model.loadBudget ? .red : (value > 0.5 ? .orange : .green)
        return HStack {
            Text(verbatim: label).frame(width: 50, alignment: .leading)
            ProgressView(value: max(0.0, min(value, 1.0)))
                .tint(color)
            Text(String(format: "%5.1f %%", value * 100.0))
                .monospacedDigit()
                .frame(width: 72, alignment: .trailing)
        }
    }
}

struct KeyboardStrip: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        let octaves = PlayView.keyboardOctaves
        VStack(spacing: 4) {
            HStack {
                Button("Octave -") { model.shiftKeyboard(octaves: -1, shown: octaves) }
                Text(verbatim: MIDIMonitor.noteName(model.lowestNote) + " - " + MIDIMonitor.noteName(model.lowestNote + 12 * octaves))
                    .monospacedDigit()
                Button("Octave +") { model.shiftKeyboard(octaves: 1, shown: octaves) }
                Spacer()
                Text("Touch lower on a key to play louder")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            .padding(.horizontal)
            KeyboardView(lowestNote: model.lowestNote,
                         octaves: octaves,
                         onNoteOn: { note, velocity in model.noteOn(note, velocity: velocity) },
                         onNoteOff: { note in model.noteOff(note) })
                .frame(height: 180)
        }
        .padding(.bottom, 4)
    }
}
