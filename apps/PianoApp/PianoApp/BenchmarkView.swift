// Benchmark screen: runs the bench_engine scenarios (core/pm_bench) offline
// on this device and shows the table, with a Copy button. Audio output is
// paused for the run (about a minute) so that the numbers are the engine's
// alone; it resumes by itself afterwards. Paced runs sleep to real-time
// deadlines between buffers; "Run on audio thread" plays one scene through
// the real audio path, so the same material is measured three ways.
import Foundation
import SwiftUI

struct BenchmarkView: View {
    @EnvironmentObject var model: AppModel
    @State private var copied = false

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(spacing: 12) {
                Button(model.benchRunning ? "Running..." : "Run") {
                    copied = false
                    model.runBenchmark()
                }
                .buttonStyle(.borderedProminent)
                .disabled(model.benchRunning)
                Toggle("All tiers", isOn: $model.benchAllTiers)
                    .fixedSize()
                    .disabled(model.benchRunning)
                Toggle("512 frames at 44.1 kHz", isOn: $model.benchLiveIO)
                    .fixedSize()
                    .disabled(model.benchRunning)
                Toggle("Paced", isOn: $model.benchPaced)
                    .fixedSize()
                    .disabled(model.benchRunning)
                Toggle("Evict", isOn: $model.benchEvict)
                    .fixedSize()
                    .disabled(model.benchRunning)
                Toggle("Phases", isOn: $model.benchPhases)
                    .fixedSize()
                    .disabled(model.benchRunning || model.sceneRunning)
                Button(copied ? "Copied" : "Copy") {
                    MonitorView.copyToClipboard(model.benchmarkExport())
                    copied = true
                }
                .buttonStyle(.bordered)
                .disabled(model.benchLines.isEmpty)
                Spacer()
            }
            HStack(spacing: 12) {
                Picker("Scene", selection: $model.sceneItem) {
                    ForEach(Array(model.sceneNames.enumerated()), id: \.offset) { item in
                        Text(verbatim: item.element).tag(item.offset)
                    }
                }
                .fixedSize()
                .disabled(model.sceneRunning)
                Toggle("Benchmark engine", isOn: $model.sceneFreshEngine)
                    .fixedSize()
                    .disabled(model.sceneRunning)
                Button(model.sceneRunning ? "Playing..." : "Run on audio thread") {
                    model.runOnAudioThread()
                }
                .buttonStyle(.bordered)
                .disabled(model.benchRunning || model.sceneRunning)
                if model.sceneRunning {
                    Button("Cancel") { model.cancelScene() }
                        .buttonStyle(.bordered)
                }
                Spacer()
            }
            Text(verbatim: model.benchProgress)
                .foregroundColor(.secondary)
            Text(verbatim: "48 kHz, 64-frame buffers (or 512 frames at 44.1 kHz, as an iPad app on a Mac is granted), "
                 + "the selected quality tier; audio is paused during the run. "
                 + "Mean, p99 and max of the render time per buffer, in us and in % of that buffer's period; "
                 + "the live-playing scenes (legato, pedalling, chord release, idle) come after the others; "
                 + "then one hammer alone per key. Paced: the benchmark thread sleeps to each buffer's real-time "
                 + "deadline between buffers, as an audio thread does (Evict: and walks 32 MiB in between). "
                 + "Run on audio thread: the selected scene's events played through the real audio path, "
                 + "audible, on the app's engine (reset first) or a benchmark-prepared one; the row has the "
                 + "same statistics, phase split and counts (switch the speed probe on in Diagnostics for its line).")
                .font(.caption)
                .foregroundColor(.secondary)
            ScrollView([.vertical, .horizontal]) {
                VStack(alignment: .leading, spacing: 2) {
                    Text(verbatim: AppModel.benchColumns)
                        .foregroundColor(.secondary)
                    ForEach(Array(model.benchLines.enumerated()), id: \.offset) { item in
                        Text(verbatim: item.element)
                    }
                }
                .font(.system(.caption, design: .monospaced))
                .textSelection(.enabled)
                .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
        .padding()
    }
}
