// Benchmark screen: runs the bench_engine scenarios (core/pm_bench) offline
// on this device and shows the table, with a Copy button. Audio output is
// paused for the run (about a minute) so that the numbers are the engine's
// alone; it resumes by itself afterwards. Paced runs sleep to real-time
// deadlines between buffers (optionally on a time-constraint thread
// policy, without a workgroup deadline); "Run on audio thread" plays one scene, "Run all on audio thread"
// every scene in turn, through the real audio path, so the same material is
// measured three ways. "Load xK" multiplies the audio-thread work (K
// engines per buffer) and "Headroom sweep" runs one scene at increasing K.
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
                .disabled(model.benchRunning || model.sceneRunning || model.sweepRunning)
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
                Toggle("Real-time paced", isOn: $model.benchRealtime)
                    .fixedSize()
                    .disabled(model.benchRunning)
                Toggle("Phases", isOn: $model.benchPhases)
                    .fixedSize()
                    .disabled(model.benchRunning || model.sceneRunning || model.sweepRunning)
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
                Stepper("Load \u{00D7}\(model.sceneLoadK)", value: $model.sceneLoadK, in: 1...AppModel.maxLoad)
                    .fixedSize()
                    .disabled(model.sceneRunning)
                Button(model.sceneRunning ? "Playing..." : "Run on audio thread") {
                    model.runOnAudioThread()
                }
                .buttonStyle(.bordered)
                .disabled(model.benchRunning || model.sceneRunning || model.sweepRunning)
                Button("Run all on audio thread") {
                    model.runAllOnAudioThread()
                }
                .buttonStyle(.bordered)
                .disabled(model.benchRunning || model.sceneRunning || model.sweepRunning)
                Button(model.sweepRunning ? "Sweeping..." : "Headroom sweep") {
                    model.runHeadroomSweep()
                }
                .buttonStyle(.bordered)
                .disabled(model.benchRunning || model.sceneRunning || model.sweepRunning)
                if model.sceneRunning || model.sweepRunning {
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
                 + "same statistics, phase split and counts; the speed probe is on for the run. "
                 + "Load \u{00D7}K renders every buffer on K benchmark-prepared engines fed the same events (the first "
                 + "one heard; about 6 MB each, up to 8): K times the work per buffer. "
                 + "Run all on audio thread: every scene in turn, one row each (Cancel stops the series). "
                 + "Headroom sweep: the selected scene at load 1, 2, 3, 4, 6, 8 (stops after a load with more than "
                 + "20 % overruns), then a table against the full-speed prediction (load \u{00D7} the last back-to-back "
                 + "mean of the scene at the same buffer size): time growing load-fold from load 1 means no headroom. "
                 + "Scheduling classes differ: the benchmark runs on an ordinary thread (QoS user-interactive); "
                 + "Real-time paced puts its paced loop on a time-constraint (real-time) thread policy, but without "
                 + "a workgroup deadline (not available from Swift), so the system does not see per-buffer deadlines "
                 + "as it does for the audio I/O thread. Only the audio-thread rows (Run all on audio thread) reflect "
                 + "the I/O thread's scheduling and are what playing "
                 + "costs; on Apple silicon the system picks the core type and clock for the audio thread from its "
                 + "deadlines, so a low % of the buffer at light load is not headroom. Each row lists the CPUs the "
                 + "render calls ran on (core types assumed from the CPU numbers) and, with the speed probe on, its "
                 + "latency-bound and throughput-bound parts against the benchmark thread.")
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
