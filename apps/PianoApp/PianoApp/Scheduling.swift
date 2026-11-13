// Core layout and scheduling helpers for the diagnostics and the benchmark.
//
// CoreLayout: the CPU's performance levels from sysctl (hw.nperflevels,
// hw.perflevelN.name / .logicalcpu; perflevel0 is the fastest) and a label
// per logical CPU number. Which CPU numbers belong to which level is NOT
// published by the system for apps: the labels assume that the lowest
// performance level has the lowest CPU numbers (true of every Apple silicon
// Mac and iPad seen so far: M1's CPUs 0-3 are its efficiency cores). The
// speed probe's per-CPU times, printed next to them, check the assumption
// empirically (an efficiency core is several times slower on the
// throughput-bound part).
//
// PacedRealtime: puts the benchmark's paced loop into the scheduling class
// of an audio I/O thread (pm_bench_set_pace_hook): Mach time-constraint
// policy with the buffer period, and an audio work interval
// (AudioWorkIntervalCreate) joined by the thread, started with each buffer's
// deadline and finished after its render call, so that the performance
// controller sees deadlines as it does for the real I/O thread. Only for the
// paced loop: the engine's prepare (about a second of CPU) and back-to-back
// runs stay at the thread's normal QoS, since a real-time thread that
// computes far beyond its declared computation time is demoted by the
// kernel.
import AudioToolbox
import Darwin
import Foundation
import PianoCore
import os

struct CoreLayout {
    struct Level {
        let name: String     // "Performance", "Efficiency" (hw.perflevelN.name)
        let logical: Int     // logical CPUs at this level
        var letter: String { name.isEmpty ? "?" : String(name.prefix(1)) }
    }

    let levels: [Level]      // perflevel0 (fastest) first
    let ncpu: Int

    static func read() -> CoreLayout {
        let ncpu = sysctlInt("hw.ncpu") ?? ProcessInfo.processInfo.activeProcessorCount
        var levels: [Level] = []
        let n = sysctlInt("hw.nperflevels") ?? 0
        for i in 0..<n {
            let name = sysctlString("hw.perflevel\(i).name") ?? "level \(i)"
            let logical = sysctlInt("hw.perflevel\(i).logicalcpu") ?? 0
            levels.append(Level(name: name, logical: logical))
        }
        return CoreLayout(levels: levels, ncpu: ncpu)
    }

    /// The level of a CPU number under the assumption above (nil: unknown).
    func level(of cpu: Int) -> Level? {
        guard cpu >= 0, !levels.isEmpty else { return nil }
        var first = 0
        for lv in levels.reversed() {  // slowest level first
            if cpu < first + lv.logical { return lv }
            first += lv.logical
        }
        return nil
    }

    /// "E0", "P6": the assumed level's initial and the CPU number.
    func label(_ cpu: Int) -> String {
        guard cpu >= 0 else { return "?" }
        return (level(of: cpu)?.letter ?? "cpu") + "\(cpu)"
    }

    var description: String {
        guard !levels.isEmpty else { return "\(ncpu) CPUs, no performance levels reported" }
        var parts: [String] = []
        var first = 0
        for lv in levels.reversed() {
            let last = first + lv.logical - 1
            parts.append("\(lv.name) \(lv.logical) (CPUs \(first)...\(last), assumed)")
            first += lv.logical
        }
        return "\(ncpu) CPUs: " + parts.joined(separator: ", ")
    }

    static func sysctlInt(_ name: String) -> Int? {
        var v: Int32 = 0
        var size = MemoryLayout<Int32>.size
        return sysctlbyname(name, &v, &size, nil, 0) == 0 ? Int(v) : nil
    }

    static func sysctlString(_ name: String) -> String? {
        var size = 0
        guard sysctlbyname(name, nil, &size, nil, 0) == 0, size > 0 else { return nil }
        var buf = [CChar](repeating: 0, count: size)
        guard sysctlbyname(name, &buf, &size, nil, 0) == 0 else { return nil }
        return String(cString: buf)
    }

    // MARK: - Text of a pm_cpu_usage

    /// Lines describing where the render calls ran: share and mean render
    /// time per CPU and per assumed core type, mid-buffer migrations, the
    /// worst buffer's CPUs, and the speed probe's parts per CPU.
    func usageLines(_ u: pm_cpu_usage, indent: String = "    ") -> [String] {
        var usage = u
        let buffers: [UInt32] = CoreLayout.elements(&usage.buffers)
        let meanUs: [Double] = CoreLayout.elements(&usage.mean_render_us)
        let probeRuns: [UInt32] = CoreLayout.elements(&usage.probe_runs)
        let probeLat: [Double] = CoreLayout.elements(&usage.probe_latency_us)
        let probeThr: [Double] = CoreLayout.elements(&usage.probe_throughput_us)
        let total = Double(u.buffers_tracked)
        guard u.buffers_tracked > 0 else {
            return [indent + "render calls by CPU: not recorded (\(u.buffers_unknown) calls without a CPU number; "
                    + "switch on phase timing, thread CPU time or the speed probe)"]
        }
        var out: [String] = []
        var perCpu: [String] = []
        var byLevel: [String: (n: Double, us: Double)] = [:]
        for (cpu, n) in buffers.enumerated() where n > 0 {
            perCpu.append(String(format: "%@ %.0f %%", label(cpu), 100.0 * Double(n) / total))
            let key = level(of: cpu)?.letter ?? "?"
            let acc = byLevel[key] ?? (0, 0)
            byLevel[key] = (acc.n + Double(n), acc.us + Double(n) * meanUs[cpu])
        }
        var line = indent + "render calls by CPU (core types assumed: lowest CPU numbers = slowest cores): "
        line += perCpu.joined(separator: ", ")
        if u.buffers_unknown > 0 { line += "; \(u.buffers_unknown) without a CPU number" }
        line += String(format: "; %llu mid-buffer migrations (%.1f %%)", u.migrations, 100.0 * Double(u.migrations) / total)
        if u.worst_cpu_start >= 0 {
            line += "; worst buffer on \(label(Int(u.worst_cpu_start))) -> \(label(Int(u.worst_cpu_end)))"
        }
        out.append(line)
        let typeText = byLevel.keys.sorted().map { k -> String in
            let v = byLevel[k]!
            return String(format: "%@ cores %.0f (%.0f %% of calls)", k, v.us / max(v.n, 1), 100.0 * v.n / total)
        }
        var means = indent + "mean render us: " + typeText.joined(separator: " vs ")
        means += "; by CPU: " + buffers.enumerated().filter { $0.element > 0 }
            .map { String(format: "%@ %.0f", label($0.offset), meanUs[$0.offset]) }.joined(separator: ", ")
        out.append(means)
        let probed = probeRuns.enumerated().filter { $0.element > 0 }
        if !probed.isEmpty {
            out.append(indent + "speed probe by CPU, latency / throughput part us (an efficiency core or a low clock "
                       + "slows the throughput part far more): "
                       + probed.map { String(format: "%@ %.1f / %.1f (%u)", label($0.offset), probeLat[$0.offset],
                                             probeThr[$0.offset], $0.element) }.joined(separator: ", "))
        }
        return out
    }

    /// The elements of a C fixed-size array (imported as a tuple).
    static func elements<T, E>(_ tuple: inout T) -> [E] {
        let count = MemoryLayout<T>.size / MemoryLayout<E>.stride
        return withUnsafeBytes(of: &tuple) { raw in
            Array(UnsafeBufferPointer(start: raw.baseAddress!.assumingMemoryBound(to: E.self), count: count))
        }
    }
}

/// The speed probe's two parts against a reference (the benchmark thread's,
/// measured back to back): "x / y us -> a / b x".
func probeSplitText(_ s: pm_speed_probe_split, reference r: pm_speed_probe_split?) -> String {
    var out = String(format: "latency-bound part median %.2f us (mean %.2f), throughput-bound part median %.2f us (mean %.2f)",
                     s.latency.median_us, s.latency.mean_us, s.throughput.median_us, s.throughput.mean_us)
    if let r = r, s.latency.median_us > 0, s.throughput.median_us > 0, r.latency.runs > 0 {
        out += String(format: "; vs the benchmark thread back to back (%.2f / %.2f us): latency %.2fx, throughput %.2fx speed",
                      r.latency.median_us, r.throughput.median_us,
                      r.latency.median_us / s.latency.median_us, r.throughput.median_us / s.throughput.median_us)
    }
    return out
}

/// Runs the benchmark's paced loop in an audio thread's scheduling class
/// (see the file comment). Install on the benchmark thread before running
/// items; every paced item then calls back here. Not thread-safe: one
/// benchmark thread.
final class PacedRealtime {
    /// What happened, for the benchmark header (main thread reads it after the run).
    private(set) var notes: [String] = []
    private var interval: os_workgroup_interval_t?
    private var token = os_workgroup_join_token_s()
    private var joined = false
    private var realtime = false
    private var startFailures = 0
    private let numer: UInt32
    private let denom: UInt32

    init() {
        var tb = mach_timebase_info_data_t()
        mach_timebase_info(&tb)
        numer = tb.numer == 0 ? 1 : tb.numer
        denom = tb.denom == 0 ? 1 : tb.denom
    }

    func install() {
        pm_bench_set_pace_hook(PacedRealtime.hook, Unmanaged.passUnretained(self).toOpaque())
    }

    func uninstall() {
        pm_bench_set_pace_hook(nil, nil)
    }

    private static let hook: pm_pace_hook = { ctx, event, ns in
        guard let ctx = ctx else { return }
        Unmanaged<PacedRealtime>.fromOpaque(ctx).takeUnretainedValue().handle(event: event, ns: ns)
    }

    private func ticks(ns: Double) -> UInt64 {
        return UInt64(max(0, ns) * Double(denom) / Double(numer))
    }

    private func handle(event: Int32, ns: Int64) {
        switch event {
        case Int32(PM_PACE_BEGIN.rawValue):
            begin(periodNs: ns)
        case Int32(PM_PACE_BUFFER.rawValue):
            if joined, let wg = interval {
                let now = mach_absolute_time()
                if os_workgroup_interval_start(wg, now, now + ticks(ns: Double(ns)), nil) != 0 { startFailures += 1 }
            }
        case Int32(PM_PACE_BUFFER_DONE.rawValue):
            if joined, let wg = interval { _ = os_workgroup_interval_finish(wg, nil) }
        case Int32(PM_PACE_END.rawValue):
            end()
        default:
            break
        }
    }

    private func begin(periodNs: Int64) {
        startFailures = 0
        let period = Double(periodNs)
        // Computation half the period, constraint the period: the shape of
        // an audio I/O thread's policy.
        var policy = thread_time_constraint_policy_data_t(
            period: UInt32(clamping: ticks(ns: period)),
            computation: UInt32(clamping: ticks(ns: 0.5 * period)),
            constraint: UInt32(clamping: ticks(ns: period)),
            preemptible: 1)
        let count = mach_msg_type_number_t(MemoryLayout<thread_time_constraint_policy_data_t>.size / MemoryLayout<integer_t>.size)
        let kr = withUnsafeMutablePointer(to: &policy) { p in
            p.withMemoryRebound(to: integer_t.self, capacity: Int(count)) { q in
                thread_policy_set(pthread_mach_thread_np(pthread_self()), thread_policy_flavor_t(THREAD_TIME_CONSTRAINT_POLICY), q, count)
            }
        }
        realtime = kr == KERN_SUCCESS
        if !realtime { note("time-constraint policy refused (kern_return \(kr))") }
        if interval == nil {
            let wg: os_workgroup_interval_t? = AudioWorkIntervalCreate("piano.benchmark.paced", OS_CLOCK_MACH_ABSOLUTE_TIME, nil)
            interval = wg
            if wg == nil { note("AudioWorkIntervalCreate returned nil") }
        }
        if let wg = interval {
            let err = os_workgroup_join(wg, &token)
            joined = err == 0
            if !joined { note("joining the audio work interval failed (errno \(err))") }
        }
        if realtime && joined { note("paced loop on a time-constraint thread in its own audio work interval") }
    }

    private func end() {
        if joined, let wg = interval {
            os_workgroup_leave(wg, &token)
            joined = false
        }
        if startFailures > 0 { note("\(startFailures) work-interval starts failed") }
        if realtime {
            var none: integer_t = 0
            let kr = thread_policy_set(pthread_mach_thread_np(pthread_self()), thread_policy_flavor_t(THREAD_STANDARD_POLICY), &none, 0)
            if kr != KERN_SUCCESS { note("returning to the standard policy failed (kern_return \(kr))") }
            realtime = false
        }
    }

    private func note(_ s: String) {
        if !notes.contains(s) { notes.append(s) }
    }
}
