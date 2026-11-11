// Smoke test of the Swift view of the C API: `swift test` at the repository
// root (macOS or Linux). The full C++ test suite runs under CTest.
import XCTest
import PianoCore

final class PianoCoreTests: XCTestCase {
    func testVersion() {
        let v = pm_api_version()
        XCTAssertEqual(v >> 16, UInt32(PM_API_VERSION_MAJOR))
        XCTAssertEqual((v >> 8) & 0xFF, UInt32(PM_API_VERSION_MINOR))
        XCTAssertEqual(String(cString: pm_version_string()).isEmpty, false)
    }

    func testHostPlaysMidi() {
        guard let host = pm_host_create() else {
            XCTFail("pm_host_create failed")
            return
        }
        defer { pm_host_destroy(host) }
        pm_host_set_timebase(host, 1, 1)
        // Without the soundboard the engine prepares faster.
        XCTAssertEqual(pm_engine_set_param(pm_host_engine(host), Int32(PM_PARAM_SOUNDBOARD_ENABLED.rawValue), 0.0), 1)
        XCTAssertEqual(pm_host_prepare(host, 48000.0, 256), 1)

        let noteOn: [UInt8] = [0x90, 60, 100]
        noteOn.withUnsafeBufferPointer { buf in
            pm_host_midi_bytes(host, buf.baseAddress, Int32(buf.count), 0, 1)
        }
        var entry = pm_midi_monitor_entry()
        XCTAssertEqual(pm_host_monitor_pop(host, &entry), 1)
        XCTAssertEqual(entry.kind, UInt8(PM_MIDI_NOTE_ON.rawValue))
        XCTAssertEqual(entry.data1, 60)

        var left = [Float](repeating: 0, count: 256)
        var right = [Float](repeating: 0, count: 256)
        var peak: Float = 0
        for _ in 0..<20 {
            left.withUnsafeMutableBufferPointer { l in
                right.withUnsafeMutableBufferPointer { r in
                    pm_host_render(host, l.baseAddress, r.baseAddress, 256, 0)
                }
            }
            for x in left { peak = max(peak, abs(x)) }
        }
        XCTAssertGreaterThan(peak, 0.001)

        var load = pm_host_load()
        pm_host_get_load(host, &load)
        XCTAssertEqual(load.buffers, 20)
        XCTAssertEqual(load.last_frames, 256)
    }
}
