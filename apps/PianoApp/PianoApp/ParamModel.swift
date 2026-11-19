// Parameter descriptors from the engine's table (pm_param_info in
// core/pm_api/pm_api.h): ranges, units, curves and classes come from there,
// never copied into the UI.
import Foundation
import PianoCore

struct ParamDescriptor: Identifiable {
    let id: Int32
    let key: String
    let name: String
    let unit: String
    let min: Double
    let max: Double
    let defaultValue: Double
    let curve: Int32
    let cls: Int32
    let flags: Int32
    let enumNames: [String]

    var param: pm_param { pm_param(rawValue: numericCast(id)) }
    var isHidden: Bool { flags & Int32(PM_PARAM_FLAG_HIDDEN.rawValue) != 0 }
    var isPrepareClass: Bool { cls == Int32(PM_PARAM_CLASS_PREPARE.rawValue) }
    var isBoolean: Bool { curve == Int32(PM_PARAM_CURVE_BOOLEAN.rawValue) }
    var isInteger: Bool { curve == Int32(PM_PARAM_CURVE_INTEGER.rawValue) }
    var isLog: Bool { curve == Int32(PM_PARAM_CURVE_LOG.rawValue) && min > 0 }

    init(_ info: pm_param_info_t) {
        id = info.id
        key = info.key.map { String(cString: $0) } ?? ""
        name = info.name.map { String(cString: $0) } ?? ""
        unit = info.unit.map { String(cString: $0) } ?? ""
        min = info.min
        max = info.max
        defaultValue = info.default_value
        curve = info.curve
        cls = info.cls
        flags = info.flags
        var names: [String] = []
        if let list = info.enum_names, info.enum_count > 0 {
            for i in 0..<Int(info.enum_count) {
                names.append(list[i].map { String(cString: $0).capitalized } ?? "")
            }
        }
        enumNames = names
    }

    /// Every parameter, in table order.
    static let all: [ParamDescriptor] = {
        let n = Int(pm_param_count())
        var list: [ParamDescriptor] = []
        for i in 0..<n {
            var info = pm_param_info_t()
            if pm_param_info(Int32(i), &info) == 1 { list.append(ParamDescriptor(info)) }
        }
        return list
    }()

    static func byKey(_ key: String) -> ParamDescriptor? { all.first { $0.key == key } }

    /// The value as shown: enum name, On/Off, or number with unit.
    func display(_ v: Double) -> String {
        if isBoolean { return v >= 0.5 ? "On" : "Off" }
        if isInteger {
            let i = Int(v.rounded()) - Int(min.rounded())
            if i >= 0, i < enumNames.count { return enumNames[i] }
            return String(Int(v.rounded()))
        }
        let a = abs(v)
        let text: String
        if a >= 100 { text = String(format: "%.0f", v) } else if a >= 10 { text = String(format: "%.1f", v) } else { text = String(format: "%.2f", v) }
        return unit.isEmpty ? text : "\(text) \(unit)"
    }
}

/// The values of the parameters shown on the Sound tab, read from the
/// engine. Observed only by that tab, so the Play tab's meters do not
/// re-render with it.
final class ParamValues: ObservableObject {
    @Published var values: [Int32: Double] = [:]

    func value(_ d: ParamDescriptor) -> Double { values[d.id] ?? d.defaultValue }

    func refresh(from host: EngineHost) {
        var v: [Int32: Double] = [:]
        for d in ParamDescriptor.all { v[d.id] = host.param(d.param) }
        values = v
    }
}

/// A C char array imported as a tuple (pm_preset_result_t.error, .name, ...)
/// as a String, up to the first NUL.
func stringFromCTuple<T>(_ tuple: T) -> String {
    var copy = tuple
    return withUnsafeBytes(of: &copy) { raw in
        String(decoding: raw.prefix(while: { $0 != 0 }), as: UTF8.self)
    }
}
