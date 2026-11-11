// swift-tools-version:5.9
// Swift package exposing the C++ engine to Swift through its C API
// (core/pm_api/pm_api.h, module PianoCore). The CMake build (core/, tests/,
// tools/) is independent of this file. The app in apps/PianoApp uses this
// package as a local dependency.
import PackageDescription

let package = Package(
    name: "PianoEngine",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "PianoCore", targets: ["PianoCore"]),
    ],
    targets: [
        .target(
            name: "PianoCore",
            path: "core",
            exclude: [
                "pm_base/CMakeLists.txt",
                "pm_string/CMakeLists.txt",
                "pm_hammer/CMakeLists.txt",
                "pm_board/CMakeLists.txt",
                "pm_voicing/CMakeLists.txt",
                "pm_engine/CMakeLists.txt",
                "pm_host/CMakeLists.txt",
            ],
            sources: [
                "pm_string",
                "pm_hammer",
                "pm_board",
                "pm_voicing",
                "pm_engine",
                "pm_host",
                "pm_api/pm_api.cpp",
                "pm_api/pm_host_api.cpp",
            ],
            publicHeadersPath: "pm_api/include",
            cxxSettings: [
                // Sources include each other as "pm_xxx/file.h" relative to core/.
                .headerSearchPath("."),
                .define("NDEBUG"),
                // CLAUDE.md: real-time code without exceptions or RTTI. -O3 also
                // in Debug: the engine is far too slow unoptimised to play.
                .unsafeFlags(["-O3", "-fno-exceptions", "-fno-rtti"]),
            ]
        ),
        .testTarget(
            name: "PianoCoreTests",
            dependencies: ["PianoCore"],
            path: "apps/PianoApp/Tests/PianoCoreTests"
        ),
    ],
    cxxLanguageStandard: .cxx20
)
