// App entry. Audio and MIDI start when the first window appears and keep
// running in the background (UIBackgroundModes: audio).
import SwiftUI

@main
struct PianoApp: App {
    @StateObject private var model = AppModel()
    @Environment(\.scenePhase) private var scenePhase

    var body: some Scene {
        WindowGroup {
            ContentView(model: model)
                .environmentObject(model)
                .onAppear {
                    model.start()
                }
                // Meter and monitor polling only while the app is active.
                .onChange(of: scenePhase) { _, phase in
                    model.setAppActive(phase == .active)
                }
        }
    }
}

struct ContentView: View {
    // Not observed here (the views that show model state observe it).
    let model: AppModel

    var body: some View {
        TabView {
            PlayView(model: model)
                .tabItem {
                    Label("Play", systemImage: "pianokeys")
                }
            MonitorView(model: model, store: model.monitorStore)
                .tabItem {
                    Label("MIDI Monitor", systemImage: "list.bullet.rectangle")
                }
            BenchmarkView()
                .tabItem {
                    Label("Benchmark", systemImage: "speedometer")
                }
        }
    }
}
