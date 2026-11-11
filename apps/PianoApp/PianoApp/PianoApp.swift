// App entry. Audio and MIDI start when the first window appears and keep
// running in the background (UIBackgroundModes: audio).
import SwiftUI

@main
struct PianoApp: App {
    @StateObject private var model = AppModel()

    var body: some Scene {
        WindowGroup {
            ContentView(model: model)
                .environmentObject(model)
                .onAppear {
                    model.start()
                }
        }
    }
}

struct ContentView: View {
    // Not observed here (the views that show model state observe it).
    let model: AppModel

    var body: some View {
        TabView {
            PlayView()
                .tabItem {
                    Label("Play", systemImage: "pianokeys")
                }
            MonitorView(model: model, store: model.monitorStore)
                .tabItem {
                    Label("MIDI Monitor", systemImage: "list.bullet.rectangle")
                }
        }
    }
}
