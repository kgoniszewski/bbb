// MIDI monitor screen: a scrolling log of every message received (source,
// group, channel, type, data, raw words, time since the previous message),
// with pause, clear, copy and share. Used for the M32 capture session.
import Foundation
import SwiftUI
#if canImport(UIKit)
import UIKit
#elseif canImport(AppKit)
import AppKit
#endif

struct MonitorView: View {
    // Not observed: this screen re-renders only when monitor lines arrive,
    // not with every meter update of the model.
    let model: AppModel
    @ObservedObject var store: MonitorStore
    @State private var copied = false

    var body: some View {
        VStack(spacing: 0) {
            HStack(spacing: 12) {
                Toggle("Pause", isOn: $store.paused)
                    .toggleStyle(.button)
                Button("Marker") {
                    model.addMonitorMarker()
                }
                Button("Clear") {
                    model.clearMonitor()
                }
                Button(copied ? "Copied" : "Copy") {
                    MonitorView.copyToClipboard(model.monitorExport())
                    copied = true
                }
                ShareLink(item: model.monitorExport()) {
                    Label("Share", systemImage: "square.and.arrow.up")
                }
                Spacer()
                Text(verbatim: "\(store.lines.count) lines")
                    .foregroundColor(.secondary)
            }
            .padding()
            Text(verbatim: "delta ms | source | group | channel | type | data | raw")
                .font(.system(.caption, design: .monospaced))
                .foregroundColor(.secondary)
                .frame(maxWidth: .infinity, alignment: .leading)
                .padding(.horizontal)
            ScrollViewReader { proxy in
                ScrollView {
                    LazyVStack(alignment: .leading, spacing: 2) {
                        ForEach(store.lines) { line in
                            Text(verbatim: line.text.replacingOccurrences(of: "\t", with: "  "))
                                .font(.system(.caption, design: .monospaced))
                                .id(line.id)
                        }
                    }
                    .padding(.horizontal)
                }
                .onChange(of: store.lines.count) { _, _ in
                    copied = false
                    if !store.paused, let last = store.lines.last {
                        proxy.scrollTo(last.id, anchor: .bottom)
                    }
                }
            }
        }
    }

    static func copyToClipboard(_ text: String) {
        #if canImport(UIKit)
        UIPasteboard.general.string = text
        #elseif canImport(AppKit)
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(text, forType: .string)
        #endif
    }
}
