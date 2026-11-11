// On-screen keyboard strip: `octaves` octaves from `lowestNote` (a C) plus the
// top C. Each key has its own zero-distance drag gesture, so several keys can
// be held with several fingers. Velocity comes from where the key is
// touched: near its top edge soft, near its front edge loud.
import Foundation
import SwiftUI

struct KeyboardView: View {
    let lowestNote: Int
    let octaves: Int
    let onNoteOn: (Int, Double) -> Void
    let onNoteOff: (Int) -> Void

    @State private var down: Set<Int> = []

    // Semitone offsets of the white keys in an octave, and of the black keys
    // with the index of the white key whose right edge they straddle.
    private static let whiteSemitones = [0, 2, 4, 5, 7, 9, 11]
    private static let blackSemitones = [1, 3, 6, 8, 10]
    private static let blackAfterWhite = [0, 1, 3, 4, 5]

    var body: some View {
        GeometryReader { geo in
            let whiteCount = octaves * 7 + 1
            let whiteWidth = geo.size.width / CGFloat(whiteCount)
            let height = geo.size.height
            ZStack(alignment: .topLeading) {
                ForEach(0..<whiteCount, id: \.self) { i in
                    keyView(note: whiteNote(i), black: false, width: whiteWidth, height: height)
                        .position(x: (CGFloat(i) + 0.5) * whiteWidth, y: height * 0.5)
                }
                ForEach(0..<(octaves * 5), id: \.self) { j in
                    keyView(note: blackNote(j), black: true, width: whiteWidth * 0.6, height: height * 0.62)
                        .position(x: blackCenter(j, whiteWidth: whiteWidth), y: height * 0.31)
                }
            }
        }
    }

    private func whiteNote(_ i: Int) -> Int {
        return lowestNote + (i / 7) * 12 + KeyboardView.whiteSemitones[i % 7]
    }

    private func blackNote(_ j: Int) -> Int {
        return lowestNote + (j / 5) * 12 + KeyboardView.blackSemitones[j % 5]
    }

    private func blackCenter(_ j: Int, whiteWidth: CGFloat) -> CGFloat {
        let whiteIndex = (j / 5) * 7 + KeyboardView.blackAfterWhite[j % 5]
        return CGFloat(whiteIndex + 1) * whiteWidth
    }

    private func keyView(note: Int, black: Bool, width: CGFloat, height: CGFloat) -> some View {
        let pressed = down.contains(note)
        let fill: Color
        if black {
            fill = pressed ? Color.gray : Color.black
        } else {
            fill = pressed ? Color.accentColor.opacity(0.4) : Color.white
        }
        return Rectangle()
            .fill(fill)
            .overlay(Rectangle().stroke(Color.black.opacity(0.7), lineWidth: 1))
            .frame(width: width, height: height)
            .contentShape(Rectangle())
            .gesture(
                DragGesture(minimumDistance: 0, coordinateSpace: .local)
                    .onChanged { value in
                        if !down.contains(note) {
                            down.insert(note)
                            let velocity = Double(value.startLocation.y / max(height, 1))
                            onNoteOn(note, velocity)
                        }
                    }
                    .onEnded { _ in
                        if down.contains(note) {
                            down.remove(note)
                            onNoteOff(note)
                        }
                    }
            )
    }
}
