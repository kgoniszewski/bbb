#!/usr/bin/env python3
"""Spectral centroid of the attack and partial levels of single piano notes.

    analyze_note.py notes.wav --note 60 [--at 0,2.5,5] [--partials 12]

For each segment starting at an --at time (default: one segment at 0), the
onset is the first sample within 0.2 s whose magnitude exceeds 1 % of the
segment peak. Reported per segment:

  * spectral centroid (power-weighted mean frequency, 20 Hz .. fs/2) of a
    Hann-windowed 50 ms frame starting at the onset;
  * peak level, and the level of the first partials (dBFS, sinusoid
    amplitude) from a Blackman-Harris frame of max(0.1 s, 8 / f1) starting
    10 ms after the onset. Partial n is searched within +-f1/4 of
    n f1 sqrt((1 + B n^2) / (1 + B)), with B refined from the partials found.

Stereo files are summed to mono. Segments in one file give a centroid-vs-
velocity table directly (see tools/render_cli/scores/single_notes.txt).
Requires numpy only.
"""

import argparse
import sys

import numpy as np

from analyze_string import blackman_harris, read_wav_channels


def onset_index(x, start, fs):
    seg = x[start:start + int(0.2 * fs)]
    if len(seg) == 0 or np.max(np.abs(seg)) == 0.0:
        return None
    return start + int(np.argmax(np.abs(seg) >= 0.01 * np.max(np.abs(seg))))


def centroid(frame, fs):
    w = np.hanning(len(frame))
    spec = np.abs(np.fft.rfft(frame * w, n=1 << 15)) ** 2
    f = np.fft.rfftfreq(1 << 15, 1.0 / fs)
    keep = f >= 20.0
    return float(np.sum(f[keep] * spec[keep]) / np.sum(spec[keep]))


def partial_levels(frame, fs, f1, count, b_init):
    n_fft = 1 << int(np.ceil(np.log2(len(frame) * 8)))
    w = blackman_harris(len(frame))
    spec = np.abs(np.fft.rfft(frame * w, n=n_fft)) * 2.0 / np.sum(w)
    f = np.fft.rfftfreq(n_fft, 1.0 / fs)
    b = b_init
    levels = []
    for n in range(1, count + 1):
        fe = n * f1 * np.sqrt((1.0 + b * n * n) / (1.0 + b))
        if fe + f1 / 4 >= fs / 2:
            break
        sel = (f >= fe - f1 / 4) & (f <= fe + f1 / 4)
        i = np.flatnonzero(sel)[np.argmax(spec[sel])]
        levels.append((n, f[i], 20.0 * np.log10(spec[i] + 1e-30)))
        if n >= 2:
            r = (f[i] / (n * levels[0][1])) ** 2
            if n * n - r > 0:
                b = max(0.0, (r - 1.0) / (n * n - r))
    return levels


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("wav")
    ap.add_argument("--note", type=int, help="MIDI note number (equal temperament, A4 = 440 Hz)")
    ap.add_argument("--f1", type=float, help="first partial, Hz (overrides --note)")
    ap.add_argument("--B", type=float, default=0.0, help="initial inharmonicity estimate")
    ap.add_argument("--at", default="0", help="comma-separated segment start times, s")
    ap.add_argument("--partials", type=int, default=12)
    args = ap.parse_args(argv)
    if args.f1 is None and args.note is None:
        ap.error("give --note or --f1")
    f1 = args.f1 if args.f1 is not None else 440.0 * 2.0 ** ((args.note - 69) / 12.0)

    chans, fs = read_wav_channels(args.wav)
    x = chans.sum(axis=1)
    starts = [float(s) for s in args.at.split(",")]
    print(f"{args.wav}: {len(x) / fs:.2f} s, {fs} Hz, {chans.shape[1]} channel(s), f1 {f1:.2f} Hz")
    rows = []
    for k, t in enumerate(starts):
        start = int(t * fs)
        on = onset_index(x, start, fs)
        if on is None:
            print(f"  segment at {t:.3f} s: silent")
            continue
        end = int(starts[k + 1] * fs) if k + 1 < len(starts) else len(x)
        c = centroid(x[on:on + int(0.05 * fs)], fs)
        peak = 20.0 * np.log10(np.max(np.abs(x[on:end])) + 1e-30)
        a = on + int(0.01 * fs)
        frame = x[a:a + int(max(0.1, 8.0 / f1) * fs)]
        lv = partial_levels(frame, fs, f1, args.partials, args.B)
        rows.append((t, on / fs, peak, c, lv))
        print(f"  segment at {t:.3f} s (onset {on / fs:.4f} s): peak {peak:6.1f} dBFS, "
              f"centroid (first 50 ms) {c:7.1f} Hz")
        print("    partial  " + " ".join(f"{n:>6d}" for n, _, _ in lv))
        print("    Hz       " + " ".join(f"{fr:6.0f}" for _, fr, _ in lv))
        print("    dBFS     " + " ".join(f"{db:6.1f}" for _, _, db in lv))
    if len(rows) > 1:
        print("  centroid table:  " + "  ".join(f"{r[0]:.1f}s:{r[3]:.0f}Hz" for r in rows))
    return 0


if __name__ == "__main__":
    sys.exit(main())
