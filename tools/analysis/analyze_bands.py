#!/usr/bin/env python3
"""Third-octave band levels and band decay after a time point: pedal bloom,
sympathetic resonance and half-pedal release comparisons.

    analyze_bands.py render.wav --at 0.4,5.2 [--start 0.3] [--window 0.7]
                     [--late 0.7] [--fmin 63] [--fmax 4000] [--labels down,up]

For each segment start t in --at (typically a chord release time):

  * band levels (dB, Hann-windowed power summed over the band's FFT bins)
    over [t + start, t + start + window), third-octave centres from --fmin
    to --fmax;
  * the same over the next window [t + start + window, + window) and from
    the two a band decay time T60 = 60 * window / (level drop in dB)
    ('-' where the level does not fall).

The first segment is the reference: the table also lists every other
segment's levels relative to it. Stereo files are summed to mono. Requires
numpy only.

    analyze_bands.py pedal_bloom.wav --at 0.4,5.2 --labels "pedal down,pedal up"
"""

import argparse
import sys

import numpy as np

from analyze_string import read_wav_channels


def third_octave_centres(fmin, fmax):
    centres = []
    k = int(np.floor(10 * np.log10(fmin / 1000.0)))
    while True:
        fc = 1000.0 * 10 ** (k / 10.0)
        if fc > fmax * 1.01:
            break
        if fc >= fmin * 0.99:
            centres.append(fc)
        k += 1
    return centres


def band_levels(x, fs, t0, length, centres):
    a = int(round(t0 * fs))
    n = int(round(length * fs))
    seg = np.zeros(n)
    avail = max(0, min(n, len(x) - a))
    seg[:avail] = x[a:a + avail]
    spec = np.abs(np.fft.rfft(seg * np.hanning(n))) ** 2 / n
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    out = []
    for fc in centres:
        lo, hi = fc / 2 ** (1 / 6), fc * 2 ** (1 / 6)
        e = spec[(freqs >= lo) & (freqs < hi)].sum()
        out.append(10 * np.log10(e + 1e-30))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav")
    ap.add_argument("--at", required=True, help="segment start times, s (comma separated)")
    ap.add_argument("--start", type=float, default=0.3, help="analysis starts this long after each time, s")
    ap.add_argument("--window", type=float, default=0.7, help="window length, s")
    ap.add_argument("--fmin", type=float, default=63.0)
    ap.add_argument("--fmax", type=float, default=4000.0)
    ap.add_argument("--labels", default="", help="names of the segments (comma separated)")
    args = ap.parse_args()

    fs, chans = read_wav_channels(args.wav)
    x = np.sum(chans, axis=0)
    times = [float(t) for t in args.at.split(",")]
    labels = args.labels.split(",") if args.labels else ["t=%.2f s" % t for t in times]
    if len(labels) != len(times):
        sys.exit("--labels needs one name per --at time")
    centres = third_octave_centres(args.fmin, args.fmax)

    levels, decays = [], []
    for t in times:
        e1 = band_levels(x, fs, t + args.start, args.window, centres)
        e2 = band_levels(x, fs, t + args.start + args.window, args.window, centres)
        levels.append(e1)
        drop = e1 - e2
        decays.append(np.where(drop > 0.1, 60.0 * args.window / np.maximum(drop, 1e-9), np.inf))

    print("%s: third-octave levels %.2f-%.2f s after each time, band T60 from the next %.2f s"
          % (args.wav, args.start, args.start + args.window, args.window))
    head = "  band Hz " + "".join("  %-22s" % l[:22] for l in labels)
    print(head)
    for b, fc in enumerate(centres):
        row = "  %7.0f " % fc
        for s in range(len(times)):
            t60 = decays[s][b]
            t60s = "     -" if not np.isfinite(t60) else "%6.2f" % t60
            rel = "" if s == 0 else " (%+5.1f)" % (levels[s][b] - levels[0][b])
            row += "  %6.1f dB T60 %ss%s" % (levels[s][b], t60s, rel)
        print(row)
    for s in range(len(times)):
        total = 10 * np.log10(np.sum(10 ** (levels[s] / 10)))
        print("  %-24s total %.1f dB" % (labels[s], total))


if __name__ == "__main__":
    main()
