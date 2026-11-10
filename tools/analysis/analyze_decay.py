#!/usr/bin/env python3
"""Two-stage decay and beating of the partials of single piano notes.

    analyze_decay.py note.wav --note 60 [--at 0,8,16] [--seconds 7.5]
                     [--partials 8] [--B 3.8e-4] [--range 55]

For each segment (one note each, starting at an --at time and lasting
--seconds), per partial n:

  * frequency: the strongest spectral peak within +-f1/4 of
    n f1 sqrt((1 + B n^2) / (1 + B)) in the first second (zero-padded FFT,
    parabolic interpolation), B refined from the partials found;
  * envelope: complex baseband (heterodyne to the partial frequency, Hann
    window of max(8 / f1, 50 ms), every 20 ms), in dB;
  * two-stage decay fit: two straight lines in dB against time, from 0.2 s
    until the envelope has fallen --range dB, breakpoint by least squares.
    Reports prompt T60 and aftersound T60 (-60 dB / slope of each line), the
    break time and the crossover level (where the lines meet, dB re the
    level at 0.2 s);
  * beat rate: frequency difference of the two components of a second-order
    linear-prediction (Prony) model of the baseband from 0.5 s on (up to
    8 s), i.e. of the two dominant damped exponentials; '-' if the weaker
    one starts more than 30 dB below the stronger.

Stereo files are summed to mono. Requires numpy only.
"""

import argparse
import sys

import numpy as np

from analyze_string import read_wav_channels


def partial_frequencies(x, fs, f1, count, b):
    n1 = min(len(x), int(fs))
    seg = x[:n1] * np.hanning(n1)
    n_fft = 1 << int(np.ceil(np.log2(n1 * 8)))
    spec = np.abs(np.fft.rfft(seg, n=n_fft))
    f = np.fft.rfftfreq(n_fft, 1.0 / fs)
    out = []
    for n in range(1, count + 1):
        fe = n * f1 * np.sqrt((1.0 + b * n * n) / (1.0 + b))
        if fe + f1 / 4 >= fs / 2:
            break
        sel = np.flatnonzero((f >= fe - f1 / 4) & (f <= fe + f1 / 4))
        i = sel[np.argmax(spec[sel])]
        if 0 < i < len(spec) - 1:
            a, c, d = np.log(spec[i - 1:i + 2] + 1e-300)
            den = a - 2 * c + d
            fr = f[i] + (0.5 * (a - d) / den if den != 0 else 0.0) * (f[1] - f[0])
        else:
            fr = f[i]
        out.append(fr)
        if n >= 2:
            r = (fr / (n * out[0])) ** 2
            if n * n - r > 0:
                b = max(0.0, (r - 1.0) / (n * n - r))
    return out


def baseband(x, fs, freq, win, hop):
    nw = max(4, int(win * fs))
    nh = max(1, int(hop * fs))
    w = np.hanning(nw)
    t = np.arange(len(x)) / fs
    y = x * np.exp(-2j * np.pi * freq * t)
    starts = np.arange(0, len(x) - nw + 1, nh)
    frames = np.lib.stride_tricks.sliding_window_view(y, nw)[starts]
    return 2.0 * frames @ w / np.sum(w), starts / fs + 0.5 * win


def two_stage(t, env_db, t0, rng):
    keep = t >= t0
    t, y = t[keep], env_db[keep]
    if len(y) < 8:
        return None
    ref = y[0]
    below = np.flatnonzero(y < ref - rng)
    n = below[0] if len(below) else len(y)
    t, y = t[:n], y[:n] - ref
    if n < 8:
        return None
    best = None
    for b in range(4, n - 3):
        p1 = np.polyfit(t[:b], y[:b], 1)
        p2 = np.polyfit(t[b:], y[b:], 1)
        if p1[0] >= 0 or p2[0] >= 0:
            continue
        sse = np.sum((np.polyval(p1, t[:b]) - y[:b]) ** 2) + np.sum((np.polyval(p2, t[b:]) - y[b:]) ** 2)
        if best is None or sse < best[0]:
            tc = (p2[1] - p1[1]) / (p1[0] - p2[0]) if p1[0] != p2[0] else t[b]
            best = (sse, -60.0 / p1[0], -60.0 / p2[0], t[b], np.polyval(p1, tc))
    return best


def beat_rate(bb, hop, k0, n):
    x = bb[k0:k0 + n]
    if len(x) < 8:
        return None
    a = np.column_stack([x[1:-1], x[:-2]])
    coef, *_ = np.linalg.lstsq(a, x[2:], rcond=None)
    z = np.roots([1.0, -coef[0], -coef[1]])
    v = np.vander(z, len(x), increasing=True).T
    amp, *_ = np.linalg.lstsq(v, x, rcond=None)
    amp = np.abs(amp)
    df = abs(np.angle(z[0]) - np.angle(z[1])) / (2 * np.pi * hop)
    if amp.min() < 0.03 * amp.max() or df < 0.01:
        return None
    return df


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("wav")
    ap.add_argument("--note", type=int, help="MIDI note (equal temperament, A4 = 440 Hz)")
    ap.add_argument("--f1", type=float, help="first partial, Hz (overrides --note)")
    ap.add_argument("--B", type=float, default=3e-4, help="initial inharmonicity estimate")
    ap.add_argument("--at", default="0", help="comma-separated segment start times, s")
    ap.add_argument("--seconds", type=float, default=None, help="segment length (default: to the next one)")
    ap.add_argument("--partials", type=int, default=8)
    ap.add_argument("--range", type=float, default=55.0, help="dB of decay used by the fit")
    args = ap.parse_args(argv)
    if args.f1 is None and args.note is None:
        ap.error("give --note or --f1")
    f1 = args.f1 if args.f1 is not None else 440.0 * 2.0 ** ((args.note - 69) / 12.0)

    chans, fs = read_wav_channels(args.wav)
    x = chans.sum(axis=1)
    starts = [float(s) for s in args.at.split(",")]
    hop = 0.02
    win = max(8.0 / f1, 0.05)
    print(f"{args.wav}: {len(x) / fs:.2f} s, {fs} Hz, f1 ~ {f1:.2f} Hz")
    for k, t0 in enumerate(starts):
        a = int(t0 * fs)
        if args.seconds is not None:
            b = a + int(args.seconds * fs)
        else:
            b = int(starts[k + 1] * fs) if k + 1 < len(starts) else len(x)
        seg = x[a:min(b, len(x))]
        on = np.argmax(np.abs(seg) >= 0.01 * np.max(np.abs(seg))) if np.max(np.abs(seg)) > 0 else 0
        seg = seg[on:]
        freqs = partial_frequencies(seg, fs, f1, args.partials, args.B)
        print(f"  segment at {t0:.2f} s ({len(seg) / fs:.1f} s):")
        print("    n   freq Hz   level dB  prompt T60  after T60   break s  crossover dB  beat Hz")
        for n, fr in enumerate(freqs, start=1):
            bb, tt = baseband(seg, fs, fr, win, hop)
            env = 20.0 * np.log10(np.abs(bb) + 1e-30)
            i02 = int(np.searchsorted(tt, 0.2))
            level = env[i02] if i02 < len(env) else float("nan")
            fit = two_stage(tt, env, 0.2, args.range)
            k0 = int(np.searchsorted(tt, 0.5))
            br = beat_rate(bb, hop, k0, int(min(8.0, max(1.0, len(seg) / fs - 1.0)) / hop))
            beat = f"{br:7.3f}" if br is not None else "      -"
            if fit is None:
                print(f"  {n:3d} {fr:9.2f} {level:9.1f}     (too short or too quiet to fit)   {beat}")
            else:
                _, p, q, brk, cross = fit
                pt = f"{p:10.2f}" if p < 1000 else "     >1000"
                qt = f"{q:10.2f}" if q < 1000 else "     >1000"
                print(f"  {n:3d} {fr:9.2f} {level:9.1f} {pt} {qt} {brk:9.2f} {cross:12.1f}  {beat}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
