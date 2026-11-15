#!/usr/bin/env python3
"""Sustain-pedal residual per Bark band (plan §1.6 note (7); method of
Lehtonen, Penttinen, Rauhala and Valimaki, "Analysis and modeling of piano
sustain-pedal effects", JASA 122, 2007): the same tone with the pedal down
and with it up, the tone's own partials removed in the frequency domain,
the remaining (residual) energy compared per Bark band over a window after
the strike.

    analyze_pedal_residual.py render.wav --pairs 1,9 17,25 33,41 \
        --notes 48 60 72 [--start 1.0] [--length 4.0]

Each pair is (strike time with the pedal down, strike time with it up) of
the same MIDI note. Partials: located by peak search in the pedal-up
spectrum, partial n near the prediction of an inharmonic fit (f0, B) of the
partials found so far (the first from the equal-tempered note, +-3 %),
up to 16 kHz; the bins within +-(3 Hz + 0.2 %) of each are removed from
both spectra. The residual is what the rest of the instrument adds: other
strings ringing sympathetically, the board, noises. Prints per Bark band
the residual levels (dB re the pedal-up broadband residual) and their
difference, and the broadband residual difference. Requires numpy only.
"""

import argparse
import sys

import numpy as np

from analyze_string import read_wav_channels

BARK_EDGES = [20, 100, 200, 300, 400, 510, 630, 770, 920, 1080, 1270, 1480, 1720, 2000, 2320, 2700, 3150,
              3700, 4400, 5300, 6400, 7700, 9500, 12000, 15500]


def spectrum(x, fs, t0, length):
    a, n = int(round(t0 * fs)), int(round(length * fs))
    seg = np.zeros(n)
    avail = max(0, min(n, len(x) - a))
    seg[:avail] = x[a:a + avail]
    w = np.hanning(n)
    return np.abs(np.fft.rfft(seg * w)) ** 2 / np.sum(w ** 2), np.fft.rfftfreq(n, 1.0 / fs)


def find_partials(spec, freqs, f1, fmax=16000.0):
    """Peak frequencies of the tone's partials."""
    found = []  # (n, f)
    n = 1
    while True:
        if len(found) >= 3:
            ns = np.array([p[0] for p in found], float)
            fs_ = np.array([p[1] for p in found])
            # f_n^2 = n^2 f0^2 + n^4 f0^2 B: linear least squares in (f0^2, f0^2 B)
            A = np.stack([ns ** 2, ns ** 4], 1)
            c, *_ = np.linalg.lstsq(A, fs_ ** 2, rcond=None)
            pred = np.sqrt(max(c[0] * n * n + c[1] * n ** 4, 1.0))
            tol = 0.01
        else:
            pred = n * (found[-1][1] / found[-1][0] if found else f1)
            tol = 0.03
        if pred > fmax:
            break
        sel = (freqs > pred * (1 - tol)) & (freqs < pred * (1 + tol))
        if not np.any(sel):
            break
        idx = np.flatnonzero(sel)
        k = idx[np.argmax(spec[idx])]
        found.append((n, freqs[k]))
        n += 1
    return [f for _, f in found]


def residual_bands(spec, freqs, mask):
    out = []
    for lo, hi in zip(BARK_EDGES[:-1], BARK_EDGES[1:]):
        sel = (freqs >= lo) & (freqs < hi) & mask
        out.append(spec[sel].sum())
    return np.array(out)


def analyse(x, fs, t_down, t_up, note, start, length):
    f1 = 440.0 * 2 ** ((note - 69) / 12.0)
    s_up, freqs = spectrum(x, fs, t_up + start, length)
    s_dn, _ = spectrum(x, fs, t_down + start, length)
    partials = find_partials(s_up, freqs, f1)
    mask = np.ones_like(freqs, bool)
    for f in partials:
        mask &= np.abs(freqs - f) > 3.0 + 0.002 * f
    e_up, e_dn = residual_bands(s_up, freqs, mask), residual_bands(s_dn, freqs, mask)
    return partials, e_up, e_dn


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('wav')
    ap.add_argument('--pairs', nargs='+', required=True, help='down,up strike times (s) per note')
    ap.add_argument('--notes', nargs='+', type=int, required=True, help='MIDI note of each pair')
    ap.add_argument('--start', type=float, default=1.0, help='window start after the strike (s)')
    ap.add_argument('--length', type=float, default=4.0, help='window length (s)')
    a = ap.parse_args(argv)
    if len(a.pairs) != len(a.notes):
        sys.exit('one note per pair')
    data, fs = read_wav_channels(a.wav)
    x = data.sum(axis=1) if data.ndim == 2 else data
    for pair, note in zip(a.pairs, a.notes):
        t_dn, t_up = (float(v) for v in pair.split(','))
        partials, e_up, e_dn = analyse(x, fs, t_dn, t_up, note, a.start, a.length)
        ref = e_up.sum()
        print(f'note {note}: {len(partials)} partials removed (f1 {partials[0]:.1f} Hz); residual over '
              f'{a.start:.1f}-{a.start + a.length:.1f} s after the strike, dB re the pedal-up broadband residual')
        print('  Bark band (Hz)    pedal up   pedal down   difference')
        for i, (lo, hi) in enumerate(zip(BARK_EDGES[:-1], BARK_EDGES[1:])):
            u = 10 * np.log10(e_up[i] / ref + 1e-30)
            d = 10 * np.log10(e_dn[i] / ref + 1e-30)
            print(f'  {lo:5d}-{hi:5d}      {u:7.1f}    {d:7.1f}     {d - u:+7.1f}')
        diff = 10 * np.log10(e_dn.sum() / ref)
        print(f'  broadband residual, pedal down re pedal up: {diff:+.1f} dB')


if __name__ == '__main__':
    main()
