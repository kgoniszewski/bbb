#!/usr/bin/env python3
"""Measure partial frequencies and per-partial T60 of a single-string render.

    analyze_string.py c4.wav --f0 261.63 --B 3.8e-4 [--b1 0.5 --b3 6.25e-9]

For each partial n the signal is projected onto exp(-j 2 pi f_n t), with f_n
from n*f0*sqrt(1 + B n^2), using a 4-term Blackman-Harris window long enough
(8 / f0 seconds) that neighbouring partials fall outside its mainlobe. For a
damped sinusoid that projection is C * exp((-sigma + j 2 pi (f - f_n)) t), so
straight-line fits of unwrapped phase and log magnitude against frame time
give the frequency offset and the decay rate; T60 = ln(1000) / sigma.

Requires numpy only.
"""

import argparse
import struct
import sys

import numpy as np

LN1000 = np.log(1000.0)


def read_wav(path):
    """Returns (samples as float64 mono array, sample rate). Supports PCM
    16/24/32-bit and IEEE float 32/64-bit, including WAVE_FORMAT_EXTENSIBLE.
    Multi-channel files are reduced to their first channel."""
    with open(path, "rb") as f:
        data = f.read()
    if data[0:4] != b"RIFF" or data[8:12] != b"WAVE":
        raise ValueError(f"{path}: not a RIFF/WAVE file")
    pos, fmt, raw = 12, None, None
    while pos + 8 <= len(data):
        tag, size = data[pos:pos + 4], struct.unpack("<I", data[pos + 4:pos + 8])[0]
        body = data[pos + 8:pos + 8 + size]
        if tag == b"fmt ":
            fmt = struct.unpack("<HHIIHH", body[:16])
            if fmt[0] == 0xFFFE and len(body) >= 26:  # extensible: real tag in the GUID
                fmt = (struct.unpack("<H", body[24:26])[0],) + fmt[1:]
        elif tag == b"data":
            raw = body
        pos += 8 + size + (size & 1)
    if fmt is None or raw is None:
        raise ValueError(f"{path}: missing fmt or data chunk")
    tag, channels, rate, _, block_align, bits = fmt
    n = len(raw) // block_align
    raw = raw[:n * block_align]
    if tag == 3 and bits == 32:
        x = np.frombuffer(raw, dtype="<f4").astype(np.float64)
    elif tag == 3 and bits == 64:
        x = np.frombuffer(raw, dtype="<f8").copy()
    elif tag == 1 and bits == 16:
        x = np.frombuffer(raw, dtype="<i2") / 32768.0
    elif tag == 1 and bits == 32:
        x = np.frombuffer(raw, dtype="<i4") / 2147483648.0
    elif tag == 1 and bits == 24:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        v = np.where(v >= 1 << 23, v - (1 << 24), v)
        x = v / 8388608.0
    else:
        raise ValueError(f"{path}: unsupported format tag {tag}, {bits} bits")
    return x.reshape(-1, channels)[:, 0], rate


def blackman_harris(n):
    a = 2.0 * np.pi * np.arange(n) / (n - 1)
    return 0.35875 - 0.48829 * np.cos(a) + 0.14128 * np.cos(2 * a) - 0.01168 * np.cos(3 * a)


def measure_partial(x, fs, f_expected, spacing_hz, range_db=60.0, floor_dbfs=-90.0, start_s=0.0):
    """Returns (frequency Hz, sigma 1/s, frames used); NaNs if too few frames.
    Frames more than range_db below the partial's strongest frame, or whose
    amplitude is below floor_dbfs, are not used."""
    nw = int(np.ceil(8.0 * fs / spacing_hz))
    hop = max(1, nw // 4)
    first = int(start_s * fs)
    starts = np.arange(first, len(x) - nw + 1, hop)
    if len(starts) < 3:
        return np.nan, np.nan, 0
    w = blackman_harris(nw)
    idx = np.arange(nw)
    frames = np.lib.stride_tricks.sliding_window_view(x, nw)[starts]
    # Phase reference is absolute time, so the frame phase advances at
    # 2 pi (f - f_expected). Reduce cycles modulo 1 for precision.
    cyc = np.mod((starts[:, None] + idx[None, :]) * (f_expected / fs), 1.0)
    proj = np.sum(frames * w * np.exp(-2j * np.pi * cyc), axis=1)
    mag = 2.0 * np.abs(proj) / np.sum(w)  # sinusoid amplitude
    t = starts / fs
    phase = np.unwrap(np.angle(proj))
    # Fit only the first contiguous run of strong frames: once a partial has
    # decayed into the floor, its phase is meaningless.
    strong = (mag >= mag.max() * 10.0 ** (-range_db / 20.0)) & (mag >= 10.0 ** (floor_dbfs / 20.0))
    i0 = int(np.argmax(strong))
    tail = strong[i0:]
    i1 = i0 + (len(tail) if tail.all() else int(np.argmin(tail)))
    keep = slice(i0, i1)
    if i1 - i0 < 3:
        return np.nan, np.nan, i1 - i0
    phase_slope = np.polyfit(t[keep], phase[keep], 1)[0]
    log_slope = np.polyfit(t[keep], np.log(mag[keep]), 1)[0]
    return f_expected + phase_slope / (2.0 * np.pi), -log_slope, i1 - i0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("wav")
    ap.add_argument("--f0", type=float, required=True, help="flexible-string fundamental, Hz")
    ap.add_argument("--B", type=float, required=True, help="inharmonicity coefficient")
    ap.add_argument("--b1", type=float, help="loss 1/s, to show expected T60")
    ap.add_argument("--b3", type=float, help="loss s, to show expected T60")
    ap.add_argument("--partials", type=int, default=20)
    ap.add_argument("--range-db", type=float, default=60.0,
                    help="ignore frames this far below a partial's strongest frame")
    ap.add_argument("--floor-db", type=float, default=-90.0,
                    help="ignore frames whose partial amplitude is below this (dBFS); keeps the "
                         "fit above window leakage and the renderer's culling threshold")
    ap.add_argument("--start", type=float, default=0.0, help="skip this many seconds")
    args = ap.parse_args(argv)

    x, fs = read_wav(args.wav)
    print(f"{args.wav}: {len(x)} samples, {fs} Hz, {len(x) / fs:.2f} s, "
          f"peak {20 * np.log10(np.max(np.abs(x)) + 1e-300):.1f} dBFS")
    have_loss = args.b1 is not None and args.b3 is not None
    header = f"{'n':>3} {'expected Hz':>12} {'measured Hz':>12} {'err cents':>10} {'T60 s':>8}"
    if have_loss:
        header += f" {'exp T60 s':>10} {'T60 err %':>10}"
    print(header)
    worst_c, worst_t = 0.0, 0.0
    for n in range(1, args.partials + 1):
        fe = n * args.f0 * np.sqrt(1.0 + args.B * n * n)
        if fe >= 0.5 * fs:
            break
        f, sigma, used = measure_partial(x, fs, fe, args.f0, args.range_db, args.floor_db,
                                         args.start)
        if used < 3:
            print(f"{n:3d} {fe:12.4f} {'-':>12} {'-':>10} {'-':>8}   (too weak)")
            continue
        cents = 1200.0 * np.log2(f / fe)
        t60 = LN1000 / sigma
        worst_c = max(worst_c, abs(cents))
        line = f"{n:3d} {fe:12.4f} {f:12.4f} {cents:10.4f} {t60:8.3f}"
        if have_loss:
            t60e = LN1000 / (args.b1 + args.b3 * (2.0 * np.pi * fe) ** 2)
            err = 100.0 * (t60 / t60e - 1.0)
            worst_t = max(worst_t, abs(err))
            line += f" {t60e:10.3f} {err:10.3f}"
        print(line)
    summary = f"worst |frequency error| {worst_c:.4f} cents"
    if have_loss:
        summary += f", worst |T60 error| {worst_t:.3f} %"
    print(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
