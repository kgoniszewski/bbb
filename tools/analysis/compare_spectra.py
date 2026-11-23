#!/usr/bin/env python3
"""Long-term average spectra of a recording and of the model playing the same
notes.

    compare_spectra.py REC.wav SCORE.txt [--render-cli PATH] [--room on|off]
                       [--render OUT.wav]

SCORE.txt is a render_cli score, normally written by transcribe.py from
REC.wav, so the model plays the same notes at the same times. The score is
rendered at the recording's sample rate (room on by default, since reference
recordings include their room) and both signals, mixed to mono, are analysed
with a 4096-sample Hann STFT every 2048 samples:

  * frames more than 50 dB below the file's 95th-percentile frame power are
    skipped as silent;
  * the model is loudness-matched to the recording by the power in the
    250-1000 Hz band (the gain is printed);
  * octave bands 63 Hz .. 16 kHz (edges fc / sqrt 2 .. fc sqrt 2) of the
    long-term average power spectrum, in dB relative to the larger of the
    250 and 500 Hz bands; the difference model minus recording per band;
  * the 1-8 kHz slope in dB/octave: least-squares line through the
    third-octave band levels from 1 to 8 kHz against log2 frequency;
  * the same split by register, labelling frames from the score: each
    sounding note (note_on to note_off) weighs (velocity / 127)^2, and a
    frame is bass, middle or treble when notes below C3 (48), from C3 to C5
    (48..72) or above C5 carry at least 60 % of the weight. A register with
    under 5 s of frames is skipped. Both signals use the same labels, since
    the score's timing is the recording's.

Requires numpy only.
"""

import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

from analyze_string import read_wav_channels

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CLI = os.path.join(os.path.dirname(os.path.dirname(HERE)),
                           "build-vs", "tools", "render_cli", "render_cli")
N_FFT, HOP = 4096, 2048
OCTAVES = [63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000]
SILENCE_DB = 50.0
REGIONS = [("bass", 0, 48), ("middle", 48, 73), ("treble", 73, 128)]
NOTE_BASE = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}


def parse_key(s):
    if s[0].isdigit():
        return int(s)
    semi = NOTE_BASE[s[0].upper()]
    i = 1
    if i < len(s) and s[i] in "#b":
        semi += 1 if s[i] == "#" else -1
        i += 1
    return 12 * (int(s[i:]) + 1) + semi


def read_notes(path):
    """(on, off, key, velocity) from a score; unreleased notes end at +inf."""
    open_, notes = {}, []
    with open(path) as f:
        rows = [l.split("#")[0].split() for l in f]
    rows = sorted((float(r[0]), r) for r in rows if len(r) >= 3)
    for t, r in rows:
        if r[1] == "note_on":
            k = parse_key(r[2])
            if k in open_:
                notes.append((open_[k][0], t, k, open_[k][1]))
            open_[k] = (t, int(r[3]))
        elif r[1] == "note_off":
            k = parse_key(r[2])
            if k in open_:
                notes.append((open_[k][0], t, k, open_[k][1]))
                del open_[k]
    notes += [(t, np.inf, k, v) for k, (t, v) in open_.items()]
    return notes


def power_frames(x):
    win = np.hanning(N_FFT)
    n = max(1, 1 + (len(x) - N_FFT) // HOP)
    out = np.empty((n, N_FFT // 2 + 1))
    for i0 in range(0, n, 1024):
        idx = (np.arange(i0, min(n, i0 + 1024)) * HOP)[:, None] + np.arange(N_FFT)[None, :]
        out[i0:i0 + len(idx)] = np.abs(np.fft.rfft(x[idx] * win, axis=1)) ** 2
    return out


def band_levels(ltas, freqs, centres, ratio):
    lo, hi = np.array(centres) / ratio, np.array(centres) * ratio
    return np.array([10 * np.log10(ltas[(freqs >= a) & (freqs < b)].sum() + 1e-30)
                     for a, b in zip(lo, hi)])


def analyse(P, freqs, mask):
    ltas = P[mask].mean(axis=0)
    oct_db = band_levels(ltas, freqs, OCTAVES, np.sqrt(2))
    oct_db -= max(oct_db[2], oct_db[3])
    thirds = 1000 * 2 ** (np.arange(10) / 3)
    third_db = band_levels(ltas, freqs, thirds, 2 ** (1 / 6))
    slope = np.polyfit(np.log2(thirds), third_db, 1)[0]
    return oct_db, slope


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording")
    ap.add_argument("score")
    ap.add_argument("--render-cli", default=DEFAULT_CLI)
    ap.add_argument("--room", choices=["on", "off"], default="on")
    ap.add_argument("--render", help="keep the model render at this path")
    a = ap.parse_args(argv)

    xr, fs = read_wav_channels(a.recording)
    xr = xr.mean(axis=1)
    out = a.render or os.path.join(tempfile.mkdtemp(), "model.wav")
    cmd = [a.render_cli, "--score", a.score, "--out", out, "--fs", str(fs), "--quiet"]
    if a.room == "off":
        cmd.append("--dry")
    r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if r.returncode != 0:
        sys.exit("render_cli failed: " + r.stderr.decode(errors="replace")[:500])
    xm, _ = read_wav_channels(out)
    xm = xm.mean(axis=1)
    if not a.render:
        os.remove(out)
    n = min(len(xr), len(xm))
    Pr, Pm = power_frames(xr[:n]), power_frames(xm[:n])
    freqs = np.arange(N_FFT // 2 + 1) * fs / N_FFT

    def loud(P):
        e = P.sum(axis=1)
        return 10 * np.log10(e + 1e-30) > 10 * np.log10(np.percentile(e, 95) + 1e-30) - SILENCE_DB

    mr, mm = loud(Pr), loud(Pm)
    band = (freqs >= 250) & (freqs < 1000)
    gain_db = 10 * np.log10(Pr[mr][:, band].sum(axis=1).mean() / Pm[mm][:, band].sum(axis=1).mean())
    Pm = Pm * 10 ** (gain_db / 10)

    # Register labels from the score.
    t = (np.arange(len(Pr)) * HOP + N_FFT / 2) / fs
    w = np.zeros((len(REGIONS), len(t)))
    for on, off, k, v in read_notes(a.score):
        i0, i1 = np.searchsorted(t, [on, off])
        ri = next(i for i, (_, lo, hi) in enumerate(REGIONS) if lo <= k < hi)
        w[ri, i0:i1] += (v / 127.0) ** 2
    tot = w.sum(axis=0)
    frame_s = HOP / fs

    print(f"{os.path.basename(a.recording)} vs model ({os.path.basename(a.score)}, room {a.room}); "
          f"{n / fs:.0f} s, model gain {gain_db:+.1f} dB (250-1000 Hz match)")
    groups = [("all", np.ones(len(t), bool))]
    for ri, (name, _, _) in enumerate(REGIONS):
        groups.append((name, (tot > 0) & (w[ri] >= 0.6 * np.maximum(tot, 1e-12))))
    head = "band Hz      " + "".join(f"{f:>7d}" for f in OCTAVES) + "   1-8k dB/oct"
    for name, g in groups:
        gr, gm = g & mr, g & mm
        secs = min(gr.sum(), gm.sum()) * frame_s
        if secs < 5.0:
            print(f"\n[{name}] {secs:.1f} s of frames: skipped")
            continue
        lr, sr = analyse(Pr, freqs, gr)
        lm, sm = analyse(Pm, freqs, gm)
        print(f"\n[{name}] {gr.sum() * frame_s:.0f} s recording / {gm.sum() * frame_s:.0f} s model frames")
        print(head)
        print("recording    " + "".join(f"{v:7.1f}" for v in lr) + f"   {sr:7.1f}")
        print("model        " + "".join(f"{v:7.1f}" for v in lm) + f"   {sm:7.1f}")
        print("model - rec  " + "".join(f"{v:+7.1f}" for v in lm - lr) + f"   {sm - sr:+7.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
