#!/usr/bin/env python3
"""Transcribe a piano recording into a render_cli score (notes and dynamics).

    transcribe.py REC.wav --out score.txt [--templates DIR] [--render-cli PATH]
                  [--no-level-match] [--level-seconds 90]
    transcribe.py --selftest [--templates DIR] [--render-cli PATH]

Used to compare the model with reference recordings of music performances:
the recording is turned into notes with velocities, the same notes are
rendered with the model and the two are compared (compare_spectra.py).

  * Spectrogram: mono mix, STFT with an 8192-sample Hann window (186 ms at
    44.1 kHz; 4096 samples left neighbouring bass keys unresolved and roughly
    doubled the false notes in the self-test) every 10 ms, power summed into
    log-frequency bins (3 per semitone, 26 Hz .. 8 kHz) with triangular
    weights at least one FFT bin wide; magnitude (square root of the bin
    power) is factorised. Onsets are gated by the spectral flux, so the long
    window costs little onset precision.
  * Templates come from the model itself: every key 21..108 is rendered once
    (velocity 80, dry), batched into four scores rendered in parallel. Two
    templates per key: the attack (first 150 ms after the onset) and the
    decay (0.5 .. 1.5 s), each normalised to unit sum. They are cached as an
    npz in the templates directory, keyed by the render_cli binary's mtime
    and the sample rate.
  * Semi-supervised NMF: templates fixed, activations learnt by KL-divergence
    multiplicative updates with a light L1/2 sparsity penalty (an L1 penalty
    only rescales KL activations). With W fixed every frame
    is independent, so the spectrogram is processed in chunks of frames.
  * Non-note components are added to the fixed dictionary: the hammer knock
    (key 108's attack spectrum below its fundamental) and 12 smooth bumps
    over log frequency for noise and reverberation.
  * Onsets: global onset frames are peaks of the log-spectral flux above
    its 1 s running median plus 0.3 x its 99th percentile. At each, a key
    is a candidate if its attack activation rises (mean of the 100 ms before
    vs max at +50 ms) by at least -20 dB re the largest rise there and at
    least doubles; a candidate a semitone, a whole tone, an octave, a twelfth
    or two octaves from one rising 10 dB more is dropped as leakage. The
    onset is the largest frame-to-frame rise of the key near the global
    onset; onsets of one key are at least 60 ms apart. The onset
    latency of this detector is measured on the calibration renders (known
    onsets) and subtracted. The note-off is where the summed activation falls
    below 15 % of its peak after the onset, or the next onset of the key.
    There is no pedal detection: the score has no cc64 events, and the
    note-offs taken from the activation decay approximate the pedalled sound.
  * Velocity: the level of a note is the peak summed activation (dB) in the
    first 120 ms. Every third key from 21 to 108 rendered at velocities 20,
    40, 60, 80, 100, 120, 127 and analysed with the same NMF give a level ->
    velocity table per key, interpolated linearly over key (and extrapolated
    in level). One global level offset (dB) maps the recording's activation
    levels onto the model's. A recording's gain is arbitrary (the four
    commercial reference recordings are 13-22 dB louder than the model at
    velocity 127, so matching absolute RMS drove every velocity to 127), so
    by default the offset puts the median velocity at --median-velocity
    (64) and the first --level-seconds of the score are rendered (room on)
    once to report the remaining gain gap, recording minus render RMS.
    With --absolute-level the offset is instead found by matching the RMS
    of that span (secant steps, up to four renders, stopping when the
    velocities saturate); the self-test does this, dry, since its gain is
    known. Both numbers are reported and written in the score's header.
  * --selftest renders tools/render_cli/scores/ab_phrase.txt and
    chord_pedal.txt (dry), transcribes the renders and reports onset
    precision, recall and F1 (pitch exact, onset within +-50 ms) and the
    median absolute velocity error of matched notes, with the true offset
    (0 dB) and with the automatic level match.

Requires numpy only.
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

from analyze_string import read_wav_channels

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DEFAULT_CLI = os.path.join(REPO, "build-vs", "tools", "render_cli", "render_cli")

N_FFT = 4096  # short window; the long one (2 * N_FFT) is used below F_SPLIT
HOP_S = 0.010
BINS_PER_SEMI = 3
F_MIN = 27.5 * 2.0 ** (-1.0 / 12.0)
F_MAX = 8000.0
F_SPLIT = 1e9  # all bins from the long window; see log_spectrogram
KEYS = np.arange(21, 109)
NK = len(KEYS)
CAL_KEYS = list(range(21, 109, 3))
CAL_VELS = [20, 40, 60, 80, 100, 120, 127]
TEMPLATE_SPACING = 2.0
CACHE_VERSION = 7

NMF_ITERS = 60
SPARSITY = 0.02
REL_FRAME_DB = -20.0
REL_GLOBAL_DB = -50.0
RISE_RATIO = 2.0
RISE_S = 0.05
FLUX_DELTA = 0.3
N_BUMPS = 12
SUPPRESS_DB = 10.0
MIN_GAP_S = 0.060
OFF_FRACTION = 0.15
LEVEL_WINDOW_S = 0.120

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


def read_score_notes(path):
    """note_on events of a render_cli score as a list of (time, key, velocity)."""
    notes = []
    with open(path) as f:
        for line in f:
            tok = line.split("#")[0].split()
            if len(tok) >= 4 and tok[1] == "note_on":
                notes.append((float(tok[0]), parse_key(tok[2]), int(tok[3])))
    return sorted(notes)


def read_mono(path):
    x, fs = read_wav_channels(path)
    return x.mean(axis=1), fs


def log_filterbank(fs, n_fft=N_FFT):
    n_bins = int(np.ceil(12 * BINS_PER_SEMI * np.log2(F_MAX / F_MIN)))
    centres = F_MIN * 2.0 ** (np.arange(n_bins) / (12.0 * BINS_PER_SEMI))
    df = fs / n_fft
    f = np.arange(n_fft // 2 + 1) * df
    half = np.maximum(centres * (2.0 ** (1.0 / (12 * BINS_PER_SEMI)) - 1.0), df)
    fb = np.maximum(0.0, 1.0 - np.abs(f[None, :] - centres[:, None]) / half[:, None])
    return fb, centres


def log_spectrogram(x, fs, chunk=1024):
    """Magnitude log-frequency spectrogram (bins x frames); frame t is centred
    at t * HOP_S seconds. Bins below F_SPLIT come from a window of
    2 * N_FFT samples: with 4096 samples the partials of neighbouring bass
    keys are not resolved and the bass templates explain one another. On the
    self-test, taking every bin from the long window (F_SPLIT above F_MAX)
    gave fewer false notes than splitting at 0.5, 1 or 2 kHz."""
    hop = int(round(HOP_S * fs))
    n_frames = 1 + len(x) // hop
    fb, centres = log_filterbank(fs, N_FFT)
    out = np.empty((fb.shape[0], n_frames), dtype=np.float32)
    for n_fft, rows in ((N_FFT, centres >= F_SPLIT), (2 * N_FFT, centres < F_SPLIT)):
        if not rows.any():
            continue
        fbr = log_filterbank(fs, n_fft)[0][rows]
        xp = np.concatenate([np.zeros(n_fft // 2), x, np.zeros(n_fft)])
        win = np.hanning(n_fft)
        for t0 in range(0, n_frames, chunk):
            t1 = min(n_frames, t0 + chunk)
            idx = (np.arange(t0, t1) * hop)[:, None] + np.arange(n_fft)[None, :]
            p = np.abs(np.fft.rfft(xp[idx] * win, axis=1)) ** 2
            out[rows, t0:t1] = np.sqrt(fbr @ p.T)
    return out


def run_render(cli, score_path, out_path, fs, dry=True, extra=()):
    cmd = [cli, "--score", score_path, "--out", out_path, "--fs", str(int(fs)), "--quiet"]
    if dry:
        cmd.append("--dry")
    cmd += list(extra)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def wait_all(procs):
    for p in procs:
        _, err = p.communicate()
        if p.returncode != 0:
            raise RuntimeError("render_cli failed: " + err.decode(errors="replace")[:500])


def nmf_activations(V, W, iters=NMF_ITERS, sparsity=SPARSITY, chunk=4000):
    """KL-NMF activations with fixed W (columns sum to 1). The sparsity term
    is an L1/2 penalty lambda * sum(sqrt(s_t h)) with s_t the frame's total
    magnitude: an L1 penalty would only rescale all activations (KL and L1 are
    both homogeneous of degree one), whereas L1/2 suppresses activations that
    are small relative to the frame."""
    V = V.astype(np.float32)
    W = W.astype(np.float32)
    Wt = W.T.copy()
    eps = np.float32(1e-12)
    H = np.empty((W.shape[1], V.shape[1]), dtype=np.float32)
    wsum = W.sum(axis=0)[:, None].astype(np.float32)
    for t0 in range(0, V.shape[1], chunk):
        v = V[:, t0:t0 + chunk] + eps
        s = v.sum(axis=0, keepdims=True)
        h = np.maximum(Wt @ v, eps) / W.shape[1]
        h *= s / np.maximum((W @ h).sum(axis=0, keepdims=True), eps)
        for _ in range(iters):
            h *= (Wt @ (v / (W @ h + eps))) / (wsum + np.float32(0.5 * sparsity) * np.sqrt(s / (h + eps)))
            h = np.maximum(h, eps)
        H[:, t0:t0 + chunk] = h
    return H


def add_noise_components(W, fs):
    """Appends non-note components: the hammer knock / board thump (attack
    spectrum of key 108 below 0.7 of its fundamental, where it has no
    partials) and N_BUMPS smooth bumps over the log-frequency axis for noise
    and reverberation. Without them the knock of every strike is explained
    by the bass keys' attack templates."""
    _, centres = log_filterbank(fs)
    f108 = 440.0 * 2 ** ((108 - 69) / 12)
    knock = np.where(centres < 0.7 * f108, W[:, NK - 1], 0.0)
    nb = W.shape[0]
    b = np.arange(nb)
    pos = np.linspace(0, nb - 1, N_BUMPS)
    bumps = np.exp(-0.5 * ((b[:, None] - pos[None, :]) / (0.7 * nb / N_BUMPS)) ** 2)
    extra = np.hstack([knock[:, None], bumps]) + 1e-9
    return np.hstack([W, extra / extra.sum(axis=0, keepdims=True)])


def global_onsets(V):
    """Frames of note onsets in the whole spectrum: peaks of the log-spectral
    flux above a running median plus a margin."""
    L = np.log(V + 1e-3 * V.max() + 1e-12)
    flux = np.maximum(np.diff(L, axis=1, prepend=L[:, :1]), 0.0).mean(axis=0)
    r = int(round(0.5 / HOP_S))
    pad = np.pad(flux, r, mode="edge")
    med = np.array([np.median(pad[t:t + 2 * r + 1]) for t in range(0, len(flux))]) \
        if len(flux) < 20000 else _running_median(pad, r, len(flux))
    thr = med + FLUX_DELTA * np.percentile(flux, 99)
    left = np.concatenate([[0.0], flux[:-1]])
    right = np.concatenate([flux[1:], [0.0]])
    return np.nonzero((flux > thr) & (flux >= left) & (flux > right))[0]


def _running_median(pad, r, n, step=5):
    idx = np.arange(0, n, step)
    m = np.array([np.median(pad[t:t + 2 * r + 1]) for t in idx])
    return np.interp(np.arange(n), idx, m)


def detect_notes(H, V, latency_frames=0.0):
    """Returns a list of (onset s, offset s, key, level dB) from activations
    (attack rows 0..NK-1, decay rows NK..2NK-1) and the spectrogram V."""
    Ha, Hs = H[:NK], H[:NK] + H[NK:2 * NK]
    T = H.shape[1]
    lw = int(round(LEVEL_WINDOW_S / HOP_S))
    w = int(round(RISE_S / HOP_S))
    cands = []
    for g in global_onsets(V):
        a0, a1 = max(0, g - w), min(T - 1, g + w)
        before = Ha[:, max(0, a0 - w):a0 + 1].mean(axis=1)
        after = Ha[:, a1 - 1:a1 + 2].max(axis=1)
        rise = after - before
        if rise.max() <= 0:
            continue
        ok = (rise >= rise.max() * 10 ** (REL_FRAME_DB / 20)) & (after >= RISE_RATIO * before)
        ks = np.nonzero(ok)[0]
        for k in ks:
            # A candidate within two semitones, or at an octave, twelfth or
            # double octave, of a candidate that rises SUPPRESS_DB more is
            # taken as leakage from that note.
            rel = [j for j in ks if abs(int(j) - int(k)) in (1, 2, 12, 19, 24)]
            if any(rise[j] > rise[k] * 10 ** (SUPPRESS_DB / 20) for j in rel):
                continue
            seg = np.diff(Ha[k, a0:a1 + 1])
            c = a0 + 1 + int(seg.argmax())
            cands.append((k, c, rise[k]))
    min_gap = int(round(MIN_GAP_S / HOP_S))
    notes = []
    gmax = max([c[2] for c in cands], default=0.0)
    for k in range(NK):
        kc = [(c, r) for kk, c, r in cands if kk == k and r >= gmax * 10 ** (REL_GLOBAL_DB / 20)]
        keep = []
        for c, r in sorted(kc, key=lambda q: -q[1]):
            if all(abs(c - q) >= min_gap for q in keep):
                keep.append(c)
        keep.sort()
        for i, c in enumerate(keep):
            nxt = keep[i + 1] if i + 1 < len(keep) else T
            seg = Hs[k, c:min(c + lw, nxt)]
            peak = float(seg.max())
            p_at = c + int(seg.argmax())
            below = np.nonzero(Hs[k, p_at:nxt] < OFF_FRACTION * peak)[0]
            off = p_at + below[0] if below.size else nxt
            on_s = max(0.0, (c - latency_frames) * HOP_S)
            off_s = max(on_s + 0.05, (off - latency_frames) * HOP_S)
            notes.append((on_s, off_s, int(KEYS[k]), 20 * np.log10(peak + 1e-12)))
    notes.sort()
    return notes


def note_level(Hs_k, t_s):
    """Calibration level: peak summed activation near a known onset."""
    a, b = int(round((t_s - 0.05) / HOP_S)), int(round((t_s + LEVEL_WINDOW_S) / HOP_S))
    return 20 * np.log10(Hs_k[max(a, 0):b].max() + 1e-12)


def build_templates(cli, tdir, fs):
    os.makedirs(tdir, exist_ok=True)
    mtime = os.stat(cli).st_mtime_ns
    cache = os.path.join(tdir, f"templates_{mtime}_{int(fs)}_v{CACHE_VERSION}.npz")
    if os.path.exists(cache):
        z = np.load(cache)
        return z["W"], z["cal_levels"], float(z["latency"])
    t_start = time.time()
    work = tempfile.mkdtemp(dir=tdir)
    groups = np.array_split(KEYS, 4)
    jobs = []
    for gi, g in enumerate(groups):
        lines = []
        for i, k in enumerate(g):
            t = 0.5 + i * TEMPLATE_SPACING
            lines += [f"{t:.3f} note_on {k} 80", f"{t + 1.5:.3f} note_off {k}"]
        jobs.append((g, [0.5 + i * TEMPLATE_SPACING for i in range(len(g))], lines))
    cal = [(k, v) for k in CAL_KEYS for v in CAL_VELS]
    cal_groups = np.array_split(np.arange(len(cal)), 8)
    cal_jobs = []
    for idx in cal_groups:
        lines, times = [], []
        for i, j in enumerate(idx):
            k, v = cal[j]
            t = 0.5 + i * TEMPLATE_SPACING
            lines += [f"{t:.3f} note_on {k} {v}", f"{t + 1.5:.3f} note_off {k}"]
            times.append(t)
        cal_jobs.append((idx, times, lines))
    procs, paths = [], []
    for n, (_, _, lines) in enumerate(jobs + cal_jobs):
        sp, wp = os.path.join(work, f"s{n}.txt"), os.path.join(work, f"s{n}.wav")
        with open(sp, "w") as f:
            f.write("\n".join(lines) + "\n")
        procs.append(run_render(cli, sp, wp, fs, extra=["--tail", "1"]))
        paths.append(wp)
    wait_all(procs)
    nb = log_filterbank(fs)[0].shape[0]
    W = np.zeros((nb, 2 * NK))
    fr = lambda s: int(round(s / HOP_S))
    for n, (g, times, _) in enumerate(jobs):
        x, _ = read_mono(paths[n])
        S = log_spectrogram(x, fs)
        for k, t in zip(g, times):
            c = fr(t)
            W[:, k - 21] = S[:, c:c + fr(0.15)].mean(axis=1)
            W[:, NK + k - 21] = S[:, c + fr(0.5):c + fr(1.5)].mean(axis=1)
    W = W / W.sum(axis=0, keepdims=True) + 1e-9
    W = add_noise_components(W / W.sum(axis=0, keepdims=True), fs)
    # Calibration: analyse with the same NMF and detector.
    cal_levels = np.full((len(CAL_KEYS), len(CAL_VELS)), np.nan)
    lat = []
    for n, (idx, times, _) in enumerate(cal_jobs):
        x, _ = read_mono(paths[len(jobs) + n])
        V = log_spectrogram(x, fs)
        H = nmf_activations(V, W)
        found = detect_notes(H, V)
        for j, t in zip(idx, times):
            k, v = cal[j]
            cal_levels[CAL_KEYS.index(k), CAL_VELS.index(v)] = note_level(H[k - 21] + H[NK + k - 21], t)
            near = [m[0] - t for m in found if m[2] == k and abs(m[0] - t) < 0.1]
            if near:
                lat.append(min(near, key=abs))
    latency = float(np.median(lat)) / HOP_S if lat else 0.0
    for i in range(len(CAL_KEYS)):  # fill gaps, force monotonic
        row = cal_levels[i]
        ok = ~np.isnan(row)
        cal_levels[i] = np.interp(CAL_VELS, np.array(CAL_VELS)[ok], row[ok])
        cal_levels[i] = np.maximum.accumulate(cal_levels[i] + 1e-3 * np.arange(len(CAL_VELS)))
    np.savez(cache, W=W, cal_levels=cal_levels, latency=latency)
    for p in paths:
        os.remove(p)
    print(f"templates: rendered and analysed in {time.time() - t_start:.0f} s, "
          f"onset latency {latency * HOP_S * 1000:.0f} ms -> {cache}")
    return W, cal_levels, latency


def level_to_velocity(key, level_db, cal_levels):
    vel = []
    for row in cal_levels:
        lo = (CAL_VELS[1] - CAL_VELS[0]) / (row[1] - row[0])
        hi = (CAL_VELS[-1] - CAL_VELS[-2]) / max(row[-1] - row[-2], 1e-3)
        if level_db < row[0]:
            v = CAL_VELS[0] + (level_db - row[0]) * lo
        elif level_db > row[-1]:
            v = CAL_VELS[-1] + (level_db - row[-1]) * hi
        else:
            v = np.interp(level_db, row, CAL_VELS)
        vel.append(v)
    v = np.interp(key, CAL_KEYS, vel)
    return int(np.clip(round(v), 1, 127))


def notes_to_score(notes, cal_levels, offset_db, t_max=None):
    ev = []
    for on, off, k, lev in notes:
        if t_max is not None and on >= t_max:
            continue
        ev.append((on, 1, f"{on:.3f} note_on {k} {level_to_velocity(k, lev + offset_db, cal_levels)}"))
        ev.append((off, 0, f"{off:.3f} note_off {k}"))
    ev.sort(key=lambda e: (e[0], e[1]))
    return "\n".join(e[2] for e in ev) + "\n"


def rms_db(x):
    return 10 * np.log10(np.mean(x * x) + 1e-20)


def level_match(x, fs, notes, cal_levels, cli, work, seconds, dry=False,
                absolute=True, median_velocity=64):
    """Global level offset (dB). Starts where the median velocity is
    `median_velocity`; with `absolute`, secant iteration on (recording -
    render) RMS over the first `seconds`, otherwise one render that only
    reports the gain gap. Returns (offset dB, recording - render dB)."""
    levels = np.array([n[3] for n in notes])
    keys = np.array([n[2] for n in notes])
    # Start where the median velocity is 64.
    lo, hi = -120.0, 120.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        med = np.median([level_to_velocity(k, l + mid, cal_levels) for k, l in zip(keys, levels)])
        lo, hi = (mid, hi) if med < median_velocity else (lo, mid)
    offset = 0.5 * (lo + hi)
    seconds = min(seconds, len(x) / fs)
    n = int(seconds * fs)
    target = rms_db(x[:n])
    prev = None
    sp, wp = os.path.join(work, "level.txt"), os.path.join(work, "level.wav")
    for it in range(4):
        with open(sp, "w") as f:
            f.write(notes_to_score(notes, cal_levels, offset, t_max=seconds))
        wait_all([run_render(cli, sp, wp, fs, dry=dry, extra=["--tail", "0.5"])])
        y, _ = read_mono(wp)
        diff = target - rms_db(y[:n])
        print(f"  level match {it + 1}: offset {offset:+.1f} dB, recording - render {diff:+.2f} dB")
        if not absolute or abs(diff) < 0.3:
            break
        slope = 1.0
        if prev is not None and abs(offset - prev[0]) > 1e-3:
            slope = float(np.clip((prev[1] - diff) / (offset - prev[0]), 0.3, 1.5))
        prev = (offset, diff)
        if slope <= 0.3 and prev is not None and it > 0:
            break  # velocities saturated: the gain gap cannot be closed
        offset += diff / slope
    os.remove(wp)
    return offset, diff


def transcribe(x, fs, W, latency):
    V = log_spectrogram(x, fs)
    return detect_notes(nmf_activations(V, W), V, latency)


def evaluate(truth, est, tol=0.05):
    """Greedy matching, pitch exact and onset within tol. Returns
    (precision, recall, f1, matched pairs)."""
    used = np.zeros(len(est), bool)
    pairs = []
    for tr in truth:
        t, k, _ = tr
        best, bi = tol + 1e-9, -1
        for i, (on, _, kk, vv) in enumerate(est):
            if not used[i] and kk == k and abs(on - t) <= best:
                best, bi = abs(on - t), i
        if bi >= 0:
            used[bi] = True
            pairs.append((tr, est[bi]))
    p = len(pairs) / max(len(est), 1)
    r = len(pairs) / max(len(truth), 1)
    return p, r, 2 * p * r / max(p + r, 1e-12), pairs


def selftest(cli, tdir, work):
    fs = 44100
    W, cal_levels, latency = build_templates(cli, tdir, fs)
    tot = {"tp": 0, "ne": 0, "nt": 0}
    errs0, errs1 = [], []
    for name in ["ab_phrase.txt", "chord_pedal.txt"]:
        sp = os.path.join(REPO, "tools", "render_cli", "scores", name)
        wp = os.path.join(work, "selftest_" + name.replace(".txt", ".wav"))
        wait_all([run_render(cli, sp, wp, fs)])
        x, _ = read_mono(wp)
        truth = read_score_notes(sp)
        notes = transcribe(x, fs, W, latency)
        p, r, f1, pairs = evaluate(truth, notes)
        offset, _ = level_match(x, fs, notes, cal_levels, cli, work, 90.0, dry=True)
        e0 = [abs(level_to_velocity(e[2], e[3], cal_levels) - t[2]) for t, e in pairs]
        e1 = [abs(level_to_velocity(e[2], e[3] + offset, cal_levels) - t[2]) for t, e in pairs]
        errs0 += e0
        errs1 += e1
        tot["tp"] += len(pairs)
        tot["ne"] += len(notes)
        tot["nt"] += len(truth)
        print(f"{name}: {len(truth)} notes, {len(notes)} detected, P {p:.2f} R {r:.2f} F1 {f1:.2f}; "
              f"median |vel err| {np.median(e0) if e0 else float('nan'):.1f} (offset 0), "
              f"{np.median(e1) if e1 else float('nan'):.1f} (auto offset {offset:+.1f} dB)")
        fp = [n for n in notes if not any(n is e for _, e in pairs)]
        if fp:
            print("  unmatched detections (t, key):",
                  " ".join(f"{n[0]:.2f}/{n[2]}" for n in fp[:12]))
        miss = [t for t in truth if not any(t is q for q, _ in pairs)]
        if miss:
            print("  missed (t, key):", " ".join(f"{t[0]:.2f}/{t[1]}" for t in miss[:12]))
    p = tot["tp"] / max(tot["ne"], 1)
    r = tot["tp"] / max(tot["nt"], 1)
    f1 = 2 * p * r / max(p + r, 1e-12)
    print(f"overall: P {p:.2f} R {r:.2f} F1 {f1:.2f}; median |vel err| "
          f"{np.median(errs0):.1f} (offset 0), {np.median(errs1):.1f} (auto offset)")
    return f1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording", nargs="?")
    ap.add_argument("--out")
    ap.add_argument("--templates", default=os.path.join(tempfile.gettempdir(), "pm_templates"))
    ap.add_argument("--render-cli", default=DEFAULT_CLI)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--no-level-match", action="store_true")
    ap.add_argument("--level-seconds", type=float, default=90.0)
    ap.add_argument("--absolute-level", action="store_true",
                    help="set the offset by matching the recording's absolute RMS (only for "
                         "recordings whose gain is known relative to the model)")
    ap.add_argument("--median-velocity", type=float, default=64.0)
    a = ap.parse_args(argv)
    t0 = time.time()
    os.makedirs(a.templates, exist_ok=True)
    work = tempfile.mkdtemp(dir=a.templates)
    if a.selftest:
        selftest(a.render_cli, a.templates, work)
        print(f"runtime {time.time() - t0:.0f} s")
        return 0
    if not a.recording or not a.out:
        ap.error("REC.wav and --out are required")
    x, fs = read_mono(a.recording)
    W, cal_levels, latency = build_templates(a.render_cli, a.templates, fs)
    t1 = time.time()
    notes = transcribe(x, fs, W, latency)
    print(f"{len(notes)} notes from {len(x) / fs:.0f} s of audio (NMF and onsets {time.time() - t1:.0f} s)")
    offset, gap = 0.0, float("nan")
    if not a.no_level_match and notes:
        offset, gap = level_match(x, fs, notes, cal_levels, a.render_cli, work, a.level_seconds,
                                  absolute=a.absolute_level, median_velocity=a.median_velocity)
    vels = [level_to_velocity(k, l + offset, cal_levels) for _, _, k, l in notes]
    with open(a.out, "w") as f:
        f.write(f"# transcribed from {os.path.basename(a.recording)} by transcribe.py: "
                f"{len(notes)} notes, level offset {offset:+.1f} dB, recording - render {gap:+.1f} dB\n")
        f.write(notes_to_score(notes, cal_levels, offset))
    if vels:
        print(f"level offset {offset:+.1f} dB, recording - render gap {gap:+.1f} dB; velocity median {np.median(vels):.0f}, "
              f"10-90 % {np.percentile(vels, 10):.0f}-{np.percentile(vels, 90):.0f}")
    print(f"wrote {a.out}; runtime {time.time() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
