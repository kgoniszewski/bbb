# Physically modelled piano engine

A portable C++20 engine for a physically modelled concert grand, built
milestone by milestone from the engineering plan in
[docs/steinway-d-physical-model-plan.md](docs/steinway-d-physical-model-plan.md).

Current state: milestones M0 (build, CI, tooling skeleton), M1 (modal string
bank with scalar and SIMD kernels, culling, kernel benchmark), the portable
part of M2 (nonlinear hammer per key, per-key table, dampers and sustain, a
sample-accurate engine with an event queue, a C API, a piano mode in
`render_cli`) and M3: coupled-mode unisons of 1-3 strings in two
polarizations (double decay and beating), one hammer on all strings of a
unison with una corda, sostenuto, stretch tuning by a virtual tuner, and an
offline finite-difference reference, and M4: a modal soundboard (v0 mode
set from an orthotropic plate) coupled to every string through the bridge
by a one-sample run-time loop (sympathetic resonance, pedal bloom),
continuous half-pedalling, hammer-knock and damper-lift noise hooks, and a
placeholder two-pickup output. There is no radiation or microphone model,
no room, no longitudinal modes or duplex and no Apple code yet.

## Layout

| Path | Contents |
|---|---|
| `core/pm_base/` | Header-only: 64-byte aligned arrays, SIMD wrapper (NEON / AVX2+FMA / SSE2 / scalar), flush-denormals guard (x86 MXCSR, ARM FPCR), SPSC queue, fast `pow` for the felt law |
| `core/pm_string/` | `ModeBank` (complex one-pole resonators, structure of arrays, group culling, per-key segments with their own I/O and damper loss, complex input weights, free-running kernel), string parameters to modal coefficients, pluck |
| `core/pm_hammer/` | Hunt-Crossley felt hammer and the contact solver for one hammer on 1-3 strings, on normal modes |
| `core/pm_board/` | Real-time soundboard modal bank (zone ports in and out, pickup read-outs, sleep) and the key/zone mixing of the coupling loop |
| `core/pm_voicing/` | Non-real-time: 88-key table v0, complex eigen-solver, bridge admittance interface (smooth placeholder kept for unit tests), soundboard mode set v0 and its admittances (`soundboard_model.h`), unison normal modes, virtual tuner, velocity curve, key parameters to bank and contact coefficients |
| `core/pm_engine/` | Engine: event queue and scheduling, hammer pool, dampers and the three pedals (continuous sustain), soundboard loop, noise hooks, render loop, output stage |
| `core/pm_api/` | Stable C API (`pm_api.h`) for Swift and Python |
| `tools/render_cli/` | Offline renderer: one plucked string to a mono WAV, or a text score through the engine to a stereo WAV |
| `tools/reference_fdtd/` | Offline finite-difference stiff string + hammer (library and CLI), the reference for the real-time model |
| `tools/bench/` | Modal kernel benchmark and engine benchmark |
| `tools/analysis/` | Python (numpy): partial frequencies and T60 of a string; attack centroid and partial levels; two-stage decay and beating of piano notes; third-octave band levels and band decay (bloom, sympathetic, half-pedal) |
| `tests/` | Self-contained C++ unit tests (and one C test of the API), run by CTest |

## Engine

- **Keys and unisons (plan §1.3).** Keys 1-8 have one string, 9-20 two,
  21-88 three. For every partial n of a key, its 2S oscillators (S strings
  x vertical/horizontal, Im z = modal displacement) are coupled through the
  2x2 bridge admittance Y(f_n):
  `dz/dt = A z,  A = diag(j w_k - sigma_k) - kappa E^T Y E,  kappa = (T/L)(1 + B n^2)`
  (Weinreich; the resonant part of the moving-termination force, exact to
  first order in the weak coupling; a single string on a resistive bridge
  decays at `2 f0 Z0 Re Y`). A is diagonalised offline by a small complex
  eigen-solver (Householder-Hessenberg + shifted QR + back-substitution;
  exceptional points are detected through the condition number of V and
  handled by a deterministic perturbation, falling back to uncoupled
  strings). Each normal mode is one bank resonator; its state is rotated so
  that its output weight is real (complex output weights cost nothing in
  the kernel), and it carries complex strike-point projections and force
  inputs per string for the hammer. Partials 1-20 keep all 2S normal modes,
  higher ones the 3 with the largest excitation x output (configurable):
  17,034 modes for the default table at 48 kHz. The first-partial pitch
  shift caused by the bridge is compensated per key (see Tuning).
- **Soundboard (plan §1.6, `core/pm_voicing/soundboard_model.h`).** v0
  mode set from a simply supported orthotropic plate, 1.93 x 1.25 m, ribs
  and bridges folded into equivalent stiffnesses (Dx 2,430, Dy 20,000,
  Dxy 2,090 N m) and 6.6 kg/m^2 (15.9 kg); loss factor 3 % above about
  1 kHz, rising to 10 % at the lowest modes (+ 0.08 / (1 + (f / 150 Hz)^2):
  rim and case participation, an estimate), mode frequencies scattered by
  +-2 % (seeded) to break the uniform plate's regularity. 190 modes below
  5 kHz (configurable cap and mode count), first mode 57.6 Hz, one mode per
  24-26 Hz above 1 kHz; mean driving-point impedance at the bridge 1,400-
  1,900 kg/s (0.2-8 kHz). Bass bridge, 40-500 Hz (keys 1-52): median |Z|
  1,130 kg/s, every key's minimum above 150 kg/s, Re Y peaks within 8x its
  mean (the plain 3 % plate dipped to 50-70 kg/s, peaks 18-25x). Treble bridge (keys 21-88) and bass bridge (1-20)
  are Bezier curves on the plate; every key has its own bridge point (shape
  values averaged over a 5 cm contact) and reaches the run-time board
  through 18 zones (4 bass, 14 treble) by hat-function weights. Above 5 kHz
  a mean-value continuum of modes continues the admittance (real part ->
  n / 4M). The data structure (frequencies, damping, shapes at keys, zones
  and pickups) is what a measured or finite-element set would fill.
- **Bridge admittance seen by a key.** Vertical: the board's driving point
  (modes + continuation). Horizontal and cross terms remain parametric: the
  bridge rocks with the vertical motion (r = 0.25 / sqrt(12)) plus an
  independent horizontal compliance (mean conductance / 12). The real part
  is positive semi-definite.
- **Baked versus fed back (no double counting).** The run-time loop feeds
  the board modes back to the strings of every key whose partials lie in the
  loop band: the modal range (5 kHz) at the Full quality tier, 2 kHz at
  Balanced, 1 kHz at Eco (`SoundboardConfig::tier`, `loopCutoffHz`; C API
  `PM_PARAM_QUALITY_TIER`, `PM_PARAM_LOOP_CUTOFF_HZ`). Its exact self-term at key k (the one-sample discrete loop
  with the zone-interpolated shape, the sympathetic gain g and the string's
  residual end stiffness) is removed from that key's bake, so bake + loop =
  driving point (real part of the remainder clamped at 0 where interpolated
  shapes exceed the true point: 1.6 % of frequency points). Partials above
  the loop band bake the full admittance and only drive the board. The
  admittance is evaluated at the complex frequency of each partial's
  dominant mode. `test_soundboard_coupling` measures a key alone on the
  board with the real loop: 20 partials of C2, C4, C6 decay as predicted,
  median deviation 0.23 %, worst 3.5 % (C2's fundamental, next to the
  board's two lowest modes).
- **Loop (one-sample feedback, solved per sub-block, `core/pm_board/soundboard.h`).**
  The recursion: hammers and the coupled string modes advance with the input
  computed at the previous sample, each key's bridge force (minus its
  static correction, below) goes into the zone forces, the board advances
  and returns zone displacements, which become each key's input. With this
  alignment every board mode's loop transfer is positive real, so the
  zone-port loop is passive (`test_passivity`: 10^4 random reduced systems,
  no energy growth; a loop fed back with the wrong sign is flagged). It is
  evaluated exactly but block-wise (`CouplingLoop`): over a sub-block a
  key's bridge force is its free response (the ordinary block kernel) plus
  its input convolved with its impulse response; summed into the zones
  this is a banded zone-to-zone kernel (about 50 entries for 18 zones), so
  only the hammers, that kernel and the board run per sample, and at the end
  of the sub-block each coupled mode adds the input's share to its state
  (Horner in c). The same recursion, rearranged: its trajectory equals the
  per-sample loop to rounding.
  Static correction: driven by a bridge displacement, each string mode
  responds quasi-statically with 2T/L (1 + B n^2) u, the same for every
  partial, so the truncated modal sum is a large negative stiffness on the
  board (uncorrected, a few keys in a zone diverge at 0 Hz); each key
  subtracts its exact static response, K0 u[n] + K1 u[n-1] matched to the
  discrete bank's static gain and slope, leaving the convergent, passive
  end stiffness of the string.
- **Output (PLACEHOLDER until the radiation model).** Board velocity at two
  pickup points (bass side left, treble side right) plus a direct path for
  partials above the modal range (panned bridge forces, high-passed at
  4.5 kHz, scaled by the board's mean transfer admittance near 5 kHz).
  Level, with mf = MIDI 80 and ff = 112: mf single notes C3-C5 peak at -33
  to -22 dBFS, velocity 127 at -17 dBFS at most, 12-note clusters every
  100 ms at -7 (ff) to -5 dBFS (127, pedal down), all 88 keys at 127 at once
  -1.4 dBFS: the limiter (above -1 dBFS) is not reached.
  `SoundboardConfig::enabled = false` gives the M3 output (A/B).
- **Decay times.** The bake follows the board, so decay varies note to
  note and partial to partial. Fundamentals at mf, prompt / aftersound T60
  measured on the engine output: C2 7.6 / 52 s, C3 16 / 6.8 s, C4 2.6 /
  24 s, C5 0.44 / 12 s, C6 1.9 / 7.5 s. Neighbouring keys' prompt decay of
  the fundamental differs by a median factor 1.6 (90 % below 3.3, worst
  8.8, around C5): fewer extremes than the plain 3 % plate (90 % below 5.9,
  worst 14, one growing mode), but still a v0 board.
- **Unison detuning.** Per-string cents offsets from a deterministic seed
  (a spread pattern, randomly permuted, +-30 % jitter, zero mean; about
  +-0.5 cent), times a global unison width; horizontal modes 0.2 cent sharp.
- **Tuning (plan §2.3).** A virtual tuner sets each key's sounding first
  partial: A4 = 440 Hz, temperament octave A3-A4 as a 4:2 octave divided
  equally (on the second partials), 4:2 octaves outwards blending to 2:1
  (E5-E6) and towards the 4:1 double octave above C7, and to 6:3 (C3-D2)
  and towards 10:5 at A0. Result on the table: -24 cents at A0, -3.5 at C2,
  -1.5 at C4, 0 at A4, +4.2 at C6, +34 at C8. Stretch amount 0-150 %, A4
  and plain equal temperament are options. With the soundboard, the engine
  re-runs the tuner on the partials as they sound on the board (bridge
  shifts and string-board hybrids included; targets move by up to 13 cents
  where a board mode pulls a fundamental against its upper partials, D2),
  and compensates each key so that its sounding first partial (energy-
  weighted mean of the string-dominated normal modes over the first second;
  a partial on a board resonance is solved together with the nearby board
  modes) is on target: every key within 1 cent measured alone (one listed
  exception at 1.1 cents, `test_engine_sound`).
- **Hammer contact.** One hammer (whole hammer mass) meets the S strings of
  its key at small per-string offsets (hammer-string mating, +-15 um), one
  Hunt-Crossley felt contact per string. While active it owns its key's
  normal modes and advances them in substeps of 1/(R fs) (R = 1 below C4, 2
  to key 75, 4 in the top octave); modes and hammer are integrated exactly
  under forces held over each substep, each felt force is the discrete
  gradient of its felt potential, and the S coupled equations are solved by
  Newton's method (a bracketed nested solve is the counted fallback). The
  scheme is energy-exact for lossless felt and strings and passive for any
  felt stiffness. Felt `x^p` uses a table-driven `fastPow` (3e-14 relative
  error). The static compliance of the partials above the bank's limit acts
  as a spring in series with each felt. Each substep's force is applied in
  the next substep's propagation pass (one pass over the modes per substep
  instead of two); the hammer also takes the key's sympathetic input. See
  `core/pm_hammer/hammer.h`.
- **Pedals (plan §1.9).** Sustain (CC64) is continuous: the position is
  smoothed (10 ms, from the exact event time) and mapped per register
  (bass dampers lift later: transition zone from 0.40 in the bass and 0.30
  in the treble, 0.25 wide) to a felt position: pressed into the string
  (full damping), light contact (damping falls to 15 %), then a gap: a
  vibration of amplitude A reaches the felt for acos(gap / A) / pi of each
  cycle (amplitude-dependent damping), clear 3 mm at full pedal. Only decay
  rates change, so pedal moves never click and re-pedalling keeps what is
  left in the strings. Bloom and sympathetic resonance come from the loop
  alone. Sostenuto (CC66): the keys whose dampers are held up by their keys
  when it goes down stay undamped until it comes up. Una corda (CC67,
  continuous, latched per strike): on trichords string 0 is reached
  u^2 x 2.5 mm later (grazed, then missed), and the felt K falls by
  35 % x u on every key. A silent key-down event lifts a damper without a
  strike.
- **Dampers.** Extra loss per mode, weighted by the mode shape under the
  felt; landing follows a raised-cosine curve over 20-80 ms (faster with
  higher release velocity). Keys 69-88 have no damper.
- **Mechanical noises (plan §1.8, switchable, modest by default).** Hammer
  knock: a 1.5 ms raised-cosine force into the key's zones, 0.02 N per m/s
  of hammer speed. Damper-lift noise: low-passed noise into every zone,
  proportional to pedal speed and to the share of dampers the pedal moves.
- **Culling.** Groups sleep when their output bound (through the board's
  peak gain, +26 dB) is below the threshold; a sleeping coupled group wakes
  when its input could have moved it above. Keys with a seated damper are
  not woken by bridge input (plan §1.5: with the pedal up only held notes
  and the undamped top section stay awake); their release tails decay
  normally.
- **Events.** `pm::Event` through a wait-free SPSC queue (one producer).
  Times are host-clock ticks (mapped per render call by a `ClockMapping`)
  or absolute engine sample indices. Lowest-latency mode puts due events at
  the start of the buffer; constant-latency mode adds a fixed delay
  (default: one buffer) to host-time events. Buffers are cut at event times
  and on fixed grids, so output is the same for any buffer size and a
  note-on lands on its exact sample (to rounding, -100 to -110 dB: the
  soundboard loop sums per sub-block).
- **Real-time.** `render()` does not allocate, lock, throw or do I/O
  (checked by a test that counts every `operator new`).

## Build and test

Requires CMake 3.20+ and a C++20 compiler.

```sh
cmake -S . -B build                 # Release by default
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Options: `-DPM_NATIVE_ARCH=ON` compiles for the host CPU (selects the AVX2
kernel on x86; the default x86 build uses SSE2), `-DPM_BUILD_TESTS=OFF`,
`-DPM_BUILD_TOOLS=OFF`, `-DPM_BUILD_BENCH=OFF`. Adding
`-DPM_SIMD_FORCE_SCALAR` to `CMAKE_CXX_FLAGS` builds the portable fallback
kernel, and `-DPM_GROUPS_PER_STEP=N` overrides how many 16-mode groups the
SIMD kernel interleaves.

## Render

Single string:

```sh
./build/tools/render_cli/render_cli --f0 261.63 --B 3.8e-4 --b1 0.5 --b3 6.25e-9 \
    --pluck 0.12 --seconds 5 --fs 48000 --out c4.wav
```

Piano, from a text score (`time note_on KEY VEL`, `time note_off KEY`,
`time key_down KEY` (silent key press), `time cc64 VALUE` (sustain,
continuous), `time cc66 VALUE` (sostenuto), `time cc67 VALUE` (una corda),
`time all_notes_off`, `time panic`; KEY is a MIDI note number or a name such
as C4 or Bb3 (`#` starts a comment, so write sharps as flats)):

```sh
./build/tools/render_cli/render_cli --score tools/render_cli/scores/single_notes.txt --out notes.wav
```

It prints every strike's hammer velocity, contact duration, number of
contacts and una corda amount. Example scores: `single_notes.txt` (C2, C4,
C7 at five velocities), `chord_pedal.txt` (a chord with and without pedal),
`repeated_note.txt` (A4 at 12 strikes per second), `double_decay.txt` (one
25 s mf C4), `una_corda.txt` (C4 with the una corda up, half and fully
down), `pedal_bloom.txt` (a staccato chord with the pedal down and up),
`half_pedal.txt` (C3 releases over fixed pedal positions, then a pedal
sweep over a ringing note), `sympathetic.txt` (C3 held silently while C4
is struck staccato, then Db3 held, then nothing held). Tuning options:
`--a4 HZ`, `--stretch X` (0-1.5), `--equal-temperament`, `--unison-width X`;
soundboard: `--no-board` (A/B), `--sympathetic G`, `--knock N`,
`--damper-noise N`. `--help` lists all options.

## Analyse

```sh
python3 tools/analysis/analyze_string.py c4.wav --f0 261.63 --B 3.8e-4 --b1 0.5 --b3 6.25e-9
```

Prints, per partial, expected and measured frequency (error in cents) and
measured T60, plus the expected T60 when `--b1` and `--b3` are given.

```sh
python3 tools/analysis/analyze_note.py notes.wav --note 60 --at 12.5,15,17.5,20,22.5
```

For each segment (one note each): the spectral centroid of the first 50 ms
after the onset and the levels of the first partials, then a
centroid-per-segment table (centroid against velocity for the example score).

```sh
python3 tools/analysis/analyze_decay.py double_decay.wav --note 60 --at 0 --seconds 25
python3 tools/analysis/analyze_decay.py una_corda.wav --note 60 --at 0,8,16 --seconds 7.5
```

Per partial: frequency, level, a two-stage decay fit (prompt and aftersound
T60, break time, crossover level) and the beat rate (two-component
linear-prediction fit of the partial's baseband).

```sh
python3 tools/analysis/analyze_bands.py pedal_bloom.wav --at 0.4,5.2 --labels "pedal down,pedal up"
python3 tools/analysis/analyze_bands.py sympathetic.wav --at 0.7,5.7,10.7 --window 1 --labels "C3,Db3,none"
```

Third-octave band levels some time after each given instant (a release)
and the band decay time from the following window, each segment also
relative to the first.

## Offline FDTD reference

```sh
./build/tools/reference_fdtd/reference_fdtd --key 40 --velocity 2.2 --points 400 \
    --seconds 0.5 --wav fdtd_c4.wav --csv fdtd_c4.csv
```

One string of a key (with its share of the hammer mass) as an explicit
finite-difference stiff string (Bensa form of the plan's equation, the
mixed-derivative loss b3' = b3 c^2; simply supported ends; see
`tools/reference_fdtd/fdtd_string.h`) struck by the same Hunt-Crossley
hammer. It prints the grid, the contact and the partial energies after the
strike, and writes the bridge force (WAV) and hammer force, strike-point
displacement and bridge force (CSV). `test_fdtd` compares the real-time
model against it: partial energies of the first 30 partials agree within
1 dB from A0 to C7 at pp, mf and ff (largest +0.89 dB, C2 ff); see the test
output for the top octave, where both models put only 1e-4 to 1e-8 of the
hammer's energy into the strings.

## Benchmark

```sh
./build/tools/bench/bench_modal            # 26,000 modes, 48 kHz, 64-frame blocks
```

Reports nanoseconds per mode-sample and the percentage of one core needed in
real time, for the scalar reference and the SIMD kernel, with a (complex)
input and free-running (the engine's case: no input).

```sh
./build/tools/bench/bench_engine           # 48 kHz, 64-frame buffers
```

Engine cost in % of one core: all 88 keys struck and held with the pedal
down (with culling, and with culling off so every mode runs), a typical
case (a four-note chord re-struck twice a second, pedal up), a 12-note ff
cluster struck every 100 ms, a pedal-down sympathetic case (staccato chords
every 0.5 s, every string free), the worst and cluster cases with the board
off (A/B), and the cost of one hammer per buffer for eight keys;
`--tier balanced|eco` and `--tiers` run the board cases at the other
quality tiers. Numbers from x86 machines are only indicative for the Apple
silicon target (SSE2, 4-wide like NEON, is the closer proxy). A coupled
string mode costs the free block kernel plus a forced pass of about 0.6
times that.

## Tests

`ctest` runs 25 suites. The M4 ones: `test_soundboard_model` (board
targets, positive-real loop, admittance against a time-domain drive of the
run-time bank, bake + loop = driving point), `test_soundboard_coupling`
(no double counting), `test_passivity` (10^4 random reduced systems with
the production components; a sign-flipped loop is flagged), `test_sympathetic` (silently held key, pedal
bloom per third octave, monotonic in the sympathetic gain) and
`test_half_pedal` (graded T60 against pedal position, no click on pedal
moves, re-pedalling keeps energy). Unit tests of the unison solver and the
M3 double-decay tests use the smooth placeholder admittance, which the
soundboard replaced in the engine.
