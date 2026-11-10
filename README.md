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
offline finite-difference reference. There is no soundboard (the bridge
admittance is a placeholder), no sympathetic coupling between keys, no
longitudinal modes and no Apple code yet.

## Layout

| Path | Contents |
|---|---|
| `core/pm_base/` | Header-only: 64-byte aligned arrays, SIMD wrapper (NEON / AVX2+FMA / SSE2 / scalar), flush-denormals guard (x86 MXCSR, ARM FPCR), SPSC queue, fast `pow` for the felt law |
| `core/pm_string/` | `ModeBank` (complex one-pole resonators, structure of arrays, group culling, per-key segments with their own I/O and damper loss, complex input weights, free-running kernel), string parameters to modal coefficients, pluck |
| `core/pm_hammer/` | Hunt-Crossley felt hammer and the contact solver for one hammer on 1-3 strings, on normal modes |
| `core/pm_voicing/` | Non-real-time: 88-key table v0, complex eigen-solver, bridge admittance interface and placeholder, unison normal modes, virtual tuner, velocity curve, key parameters to bank and contact coefficients |
| `core/pm_engine/` | Engine: event queue and scheduling, hammer pool, dampers and the three pedals, render loop, stereo output stage |
| `core/pm_api/` | Stable C API (`pm_api.h`) for Swift and Python |
| `tools/render_cli/` | Offline renderer: one plucked string to a mono WAV, or a text score through the engine to a stereo WAV |
| `tools/reference_fdtd/` | Offline finite-difference stiff string + hammer (library and CLI), the reference for the real-time model |
| `tools/bench/` | Modal kernel benchmark and engine benchmark |
| `tools/analysis/` | Python (numpy): partial frequencies and T60 of a string; attack centroid and partial levels; two-stage decay and beating of piano notes |
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
  shift caused by the bridge reactance is compensated per key.
- **Bridge (placeholder until M4).** `PlaceholderBridgeAdmittance`
  (`core/pm_voicing/bridge_admittance.h`): Re Yvv = G(key) (f / f_key)^0.3
  with G = 2.5-5e-4 s/kg at each key's fundamental (impedance 2,000-4,000
  kg/s), a reactive part from +0.3 (spring-like, low f) to -0.2 (mass-like,
  high f), Yhh = Yvv / 12, Yvh = 0.25 Yvv / sqrt(12). Behind the
  `BridgeAdmittance` interface so that M4 can supply the soundboard's
  driving-point admittance. Internal losses b1, b3 were reduced so that the
  bridge supplies the prompt-sound loss: aftersound (internal) T60 of the
  fundamental about 60, 45, 20, 7, 3.5, 1.5 s and prompt T60 about 25, 14,
  3.5, 1.2, 0.6, 0.3 s at A0, C2, C4, C6, C7, C8.
- **Unison detuning.** Per-string cents offsets from a deterministic seed
  (a spread pattern, randomly permuted, +-30 % jitter, zero mean; about
  +-0.5 cent), times a global unison width; horizontal modes 0.2 cent sharp.
- **Tuning (plan §2.3).** A virtual tuner sets each key's sounding first
  partial: A4 = 440 Hz, temperament octave A3-A4 as a 4:2 octave divided
  equally, 4:2 octaves outwards blending to 2:1 (E5-E6) and towards the 4:1
  double octave above C7, and to 6:3 (C3-D2) and towards 10:5 at A0. Result:
  -24 cents at A0, -3.5 at C2, -1.5 at C4, 0 at A4, +4.2 at C6, +34 at C8.
  Stretch amount 0-150 %, A4 and plain equal temperament are options.
- **Output.** The vertical bridge force of each key (plus the vertical
  bridge motion caused by horizontal string force, Yvh / Yvv), summed
  through a fixed stereo spread, a 5 Hz DC blocker, master gain and a soft
  clipper (transparent below -1 dBFS, never reaches full scale).
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
  as a spring in series with each felt. See `core/pm_hammer/hammer.h`.
- **Pedals.** Sustain (CC64, on/off at half travel). Sostenuto (CC66): the
  keys whose dampers are held up by their keys when it goes down stay
  undamped until it comes up. Una corda (CC67, continuous, latched per
  strike): on trichords string 0 is reached u^2 x 2.5 mm later (grazed,
  then missed), and the felt K falls by 35 % x u on every key.
- **Dampers.** Extra loss per mode, weighted by the mode shape under the
  felt; landing follows a raised-cosine curve over 20-80 ms (faster with
  higher release velocity). Keys 69-88 have no damper.
- **Events.** `pm::Event` through a wait-free SPSC queue (one producer).
  Times are host-clock ticks (mapped per render call by a `ClockMapping`)
  or absolute engine sample indices. Lowest-latency mode puts due events at
  the start of the buffer; constant-latency mode adds a fixed delay
  (default: one buffer) to host-time events. Buffers are cut at event times
  and on fixed grids, so output is bit-identical for any buffer size and a
  note-on lands on its exact sample.
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
`time cc64 VALUE` (sustain), `time cc66 VALUE` (sostenuto), `time cc67 VALUE`
(una corda), `time all_notes_off`, `time panic`; KEY is a MIDI note number or
a name such as C4 or F#3):

```sh
./build/tools/render_cli/render_cli --score tools/render_cli/scores/single_notes.txt --out notes.wav
```

It prints every strike's hammer velocity, contact duration, number of
contacts and una corda amount. Example scores: `single_notes.txt` (C2, C4,
C7 at five velocities), `chord_pedal.txt` (a chord with and without pedal),
`repeated_note.txt` (A4 at 12 strikes per second), `double_decay.txt` (one
25 s mf C4), `una_corda.txt` (C4 with the una corda up, half and fully
down). Tuning options: `--a4 HZ`, `--stretch X` (0-1.5),
`--equal-temperament`, `--unison-width X`. `--help` lists all options.

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
cluster struck every 100 ms, and the cost of one hammer per buffer for
eight keys. Numbers from x86 machines are only indicative for the Apple
silicon target.
