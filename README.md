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
placeholder two-pickup output, and the app half of M2: a portable host
layer (MIDI 1.0 / UMP parsing, event queues from the MIDI and UI threads,
a prepare gate, a render-load meter with diagnostics and a speed probe, benchmark scenes on the audio thread, per-CPU render statistics; `core/pm_host`, C API 0.11), an
offline benchmark library shared by `bench_engine` and the app (`core/pm_bench`), a Swift
package exposing the C API, and a SwiftUI app for iPad and Mac (see
[iPad app](#ipad-app); not yet built by anyone, see its checklist). There
is no radiation or microphone model, no room, no longitudinal modes or
duplex yet.

## Layout

| Path | Contents |
|---|---|
| `core/pm_base/` | Header-only: 64-byte aligned arrays, SIMD wrapper (NEON / AVX2+FMA / SSE2 / scalar), flush-denormals guard (x86 MXCSR, ARM FPCR), SPSC queue, fast `pow` for the felt law |
| `core/pm_string/` | `ModeBank` (complex one-pole resonators, structure of arrays, group culling, per-key segments with their own I/O and damper loss, complex input weights, free-running kernel), string parameters to modal coefficients, pluck |
| `core/pm_hammer/` | Hunt-Crossley felt hammer and the contact solver for one hammer on 1-3 strings, on normal modes |
| `core/pm_board/` | Real-time soundboard modal bank (zone ports in and out, pickup read-outs, sleep) and the key/zone mixing of the coupling loop |
| `core/pm_voicing/` | Non-real-time: 88-key table v0, complex eigen-solver, bridge admittance interface (smooth placeholder kept for unit tests), soundboard mode set v0 and its admittances (`soundboard_model.h`), unison normal modes, virtual tuner, velocity curve, key parameters to bank and contact coefficients |
| `core/pm_engine/` | Engine: event queue and scheduling, hammer pool, dampers and the three pedals (continuous sustain), soundboard loop, noise hooks, render loop, output stage |
| `core/pm_host/` | App host glue, real-time and portable: MIDI 1.0 byte-stream parser (running status), UMP decoder (MIDI 1.0 and 2.0 protocol), Core MIDI `MIDIEventList` reader, MIDI to engine events, MIDI/UI/monitor queues, prepare gate, render-load meter |
| `core/pm_bench/` | Non-real-time: the engine benchmark scenarios (worst, typical, cluster, bloom, board-off A/Bs, then the live-playing scenes legato, pedalling, chord release, idle, hammer burst, pedalled chords and the fresh- and same-key bursts, and the hammer per key; back to back or paced to real time), shared by `bench_engine` and the app's Benchmark screen |
| `core/pm_api/` | Stable C API (`pm_api.h`) for Swift and Python: engine, (0.6) the app host `pm_host_*`, (0.7) load diagnostics, build information and the offline benchmark `pm_bench_*`, (0.8) phase timing and the worst buffer's events, (0.9) the worst buffer's hammer and zone-kernel detail, the speed probe and `PM_BENCH_PHASE_TIMING`, (0.10) paced benchmark runs, per-buffer work counts (`pm_work_detail`), the effective engine configuration (`pm_engine_describe`) and benchmark scenes played on the audio thread (`pm_host_scene_*`), (0.11) the speed probe's latency- and throughput-bound parts, per-CPU render statistics (`pm_cpu_usage`) and the paced benchmark's pacing hook; `include/` holds the Swift module map |
| `Package.swift` | Swift package `PianoEngine`, library `PianoCore` (the C++ core, imported in Swift through the C API) |
| `apps/PianoApp/` | SwiftUI app (iPadOS 17+, macOS 14+): XcodeGen spec `project.yml`, sources, and the package's Swift smoke test |
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
./build/tools/bench/bench_engine                         # 48 kHz, 64-frame buffers
./build/tools/bench/bench_engine --block 512 --fs 44100  # what an iPad app on a Mac is granted
./build/tools/bench/bench_engine --timing both           # every case with phase timing off and on
./build/tools/bench/bench_engine --block 512 --fs 44100 --compare   # back to back vs paced vs paced + eviction
./build/tools/bench/bench_engine --paced --only burst    # paced to real time, only the cases named "...burst..."
```

Engine cost in % of one core: all 88 keys struck and held with the pedal
down (with culling, and with culling off so every mode runs), a typical
case (a four-note chord re-struck twice a second, pedal up), a 12-note ff
cluster struck every 100 ms, a pedal-down sympathetic case (staccato chords
every 0.5 s, every string free), the worst and cluster cases with the board
off (A/B), then (appended, so that the earlier lines stay comparable)
scenes with the events of live playing: a legato melody with overlapping
notes released while they sound, chords with legato pedalling (CC64 as
ramps of many values and as plain on/off), a ten-note chord released
together, an idle engine, a hammer burst (ten bass and middle notes at
velocity 127 struck at the same sample, every second) and pedalled chords
(eight different roots across the keyboard with legato on/off pedalling,
which keeps the loop's awake set changing as in live playing), then a
fresh-key burst (pedal down, a bass note and a chord held, and every 0.6 s
six notes at velocity 100-127 in one buffer on keys not struck for at
least 6 s, two of them released after 30 ms; every fourth burst lands in a
legato pedal change, so those staccato notes are damped while their
hammers are still active) and the same with the same six keys every time
(same-key burst, the warm counterpart); last, the
cost of one hammer per buffer for eight keys (at mf, 2.2 m/s; the scenes
use MIDI velocities 70-127, the clusters 120, the burst 127, i.e. 5.5
m/s). `--tier balanced|eco` and `--tiers` run the board cases at the
other quality tiers; `--block` and `--fs` set the buffer size and sample
rate (every load is relative to the buffer's own period). Numbers from x86 machines are only indicative for the Apple
silicon target (SSE2, 4-wide like NEON, is the closer proxy). A coupled
string mode costs the free block kernel plus a forced pass of about 0.6
times that.

After the summary lines comes a per-phase breakdown of every case, in µs
per buffer: the most expensive buffer, the mean over the top 1 % (the
buffers at or above p99) and the mean over all buffers, split into
`events` (event handling, control tick, wake-ups, impulse-response cache
and loop-kernel updates), `hammer` (contact solves), `free` (the bank's
block pass; board off: the whole bank), `forced` (input added to the
coupled modes), `board` (zone kernel, noise, knocks, board step) and
`other` (output, filters, limiter, cull passes). It lets a run on the
target attribute peaks without a profiler: note-onset buffers are mostly
`hammer`, whose cost per hammer can be compared with the hammer-solver
lines. The breakdown uses the engine's run-time phase timing
(`Engine::setPhaseTiming`, C API `pm_host_set_phase_timing`, the app's
Phase timing toggle; there is no separate profiling build any more). It
reads the clock about five times per 32-frame sub-block, plus twice per
sample while a hammer is active, included in every number; `--timing
both` measures what that costs (on the x86 container: within the run-to-run
noise for every playing scene, about +1.3 µs per idle 512-frame buffer),
`--timing off` skips it.

Pacing. By default every buffer is rendered back to back, as fast as the
thread can, with everything the engine touches hot in the caches and the
branch predictors trained. `--paced` makes the benchmark thread sleep
after each buffer until its real-time deadline (start + (b + 1) × period,
absolute, as an audio thread waits for its next callback), timing only the
render call; `--paced-evict [MiB]` also walks a scratch buffer (default
32 MiB, one read-modify-write per cache line) between buffers, outside the
timing, to emulate other work on the core. `--compare` runs every case
five ways: back to back (`b2b`), paced busy-waiting to each deadline
(`spin`: the core stays awake and the caches keep what the engine left),
paced sleeping (`sleep`, as an audio thread), sleeping with the eviction
walk (`evict`) and busy-waiting with it (`sp+ev`: the caches alone); it
prints mean / p99 / max, the buffers over the period, the mean phase
split, the hammers phase per hammer-sample and the median of the speed
probe run right before every 4th render call (untimed, as the app's host
runs it), with each mode's ratio to back to back. Sleep against spin is
the core's power state and clock after the gap, sp+ev against spin the
caches. `--only text` restricts any run to the cases whose name contains
the text.

The hammers phase (board on) is, for every sample during which a hammer
is active (from its launch, through contact, until it is released on the
32-sample grid after it has provably left the strings or its release time
has passed), and for every active hammer: the key's loop input, one pass
over that key's modes through its interleaved sample-path record (B, wIn,
P, C^R, wOut: (4SR + 5) floats per mode, 20-25 KB for a bass or middle
key) plus the state, the R contact solves while it touches, and, when the
key's damper is on (a note released while its hammer is still active),
two more passes per sample (flush and damper). It does not include the
free, forced or board work of those samples (each is timed separately),
nor other keys' modes. Its cost therefore scales with hammer-samples
stepped, not with hammer-samples in contact; the work counts give both
(`us/hs`, `us/hc`).

After the per-case hammer lines, the work counts: for the most expensive
buffer the hammer-samples stepped (and with the damper on), the bank's
group-blocks in the free and forced passes, the loop's members, listeners,
zone-kernel entries and zones read; per buffer on average the free and
forced group-blocks, members and listeners; and the hammers phase per
hammer-sample stepped and in contact. The same counts are in the app's
diagnostics and in its audio-thread runs, so two instances or two kinds of
run can be compared count by count. The effective engine configuration
(`Engine::describe`, C API `pm_engine_describe`: sample rate, maximum
frames, tier and loop band, board, culling, parameters, mode counts,
SIMD kernel and where the hot arrays fall modulo 16 KiB) is printed at the
top, for a diff against the app's.

The first line is the speed probe on the benchmark thread (see the Play
screen's Speed probe toggle), with its latency-bound and throughput-bound
parts and the CPU the thread is on; `--compare` rows add the parts and
how many CPUs the render calls ran on. Before the breakdown, per case, the most
expensive buffer's hammer work (hammers that touched a string, the most
at once, hammer-samples in contact, contact-solver Newton evaluations and
fallbacks) and the zone-kernel full rebuilds and in-place key updates per
buffer.

The scenarios live in `core/pm_bench` (`bench.h`) and are also reachable
through the C API (`pm_bench_item_count`, `pm_bench_item_name`,
`pm_bench_run_item`; flag `PM_BENCH_PHASE_TIMING` runs them with phase
timing on, `PM_BENCH_PACED` / `PM_BENCH_PACED_EVICT` paced,
`PM_BENCH_SPEED_PROBE` with the probe before every 8th buffer; the result
carries the phase splits and work counts, `pm_bench_last_config` the
engine configuration, `pm_bench_last_cpu_usage` / `_probe_split` the CPUs
and probe parts, and `pm_bench_set_pace_hook` lets the caller run the
paced loop in another scheduling class): the app's Benchmark tab runs exactly these cases on
the device. `pm_host_scene_start` plays a case's events through the real
host and audio render path instead (see the Benchmark screen below).

## Tests

`ctest` runs 29 suites. The app host ones: `test_midi_parser` (running
status, real-time bytes inside messages, SysEx, malformed input, UMP in
MIDI 1.0 and 2.0 protocol, the `MIDIEventList` layout, controller mapping,
fuzzing), `test_load_meter` and `test_host` (MIDI host timestamps land on
the same samples as the equivalent sample-time events, through all three
MIDI entry points; monitor ring and overflow; prepare under a running
audio thread; no allocation on the MIDI, UI and audio paths; a benchmark
scene played through `pm_host_render` is bit-identical to a freshly
prepared engine fed the same events, measures exactly the benchmark's
buffers without allocating, and runs on a private engine on request; the
burst scenes' key rules; a paced run takes real time). The M4 ones: `test_soundboard_model` (board
targets, positive-real loop, admittance against a time-domain drive of the
run-time bank, bake + loop = driving point), `test_soundboard_coupling`
(no double counting), `test_passivity` (10^4 random reduced systems with
the production components; a sign-flipped loop is flagged), `test_sympathetic` (silently held key, pedal
bloom per third octave, monotonic in the sympathetic gain) and
`test_half_pedal` (graded T60 against pedal position, no click on pedal
moves, re-pedalling keeps energy). `test_hammer` also checks the hammer's
sample path (one pass over a key's modes per base sample) against its
substep path (one pass per substep) and against a double-precision
recursion. Unit tests of the unison solver and the
M3 double-decay tests use the smooth placeholder admittance, which the
soundboard replaced in the engine.

## iPad app

A minimal SwiftUI app (M2, "first playable") that plays the engine from a
USB MIDI keyboard (target: Komplete Kontrol M32) on an iPad Pro M2, and
also runs on a Mac. **It has been written without access to Xcode: nobody
has compiled it yet.** The C++ part it relies on (`core/pm_host`, the C
API) is tested by CTest, and the Swift package builds and passes its
smoke test with `swift build` / `swift test` on Linux (Swift 6.1); the app
sources were only syntax-checked (`swiftc -parse`), apart from two files
type-checked against the C API.

### Structure

- `Package.swift` (repository root): package `PianoEngine`, library
  `PianoCore` = every `.cpp` of `core/` compiled as C++20 with `-O3
  -fno-exceptions -fno-rtti -DNDEBUG` (also in Debug: the engine is far
  too slow unoptimised). Swift sees only `core/pm_api/pm_api.h` through
  `core/pm_api/include/module.modulemap`. CMake does not use these files.
- `apps/PianoApp/project.yml`: XcodeGen spec. Two targets sharing the
  sources: `PianoApp` (iPadOS 17+, background audio) and `PianoAppMac`
  (macOS 14+). Swift 5 language mode (`SWIFT_VERSION = 5.0`, strict
  concurrency minimal), so Swift 6 data-race checking does not apply. The
  schemes run the Release configuration.
- `apps/PianoApp/PianoApp/`:
  `EngineHost.swift` (owns `pm_host`, background prepare queue, events
  from the UI), `AudioController.swift` (AVAudioEngine + AVAudioSourceNode
  whose render block calls `pm_host_render` with the C pointer only; on
  iOS the AVAudioSession: playback category, preferred 48 kHz and 64
  frames, granted values read back, interruptions, route changes,
  media-services reset, re-prepare on a sample-rate change),
  `MIDIController.swift` (Core MIDI client, event-list input port with the
  MIDI 1.0 protocol, every source connected, hot-plug; the receive block
  passes the raw `MIDIEventList` to `pm_host_midi_event_list`),
  `MIDIMonitor.swift` + `MonitorView.swift` (log drained from the host's
  lock-free ring on the main thread, captured only while the monitor tab
  is visible), `PlayView.swift` (status, sources, sustain, panic, quality
  tier, soundboard on/off, CPU meter with diagnostics, keyboard strip),
  `BenchmarkView.swift` (the offline benchmark, see below),
  `KeyboardView.swift`, `AppModel.swift` (the meters live in their own
  `MeterStore`, published at 10 Hz and only on change, so a meter update
  never re-renders the Play screen or the keyboard), `PianoApp.swift`.
  After pulling a change that adds a Swift file, run `xcodegen` again.

Threads: Core MIDI's thread parses and queues (one SPSC queue), the main
thread queues on-screen notes and pedals (a second queue), the audio
thread moves both into the engine at the start of each buffer and
renders. `pm_host_prepare` closes a gate and waits until the audio
thread has left `pm_host_render` (the audio thread never waits; it outputs
silence while the engine prepares, about a second).

Timing: Core MIDI timestamps and `AudioTimeStamp.mHostTime` are both
`mach_absolute_time` ticks; `pm_host_set_timebase` takes
`mach_timebase_info` (ticks per sample = 1e9 / fs x denom / numer, 500 at
48 kHz on Apple silicon) and an event lands at frame floor((t - buffer
start) / ticks per sample). The default lowest-latency mode puts events
that are already due (every live MIDI event) at frame 0 of the next
buffer; events stamped beyond the buffer wait for theirs; timestamp 0
means "now".

### Build and run on the Mac

```sh
# 1. Optional sanity check of the package (C++ through SwiftPM, Swift sees the C API):
swift build
swift test                      # PianoCoreTests: version, MIDI in, audio out

# 2. Generate and open the Xcode project:
brew install xcodegen
cd apps/PianoApp
xcodegen                        # writes PianoApp.xcodeproj (not committed)
open PianoApp.xcodeproj
```

In Xcode: select the `PianoApp` target, Signing & Capabilities, choose
your Team (bundle id placeholder `com.example.piano.app`: change it if
Xcode says it is taken). Then:

- **Mac first:** scheme `PianoAppMac`, destination "My Mac", Run. Plug the
  M32 into the Mac; the Play tab lists it under MIDI sources. The native
  Mac app asks the output device (HAL) for 48 kHz (only if the device
  supports it; this changes the device's rate for every app, as audio
  apps do) and a 64-frame buffer (clamped to the device's range); the
  Play tab shows the request ("I/O request") and the grant ("I/O buffer",
  "Not as requested"). An iPad app on a Mac (scheme `PianoApp`, "My Mac
  (Designed for iPad)") cannot do this and runs at the device's current
  settings.
- **iPad:** connect the iPad (Developer Mode on), scheme `PianoApp`,
  destination the iPad, Run. Then connect the M32 to the iPad's USB-C
  port (directly, or through a powered hub if it does not power up).

Without XcodeGen: in Xcode, File > New > Project > Multiplatform App
(SwiftUI, Swift), name `PianoApp`; delete the generated `ContentView.swift`
and app file; drag the files of `apps/PianoApp/PianoApp/` into the
project (both targets); File > Add Package Dependencies > Add Local... >
the repository root, add the product `PianoCore` to the app target; in
Build Settings set Swift Language Version to 5 and the iOS deployment
target to 17.0; in Signing & Capabilities add Background Modes > Audio;
edit the scheme's Run configuration to Release.

### What to test (M2 acceptance, plan §7)

- [ ] **Playable from the M32 over USB.** Notes with velocity, sustain
      pedal in the M32's pedal socket (CC64; continuous values give
      half-pedal), unplug and replug the M32 while holding a chord with the
      pedal down: no hanging notes (removal releases all notes and pedals),
      the M32 reappears in the source list and plays again.
- [ ] **Key-to-sound latency, median <= 10 ms.** The app shows the granted
      I/O buffer and (iPad) the session's output latency. Expected budget
      at 48 kHz / 64 frames: USB MIDI about 1 ms + waiting for the next
      buffer 0-1.3 ms + one buffer 1.3 ms + output latency (wired
      headphones typically 1-3 ms; the built-in speakers more; Bluetooth
      100+ ms, unsuitable). Measuring: record, with a second device (phone
      or the Mac with a microphone), the click of a key being struck hard
      together with the iPad's output (a speaker or headphone right next
      to the microphone); in an audio editor measure from the key click to
      the tone onset for 20 or more strikes; report the median and the
      worst. The key's own travel before the switch closes adds a few ms
      the test cannot separate, so the result is an upper bound.
- [ ] **No audio-thread allocations (Instruments).** Product > Profile
      (Release) > Allocations. Record while playing chords with the pedal
      for 30 s. In the Call Tree, check "Separate by Thread" and find the
      audio I/O thread (on iOS named like `AURemoteIO::IOThread`, on macOS
      `com.apple.audio.IOThread.client`): there must be no allocations
      under `pm_host_render` or the render block. The same check for Core
      MIDI's receive thread under `pm_host_midi_event_list` (Core MIDI may
      allocate in its own frames above our call; that is outside our
      code). On the host side the same property is checked by
      `test_host` and `test_rt_alloc`.
- [ ] Hammer energy test: covered by the CTest suites (M2 portable part).

Also: plugging headphones in and out (route change; the status shows the
new output and sample rate), a phone call or Siri (interruption; the
piano comes back afterwards), backgrounding the app while a note sounds
(it keeps playing: background audio), quality tier and soundboard
switches (each re-prepares, about a second of silence).

### Reading the CPU meter

The meter is the render time of each buffer divided by the buffer
period (frames / sample rate), measured inside `pm_host_render`:
"Now" is smoothed over about 0.5 s, "Mean", "p99" and "Max" are since the
last Reset, and the counter shows buffers above 70 %. Targets on the iPad
Pro M2 (CLAUDE.md): mean at most 50 %, worst buffer at most 70 %. Above
100 % a buffer was late and you hear a click. Note-on buffers carry the
hammer solves and are the peaks; the first buffers after a prepare can be
slow too, so press Reset after the app has settled.

Below the bars (C API `pm_host_get_diagnostics`):

- **Not as requested** (status, orange): the sample rate or I/O buffer the
  system granted differs from the requested 48 kHz / 64 frames. An iPad
  app running on a Mac gets the Mac's output settings (typically 512
  frames at 44.1 kHz) whatever it asks for.
- **Render buffer**: the frames of the last render call and its period in
  µs, the smallest and largest frame count seen and how many distinct
  counts. More than one means the render block is not called with a fixed
  size (for example behind a sample-rate converter, or 4096 while the
  screen is locked); a load is always relative to that call's own period.
- **Worst buffer**: for the buffer with the highest load since Reset, its
  frame count, render time and period, how many hammers touched a string
  in it (including hammers that left the strings and were released before
  its end; API 0.9) and how many were still active at its end, awake
  modes, events applied and the number of sub-blocks the engine cut it
  into.
- **Thread CPU time** (toggle, off by default: two kernel clock reads per
  buffer): adds the CPU time of the worst buffer and the largest
  wall-clock minus CPU time of any buffer. A worst buffer whose CPU time
  is close to its wall time was slow to compute; a large difference means
  the audio thread was preempted.
- **Phase timing** (toggle, off by default: about five user-space clock
  reads per 32-frame sub-block, two per sample while a hammer is active):
  splits the render time into events (event handling, control ticks,
  damper updates, loop set-up with impulse-response and zone-kernel
  recomputes), hammers, free (the strings' block pass), forced (bridge
  input into the strings), board (zone kernel and board step) and other,
  for the worst buffer, the top 1 % and the mean. Turning it on resets
  the meter so that all of them are timed.
- **Speed probe** (toggle, off by default): every 8th render call first
  runs a small fixed workload (`core/pm_host/speed_probe.h`, about 15 µs on
  the x86 container, less on a fast Apple core; 16 KiB of stack, no
  allocation) on the audio thread and records its time, in two parts
  (API 0.11): latency-bound (a dependent scalar sqrt/divide chain and a
  dependent vector multiply-add chain) and throughput-bound (the mode-bank
  kernel's shape: 1024 complex one-pole modes, many independent vector
  operations in flight). Copy diagnostics compares both with the same
  workload on the benchmark thread (from the last Benchmark run; without
  one, on the main thread at the moment of copying) and states the speed
  the audio thread ran at, e.g. "latency 0.80x, throughput 0.30x". An
  efficiency core or a low clock costs the throughput part (and the
  engine's inner loops) far more than the latency part; the API 0.9-0.10
  probe was mostly latency-bound and so could not see it.
- **CPU numbers** (API 0.11; recorded while phase timing, thread CPU time
  or the speed probe is on: two reads of a per-CPU register value per
  render call, `core/pm_host/cpu_id.h`): which logical CPU each render
  call started and ended on. Copy diagnostics lists the share of render
  calls and their mean render time per CPU and per core type, the calls
  that migrated mid-buffer, the worst buffer's CPUs, and the speed probe's
  parts per CPU. The core types come from `sysctl` (hw.nperflevels,
  hw.perflevelN.name / .logicalcpu), but which CPU numbers belong to which
  type is an assumption (the slowest cores have the lowest numbers, as on
  every Apple silicon chip so far); the per-CPU probe times check it.
- **Process CPU** (once a second, main thread): the whole app's CPU over
  the last second in % of one core, split into the audio render (from the
  meter), the main thread (UI) and the other threads (Core MIDI, system
  audio). This is what Activity Monitor shows for the app.

The meters are polled four times a second, and only while the Play tab is
shown and the app is active; nothing on screen is updated unless what it
shows changed (the load rows, the status rows and the CPU line are
separate observed leaves, so a changing load re-renders only the rows),
so an idle app should leave the main thread asleep apart from the
once-a-second CPU line. Measured on a Mac (iPad app) it still used about
15 % of a core at idle; the cause could not be found from the code (the
SwiftUI app cannot be built or profiled in the development container).
**UI cost test** (segmented control at the bottom of the Play tab) hides
the keyboard ("No keyboard"), stops the meter updates ("No meters") or
both ("CPU line only"); the process-CPU line keeps updating in every
mode, so comparing its "main thread" figure across the modes shows what
each part costs. If even "CPU line only" stays high, the cost is outside
these views (the tab bar, the window, the system) and needs Instruments
(Time Profiler on the main thread).

**Copy diagnostics** builds the full text at that moment: the above plus
render time (mean, p99, max in µs and %; p99 from a histogram with 0.25 %
bins), MIDI message counts, the worst buffer's events (note-ons, note-offs,
pedal changes) and the recomputes they caused (damper updates, impulse
responses, zone-kernel full rebuilds and in-place key updates), its
hammer work (hammers that touched a string, the most at once,
hammer-samples in contact, Newton evaluations, fallbacks), its work counts
(API 0.10: hammer-samples stepped and with the damper on, free and forced
group-blocks, loop members, listeners, zone-kernel entries and zones),
the phase split when timed, the speed probe and its parts, the core
layout and the render calls per CPU, the engine's effective
configuration (for a diff against the benchmark's), the UI cost test mode, the build
information (SIMD kernel, optimisation, NDEBUG, compiler), device, thermal
state and audio route.

### Benchmark screen

The Benchmark tab runs the `bench_engine` scenarios (`core/pm_bench`) on
the device at 48 kHz in 64-frame buffers (or, with "512 frames at 44.1
kHz", at what an iPad app on a Mac is granted), at the selected quality
tier ("All tiers" adds Balanced and Eco): render time per buffer (mean, p99,
max, in µs and %) per case, then one hammer alone per key. The first line
is the speed probe on the benchmark thread (whole, and its two parts),
which Copy diagnostics and the audio-thread rows use as the reference.
Audio output is paused for the run (about a minute; each case prepares its
own engine) so that the numbers are the engine's alone, then resumes by
itself. The run is on a dedicated thread of QoS user-interactive. Every
scenario row lists the CPUs its render calls ran on (as in Copy
diagnostics), and with the Diagnostics speed probe on, the probe's parts
run before every 8th buffer against the back-to-back reference. Copy puts the table,
with the build and device information and the engine configurations (the
benchmark's last scenario, the last audio-thread run, the app's engine
now), on the clipboard.

Switches: **Paced** sleeps to each buffer's real-time deadline between
render calls, as an audio thread waits for its callback (only the render
call is timed); **Evict** also walks 32 MiB between buffers (other work on
the core); **Real-time paced** (on by default) runs the paced loop in an
audio I/O thread's scheduling class: Mach time-constraint policy (period
the buffer, computation half of it) and an audio work interval
(`AudioWorkIntervalCreate`) joined by the thread, started with each
buffer's deadline before the render call and finished after it, through
the C API's pacing hook (`pm_bench_set_pace_hook`); the thread returns to
the standard policy after the paced loop (the engine's prepare runs
outside it). Without it the paced loop runs at the thread's QoS, which is
not an audio thread's class at all, so its numbers say nothing about the
audio thread. Even with it the thread is not the I/O thread (not in the
output device's workgroup); only the audio-thread rows measure playing; **Phases** (on by default) runs with phase timing, and each
scenario row is then followed by its phase split (mean, top 1 %, most
expensive buffer) and its work counts (per buffer: awake modes, free and
forced group-blocks, loop members and listeners, zone-kernel rebuilds and
key updates, hammers phase per hammer-sample stepped and in contact,
overruns; for the most expensive buffer: events, hammers, hammer-samples
stepped, damped and in contact, Newton evaluations, recomputes,
group-blocks, loop members, listeners, entries and zones).

**Run on audio thread** plays the scene chosen in the picker through the
real host and audio render path, in real time and audible (C API
`pm_host_scene_start`): the app's engine is reset to its just-prepared
state (a buffer of silence; parameters, tier and tables kept), the
scene's events (the same as the benchmark's, at the device's sample rate)
are pushed into the engine at their sample times by the audio thread
itself (pre-built list, no allocation, no lock), the load meter is reset
when the warm-up (0.5 s, as the benchmark) is over, and after the
Benchmark tab's 3 s a row is appended with the same statistics (mean, p99,
max; p99 to the meter's 1.5 % bins), phase split and work counts as a
benchmark row, plus the speed probe over the run (switch the probe on in
Diagnostics) against the benchmark thread's. Leave the keyboard alone
while it plays; afterwards all notes are released. With **Benchmark
engine** the scene runs instead on a private engine prepared exactly as
the benchmark prepares that case (the app's engine keeps its state and
resumes afterwards). The same material can so be measured four ways:
back to back and paced on the benchmark thread, and on the audio thread
with the app's engine and with a benchmark engine; which of these differ
tells the thread (scheduling, core, clock) from the engine instance and
from the material. **Run all on audio thread** plays every scene in the
list in turn (one row each, the same format; Cancel stops the series),
so that a full table on the audio thread takes one tap; a header line
before the rows gives the I/O configuration and engine.

What to send back from a device: the Benchmark table (Copy), with the
Speed probe and Phases on, at both buffer settings, back to back and
paced (Real-time paced on, and once off), with "Run all on audio thread"
rows (benchmark engine) appended, and Copy diagnostics with Phase timing
and Speed probe on (after a Benchmark run, for the reference) after (a) a
minute of normal playing, (b) Reset, then ten notes hard together and
released together a few times, (c) Reset, then chords with the sustain
pedal pressed and released, and (d) Reset, then a minute with nothing
played (idle).

### M32 capture session (plan §3.2, M0 item)

Goal: learn exactly what the M32 sends on iPadOS in its default mode.
Open the MIDI Monitor tab, press Clear, then work through the list below.
Before each item press **Marker** and write down "marker N = item".
Move each control slowly over its whole range, then quickly.

1. Keys: the lowest and highest key, softly and hard (velocity range).
2. Octave - and + buttons, each a few times, then a key after each press
   (do the buttons send anything? which notes do keys send after them?).
3. Each of the eight knobs: turn clockwise then counter-clockwise; touch
   without turning (touch-sensitive knobs).
4. Pitch strip: touch, slide, release (does it return to centre?).
5. Mod strip: touch, slide, release (does it hold?).
6. Every transport button (Play, Rec, Stop, Loop, Metro, Tempo, ...),
   the 4-D encoder (rotate both ways, push, push in four directions) and
   every other button (Shift, Scale, Arp, Undo, Quantize, Preset up/down,
   Browse, ...).
7. A sustain pedal in the pedal socket: press and release slowly
   (on/off only, or continuous?).

Then press Pause, Copy (or Share to Files / Mail) and send back the text
together with the marker list. The log is tab-separated: time since the
previous message (ms), source, group, channel, type, decoded data, raw
UMP words. Also note the iPad model and iPadOS version and whether the
M32 was connected directly or through a hub.
