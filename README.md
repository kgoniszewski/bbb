# Physically modelled piano engine

A portable C++20 engine for a physically modelled concert grand, built
milestone by milestone from the engineering plan in
[docs/steinway-d-physical-model-plan.md](docs/steinway-d-physical-model-plan.md).

Current state: milestones M0 (build, CI, tooling skeleton), M1 (modal string
bank with scalar and SIMD kernels, culling, kernel benchmark) and the
portable part of M2: a nonlinear hammer per key, 88 keys of one string each
from a per-key table, dampers and sustain pedal, a sample-accurate engine
with an event queue, a C API, and a piano mode in `render_cli`. There is no
soundboard, no unisons, no stretch tuning and no Apple code yet.

## Layout

| Path | Contents |
|---|---|
| `core/pm_base/` | Header-only: 64-byte aligned arrays, SIMD wrapper (NEON / AVX2+FMA / SSE2 / scalar), flush-denormals guard (x86 MXCSR, ARM FPCR), SPSC queue |
| `core/pm_string/` | `ModeBank` (complex one-pole resonators, structure of arrays, group culling, per-key segments with their own I/O and damper loss), string parameters to modal coefficients, pluck |
| `core/pm_hammer/` | Hunt-Crossley felt hammer and the hammer-string contact solver |
| `core/pm_voicing/` | Non-real-time: 88-key parameter table v0, velocity curve, key parameters to bank and contact coefficients |
| `core/pm_engine/` | Engine: event queue and scheduling, hammer pool, dampers and pedal, render loop, stereo output stage |
| `core/pm_api/` | Stable C API (`pm_api.h`) for Swift and Python |
| `tools/render_cli/` | Offline renderer: one plucked string to a mono WAV, or a text score through the engine to a stereo WAV |
| `tools/bench/` | Modal kernel benchmark and engine benchmark |
| `tools/analysis/` | Python (numpy): partial frequencies and T60 of a string; attack centroid and partial levels of piano notes |
| `tests/` | Self-contained C++ unit tests (and one C test of the API), run by CTest |

## Engine (M2)

- **Keys.** One string per key. Parameters are interpolated (monotone cubic,
  geometric for scale-like quantities) between the anchor rows A0, C2, C4,
  C6, C7, C8 of plan §2.1. The sounding first partial f0·sqrt(1 + B) is in
  equal temperament at A4 = 440 Hz. Partials stay below min(18 kHz, 0.45 fs),
  at most 120 per string: 4,897 modes at 48 kHz.
- **Output.** The transverse force of each string on the bridge, summed
  through a fixed stereo spread (bass left), a 5 Hz DC blocker, master gain
  and a soft clipper (transparent below -1 dBFS, never reaches full scale).
- **Hammer contact.** While a hammer is active it owns its key's modes and
  advances them in substeps of 1/(R fs) (R = 1 below C4, 2 to key 75, 4 in
  the top octave). Modes and hammer are integrated exactly under a force
  held over each substep; the felt force is the discrete gradient of the
  felt potential, solved by a safeguarded scalar Newton iteration. The
  scheme is energy-exact for a lossless felt and string and passive for any
  felt stiffness. The static compliance of the partials above the bank's
  limit acts as a spring in series with the felt. The hammer is followed
  until it has stayed off the string for 1.5 round trips (multiple contacts
  in the bass), then the modes go back to the bank. A re-strike launches a
  hammer into the moving string with no state reset. See
  `core/pm_hammer/hammer.h`.
- **Dampers.** Extra loss per mode, weighted by the mode shape under the
  felt; landing follows a raised-cosine curve over 20-80 ms (faster with
  higher release velocity). Keys 69-88 have no damper. CC64 is stored as
  0..1 and used as on/off.
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
`time cc64 VALUE`, `time all_notes_off`, `time panic`; KEY is a MIDI note
number or a name such as C4 or F#3):

```sh
./build/tools/render_cli/render_cli --score tools/render_cli/scores/single_notes.txt --out notes.wav
```

It prints every strike's hammer velocity, contact duration and number of
contacts. Example scores: `single_notes.txt` (C2, C4, C7 at five
velocities), `chord_pedal.txt` (a chord with and without pedal),
`repeated_note.txt` (A4 at 12 strikes per second). `--help` lists all
options.

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

## Benchmark

```sh
./build/tools/bench/bench_modal            # 26,000 modes, 48 kHz, 64-frame blocks
```

Reports nanoseconds per mode-sample and the percentage of one core needed in
real time, for the scalar reference and the SIMD kernel.

```sh
./build/tools/bench/bench_engine           # 48 kHz, 64-frame buffers
```

Engine cost in % of one core: all 88 keys struck and held with the pedal
down (with culling, and with culling off so every mode runs), and a typical
case (a four-note chord re-struck twice a second, pedal up). Numbers from
x86 machines are only indicative for the Apple silicon target.
