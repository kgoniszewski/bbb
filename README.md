# Physically modelled piano engine

A portable C++20 engine for a physically modelled concert grand, built
milestone by milestone from the engineering plan in
[docs/steinway-d-physical-model-plan.md](docs/steinway-d-physical-model-plan.md).

Current state: milestones M0 (build, CI, tooling skeleton) and M1 (a single
inharmonic modal string with scalar and SIMD kernels, pluck excitation,
force input, energy-based culling, and a kernel benchmark). There is no
hammer, soundboard or MIDI yet.

## Layout

| Path | Contents |
|---|---|
| `core/pm_base/` | Header-only: 64-byte aligned arrays, SIMD wrapper (NEON / AVX2+FMA / SSE2 / scalar), flush-denormals guard (x86 MXCSR, ARM FPCR), SPSC queue |
| `core/pm_string/` | `ModeBank` (complex one-pole resonators, structure of arrays, group culling), string parameters to modal coefficients, pluck |
| `tools/render_cli/` | Offline renderer: one plucked string to a mono WAV (32-bit float or 24-bit PCM) |
| `tools/bench/` | Modal kernel benchmark |
| `tools/analysis/` | Python (numpy): measured partial frequencies and T60 per partial |
| `tests/` | Self-contained C++ unit tests, run by CTest |

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

```sh
./build/tools/render_cli/render_cli --f0 261.63 --B 3.8e-4 --b1 0.5 --b3 6.25e-9 \
    --pluck 0.12 --seconds 5 --fs 48000 --out c4.wav
```

`--help` lists all options (string length and density, pickup position,
partial limits, culling threshold, kernel, `--format f32|s24`).

## Analyse

```sh
python3 tools/analysis/analyze_string.py c4.wav --f0 261.63 --B 3.8e-4 --b1 0.5 --b3 6.25e-9
```

Prints, per partial, expected and measured frequency (error in cents) and
measured T60, plus the expected T60 when `--b1` and `--b3` are given.

## Benchmark

```sh
./build/tools/bench/bench_modal            # 26,000 modes, 48 kHz, 64-frame blocks
```

Reports nanoseconds per mode-sample and the percentage of one core needed in
real time, for the scalar reference and the SIMD kernel. Numbers from x86
machines are only indicative for the Apple silicon target.
