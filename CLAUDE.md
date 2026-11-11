# Project guidance

Physically modelled concert grand piano (target: Steinway Model D) for iPad Pro M2. Portable C++20 engine, later a SwiftUI app and AUv3. No samples. The plan is `docs/steinway-d-physical-model-plan.md`; build and run instructions are in `README.md`.

## Which model does what

- **Design, DSP and coding:** Opus 5.5 at medium effort.
- **Simple mechanical tasks:** Sonnet 5.5 or Haiku 5.5 subagents. This covers building, running `ctest` across the configuration matrix, sanitizer runs, benchmarks, rendering example scores, running analysis scripts, and cleanup. Use Haiku by default and Sonnet when failures need interpreting.
- Give such a subagent the exact commands and ask for a compact result: pass/fail per configuration, the output of failing tests, and benchmark tables.

## Working rules

- Work milestone by milestone as defined in §7 of the plan. State results against that milestone's acceptance criteria, with measured numbers.
- Keep the plan accurate: when the implementation departs from it or a formula in it is wrong, correct the plan in the same change.
- Never loosen a test to make it pass without saying so and giving the reason in a comment next to the assertion.
- Unrelated files in the repository root (`drugi_plik.py`, `nowy.py`, `tekst.txt`) are not part of the project; leave them alone.

## Code conventions

- `core/` is portable C++20 with no platform dependencies.
- Real-time code: no allocation, locks, exceptions, RTTI or I/O in processing functions. Real-time libraries build with `-fno-exceptions -fno-rtti`.
- Data layout is structure-of-arrays with 64-byte alignment.
- SIMD goes through `core/pm_base/simd.h` only. Every kernel is mirrored across NEON, AVX2, SSE2 and scalar, and has a scalar reference in the tests.
- Only the C API in `core/pm_api` is a stable surface for Swift and Python.
- Tests use the small harness in `tests/test_harness.h` and are registered with CTest. No dependencies are downloaded at configure time.
- Comments explain non-obvious DSP or real-time reasoning only.

## Verification

- Before a change is committed: build with g++ and clang++, with and without `-DPM_NATIVE_ARCH=ON`, with forced-scalar SIMD, and once under ASan+UBSan; all CTest suites must pass.
- The development container is Linux x86_64. NEON code cannot run there, so any new or changed kernel is unverified until it has run on Apple Silicon. List such kernels in the report.
- For performance, the SSE2 build is the proxy for NEON (both are 4-wide); AVX2 numbers overstate what Apple Silicon will do.
- CPU limits on the iPad Pro M2 at 48 kHz and 64-frame buffers: mean at most 50 % of one core, worst single buffer at most 70 %.

## Git

- Develop on the branch named in the session instructions; do not push elsewhere.
- Build directories and generated audio are never committed.
