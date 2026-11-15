# Project guidance

Physically modelled concert grand piano (target: Steinway Model D) for iPad Pro M2. Portable C++20 engine, later a SwiftUI app and AUv3. No samples. The plan is `docs/steinway-d-physical-model-plan.md`; build and run instructions are in `README.md`.

## Which model does what

- **Planning, briefs, review, plan and README updates:** the main session.
- **Design, DSP and coding:** Opus 5.5 subagents at medium effort.
- **Mechanical work that needs judgement** (interpreting failures, pull requests): Sonnet 5.5 or Haiku 5.5 subagents.

## Keeping token use down

Every subagent turn re-sends its whole context, so cost grows with turns times context size. Measured on M1–M4: workers given a whole job ran 100–200 turns at about 320 k tokens of context and were 87 % of all usage.

- **One focused change per Opus worker.** The brief names the files and line ranges to read and gives the relevant plan excerpt. Target: under 40 turns and under 100 k tokens of context. Split anything larger.
- **Workers do not read or edit `docs/steinway-d-physical-model-plan.md` or `README.md`** (together about 58 k tokens). They report what changed and the measured numbers; the main session updates the documents.
- **Workers do not run the matrix or the benchmarks.** They do one g++ Release build and the affected test suites, with output filtered to warnings, errors and failures.
- **Verification and benchmarks run as a background script** started by the main session, which writes a short summary file (pass/fail per configuration, output of failing tests only). No subagent waits on or polls a script.
- **Never print whole long files or unfiltered build, test or benchmark logs** into any context. The plan has paragraphs of several thousand tokens: search it with `grep -o` or `cut -c1-200`, and read by line range.
- **One session per milestone.** The plan and this file carry the state between sessions.

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

- Before each commit: g++ and clang++ Release builds and one g++ ASan+UBSan build; all CTest suites must pass.
- Before a pull request, and after any change to a SIMD kernel: the full matrix. That is g++ and clang++, each with and without `-DPM_NATIVE_ARCH=ON`, forced-scalar SIMD, and ASan+UBSan (g++; the container has no clang ASan runtime).
- The development container is Linux x86_64. NEON code cannot run there, so any new or changed kernel is unverified until it has run on Apple Silicon. List such kernels in the report.
- For performance, the SSE2 build is the proxy for NEON (both are 4-wide); AVX2 numbers overstate what Apple Silicon will do.
- CPU limits on the iPad Pro M2 at 48 kHz and 64-frame buffers: mean at most 50 % of one core, worst single buffer at most 70 %.

## Git

- Develop on the branch named in the session instructions; do not push elsewhere.
- Build directories and generated audio are never committed.
