// Modal kernel benchmark: N always-active modes, 64-frame blocks, per-mode
// input weight on a shared input signal, summed output (the runModes shape of
// plan §1.5). Reports ns per mode-sample and % of one core in real time.
//
//   bench_modal [--modes 26000] [--fs 48000] [--block 64] [--seconds 1] [--reps 3]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_string/mode_bank.h"

namespace {

struct Result {
    double nsPerModeSample;
    double percentRealTime;
    float checksum;
};

Result run(pm::ModeBank& bank, pm::Kernel kernel, double fs, int block, double seconds,
           int reps, const std::vector<float>* input) {
    const int blocks = static_cast<int>(seconds * fs / block);
    std::vector<float> out(static_cast<std::size_t>(block));
    double best = 1e30;
    float checksum = 0.0f;
    pm::ScopedFlushDenormals ftz;
    for (int r = 0; r < reps + 1; ++r) {  // first rep is warm-up
        const auto t0 = std::chrono::steady_clock::now();
        for (int b = 0; b < blocks; ++b) {
            std::fill(out.begin(), out.end(), 0.0f);
            bank.process(input ? input->data() + (b % 16) * block : nullptr, out.data(), block, kernel);
            checksum += out[0];
        }
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (r > 0) best = std::min(best, t);
    }
    const double modeSamples = double(bank.size()) * blocks * block;
    const double audioSeconds = double(blocks) * block / fs;
    return {best * 1e9 / modeSamples, 100.0 * best / audioSeconds, checksum};
}

}  // namespace

int main(int argc, char** argv) {
    int modes = 26000, block = 64, reps = 3;
    double fs = 48000.0, seconds = 1.0;
    for (int i = 1; i + 1 < argc; i += 2) {
        const double v = std::atof(argv[i + 1]);
        if (!std::strcmp(argv[i], "--modes")) modes = static_cast<int>(v);
        else if (!std::strcmp(argv[i], "--fs")) fs = v;
        else if (!std::strcmp(argv[i], "--block")) block = static_cast<int>(v);
        else if (!std::strcmp(argv[i], "--seconds")) seconds = v;
        else if (!std::strcmp(argv[i], "--reps")) reps = static_cast<int>(v);
        else {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        }
    }

    pm::ModeBank bank;
    if (!bank.allocate(modes)) return 1;
    bank.resize(modes);
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> uf(27.0, 18000.0), us(0.3, 30.0), uw(-1.0, 1.0);
    for (int i = 0; i < modes; ++i) {
        const double f = uf(rng), sigma = us(rng), wIn = 1e-3 * uw(rng), wOut = uw(rng) / 100.0;
        bank.setMode(i, f, sigma, fs);
        bank.setWeights(i, static_cast<float>(wIn), static_cast<float>(0.5 * wIn), static_cast<float>(wOut));
    }
    bank.setCulling(false);  // every group processed every block

    // Continuous noise input keeps every mode at a steady, normal-range level.
    std::vector<float> input(static_cast<std::size_t>(16 * block));
    std::uniform_real_distribution<float> un(-1.0f, 1.0f);
    for (auto& x : input) x = un(rng);

    const double hotKiB = 6.0 * 4.0 * modes / 1024.0;
    std::printf("modes %d (%d groups of %d), fs %.0f Hz, block %d, %.1f s per rep, best of %d\n",
                modes, bank.groupCount(), pm::ModeBank::kGroupSize, fs, block, seconds, reps);
    std::printf("hot state+coefficient arrays: %.0f KiB\n", hotKiB);
    // With an input (complex input weight, as the M4 sympathetic drive will
    // need) and free-running (no input: every key in the M3 engine).
    for (bool withInput : {true, false}) {
        for (pm::Kernel k : {pm::Kernel::Scalar, pm::Kernel::Simd}) {
            bank.clearState();
            for (int i = 0; i < modes; ++i) bank.setState(i, 1e-3f, 0.0f);
            const Result r = run(bank, k, fs, block, seconds, reps, withInput ? &input : nullptr);
            std::printf("%-8s %-16s %-10s %7.3f ns/mode-sample   %6.1f %% of one core   (checksum %g)\n",
                        k == pm::Kernel::Simd ? "simd" : "scalar",
                        k == pm::Kernel::Simd ? pm::simdKernelName() : "reference",
                        withInput ? "input" : "free", r.nsPerModeSample, r.percentRealTime,
                        static_cast<double>(r.checksum));
        }
    }
    return 0;
}
