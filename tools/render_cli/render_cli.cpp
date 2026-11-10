// Offline renderer: plucks one modal string and writes a mono WAV file.
//
//   render_cli --f0 261.63 --B 3.8e-4 --b1 0.5 --b3 6.25e-9 --pluck 0.12
//              --seconds 5 --fs 48000 --out c4.wav
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_string/mode_bank.h"
#include "pm_string/string_modes.h"
#include "wav_writer.h"

namespace {

void usage() {
    const pm::StringParams d;
    std::printf(
        "usage: render_cli [options]\n"
        "  --f0 HZ            flexible-string fundamental (default %.2f)\n"
        "  --B X              inharmonicity coefficient (default %g)\n"
        "  --b1 X             loss, 1/s (default %g)\n"
        "  --b3 X             frequency-dependent loss, s (default %g)\n"
        "  --L M              speaking length, m (default %g)\n"
        "  --mu KG_PER_M      linear density (default %g)\n"
        "  --pluck X          pluck position, fraction of L (default 0.12)\n"
        "  --pickup X         observation position, fraction of L (default %g)\n"
        "  --amp M            pluck height, m (default 1e-3)\n"
        "  --gain X           output units per metre (default %g)\n"
        "  --max-partials N   (default %d)\n"
        "  --max-freq HZ      (default %g; also capped at 0.45 fs)\n"
        "  --seconds S        (default 5)\n"
        "  --fs HZ            (default 48000)\n"
        "  --block N          frames per process call (default 64)\n"
        "  --cull-db DB       culling threshold re full scale (default -100)\n"
        "  --no-cull          disable culling\n"
        "  --kernel K         simd | scalar (default simd)\n"
        "  --format F         f32 | s24 (default f32)\n"
        "  --out FILE         output WAV (default out.wav)\n",
        d.f0, d.B, d.b1, d.b3, d.length, d.linearDensity, d.outputPos, d.outputGain,
        d.maxPartials, d.maxFreqHz);
}

bool parseDouble(const char* s, double& v) {
    char* end = nullptr;
    v = std::strtod(s, &end);
    return end != s && *end == '\0';
}

}  // namespace

int main(int argc, char** argv) {
    pm::StringParams p;
    double pluckPos = 0.12, amp = 1e-3, seconds = 5.0, fs = 48000.0, cullDb = -100.0;
    int block = 64;
    bool cull = true;
    pm::Kernel kernel = pm::Kernel::Simd;
    pm_tools::WavFormat format = pm_tools::WavFormat::Float32;
    std::string out = "out.wav";

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            usage();
            return 0;
        }
        if (a == "--no-cull") {
            cull = false;
            continue;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", a.c_str());
            return 2;
        }
        const char* v = argv[++i];
        double d = 0.0;
        const bool num = parseDouble(v, d);
        bool ok = true;
        if (a == "--f0") ok = num, p.f0 = d;
        else if (a == "--B") ok = num, p.B = d;
        else if (a == "--b1") ok = num, p.b1 = d;
        else if (a == "--b3") ok = num, p.b3 = d;
        else if (a == "--L") ok = num, p.length = d;
        else if (a == "--mu") ok = num, p.linearDensity = d;
        else if (a == "--pluck") ok = num, pluckPos = d;
        else if (a == "--pickup") ok = num, p.outputPos = d;
        else if (a == "--amp") ok = num, amp = d;
        else if (a == "--gain") ok = num, p.outputGain = d;
        else if (a == "--max-partials") ok = num, p.maxPartials = static_cast<int>(d);
        else if (a == "--max-freq") ok = num, p.maxFreqHz = d;
        else if (a == "--seconds") ok = num, seconds = d;
        else if (a == "--fs") ok = num, fs = d;
        else if (a == "--block") ok = num, block = static_cast<int>(d);
        else if (a == "--cull-db") ok = num, cullDb = d;
        else if (a == "--kernel") {
            if (std::strcmp(v, "simd") == 0) kernel = pm::Kernel::Simd;
            else if (std::strcmp(v, "scalar") == 0) kernel = pm::Kernel::Scalar;
            else ok = false;
        } else if (a == "--format") {
            if (std::strcmp(v, "f32") == 0) format = pm_tools::WavFormat::Float32;
            else if (std::strcmp(v, "s24") == 0) format = pm_tools::WavFormat::Pcm24;
            else ok = false;
        } else if (a == "--out") {
            out = v;
        } else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            usage();
            return 2;
        }
        if (!ok) {
            std::fprintf(stderr, "bad value for %s: %s\n", a.c_str(), v);
            return 2;
        }
    }
    if (fs <= 0.0 || seconds <= 0.0 || block <= 0 || p.f0 <= 0.0 || pluckPos <= 0.0 ||
        pluckPos >= 1.0) {
        std::fprintf(stderr, "invalid parameters\n");
        return 2;
    }

    pm::ModeBank bank;
    const int count = pm::countStringModes(p, fs);
    if (count == 0 || !bank.allocate(count) || pm::loadString(bank, p, fs) != count) {
        std::fprintf(stderr, "could not set up %d modes\n", count);
        return 1;
    }
    bank.setCulling(cull, static_cast<float>(cullDb));
    pm::pluck(bank, pluckPos, amp);

    const int n = static_cast<int>(std::lround(seconds * fs));
    std::vector<float> y(static_cast<std::size_t>(n), 0.0f);
    const auto t0 = std::chrono::steady_clock::now();
    {
        pm::ScopedFlushDenormals ftz;
        for (int pos = 0; pos < n; pos += block)
            bank.process(nullptr, y.data() + pos, std::min(block, n - pos), kernel);
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    float peak = 0.0f;
    for (float v : y) peak = std::max(peak, std::fabs(v));
    std::size_t clipped = 0;
    if (!pm_tools::writeWav(out.c_str(), y, static_cast<int>(std::lround(fs)), format, &clipped)) {
        std::fprintf(stderr, "could not write %s\n", out.c_str());
        return 1;
    }
    std::printf("%s: %d modes (%d groups), first partial %.3f Hz, highest %.1f Hz\n", out.c_str(),
                count, bank.groupCount(), pm::partialFrequency(p.f0, p.B, 1),
                pm::partialFrequency(p.f0, p.B, count));
    std::printf("  %.2f s at %.0f Hz, peak %.4f (%.1f dBFS), kernel %s, culling %s\n", seconds, fs,
                peak, 20.0 * std::log10(std::max(peak, 1e-30f)),
                kernel == pm::Kernel::Simd ? pm::simdKernelName() : "scalar",
                cull ? "on" : "off");
    std::printf("  render time %.1f ms (%.1fx real time), groups awake at end %d, group-blocks %llu\n",
                elapsed * 1e3, seconds / elapsed, bank.activeGroups(),
                static_cast<unsigned long long>(bank.groupBlocksProcessed()));
    if (clipped) std::printf("  warning: %zu samples clipped in 24-bit output\n", clipped);
    return 0;
}
