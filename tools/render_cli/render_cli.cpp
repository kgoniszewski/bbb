// Offline renderer.
//
// Single-string mode: plucks one modal string and writes a mono WAV file.
//   render_cli --f0 261.63 --B 3.8e-4 --b1 0.5 --b3 6.25e-9 --pluck 0.12
//              --seconds 5 --fs 48000 --out c4.wav
//
// Piano mode: renders a text score (see score.h) through the engine to a
// stereo WAV file.
//   render_cli --score scores/single_notes.txt --out notes.wav
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_engine/engine.h"
#include "pm_string/mode_bank.h"
#include "pm_string/string_modes.h"
#include "score.h"
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
        "  --out FILE         output WAV (default out.wav)\n"
        "\n"
        "piano mode: render_cli --score FILE [options]\n"
        "  --score FILE       text score (time note_on KEY VEL | note_off KEY | cc64/cc66/cc67 V | ...)\n"
        "  --tail S           seconds rendered after the last event (default 4)\n"
        "  --fs, --block, --format, --out, --cull-db, --no-cull as above\n"
        "  --gain-db DB       master gain (default 0)\n"
        "  --width W          stereo width 0..1 (default 0.35)\n"
        "  --quiet            do not list the strikes\n"
        "  --a4 HZ            reference pitch (default 440)\n"
        "  --stretch X        stretch tuning amount 0..1.5 (default 1)\n"
        "  --equal-temperament  plain 12-TET instead of stretch tuning\n"
        "  --unison-width X   scales the unison detuning (default 1; 0 = strings in unison)\n"
        "  --no-board         soundboard off (A/B: M3 output, no sympathetic loop)\n"
        "  --sympathetic G    sympathetic loop gain 0..1.5 (default 1)\n"
        "  --knock N          hammer knock level, N per m/s (default 0.02; 0 = off)\n"
        "  --damper-noise N   damper-lift noise level (default 0.004; 0 = off)\n",
        d.f0, d.B, d.b1, d.b3, d.length, d.linearDensity, d.outputPos, d.outputGain,
        d.maxPartials, d.maxFreqHz);
}

bool parseDouble(const char* s, double& v) {
    char* end = nullptr;
    v = std::strtod(s, &end);
    return end != s && *end == '\0';
}

int pianoMain(int argc, char** argv) {
    std::string scorePath, out = "out.wav";
    double fs = 48000.0, tail = 4.0, gainDb = 0.0, width = 0.35, cullDb = -100.0;
    int block = 64;
    bool cull = true, quiet = false;
    pm::TableOptions voicing;
    pm::SoundboardConfig board;
    double knock = -1.0, damperNoise = -1.0;
    pm_tools::WavFormat format = pm_tools::WavFormat::Float32;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--no-cull") { cull = false; continue; }
        if (a == "--quiet") { quiet = true; continue; }
        if (a == "--equal-temperament") { voicing.equalTemperament = true; continue; }
        if (a == "--no-board") { board.enabled = false; continue; }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", a.c_str());
            return 2;
        }
        const char* v = argv[++i];
        double d = 0.0;
        const bool num = parseDouble(v, d);
        bool ok = true;
        if (a == "--score") scorePath = v;
        else if (a == "--out") out = v;
        else if (a == "--fs") ok = num, fs = d;
        else if (a == "--tail") ok = num, tail = d;
        else if (a == "--block") ok = num, block = static_cast<int>(d);
        else if (a == "--gain-db") ok = num, gainDb = d;
        else if (a == "--width") ok = num, width = d;
        else if (a == "--cull-db") ok = num, cullDb = d;
        else if (a == "--a4") ok = num && d > 0.0, voicing.a4Hz = d;
        else if (a == "--stretch") ok = num && d >= 0.0 && d <= 1.5, voicing.stretch = d;
        else if (a == "--unison-width") ok = num && d >= 0.0, voicing.unisonWidth = d;
        else if (a == "--sympathetic") ok = num && d >= 0.0 && d <= 1.5, board.sympathetic = d;
        else if (a == "--knock") ok = num && d >= 0.0, knock = d;
        else if (a == "--damper-noise") ok = num && d >= 0.0, damperNoise = d;
        else if (a == "--format") {
            if (std::strcmp(v, "f32") == 0) format = pm_tools::WavFormat::Float32;
            else if (std::strcmp(v, "s24") == 0) format = pm_tools::WavFormat::Pcm24;
            else ok = false;
        } else {
            std::fprintf(stderr, "unknown option %s in piano mode\n", a.c_str());
            return 2;
        }
        if (!ok) {
            std::fprintf(stderr, "bad value for %s: %s\n", a.c_str(), v);
            return 2;
        }
    }
    if (fs <= 0.0 || block <= 0 || tail < 0.0) {
        std::fprintf(stderr, "invalid parameters\n");
        return 2;
    }
    std::vector<pm_tools::ScoreEvent> score;
    if (!pm_tools::readScore(scorePath.c_str(), fs, score)) return 1;

    pm::Engine engine;
    engine.setMasterGainDb(static_cast<float>(gainDb));
    engine.setStereoWidth(static_cast<float>(width));
    engine.setCulling(cull, static_cast<float>(cullDb));
    engine.setSoundboardConfig(board);
    engine.setNoiseLevels(knock >= 0.0 ? static_cast<float>(knock) : engine.knockLevel(),
                          damperNoise >= 0.0 ? static_cast<float>(damperNoise) : engine.damperNoiseLevel());
    pm::PianoTable table;
    pm::buildPianoTable(table, voicing);
    if (!engine.prepare(fs, block, table)) {
        std::fprintf(stderr, "engine prepare failed\n");
        return 1;
    }
    const double lastEvent = score.empty() ? 0.0 : score.back().seconds;
    const int n = static_cast<int>(std::lround((lastEvent + tail) * fs));
    std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
    std::vector<float> stereo(static_cast<std::size_t>(n) * 2);
    std::size_t next = 0;
    std::vector<pm::StrikeInfo> strikes;
    const auto t0 = std::chrono::steady_clock::now();
    for (int pos = 0; pos < n; pos += block) {
        const int len = std::min(block, n - pos);
        // The queue holds 1024 events: feed it one buffer ahead.
        while (next < score.size() &&
               score[next].event.time < static_cast<std::uint64_t>(pos + 2 * block) &&
               engine.pushEvent(score[next].event))
            ++next;
        engine.render(l.data(), r.data(), len, nullptr);
        for (int i = 0; i < len; ++i) {
            stereo[2 * std::size_t(pos + i)] = l[i];
            stereo[2 * std::size_t(pos + i) + 1] = r[i];
        }
        pm::StrikeInfo info;
        while (engine.popStrikeInfo(info)) strikes.push_back(info);
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    float peak = 0.0f;
    for (float v : stereo) peak = std::max(peak, std::fabs(v));
    std::size_t clipped = 0;
    if (!pm_tools::writeWav(out.c_str(), stereo, static_cast<int>(std::lround(fs)), format, &clipped, 2)) {
        std::fprintf(stderr, "could not write %s\n", out.c_str());
        return 1;
    }
    const pm::EngineStats st = engine.stats();
    std::printf("%s: %zu events, %.2f s stereo at %.0f Hz, %d modes, peak %.4f (%.1f dBFS)\n",
                out.c_str(), score.size(), n / fs, fs, st.totalModes, peak,
                20.0 * std::log10(std::max(peak, 1e-30f)));
    std::printf("  render time %.1f ms (%.1fx real time), block %d, kernel %s, culling %s, soundboard %s\n",
                elapsed * 1e3, (n / fs) / elapsed, block, pm::simdKernelName(), cull ? "on" : "off",
                board.enabled ? "on" : "off");
    std::printf("  peak before limiter %.3f, limited samples %llu, deferred strikes %llu, dropped %llu\n",
                st.peakPreLimiter, static_cast<unsigned long long>(st.limitedSamples),
                static_cast<unsigned long long>(st.deferredStrikes),
                static_cast<unsigned long long>(st.droppedEvents));
    if (!quiet) {
        std::printf("  %-8s %-5s %-9s %-10s %-9s %-8s %-9s %-9s\n", "time s", "note", "v m/s",
                    "contact ms", "in cont.", "contacts", "felt loss", "una corda");
        for (const pm::StrikeInfo& s : strikes)
            std::printf("  %-8.3f %-5d %-9.2f %-10.3f %-9.3f %-8d %-9.1f %.2f\n", s.sampleTime / fs, s.note,
                        s.velocity, s.contactSpanMs, s.contactTimeMs, s.contacts,
                        100.0 * s.feltLossFraction, s.unaCorda);
    }
    if (clipped) std::printf("  warning: %zu samples clipped in 24-bit output\n", clipped);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--score") == 0) return pianoMain(argc, argv);

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
