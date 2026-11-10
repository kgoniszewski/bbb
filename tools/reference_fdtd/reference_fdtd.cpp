// Offline FDTD reference for one string of a key (see fdtd_string.h).
//
//   reference_fdtd --key 40 [--velocity 2.2] [--points 400 | --oversample 8]
//                  [--seconds 0.5] [--fs 48000] [--partials 30]
//                  [--wav bridge.wav] [--csv out.csv]
//
// The string and hammer come from the engine's per-key table: one string
// of the unison with its own share of the hammer mass (mass / strings), as
// the real-time model's single-string cross-check uses. Prints the grid,
// contact duration and, per partial, the modal energy at the end of the
// contact (extrapolated back to t = 0 with the partial's decay rate). Writes
// the bridge force (WAV, scaled by the table's outputGain) and a CSV of
// time, hammer force, strike-point displacement and bridge force at the
// output rate.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../render_cli/wav_writer.h"
#include "fdtd_string.h"
#include "pm_voicing/piano_table.h"

namespace {

pm_fdtd::StringSpec specFromTable(int key, double velocity) {
    pm::PianoTable t;
    pm::buildPianoTable(t);
    const pm::KeyParams& k = t.keys[key - 1];
    pm_fdtd::StringSpec s;
    s.length = k.length;
    s.tension = k.tension;
    s.linearDensity = k.linearDensity;
    s.B = k.B;
    s.b1 = k.b1;
    s.b3 = k.b3;
    s.strikePos = k.strikePos;
    s.hammerMass = k.hammerMass / k.strings;
    s.feltK = k.feltK;
    s.feltP = k.feltP;
    s.feltMu = k.feltMu;
    s.velocity = velocity;
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    int key = 40, points = 0, partials = 30;
    double velocity = 2.2, seconds = 0.5, fs = 48000.0, oversample = 8.0;
    std::string wav, csv;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::printf("usage: reference_fdtd --key K [--velocity V] [--points N | --oversample R]\n"
                        "       [--seconds S] [--fs HZ] [--partials P] [--wav FILE] [--csv FILE]\n");
            return 0;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", a.c_str());
            return 2;
        }
        const char* v = argv[++i];
        if (a == "--key") key = std::atoi(v);
        else if (a == "--velocity") velocity = std::atof(v);
        else if (a == "--points") points = std::atoi(v);
        else if (a == "--oversample") oversample = std::atof(v);
        else if (a == "--seconds") seconds = std::atof(v);
        else if (a == "--fs") fs = std::atof(v);
        else if (a == "--partials") partials = std::atoi(v);
        else if (a == "--wav") wav = v;
        else if (a == "--csv") csv = v;
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }
    if (key < 1 || key > 88 || velocity <= 0.0 || seconds <= 0.0 || fs <= 0.0) {
        std::fprintf(stderr, "invalid parameters\n");
        return 2;
    }
    const pm_fdtd::StringSpec spec = specFromTable(key, velocity);
    double k = 1.0 / (fs * oversample);
    if (points > 0) {
        // A whole number of steps per output sample.
        const double kMax = pm_fdtd::stepForPoints(spec, points);
        k = 1.0 / (fs * std::ceil(1.0 / (fs * kMax)));
    }
    const int perSample = static_cast<int>(std::lround(1.0 / (fs * k)));
    k = 1.0 / (fs * perSample);
    pm_fdtd::FdtdString str(spec, k);
    std::printf("key %d: L %.3f m, T %.0f N, mu %.3e kg/m, B %.2e; grid N = %d (h = %.3g mm), "
                "%d steps per output sample (%.3g MHz)\n",
                key, spec.length, spec.tension, spec.linearDensity, spec.B, str.grid().points,
                1e3 * str.grid().h, perSample, 1e-6 / k);

    const int nOut = static_cast<int>(seconds * fs);
    std::vector<float> out(static_cast<std::size_t>(nOut));
    std::FILE* f = csv.empty() ? nullptr : std::fopen(csv.c_str(), "w");
    if (f) std::fprintf(f, "t,hammer_force,strike_displacement,bridge_force\n");
    pm::PianoTable t;
    pm::buildPianoTable(t);
    const double gain = t.keys[key - 1].outputGain;
    double contactStart = -1.0, contactEnd = -1.0;
    pm_fdtd::ModalState atEnd;
    bool wasIn = false;
    for (int s = 0; s < nOut; ++s) {
        double fh = 0.0, ys = 0.0, fb = 0.0;
        for (int r = 0; r < perSample; ++r) {
            str.step();
            fh += str.hammerForce();
            ys += str.strikeDisplacement();
            fb += str.bridgeForce();
            const bool in = str.inContact();
            if (in && contactStart < 0.0) contactStart = str.time();
            if (wasIn && !in) {
                contactEnd = str.time();
                atEnd = str.modes(partials);
            }
            wasIn = in;
        }
        fh /= perSample;
        ys /= perSample;
        fb /= perSample;
        out[static_cast<std::size_t>(s)] = static_cast<float>(gain * fb);
        if (f) std::fprintf(f, "%.8f,%.6e,%.6e,%.6e\n", (s + 1) / fs, fh, ys, fb);
    }
    if (f) std::fclose(f);
    std::printf("contact %.3f to %.3f ms; hammer leaves with %.3f m/s\n", 1e3 * contactStart,
                1e3 * contactEnd, str.hammerVelocity());
    if (!atEnd.energy.empty()) {
        std::printf("partial energies at the end of the last contact (J, extrapolated to t = 0):\n");
        for (std::size_t n = 0; n < atEnd.energy.size(); ++n) {
            const double e = atEnd.energy[n] * std::exp(2.0 * str.modeDecayRate(int(n) + 1) * contactEnd);
            std::printf("  n %2zu  %8.1f Hz  %.4e  (%6.1f dB)\n", n + 1, atEnd.omega[n] / (2.0 * 3.141592653589793),
                        e, 10.0 * std::log10(e + 1e-300));
        }
    }
    if (!wav.empty() && !pm_tools::writeWav(wav.c_str(), out, static_cast<int>(fs), pm_tools::WavFormat::Float32)) {
        std::fprintf(stderr, "could not write %s\n", wav.c_str());
        return 1;
    }
    return 0;
}
