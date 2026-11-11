// Per-key parameter table v0 (plan §2.1): parameters interpolated between
// the anchor rows A0, C2, C4, C6, C7, C8 (monotone cubic, in the log domain
// for scale-like quantities), unisons of 1-3 strings, stretch tuning of the
// sounding first partial (tuning.h). Non-real-time.
#pragma once

#include <cstdint>

#include "pm_hammer/hammer.h"

namespace pm {

inline constexpr int kNumKeys = 88;
inline constexpr int kFirstMidiNote = 21;  // A0 is key 1 = MIDI note 21

struct KeyParams {
    int key = 0;                  // 1..88
    double f1 = 0.0;              // Hz, sounding first partial (tuning target)
    double f0 = 0.0;              // Hz, flexible-string fundamental, f1 / sqrt(1 + B)
    double length = 0.0;          // m
    double tension = 0.0;         // N, per string
    double B = 0.0;               // inharmonicity
    double linearDensity = 0.0;   // kg/m, from f0, L, T
    double b1 = 0.0;              // 1/s, internal (string and air) loss only
    double b3 = 0.0;              // s
    int maxPartials = 120;
    double maxFreqHz = 18000.0;   // also capped at 0.45 fs

    // Unison (plan §1.3).
    int strings = 1;                            // 1..kMaxStrings
    double detuneCents[kMaxStrings] = {};       // per string, relative to f1
    double polarizationSplitCents = 0.0;        // horizontal relative to vertical
    bool bridgeCoupling = true;                 // false: rigid bridge (uncoupled strings)
    // Normal modes kept: all 2S for partials 1..fullCoupledPartials, the
    // modesPerHighPartial largest (by excitation x output) above.
    int fullCoupledPartials = 20;
    int modesPerHighPartial = 3;

    // Hammer (plan §1.4). Mass is the whole hammer; the felt law is per
    // string contact.
    double hammerMass = 0.0;      // kg
    double feltK = 0.0;           // N / m^p
    double feltP = 0.0;
    double feltMu = 0.0;          // s/m, Hunt-Crossley damping
    double strikePos = 0.0;       // fraction of L
    int contactOversampling = 1;  // substeps per sample while the hammer is active
    double strikeOffset[kMaxStrings] = {};  // m, hammer-string mating (+ = reached later)
    double strikeHorizontal = 0.0;          // horizontal part of the hammer's direction
    // Una corda at amount u (0..1): string unaCordaString (-1: none) is
    // reached u^2 * unaCordaMiss later (grazed, then missed: felt
    // compressions are 0.3-1.5 mm, so it is missed from about u = 0.6-0.8),
    // and K is scaled by 1 - u * unaCordaSoften (the shifted hammer meets
    // softer felt).
    int unaCordaString = -1;
    double unaCordaMiss = 0.0;    // m
    double unaCordaSoften = 0.0;

    // Damper (plan §1.9). Extra decay of partial n with the damper seated:
    //   sigma_n = damperSigma * (floor + (1 - floor) * 2 <sin^2(n pi x)>)
    // averaged over the felt, x in damperPos +- damperWidth / 2.
    bool hasDamper = true;
    double damperSigma = 0.0;     // 1/s
    double damperPos = 0.0;       // fraction of L
    double damperWidth = 0.0;     // fraction of L
    double damperFloor = 0.0;

    double outputGain = 0.0;      // output units per newton of bridge force
    double pan = 0.0;             // -1 (left) .. 1 (right), before the width control
};

// Stretch tuning (tuning.h).
struct TuningOptions {
    double a4Hz = 440.0;
    double stretch = 1.0;            // 0 .. 1.5 (clamped)
    bool equalTemperament = false;   // true: ignore B, plain 12-TET
};

struct PianoTable {
    KeyParams keys[kNumKeys];
    TuningOptions tuning{};  // as the table was tuned
};

struct TableOptions {
    double a4Hz = 440.0;
    double stretch = 1.0;             // 0 .. 1.5 of the computed stretch
    bool equalTemperament = false;    // plain 12-TET on the first partial
    double unisonWidth = 1.0;         // scales every string's detuning
    double unisonSpreadCents = 0.5;   // per-string offsets about +-this at width 1
    double matingSpreadM = 15e-6;     // hammer-string offsets, +- metres
    std::uint64_t seed = 1;           // detuning and mating offsets
};

void buildPianoTable(PianoTable& table, const TableOptions& options) noexcept;
// Default options with the given A4.
void buildPianoTable(PianoTable& table, double a4Hz = 440.0) noexcept;

// String count per key (plan §2.1 assumption): 1 for keys 1-8, 2 for 9-20,
// 3 above.
int defaultStringCount(int key) noexcept;

// Monotone cubic (Fritsch-Carlson) through (x[i], y[i]), x increasing;
// constant beyond the ends.
double interpMonotone(const double* x, const double* y, int n, double at) noexcept;
// Through the six table anchors (keys 1, 16, 40, 64, 76, 88), geometric.
double interpAnchorsLog(const double* values6, double key) noexcept;

// Default MIDI-velocity mapping: v = vMin + (vMax - vMin) * x^exponent, x the
// velocity normalised to 0..1. The engine takes any function of this shape.
struct VelocityCurve {
    double vMin = 0.3;
    double vMax = 5.5;
    double exponent = 1.5;
};
using VelocityFn = double (*)(double normalizedVelocity, const void* user);
// 'user' points to a VelocityCurve.
double velocityCurveFn(double normalizedVelocity, const void* user) noexcept;

}  // namespace pm
