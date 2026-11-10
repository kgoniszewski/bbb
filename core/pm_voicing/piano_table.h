// Per-key parameter table v0 (plan §2.1): one string per key, parameters
// interpolated between the anchor rows A0, C2, C4, C6, C7, C8 (monotone
// cubic, in the log domain for scale-like quantities), equal temperament on
// the sounding first partial. Non-real-time.
#pragma once

namespace pm {

inline constexpr int kNumKeys = 88;
inline constexpr int kFirstMidiNote = 21;  // A0 is key 1 = MIDI note 21

struct KeyParams {
    int key = 0;                  // 1..88
    double f1 = 0.0;              // Hz, sounding first partial (tuning target)
    double f0 = 0.0;              // Hz, flexible-string fundamental, f1 / sqrt(1 + B)
    double length = 0.0;          // m
    double tension = 0.0;         // N
    double B = 0.0;               // inharmonicity
    double linearDensity = 0.0;   // kg/m, from f0, L, T
    double b1 = 0.0;              // 1/s
    double b3 = 0.0;              // s
    int maxPartials = 120;
    double maxFreqHz = 18000.0;   // also capped at 0.45 fs

    // Hammer (effective per-string values; see piano_table.cpp).
    double hammerMass = 0.0;      // kg
    double feltK = 0.0;           // N / m^p
    double feltP = 0.0;
    double feltMu = 0.0;          // s/m, Hunt-Crossley damping
    double strikePos = 0.0;       // fraction of L
    int contactOversampling = 1;  // substeps per sample while the hammer is active

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

struct PianoTable {
    KeyParams keys[kNumKeys];
};

// a4Hz: pitch of A4 (key 49). Fills all 88 rows.
void buildPianoTable(PianoTable& table, double a4Hz = 440.0) noexcept;

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
