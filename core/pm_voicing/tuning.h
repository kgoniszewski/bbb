// Stretch tuning: a "virtual tuner" (plan §2.3). Non-real-time.
//
// Given each key's inharmonicity B, partial n of key k sounds at
//   f_k,n = f1_k * r_k(n),   r_k(n) = n sqrt((1 + B_k n^2) / (1 + B_k))
// where f1 is the sounding first partial (the tuning target). The tuner
// sets f1 so that chosen partial pairs of octaves coincide, as an aural
// tuner does:
//
//  1. A4 (key 49) = a4Hz.
//  2. Temperament octave A3-A4: A3 is tuned to A4 as a 4:2 octave (partial 4
//     of A3 = partial 2 of A4); the eleven keys between divide that octave
//     equally on their second partials (equal temperament inside the
//     stretched octave, set on upper partials as an aural tuner hears
//     intervals; with ratios from B alone this differs from dividing the
//     first partials by the variation of B over the octave, < 0.01 cent).
// With sounding ratios (PartialRatios) every r_k(n) below is replaced by the
// ratio of the sounding frequencies.
//
//  3. Upwards, key k from key k-12 (and k-24): 4:2 up to key 56 (E5),
//     blending to 2:1 by key 68 (E6); above key 76 (C7) the 2:1 octave
//     blends towards the 4:1 double octave (partial 4 of key k-24 = partial
//     1 of key k), reaching an equal blend at key 88.
//  4. Downwards, key k from key k+12: 4:2 down to key 28 (C3), blending to
//     6:3 by key 18 (D2); below key 16 (C2) the 6:3 octave blends towards
//     the wider 10:5 octave, reached at key 1 (A0). With this table's B
//     (smallest around C2) pure 6:3 octaves give only about -11 cents at A0;
//     aural tuners widen the lowest octaves the same way.
//
// Blends are raised-cosine weights in key number, applied to the log
// frequencies of the per-type targets, so the curve has no corners. The
// deviation from equal temperament, d_k (cents), is then scaled by the user
// stretch amount (0 = plain equal temperament, 1 = as computed, up to 1.5).
#pragma once

#include "pm_voicing/piano_table.h"

namespace pm {

// Sounding partial ratios f_k,n / f_k,1 for n = 1..kTunerPartials, per key
// (e.g. including the bridge's partial shifts, key_loader.h); keys or
// entries left at 0 use r_k(n).
inline constexpr int kTunerPartials = 10;
struct PartialRatios {
    double r[kNumKeys][kTunerPartials + 1] = {};
};

// Octave-type weights used for key k (0 for keys tuned by equal division or
// fixed). Upward keys (> 49): w42, w21, w41; downward keys (< 37): w42, w63,
// w105.
struct OctaveWeights {
    double w42 = 0.0, w21 = 0.0, w41 = 0.0, w63 = 0.0, w105 = 0.0;
};
OctaveWeights octaveWeights(int key) noexcept;

// r_k(n) for inharmonicity B.
double partialRatio(double B, int n) noexcept;

// f1Out[88]: sounding first-partial targets. B[88]: per-key inharmonicity.
// With 'ratios', the octave partial pairs are matched on those ratios
// instead of r_k(n).
void virtualTuner(const double* B, const TuningOptions& opt, double* f1Out,
                  const PartialRatios* ratios = nullptr) noexcept;

// Deviation of f1 from 12-TET (A4 = a4Hz), cents.
double centsFromEqualTemperament(double f1, int key, double a4Hz) noexcept;

// Reference stretch curve (cents from 12-TET) used to validate the tuner: a
// smooth curve through typical values read off published Railsback curves
// for grand pianos (Railsback 1938; Martin & Ward 1961; Fletcher & Rossing,
// "The Physics of Musical Instruments", fig. 12.x on piano tuning):
//   A0 -25, C1 -16, C2 -7, C3 -2.5, C4 -0.5, A4 0, C5 +1, C6 +5, C7 +13,
//   C8 +30 cents
// (monotone cubic in between).
// These are representative, not measured Steinway D values.
double railsbackReferenceCents(int key) noexcept;

}  // namespace pm
