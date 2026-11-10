// Per-key parameters -> modal coefficients for the bank and the hammer
// contact solver. Non-real-time.
#pragma once

#include <vector>

#include "pm_hammer/hammer.h"
#include "pm_string/mode_bank.h"
#include "pm_voicing/piano_table.h"

namespace pm {

struct KeyModeSet {
    int count = 0;
    double modalMass = 0.0;           // mu L / 2
    std::vector<double> freq, sigma;  // Hz, 1/s (free string)
    std::vector<double> phi;          // mode shape at the strike point
    std::vector<double> wIn;          // impulse input weight at the strike point
    std::vector<double> wOut;         // output per metre of modal displacement
    std::vector<double> damperSigma;  // 1/s extra decay with the damper seated
    double residualCompliance = 0.0;  // m/N, static compliance at the strike
                                      // point of the partials not simulated
};

// Partial count per plan §1.5: below min(maxFreqHz, 0.45 fs), at most maxPartials.
int keyModeCount(const KeyParams& k, double fs);

// The string output is the transverse force on the bridge,
//   F_b = T sum_n q_n (n pi / L) (-1)^(n+1),
// times outputGain (no soundboard yet).
KeyModeSet computeKeyModes(const KeyParams& k, double fs);

HammerParams hammerParams(const KeyParams& k);

// Writes key k's modes into bank segment s (which must hold at least
// keyModeCount() modes) and the matching contact coefficients. State is not
// touched.
void loadKey(ModeBank& bank, ContactTables& contact, int s, const KeyParams& k, double fs);

}  // namespace pm
