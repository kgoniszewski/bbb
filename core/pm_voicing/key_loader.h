// Per-key parameters -> coupled normal modes for the bank and the hammer
// contact solver. Non-real-time.
#pragma once

#include <array>
#include <complex>
#include <vector>

#include "pm_hammer/hammer.h"
#include "pm_string/mode_bank.h"
#include "pm_voicing/bridge_admittance.h"
#include "pm_voicing/piano_table.h"
#include "pm_voicing/unison.h"

namespace pm {

// One partial of a key: its full unison decomposition and which of its
// normal modes are kept in the bank.
struct KeyPartial {
    int n = 0;
    int firstMode = 0;  // index of its first kept mode in KeyModeSet
    int kept = 0;
    int keptIndex[kMaxOscillators] = {};  // into unison.mode[]
    UnisonPartial unison;
};

struct KeyModeSet {
    int count = 0;                    // kept normal modes
    int strings = 1;
    double modalMass = 0.0;           // mu L / 2
    // Per kept mode, ordered by partial:
    std::vector<int> partial;                             // n
    std::vector<double> freq, sigma;                      // Hz, 1/s (free string)
    std::vector<double> wOut;                             // real, output per unit Im(w)
    std::vector<std::array<std::complex<double>, kMaxStrings>> proj, input;
    std::vector<double> damperSigma;                      // 1/s extra decay with the damper seated
    // Sympathetic input (soundboard loop): dw/dt per metre of vertical
    // bridge displacement (normal coordinates), and the mode's displacement
    // amplitude at the damper per unit |w| (for the half-pedal felt contact).
    std::vector<std::complex<double>> sympIn;
    std::vector<double> damperShape;
    // The first coupledModes modes (partials below the bridge's loop
    // cutoff) are fed back by the run-time loop; the bank stores them first,
    // padded to a whole group, then the others (bankIndex()).
    int coupledModes = 0;
    std::vector<KeyPartial> partials;
    double residualCompliance = 0.0;  // m/N, static compliance at the strike
                                      // point of the partials not simulated
    double pitchRatio = 1.0;          // applied to every string so partial 1 sounds at f1
    int perturbedPartials = 0;        // eigen-solve needed a perturbation
    int uncoupledPartials = 0;        // fell back to uncoupled oscillators
};

// Bank layout of a key: coupled modes, padding to a group boundary, the rest.
int keyBankModes(const KeyModeSet& m) noexcept;
int keyCoupledGroups(const KeyModeSet& m) noexcept;
int bankIndex(const KeyModeSet& m, int i) noexcept;

// Number of partials per string (plan §1.5): below min(maxFreqHz, 0.45 fs),
// at most maxPartials.
int keyPartialCount(const KeyParams& k, double fs);

// Builds the normal modes of key k. The bridge force of a string,
//   F_b = T (n pi / L)(1 + B n^2) (-1)^(n+1) q_n   (tension and shear at a pinned end),
// times outputGain, is the output of a vertical oscillator; a horizontal
// one contributes bridge.crossRatio() of its own bridge force (the vertical
// bridge motion it causes). By reciprocity a vertical bridge displacement u
// drives oscillator k with the force (o_k / outputGain) u, i.e.
// dz_k/dt += o_k / (outputGain m omega) u: the sympathetic input. The first-partial pitch
// shift caused by the bridge reactance is compensated: every string is
// scaled by pitchRatio so that the excitation x output weighted mean
// frequency of partial 1's normal modes equals k.f1.
KeyModeSet computeKeyModes(const KeyParams& k, double fs, const BridgeAdmittance& bridge);
// With the default placeholder admittance (RigidBridge if !k.bridgeCoupling).
KeyModeSet computeKeyModes(const KeyParams& k, double fs);

// The default bridge: the default soundboard's full driving-point
// admittance (soundboard_model.h, no run-time loop).
const BridgeAdmittance& defaultBridge() noexcept;

HammerParams hammerParams(const KeyParams& k);
// The same for una corda amount u in 0..1 (plan §1.9): the missed string's
// offset grows by u^2 * unaCordaMiss and K falls by u * unaCordaSoften.
HammerParams unaCordaHammerParams(const KeyParams& k, double u);

// Writes the key's modes into bank segment s (which must hold at least
// keyBankModes(m) modes) and the matching contact coefficients. State is
// not touched. The bank's input weight is the sympathetic input of the
// coupled modes, integrated over one sample with the input held
// (sympIn (c - 1) / lambda), and 0 for the others.
void loadKeyModes(ModeBank& bank, ContactTables& contact, int s, const KeyParams& k,
                  const KeyModeSet& m, double fs);
void loadKey(ModeBank& bank, ContactTables& contact, int s, const KeyParams& k, double fs);

// Physical energy of the key's strings (sum over string-polarization
// oscillators of (m/2)(qdot^2 + (omega^2 + sigma^2) q^2)), from the bank
// state of its kept modes (zr, zi: the segment's state arrays, bank layout).
double keyStringEnergy(const KeyModeSet& m, const float* zr, const float* zi);

}  // namespace pm
