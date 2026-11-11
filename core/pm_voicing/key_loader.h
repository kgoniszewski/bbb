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
// shift caused by the bridge is compensated: every string is scaled by
// pitchRatio so that the sounding first partial (soundingPartial) equals k.f1.
KeyModeSet computeKeyModes(const KeyParams& k, double fs, const BridgeAdmittance& bridge);
// With the default placeholder admittance (RigidBridge if !k.bridgeCoupling).
KeyModeSet computeKeyModes(const KeyParams& k, double fs);

// The sounding frequency of partial n of key k (strings scaled to f0), as a
// tuner hears it: the energy-weighted mean frequency of its normal modes,
// each weighted by its output energy over the tuning window (kTuningWindowS
// seconds from kTuningStartS after the strike, i.e. after the attack;
// amplitude = output weight x the coherent sum of the string inputs, a
// hammer driving the strings in phase). A detuned unison sounds as a beating
// cluster around this mean; weakly excited aftersound modes hardly count. Without a run-time loop for that partial, over its normal modes. With one, the
// partial's oscillators are solved against the run-time admittance (frozen
// bake plus the dynamic loop) at their own complex frequency, together with
// up to kMaxLoopPoles nearby board modes of the loop as explicit states, so
// that a partial sitting on a board resonance resolves into its string-board
// doublet; the sounding frequency is then the weighted mean over the
// string-dominated modes (string participation > 0.5), and the strongest
// board-dominated mode is reported alongside.
inline constexpr int kMaxLoopPoles = 6;
inline constexpr double kTuningStartS = 0.05, kTuningWindowS = 1.0;
struct SoundingPartial {
    double freqHz = 0.0;
    double participation = 1.0;       // strings' share of the leading sounding mode
    double sigma = 0.0;               // decay rate of the leading sounding mode, 1/s
    double boardFreqHz = 0.0;         // strongest board-dominated mode (hybrid only)
    double boardParticipation = 0.0;  // strings' share of that mode
    bool hybrid = false;              // solved with explicit board modes
};
SoundingPartial soundingPartial(const KeyParams& k, double f0, int n, const BridgeAdmittance& bridge);

// f0 at which the sounding first partial of key k (soundingPartial) is at
// k.f1 (the pitch compensation of computeKeyModes, to 1e-4 cent).
double compensatedF0(const KeyParams& k, const BridgeAdmittance& bridge);

// Re-runs the virtual tuner (tuning.h) on the table with the sounding
// partial ratios of every bridge-coupled key (partials 1..kTunerPartials,
// soundingPartial against 'bridge'), so that the octave partial pairs the
// tuner matches coincide as they sound, bridge shifts and string-board
// hybrids included. Applied as a correction of the table's targets (the
// ratio of the two tuner results), so per-key edits of f1 are kept. Updates
// f1, f0 and the linear density (tension and length kept). Nothing to do
// for equal temperament.
void tuneToSoundingPartials(PianoTable& t, const BridgeAdmittance& bridge);

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
