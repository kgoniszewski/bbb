// Stereo microphone radiation v0 (plan §1.7). Non-real-time.
//
// Sound pressure at a microphone from the baffled board, Rayleigh integral
//   p(x_m, w) = (j w rho0 / 2 pi) Int v(x) exp(-j k r) / r dS,
// plus the open lid as a mirror source: the microphone's image in the lid
// plane, times a reflection factor, wherever the reflection point lies on
// the lid. Narrowband per mode: every board mode is a resonator, so its
// contribution is its velocity phasor times
//   G_{m,i} = (j w_i rho0 / 2 pi) Sum_patches phi_i exp(-j k_i (r - r0)) / r dA
// evaluated at the mode's damped frequency w_i, with one bulk delay r0
// removed for both microphones (the mean of their distances to the board
// centre), so the inter-microphone time difference stays in the phases.
// A complex gain on a mode's complex state is a linear combination of its
// real and imaginary parts, i.e. the existing pickup row of the board
// (Soundboard::setPickupGain): no run-time cost. Limitation: a mode radiates
// with the gain of its own frequency, so its off-resonance forced response
// gets that gain's phase; the error grows with the residual path
// difference r - r0 (board extent plus half the inter-microphone difference).
#pragma once

#include <complex>
#include <vector>

#include "pm_voicing/soundboard_model.h"

namespace pm {

class Soundboard;

// Coordinates: the board's (x along the strings from the belly rail, y
// across from the bass-side straight edge (the spine), z up from the board).
inline constexpr double kMicLevelTrim = 0.67;

struct RadiationParams {
    bool enabled = true;  // false: the v0 pickups (PlateParams::pickupX/Y), bit-identical
    // A spaced pair 0.6 m apart, 0.5 m above the board, inside the lid
    // opening over the treble/curved side: left (bass) and right (treble).
    double micX[2] = {1.00, 0.60}, micY[2] = {0.55, 1.00}, micZ[2] = {0.50, 0.50};  // m
    bool lid = true;
    double lidAngleDeg = 38.0;     // full stick
    double lidReflection = 0.9;
    double lidHingeHeight = 0.15;  // m above the board, hinge along y = 0
    double lidLength = 1.45;       // m, hinge to free edge
    double c = 343.0, rho = 1.204;
    // Grid, chosen per mode: patch at most c / (patchesPerWavelength f) and
    // at most a half wave of the shape / patchesPerHalfWave. gridScale > 1
    // refines both (convergence tests).
    double patchesPerWavelength = 5.0;
    double patchesPerHalfWave = 5.0;
    int minPatches = 16;  // per axis
    double gridScale = 1.0;
    // Normalisation band: the geometric mean of |G| over the modes in it
    // equals that of the v0 pickup shapes (keeps the output level).
    double normLoHz = 200.0, normHiHz = 2000.0;
    // Level convention, not physics (the final balance is M6 voicing): a
    // trim on that match so that all 88 keys at velocity 127 at once peak
    // at about -3.5 dBFS, as the v0 pickups do, without limiting.
    double levelTrim = kMicLevelTrim;
    // false: each microphone's own distance to the board centre is removed
    // (drops the inter-microphone time difference; tests and A/B).
    bool commonBulkDelay = true;
    bool operator==(const RadiationParams&) const = default;
};

struct MicGains {
    int modes = 0;
    std::vector<std::complex<double>> gain;  // 2 x modes, on the velocity phasor, normalised
    double scale = 0.0;   // normalisation applied (output units per Pa)
    PlateParams plate{};      // what the gains were computed for
    RadiationParams params{};
    const std::complex<double>* row(int mic) const noexcept { return gain.data() + std::size_t(mic) * modes; }
};

// Raw (unnormalised, Pa per m/s of modal amplitude) gain of mode i at microphone mic.
std::complex<double> micModeGain(const BoardModel& b, const RadiationParams& r, int mic, int i);
// All modes, both microphones, normalised to the v0 pickup level.
MicGains computeMicGains(const BoardModel& b, const RadiationParams& r);
// After loadSoundboard: replaces the pickup rows by the microphones.
void loadMicGains(Soundboard& sb, const MicGains& g) noexcept;
// Modal transfer from a shape row (force) to a microphone, the counterpart of
// boardModalAdmittance(b, shapeA, pickupShape, f): jw sum sa g / (wm^2 - w^2 + 2j sigma w).
std::complex<double> micModalTransfer(const BoardModel& b, const double* shapeA, const std::complex<double>* gain,
                                      double f) noexcept;

}  // namespace pm
