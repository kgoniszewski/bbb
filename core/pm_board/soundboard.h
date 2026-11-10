// Soundboard modal bank and string-board coupling loop (plan §1.6).
// Real-time part; the mode set comes from pm_voicing/soundboard_model.h (or
// any measured / finite-element set with the same data).
//
// Modes. Each board mode is a mass-normalised oscillator
//     q'' + 2 sigma q' + omega^2 q = sum_z phi_z F_z
// (phi in 1/sqrt(kg), F in N), run as a complex one-pole p with Im p = q
// and Re p = (q' + sigma q) / omega_d, exactly like the string bank:
//     p[n+1] = c p[n] + b D[n],   c = exp(lambda / fs),
//     b = (c - 1) / (lambda omega_d)      (force held over the sample)
// with the same d = Re(c) - 1 form for precision.
//
// Ports. A port is a row of mode-shape values. Zone ports are reciprocal:
// the same row maps the zone force into the modes and reads the zone's
// displacement (sum phi Im p) back. Pickup ports read a velocity
// (sum phi (omega_d Re p - sigma Im p)).
//
// Loop timing (one sample, see CouplingLoop below and soundboard_model.h):
// strings are advanced with the input computed at the previous sample and
// produce their bridge force F[n] from their new state; the board takes
// F[n] and returns the displacement at the end of its update, which the
// strings receive at n + 1. With this alignment the per-mode loop transfer,
// read as an admittance, is positive real (soundboard_model.h,
// loopModeAdmittance), so the zone-port loop is passive.
//
// Sleep. With no input for kSleepInterval samples and every output bound
// below its threshold the state is zeroed and step() costs nothing until a
// non-zero input arrives.
//
// configure / set* are non-real-time; step() and the RT-marked members are
// real-time safe (no allocation, locks or I/O).
#pragma once

#include <cstdint>

#include "pm_base/aligned.h"
#include "pm_string/mode_bank.h"

namespace pm {

class Soundboard {
public:
    static constexpr int kMaxZones = 32;
    static constexpr int kMaxPickups = 4;
    static constexpr int kSleepInterval = 64;

    // Non-real-time. Clears everything.
    bool allocate(int modes, int zones, int pickups) noexcept;
    void setMode(int i, double freqHz, double sigma, double fs) noexcept;
    void setZoneShape(int z, int i, double phi) noexcept;
    // After setMode(i): the row holds phi times the mode's velocity readout.
    void setPickupShape(int p, int i, double phi) noexcept;
    // Sleep thresholds: zone displacement bound (m) and pickup velocity bound (m/s).
    void setSleepThresholds(double displacement, double velocity) noexcept;

    int modeCount() const noexcept { return modes_; }
    int zoneCount() const noexcept { return zones_; }
    int pickupCount() const noexcept { return pickups_; }

    // RT. One sample: zoneForce[zones] in, zoneDisp[zones] and pickupVel[pickups] out.
    // With outZones (n entries), only those zones' displacements are computed.
    void step(const float* zoneForce, float* zoneDisp, float* pickupVel, Kernel kernel = Kernel::Simd,
              const int* outZones = nullptr, int nOut = -1) noexcept;
    void clearState() noexcept;
    // RT. Multiplies the state by g (panic damping).
    void scaleState(float g) noexcept;
    bool asleep() const noexcept { return asleep_; }

    // Inspection (tests): modal energy sum (q'^2 + omega^2 q^2) / 2, and state access.
    double energy() const noexcept;
    float stateRe(int i) const noexcept { return pr_[i]; }
    float stateIm(int i) const noexcept { return pi_[i]; }
    void setState(int i, float re, float im) noexcept;
    std::uint64_t stepsProcessed() const noexcept { return steps_; }

private:
    void sleepCheck() noexcept;

    int modes_ = 0, padded_ = 0, zones_ = 0, pickups_ = 0;
    AlignedArray<float> pr_, pi_, dr_, ci_, br_, bi_;
    AlignedArray<float> vRe_, vIm_;      // velocity readout: omega_d, -sigma per mode
    AlignedArray<float> zoneShape_;      // zones x padded
    AlignedArray<float> pickRe_, pickIm_;  // pickups x padded: phi * omega_d, -phi * sigma
    AlignedArray<float> scratch_;        // padded: drive
    AlignedArray<double> omega2_;        // per mode, for energy()
    float maxZoneShape_ = 0.0f, maxPickShape_ = 0.0f;
    double dispThreshold_ = 1e-14, velThreshold_ = 1e-10;
    bool asleep_ = true;
    bool inputSeen_ = false;
    int sinceCheck_ = 0;
    std::uint64_t steps_ = 0;
};

// Key -> zone mapping of the coupling loop (plan §1.6): each key reaches the
// board through at most two adjacent zones of its own bridge with weights
// w0, w1 (hat functions along the bridge), in both directions, so the
// coupling is reciprocal. The key's bridge force enters the zone forces as
// output / outputGain (the bank's output unit is outputGain newtons); the
// key's sympathetic input is the gain-weighted zone displacement in metres.
struct KeyZone {
    int z0 = 0;
    float w0 = 0.0f, w1 = 0.0f;  // zone z0 and z0 + 1
    float forceScale = 1.0f;     // newtons per bank output unit (1 / outputGain)
};

// Mixes per-key bridge forces into zone forces and zone displacements back
// into per-key inputs. RT, no state.
inline void keysToZones(const KeyZone* kz, const float* keyOut, int keys, float* zoneForce) noexcept {
    for (int k = 0; k < keys; ++k) {
        const float f = keyOut[k];
        if (f == 0.0f) continue;
        const float s = f * kz[k].forceScale;
        zoneForce[kz[k].z0] += kz[k].w0 * s;
        if (kz[k].w1 != 0.0f) zoneForce[kz[k].z0 + 1] += kz[k].w1 * s;
    }
}
inline void zonesToKeys(const KeyZone* kz, const float* zoneDisp, int keys, float gain, float* keyIn) noexcept {
    for (int k = 0; k < keys; ++k) {
        float u = kz[k].w0 * zoneDisp[kz[k].z0];
        if (kz[k].w1 != 0.0f) u += kz[k].w1 * zoneDisp[kz[k].z0 + 1];
        keyIn[k] = gain * u;
    }
}

}  // namespace pm
