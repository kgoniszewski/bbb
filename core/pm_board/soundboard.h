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
// loopModeAdmittance), so the zone-port loop is passive. CouplingLoop
// computes exactly this recursion, but only the board and an 18-zone core
// are stepped per sample; the string modes run a sub-block at a time.
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
    // RT. Displacement of zone z from the current state (what step() last
    // returned for it, also when it was not among the zones read).
    float zoneDisplacement(int z) const noexcept;
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

// The string-board loop solved exactly per sub-block (plan §1.6, M4 note;
// mode_bank.h, "Coupled processing").
//
// Every key is a single-input single-output system for the loop: input u_k
// (gain times its interpolated zone displacement), output F_k (its bridge
// force). Over a sub-block of n samples its output is its free response F0_k
// (no input, a block pass of the bank) plus the convolution of its input
// with its impulse response h_k (n taps, static correction included). Keys
// enter the zones through at most two adjacent zones each, so the forced
// part summed into zone forces is a convolution of the zone displacements
// with a banded zone-to-zone kernel
//   G_ab[j] = g sum_k a_ak a_bk forceScale_k h_k[j]     (a, b zones of key k)
// (about 50 non-zero entries for 18 zones). Per sample only this core runs:
// zone force = free part + sum_j G[j] d[i-j], board step, new displacement.
// At the end of the sub-block each key's input sequence is known and the
// bank adds its share to the key's state (Horner, mode_bank.h). This is the
// per-sample loop rearranged by superposition, not an approximation: the
// trajectory is the same up to rounding, so stability and passivity are
// those of the per-sample loop (soundboard_model.h, test_passivity).
// h_k changes with the key's damper amount and awake set; it is recomputed
// (about one free pass of the key's coupled modes) only when it changes.
//
// Keys owned by a hammer solver (io[k].skip) are not part of G: the caller
// steps them per sample with keyInput() and adds their force itself.
//
// Culling. A sleeping coupled group that input wakes during a sub-block
// joins at its end, from the state it would have had had it woken at the
// sample its bound was crossed (mode_bank.h); its output over the rest of
// that sub-block is missing from the loop, which is negligible because the
// bound was only just crossed. A key that starts taking input (its damper
// lifted by the pedal) has its sleeping coupled groups woken at once, before
// the sub-block, since its input is then typically far above the bound.
// Every such decision falls on a sample fixed relative to the music, so the
// output does not depend on how the stream is cut into sub-blocks (to
// rounding).
//
// configure() is non-real-time; everything else is RT.
class CouplingLoop {
public:
    static constexpr int kMaxBlock = ModeBank::kResponseTaps;
    static constexpr int kMaxKeys = 128;
    static constexpr int kMaxEntries = 4 * kMaxKeys;

    static constexpr int kMaxTaps = 2;

    // kz[keys]: the keys' zones and force scales; zones of the board. 'taps'
    // extra outputs (a direct path, e.g. stereo) receive sum_k tapGain[t][k]
    // forceScale_k F_k, exactly like the zone forces.
    bool configure(const KeyZone* kz, int keys, int zones, int taps = 0) noexcept;
    // RT (rebuilds the kernel at the next sub-block when changed).
    void setTapGain(int t, int k, float g) noexcept {
        if (tapGain_[t][k] != g) dirty_ = true;
        tapGain_[t][k] = g;
    }
    void setGain(float g) noexcept {
        if (g != gain_) dirty_ = true;
        gain_ = g;
    }
    float gain() const noexcept { return gain_; }
    void reset() noexcept;

    // Sub-block of n <= kMaxBlock samples, in this order:
    //   begin(), ModeBank::processSegmentsFree(), collect(),
    //   per sample forces() / keyInput(), board step, push(), then end().
    // Keys that receive an input: awake or wakeable coupled groups (as the
    // bank reports them) and skipped keys. A key that starts to take input
    // has its sleeping coupled groups woken here (wakeAllCoupled), so that
    // they are part of the loop from this sample on. io[k].skip must be set.
    void begin(ModeBank& bank, const Soundboard& board, const SegmentIO* io, Kernel kernel = Kernel::Simd) noexcept;
    // The free zone forces, from the outputs of processSegmentsFree.
    void collect(ModeBank& bank, const SegmentIO* io, int n) noexcept;
    // Zones whose displacement the loop needs (pass to Soundboard::step).
    int outZones(const int*& list) const noexcept {
        list = outZones_;
        return nOut_;
    }
    // ADDS to zoneForce the zone forces of sample i (free responses of all
    // non-skipped keys and the forced response to d[0..i]), and to tap[t]
    // the taps' outputs (tap may be null when there are none).
    void forces(int i, float* zoneForce, float* tap, Kernel kernel = Kernel::Simd) noexcept;
    // Input of key k at sample i (gain times its zone displacement).
    float keyInput(int k, int i) const noexcept {
        const KeyZone& z = kz_[k];
        float u = z.w0 * d_[i * Soundboard::kMaxZones + z.z0];
        if (z.w1 != 0.0f) u += z.w1 * d_[i * Soundboard::kMaxZones + z.z0 + 1];
        return gain_ * u;
    }
    // The board's zone displacements after sample i (input of sample i + 1).
    void push(int i, const float* zoneDisp) noexcept;
    // Applies the inputs of the sub-block to the bank (and wakes groups).
    void end(ModeBank& bank, const SegmentIO* io, int n, Kernel kernel = Kernel::Simd) noexcept;

    // Inspection: keys in the loop (awake coupled groups) during the last sub-block.
    int memberCount() const noexcept { return nMembers_; }
    int entryCount() const noexcept { return nEnt_; }

private:
    void rebuildKernel() noexcept;

    int keys_ = 0, zones_ = 0, taps_ = 0, nEnt_ = 0, entPad_ = 0;
    float tapGain_[kMaxTaps][kMaxKeys]{};
    float gain_ = 0.0f;
    bool dirty_ = true;
    KeyZone kz_[kMaxKeys]{};
    // Destination: a zone, or zones + t for tap t.
    int entDst_[kMaxEntries + kMaxTaps * Soundboard::kMaxZones]{}, entSrc_[kMaxEntries + kMaxTaps * Soundboard::kMaxZones]{};
    int tapEnt_[kMaxTaps][Soundboard::kMaxZones]{};
    // Entries carried by the current members, packed: their destination and
    // source zones; G_ and dE_ use this packed order (nAct_, padded actPad_).
    int actDst_[kMaxEntries + kMaxTaps * Soundboard::kMaxZones]{}, actSrc_[kMaxEntries + kMaxTaps * Soundboard::kMaxZones]{};
    int nAct_ = 0, actPad_ = 0;
    int keyEnt_[kMaxKeys][4]{};       // entries of key k: (z0,z0), (z0,z1), (z1,z0), (z1,z1)
    float keyEntW_[kMaxKeys][4]{};    // their weights a_ak a_bk
    AlignedArray<float> h_;           // kMaxKeys x kMaxBlock
    float k1_[kMaxKeys]{};
    std::uint32_t ver_[kMaxKeys]{};
    bool hasH_[kMaxKeys]{}, member_[kMaxKeys]{}, listen_[kMaxKeys]{}, newListener_[kMaxKeys]{};
    int members_[kMaxKeys]{}, listenList_[kMaxKeys]{};
    int nMembers_ = 0, nListen_ = 0;
    AlignedArray<float> G_;           // kMaxBlock x entPad
    AlignedArray<float> dE_;          // kMaxBlock x entPad: source displacement per entry
    AlignedArray<float> acc_;         // entPad
    AlignedArray<float> d_;           // (kMaxBlock + 1) x kMaxZones
    AlignedArray<float> f0_;          // kMaxBlock x (kMaxZones + kMaxTaps): free zone forces and taps
    AlignedArray<float> u_;           // kMaxBlock
    int outZones_[Soundboard::kMaxZones]{};
    int nOut_ = 0;
    bool zoneOut_[Soundboard::kMaxZones]{}, zoneOutPrev_[Soundboard::kMaxZones]{};
};

}  // namespace pm
