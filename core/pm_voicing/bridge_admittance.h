// Bridge admittance seen by the strings of one key (plan §1.3, §1.6).
// Non-real-time.
//
// Y(omega) is the 2x2 driving-point admittance (velocity / force, s/kg) of
// the bridge at the key's string termination, polarizations ordered
// (vertical, horizontal). It must be complex symmetric with a positive
// semi-definite real part (a passive termination); the unison solver relies
// on both.
//
// Since M4 the engine uses the soundboard's admittance (SoundboardBridge,
// soundboard_model.h); the smooth placeholder below remains for unit tests
// of the unison solver and the hammer.
#pragma once

#include <complex>

namespace pm {

struct Admittance2 {
    std::complex<double> vv, vh, hh;  // s/kg
};

class BridgeAdmittance {
public:
    virtual ~BridgeAdmittance() = default;
    // key 1..88; freqHz > 0.
    virtual Admittance2 at(int key, double freqHz) const noexcept = 0;
    // Vertical bridge motion caused per unit of horizontal motion (Yvh / Yvv);
    // it weights the horizontal oscillators' share of the bridge force.
    virtual std::complex<double> crossRatio(int key, double freqHz) const noexcept {
        const Admittance2 y = at(key, freqHz);
        return std::abs(y.vv) > 0.0 ? y.vh / y.vv : std::complex<double>(0.0);
    }
    // Partials below this frequency are fed back by a run-time soundboard
    // loop (their bake excludes the loop's share, soundboard_model.h); 0: none.
    virtual double loopCutoffHz() const noexcept { return 0.0; }
    // What the strings see in total once a run-time loop adds its share:
    // the reference for pitch compensation (key_loader.h).
    virtual Admittance2 total(int key, double freqHz) const noexcept { return at(key, freqHz); }
    // The same at the complex frequency of a decaying mode, lambda = -sigma +
    // j 2 pi f (1/s): where the admittance varies on the scale of the mode's
    // own decay rate, the mode's exact pole satisfies lambda = lambda0 -
    // kappa Y(lambda). Smooth admittances may ignore sigma (the default).
    virtual Admittance2 atLambda(int key, std::complex<double> lambda) const noexcept {
        return at(key, lambda.imag() / 6.283185307179586);
    }
    virtual Admittance2 totalLambda(int key, std::complex<double> lambda) const noexcept {
        return total(key, lambda.imag() / 6.283185307179586);
    }
};

// No coupling at all (rigid bridge): every string and polarization is an
// independent oscillator. Used by the single-string cross-checks.
class RigidBridge final : public BridgeAdmittance {
public:
    Admittance2 at(int, double) const noexcept override { return {}; }
};

// Smooth parametric admittance (the M3 placeholder, used by unit tests).
//
//   Re Yvv(key, f) = G(key) * (f / fET(key))^slope
//   Yvv            = Re Yvv * (1 + j chi(f))
//   chi(f)         = reactLow -> reactHigh, raised-cosine in log f between
//                    fReactLow and fReactHigh (spring-like, Im > 0, below
//                    the board's main resonances; mass-like above)
//   Yhh            = Yvv / horizontalRatio      (bridge much stiffer sideways)
//   Yvh = Yhv      = cross * Yvv / sqrt(horizontalRatio)
//
// fET(key) is the equal-tempered first partial (A4 = 440 Hz), only used as
// a per-key frequency reference. G(key) is interpolated (monotone cubic in
// log) between anchors at A0, C2, C4, C6, C7, C8, chosen with the internal
// string losses of the table so that the decay of the fundamental is
// plausible (see piano_table.cpp): G is 2.5e-4 .. 5e-4 s/kg, i.e. an
// impedance of 2,000-4,000 kg/s, at the key's own fundamental.
// With |cross| < 1 the real part is positive semi-definite.
class PlaceholderBridgeAdmittance final : public BridgeAdmittance {
public:
    struct Params {
        double conductance[6] = {2.5e-4, 4.3e-4, 5.0e-4, 4.2e-4, 3.8e-4, 4.4e-4};
        double slope = 0.3;
        double reactLow = 0.3, reactHigh = -0.2;
        double fReactLow = 150.0, fReactHigh = 3000.0;
        double horizontalRatio = 12.0;
        double cross = 0.25;
        double scale = 1.0;  // multiplies every entry
    };
    PlaceholderBridgeAdmittance() = default;
    explicit PlaceholderBridgeAdmittance(const Params& p) : p_(p) {}
    Admittance2 at(int key, double freqHz) const noexcept override;
    const Params& params() const noexcept { return p_; }

private:
    Params p_{};
};

}  // namespace pm
