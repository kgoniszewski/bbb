// Bridge admittance seen by the strings of one key (plan §1.3, §1.6).
// Non-real-time.
//
// Y(omega) is the 2x2 driving-point admittance (velocity / force, s/kg) of
// the bridge at the key's string termination, polarizations ordered
// (vertical, horizontal). It must be complex symmetric with a positive
// semi-definite real part (a passive termination); the unison solver relies
// on both.
//
// M4 replaces the placeholder below with the driving-point admittance of
// the soundboard model at each key's bridge point; nothing else changes.
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
};

// No coupling at all (rigid bridge): every string and polarization is an
// independent oscillator. Used by the single-string cross-checks.
class RigidBridge final : public BridgeAdmittance {
public:
    Admittance2 at(int, double) const noexcept override { return {}; }
};

// PLACEHOLDER (until the M4 soundboard): a smooth parametric admittance.
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
