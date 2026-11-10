#include "pm_voicing/bridge_admittance.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_voicing/piano_table.h"

namespace pm {

Admittance2 PlaceholderBridgeAdmittance::at(int key, double freqHz) const noexcept {
    const double fEt = 440.0 * std::pow(2.0, (key - 49.0) / 12.0);
    const double g = p_.scale * interpAnchorsLog(p_.conductance, key) *
                     std::pow(std::max(freqHz, 1.0) / fEt, p_.slope);
    const double t = std::clamp(std::log(std::max(freqHz, 1.0) / p_.fReactLow) /
                                    std::log(p_.fReactHigh / p_.fReactLow),
                                0.0, 1.0);
    const double chi = p_.reactLow + (p_.reactHigh - p_.reactLow) * (0.5 - 0.5 * std::cos(std::numbers::pi * t));
    const std::complex<double> vv(g, g * chi);
    Admittance2 y;
    y.vv = vv;
    y.hh = vv / p_.horizontalRatio;
    y.vh = p_.cross * vv / std::sqrt(p_.horizontalRatio);
    return y;
}

}  // namespace pm
