// Stiff, lossy string -> modal coefficients (plan §1.2), and pluck excitation.
//
//   f_n     = n * f0 * sqrt(1 + B n^2)
//   sigma_n = b1 + b3 * (2 pi f_n)^2          T60_n = ln(1000) / sigma_n
//   phi_n(x) = sin(n pi x / L)                modal mass m_n = mu L / 2
//
// Note that f0 is the fundamental of the equivalent flexible string; the
// first partial sounds at f0 * sqrt(1 + B).
//
// Each mode's state z is scaled so that Im(z) is the modal displacement q_n in
// metres (and Re(z) ~ dq_n/dt / omega_n). The force input u is in newtons.
#pragma once

#include "pm_string/mode_bank.h"

namespace pm {

struct StringParams {
    double f0 = 261.63;             // Hz
    double B = 3.8e-4;              // inharmonicity coefficient
    double b1 = 0.5;                // 1/s, frequency-independent loss
    double b3 = 6.25e-9;            // s, loss growing with frequency squared
    double length = 0.62;           // m, speaking length L
    double linearDensity = 6.4e-3;  // kg/m, mu
    double inputPos = 0.12;         // force input point, as a fraction of L
    double outputPos = 0.37;        // observation point, as a fraction of L
    double outputGain = 500.0;      // output units per metre of displacement
    int maxPartials = 120;
    double maxFreqHz = 18000.0;     // partials stay below min(this, 0.45 fs)
};

struct ModeSpec {
    double freqHz;
    double sigma;  // 1/s
    double wIn;    // state units per newton per sample
    double wOut;   // output units per metre of modal displacement
};

double partialFrequency(double f0, double B, int n) noexcept;
double partialDecayRate(double b1, double b3, double freqHz) noexcept;

// Number of partials kept for these parameters at sample rate fs.
int countStringModes(const StringParams& p, double fs) noexcept;

// Fills out[0..count) for partials n = 1..count; returns count (<= maxModes).
int computeStringModes(const StringParams& p, double fs, ModeSpec* out, int maxModes) noexcept;

// Configures bank modes 0..count) for this string, clears state, and resizes
// the bank to count. Returns count, or 0 if the bank capacity is too small.
int loadString(ModeBank& bank, const StringParams& p, double fs) noexcept;

// Recomputes only the rotation coefficients (pitch and decay) of a string
// already loaded, leaving state and weights untouched, so it may be called
// while the string rings. Modes whose new frequency exceeds the limit are
// faded out with a short decay instead of being cut.
void retuneString(ModeBank& bank, const StringParams& p, double fs) noexcept;

// Ideal pluck from rest: initial triangular displacement of peak height
// 'heightM' at xPluck (fraction of L), zero velocity. Replaces the state.
//   q_n(0) = 2 h sin(n pi xp) / (pi^2 n^2 xp (1 - xp))
void pluck(ModeBank& bank, double xPluck, double heightM) noexcept;

}  // namespace pm
