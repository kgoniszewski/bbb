// Soundboard mode set v0 and the bridge admittance it implies (plan §1.6).
// Non-real-time.
//
// Plate. A rectangular, simply supported, orthotropic plate stands in for
// the ribbed board: x runs along the strings (0 at the belly rail, lengthX
// at the tail), y across the case (0 on the bass side). The ribs and the
// bridges are folded into equivalent bending stiffnesses Dx, Dy, Dxy (the
// "H" of the orthotropic plate equation) and into the mass per area. Modes:
//   f_mn  = (pi / 2) sqrt((Dx (m/a)^4 + 2 Dxy (m/a)^2 (n/b)^2 + Dy (n/b)^4) / rho_h)
//   phi_mn(x, y) = sin(m pi x / a) sin(n pi y / b) / sqrt(M / 4)   (mass-normalised, M = rho_h a b)
//   sigma = eta(f) pi f,   eta(f) = etaLow + (etaHigh - etaLow) (f / etaRefHz)^etaExponent, <= etaHigh
// all modes below maxFreqHz, at most maxModes. The default values are not
// derived from a measured board: they are chosen so that the mode set meets
// the plan's targets (first mode about 58 Hz, about one mode per 23 Hz
// above 1 kHz, loss factor 3 %, mean driving-point impedance about
// 1,500 kg/s at the bridge), with plausible ingredients (9 mm spruce plus
// ribs and bridges smeared to 6.6 kg/m^2; bare spruce alone would give
// Dx ~ 50 and Dy ~ 700 N m, the rib composite and the crown raise both).
// Plate size, loss factor and bridge placement were then adjusted within
// plausible ranges (a random search over the bridge curves) so that the
// fundamentals of C2, C4 and C6 do not fall on an extreme of the strongly
// fluctuating admittance (Re Y there is 0.86-1.0 times the M3 placeholder's
// conductance; C3 and C5 land at 0.14 and 3.7 times it: decay varies note
// to note and partial to partial). The treble bridge must stay off the
// plate's centre line x = a / 2, a nodal line of every mode with even m.
//
// Bridges. Treble bridge (keys 21-88) and bass bridge (keys 1-20) are
// quadratic Bezier curves in plate coordinates; keys sit at equal arc-length
// spacing. The bridge spreads a string's force over a short length: every
// shape value at a bridge point is the mean over bridgePoints points within
// +-bridgeHalfLength along the bridge, which softens the high modes.
//
// Zones (plan §1.6). Each bridge carries zone points at equal arc-length
// spacing, both ends included. A key reaches the board through the two zone
// points around it with hat-function weights (KeyZone): the run-time loop
// uses these interpolated shapes psi_k = w0 phi(z0) + w1 phi(z0 + 1), while
// the driving-point admittance uses the key's own bridge point.
//
// Above the modal range. Modes above maxFreqHz are replaced by a statistical
// continuum (mean-value admittance): a continuous density n of modes per Hz
// with the mean squared shape 1/M, the bridge-averaging factor and the same
// loss law,
//   Ycont(f) = j w INT_fmax^inf (n a(f') / M) / (w'^2 - w^2 + j eta w' w) df',
// whose real part tends to the infinite-plate value n / (4 M) =
// 1 / (8 sqrt(rho_h) (Dx Dy)^(1/4)) (for Dxy = sqrt(Dx Dy)) above maxFreqHz.
//
// Admittance seen by the strings of key k, polarizations (vertical,
// horizontal):
//   Y = Yb [[1, r], [r, r^2]] + [[0, 0], [0, Yh]]
//   Yb = sum_m phi_m(k)^2 j w / (w_m^2 - w^2 + 2 j sigma_m w) + Ycont   (vertical board)
//   r  = cross / sqrt(horizontalRatio)                (bridge rocking: horizontal share of the board motion)
//   Yh = Re(Ycont at high f) a(f) / horizontalRatio   (independent horizontal compliance, smooth)
// The real part is positive semi-definite whenever Re Yb >= 0.
//
// Baked versus fed back (plan §1.6 "double counting"). The run-time loop
// (pm_board/soundboard.h) carries the board modes; it presents to key k the
// self admittance g Yloop_k(f) (loopSelfAdmittance: the exact equivalent of
// the discrete one-sample loop, with the interpolated shape psi_k, times the
// sympathetic gain g). SoundboardBridge with g > 0 therefore bakes, for
// partials below the loop cutoff (the board's modal range),
//   Yb_baked = Yb - g Yloop_k        (real part clamped at 0)
// so that bake plus loop give Yb again, and bakes the full Yb above the
// cutoff (those modes are not fed back; they only drive the board). With
// g = 0 nothing is fed back and the full Yb is baked.
//
// Static correction (mode-acceleration form). A string mode driven by the
// bridge displacement u responds quasi-statically with the force
// o_n^2 / (m w_n^2) u = 2 T / L (1 + B n^2) u, the same for every partial,
// so the truncated modal sum presents the board with a large negative
// stiffness K = sum over the fed-back partials (it diverges with their
// number; the narrowband bake never sees it). Uncorrected, a few keys in one
// zone exceed the board's own stiffness and the loop diverges at 0 Hz. The
// engine therefore subtracts each key's static response from its bridge
// force, F_k - (K0 u_k[n] + K1 u_k[n-1]), with the taps matched to the
// discrete bank's exact static gain and its first derivative in frequency
// (K0 + K1 = K, K1 = -D1; about 1.5 K and -0.5 K). What is left is the
// convergent Mittag-Leffler series of the string's end stiffness,
// sum 2T/L w^2 / (w_n^2 - w^2): a small, passive, mass-like loading. (A
// one-tap correction leaves a term of order w T K, a dashpot of either
// sign; summed over the keys it undamps the first board mode.) At partial
// j's frequency the other fed-back partials respond quasi-statically and
// cancel their share of the correction; what is left, the string's
// non-resonant end stiffness D_j(w) = sum_{n != j} K_n w^2 / (w_n^2 - w^2) - K_j
// (stringResidualStiffness), is small. The self admittance of the loop is
// therefore
// (F = F_res + D_j u, u = g L F) therefore
//   g Yloop = fs (z - 1) g L / (1 - g L D_j),  L = z^-1 sum psi^2 H.
#pragma once

#include <array>
#include <complex>
#include <vector>

#include "pm_board/soundboard.h"
#include "pm_voicing/bridge_admittance.h"
#include "pm_voicing/piano_table.h"

namespace pm {

struct BridgeCurve {
    double x0, y0, cx, cy, x1, y1;  // first key, control point, last key (m)
    int firstKey, lastKey;          // 1..88
    int zones;
};

struct PlateParams {
    double lengthX = 1.93, widthY = 1.25;    // m
    double massPerArea = 6.6;                // kg/m^2
    double Dx = 2430.0, Dy = 20000.0, Dxy = 2090.0;  // N m
    double etaLow = 0.03, etaHigh = 0.03, etaRefHz = 5000.0, etaExponent = 0.6;
    double maxFreqHz = 5000.0;
    int maxModes = 600;
    double bridgeHalfLength = 0.025;         // m
    int bridgePoints = 5;
    BridgeCurve bass{1.745, 0.202, 1.660, 0.372, 1.513, 0.542, 1, 20, 4};
    BridgeCurve treble{1.345, 0.376, 0.632, 0.503, 0.303, 1.065, 21, 88, 14};
    double pickupX[2] = {1.20, 0.60}, pickupY[2] = {0.35, 0.95};  // left, right
    double horizontalRatio = 12.0;
    double cross = 0.25;
    int continuationPoints = 360;  // log-frequency grid of the continuation table
};

struct BoardModel {
    PlateParams params;
    int modes = 0;
    std::vector<double> freq, sigma;  // Hz, 1/s
    std::vector<int> mIndex, nIndex;  // plate mode numbers (0 for a measured set)
    int zones = 0;
    std::vector<double> zoneShape;    // zones x modes, 1/sqrt(kg)
    std::vector<double> keyShape;     // kNumKeys x modes, at each key's bridge point
    std::vector<double> pickupShape;  // 2 x modes
    std::array<KeyZone, kNumKeys> keyZone{};  // forceScale left at 1
    double keyX[kNumKeys] = {}, keyY[kNumKeys] = {};
    std::vector<double> zoneX, zoneY;
    double mass = 0.0;           // kg
    double density = 0.0;        // modes per Hz (asymptotic)
    double meanConductance = 0.0;  // n / (4 M), s/kg
    // Statistical continuation, tabulated on a log-frequency grid.
    std::vector<double> contF;
    std::vector<std::complex<double>> contY;
    std::vector<double> bridgeTab;  // a(f) on a log grid

    std::complex<double> continuation(double f) const noexcept;
    double eta(double f) const noexcept;
    double bridgeFactor(double f) const noexcept;  // a(f), 1 at low f
    const double* shapeOfKey(int key) const noexcept { return keyShape.data() + std::size_t(key - 1) * modes; }
    const double* shapeOfZone(int z) const noexcept { return zoneShape.data() + std::size_t(z) * modes; }
};

BoardModel buildPlateBoard(const PlateParams& p);
// Built once from the default parameters (thread-safe static).
const BoardModel& defaultBoard();

// Modal part of the vertical (transfer) admittance between two shape rows.
std::complex<double> boardModalAdmittance(const BoardModel& b, const double* shapeA, const double* shapeB,
                                          double f) noexcept;
// The same at a complex frequency s (1/s): sum sa sb s / (w_m^2 + s^2 + 2 sigma_m s).
std::complex<double> boardModalAdmittanceS(const BoardModel& b, const double* shapeA, const double* shapeB,
                                           std::complex<double> s) noexcept;
// Vertical driving-point admittance at a key's bridge point: modal + continuation.
std::complex<double> boardDrivingPoint(const BoardModel& b, int key, double f) noexcept;
std::complex<double> boardDrivingPointS(const BoardModel& b, int key, std::complex<double> s) noexcept;
// Interpolated loop shape psi_k of a key (modes values).
void keyLoopShape(const BoardModel& b, int key, double* psi) noexcept;
// Equivalent continuous admittance of one board mode inside the discrete
// one-sample loop (unit shape): fs (z - 1) z^-1 H(z), z = exp(j 2 pi f / fs).
std::complex<double> loopModeAdmittance(double freqHz, double sigma, double f, double fs) noexcept;
// At a complex frequency s (z = exp(s / fs)).
std::complex<double> loopModeAdmittanceS(double freqHz, double sigma, std::complex<double> s, double fs) noexcept;
// The loop's self admittance at key k (gain 1, no static correction):
// sum psi_k^2 loopModeAdmittance.
std::complex<double> loopSelfAdmittance(const BoardModel& b, int key, double f, double fs) noexcept;
// With loop gain g and the string's residual end stiffness D (N/m), see above.
std::complex<double> loopSelfAdmittance(const BoardModel& b, int key, double f, double fs, double g,
                                        double D) noexcept;
std::complex<double> loopSelfAdmittanceS(const BoardModel& b, int key, std::complex<double> s, double fs, double g,
                                         std::complex<double> D) noexcept;
// Static stiffness (N/m) of the fed-back partials of a key (f_n < cutoff):
// K = sum K_n, K_n = S (1 + r^2) (T n pi / L (1 + B n^2))^2 / (m w_n^2).
double stringStaticStiffness(const KeyParams& k, double fs, double cutoffHz, double r) noexcept;
// sum over the fed-back partials n not within 2 % of f of K_n w^2 / (w_n^2 - w^2):
// another key's strings seen from a board vibrating at f.
double stringLoadStiffness(const KeyParams& k, double fs, double cutoffHz, double r, double f) noexcept;
// D_j(f) above, j the fed-back partial nearest to f.
double stringResidualStiffness(const KeyParams& k, double fs, double cutoffHz, double r, double f) noexcept;
// At a complex frequency s (w^2 -> -s^2), own partial nearest to Im s.
std::complex<double> stringResidualStiffnessS(const KeyParams& k, double fs, double cutoffHz, double r,
                                              std::complex<double> s) noexcept;

class SoundboardBridge final : public BridgeAdmittance {
public:
    // loopGain = sympathetic gain g of the run-time loop (0: no loop). With a
    // table, each key's static correction enters the loop's self term.
    SoundboardBridge(const BoardModel& board, double fs, double loopGain, const PianoTable* table = nullptr);
    Admittance2 at(int key, double freqHz) const noexcept override;
    std::complex<double> crossRatio(int, double) const noexcept override { return r_; }
    double loopCutoffHz() const noexcept override { return g_ > 0.0 ? board_.params.maxFreqHz : 0.0; }
    // Driving point plus, with a loop, the first-order loading of the board
    // by the other keys' strings (their non-resonant end stiffness), which
    // the loop produces: the reference for pitch compensation. The bake
    // itself does not include it (it is not double counted: the loop adds it).
    Admittance2 total(int key, double freqHz) const noexcept override;
    Admittance2 atLambda(int key, std::complex<double> lambda) const noexcept override;
    Admittance2 totalLambda(int key, std::complex<double> lambda) const noexcept override;
    std::complex<double> loadingShift(int key, double freqHz) const noexcept;
    // Vertical board admittance as baked (after removing the loop's share).
    std::complex<double> baked(int key, double freqHz) const noexcept;
    std::complex<double> bakedS(int key, std::complex<double> s) const noexcept;
    const BoardModel& board() const noexcept { return board_; }
    double loopGain() const noexcept { return g_; }

private:
    const BoardModel& board_;
    double fs_, g_, r_;
    KeyParams keys_[kNumKeys]{};
    bool hasKeys_ = false;
};

// Configures the run-time bank (allocates; non-real-time). Pickups 0 and 1.
bool loadSoundboard(Soundboard& sb, const BoardModel& b, double fs);

}  // namespace pm
