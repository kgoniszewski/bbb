#include "pm_voicing/soundboard_model.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <cstdint>
#include <tuple>

namespace pm {

namespace {

constexpr double kPi = std::numbers::pi;
using cd = std::complex<double>;

struct Point {
    double x, y;
};

Point bezier(const BridgeCurve& c, double t) {
    const double u = 1.0 - t;
    return {u * u * c.x0 + 2.0 * u * t * c.cx + t * t * c.x1, u * u * c.y0 + 2.0 * u * t * c.cy + t * t * c.y1};
}
Point bezierTangent(const BridgeCurve& c, double t) {
    const double dx = 2.0 * (1.0 - t) * (c.cx - c.x0) + 2.0 * t * (c.x1 - c.cx);
    const double dy = 2.0 * (1.0 - t) * (c.cy - c.y0) + 2.0 * t * (c.y1 - c.cy);
    const double l = std::hypot(dx, dy);
    return {dx / l, dy / l};
}

// Arc-length table of a curve: parameter t for a fraction s of the length.
struct Arc {
    std::vector<double> len;  // cumulative, at t = i / (N - 1)
    double total = 0.0;
    explicit Arc(const BridgeCurve& c) {
        const int n = 2001;
        len.resize(n);
        Point prev = bezier(c, 0.0);
        len[0] = 0.0;
        for (int i = 1; i < n; ++i) {
            const Point q = bezier(c, double(i) / (n - 1));
            len[i] = len[i - 1] + std::hypot(q.x - prev.x, q.y - prev.y);
            prev = q;
        }
        total = len.back();
    }
    double paramAt(double frac) const {
        const double target = std::clamp(frac, 0.0, 1.0) * total;
        const auto it = std::lower_bound(len.begin(), len.end(), target);
        const int i = std::clamp(static_cast<int>(it - len.begin()), 1, static_cast<int>(len.size()) - 1);
        const double f = (target - len[i - 1]) / std::max(len[i] - len[i - 1], 1e-30);
        return (i - 1 + f) / (len.size() - 1.0);
    }
};

double plateShape(const PlateParams& p, int m, int n, double x, double y) {
    return std::sin(m * kPi * x / p.lengthX) * std::sin(n * kPi * y / p.widthY);
}

// Mean of the plate shape over the bridge contact around (x, y).
double bridgeShape(const PlateParams& p, int m, int n, Point at, Point tangent) {
    const int np = std::max(1, p.bridgePoints);
    double s = 0.0;
    for (int i = 0; i < np; ++i) {
        const double d = np == 1 ? 0.0 : p.bridgeHalfLength * (2.0 * i / (np - 1) - 1.0);
        s += plateShape(p, m, n, at.x + d * tangent.x, at.y + d * tangent.y);
    }
    return s / np;
}

// Uniform in [-1, 1), a fixed function of (m, n, seed).
double jitter(int m, int n, unsigned seed) {
    std::uint64_t x = (std::uint64_t(m) << 32) ^ (std::uint64_t(n) << 8) ^ (std::uint64_t(seed) * 0x9E3779B97F4A7C15ull);
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    x ^= x >> 31;
    return double(x >> 11) * (2.0 / 9007199254740992.0) - 1.0;
}

double directionalD(const PlateParams& p, double th) {
    const double c = std::cos(th), s = std::sin(th);
    return p.Dx * c * c * c * c + 2.0 * p.Dxy * c * c * s * s + p.Dy * s * s * s * s;
}

}  // namespace

double BoardModel::eta(double f) const noexcept {
    const PlateParams& p = params;
    const double e = p.etaLow + (p.etaHigh - p.etaLow) * std::pow(std::max(f, 0.0) / p.etaRefHz, p.etaExponent);
    const double x = f / p.etaBassHz;
    return std::min(e, p.etaHigh) + p.etaBass / (1.0 + x * x);
}

namespace {
// Mean over wave directions of |mean over the contact points of
// exp(j k s cos th)|^2, with the geometric-mean bending wavenumber.
double bridgeFactorRaw(const PlateParams& p, double f) {
    const int np = std::max(1, p.bridgePoints);
    if (np == 1 || p.bridgeHalfLength <= 0.0) return 1.0;
    const double w = 2.0 * kPi * f;
    const double k = std::pow(w * w * p.massPerArea / std::sqrt(p.Dx * p.Dy), 0.25);
    const double step = 2.0 * p.bridgeHalfLength / (np - 1);
    const int nt = 64;
    double acc = 0.0;
    for (int t = 0; t < nt; ++t) {
        const double x = 0.5 * k * std::cos(kPi * (t + 0.5) / nt) * step;
        const double den = np * std::sin(x);
        const double r = std::fabs(den) < 1e-12 ? 1.0 : std::sin(np * x) / den;
        acc += r * r;
    }
    return acc / nt;
}
constexpr double kTabLo = 10.0, kTabHi = 1e6;
constexpr int kTabN = 512;
}  // namespace

double BoardModel::bridgeFactor(double f) const noexcept {
    if (bridgeTab.empty()) return 1.0;
    const double pos = std::log(std::clamp(f, kTabLo, kTabHi) / kTabLo) / std::log(kTabHi / kTabLo) * (kTabN - 1);
    const int i = std::min(static_cast<int>(pos), kTabN - 2);
    const double t = pos - i;
    return bridgeTab[i] + t * (bridgeTab[i + 1] - bridgeTab[i]);
}

cd BoardModel::continuation(double f) const noexcept {
    if (contF.empty()) return 0.0;
    if (f <= contF.front()) return contY.front() * (f / contF.front());
    if (f >= contF.back()) return contY.back();
    const double lf = std::log(f);
    const double l0 = std::log(contF.front()), l1 = std::log(contF.back());
    const double pos = (lf - l0) / (l1 - l0) * (contF.size() - 1);
    const int i = std::min(static_cast<int>(pos), static_cast<int>(contF.size()) - 2);
    const double t = pos - i;
    return contY[i] + t * (contY[i + 1] - contY[i]);
}

BoardModel buildPlateBoard(const PlateParams& p) {
    BoardModel b;
    b.params = p;
    b.mass = p.massPerArea * p.lengthX * p.widthY;

    // Mode enumeration.
    std::vector<std::tuple<double, int, int>> list;
    const double a = p.lengthX, w = p.widthY;
    for (int m = 1; m < 400; ++m) {
        bool any = false;
        for (int n = 1; n < 400; ++n) {
            const double km = m / a, kn = n / w;
            const double f = 0.5 * kPi *
                             std::sqrt((p.Dx * km * km * km * km + 2.0 * p.Dxy * km * km * kn * kn +
                                        p.Dy * kn * kn * kn * kn) / p.massPerArea);
            if (f >= p.maxFreqHz) break;
            list.emplace_back(f * (1.0 + p.freqJitter * jitter(m, n, p.jitterSeed)), m, n);
            any = true;
        }
        if (!any) break;
    }
    std::sort(list.begin(), list.end());
    while (!list.empty() && std::get<0>(list.back()) >= p.maxFreqHz) list.pop_back();
    if (static_cast<int>(list.size()) > p.maxModes) list.resize(static_cast<std::size_t>(p.maxModes));
    b.modes = static_cast<int>(list.size());
    const double norm = 1.0 / std::sqrt(0.25 * b.mass);
    for (const auto& [f, m, n] : list) {
        b.freq.push_back(f);
        b.mIndex.push_back(m);
        b.nIndex.push_back(n);
    }
    for (int i = 0; i < b.modes; ++i) b.sigma.push_back(b.eta(b.freq[i]) * kPi * b.freq[i]);

    // Bridges, keys, zones.
    b.keyShape.assign(std::size_t(kNumKeys) * b.modes, 0.0);
    int zoneBase = 0;
    for (const BridgeCurve* c : {&p.bass, &p.treble}) {
        const Arc arc(*c);
        const int nk = c->lastKey - c->firstKey + 1;
        const int nz = std::max(1, c->zones);
        for (int z = 0; z < nz; ++z) {
            const double frac = nz == 1 ? 0.5 : double(z) / (nz - 1);
            const double t = arc.paramAt(frac);
            const Point q = bezier(*c, t), tg = bezierTangent(*c, t);
            b.zoneX.push_back(q.x);
            b.zoneY.push_back(q.y);
            for (int i = 0; i < b.modes; ++i)
                b.zoneShape.push_back(norm * bridgeShape(p, b.mIndex[i], b.nIndex[i], q, tg));
        }
        for (int j = 0; j < nk; ++j) {
            const int key = c->firstKey + j;
            const double frac = nk == 1 ? 0.5 : double(j) / (nk - 1);
            const double t = arc.paramAt(frac);
            const Point q = bezier(*c, t), tg = bezierTangent(*c, t);
            b.keyX[key - 1] = q.x;
            b.keyY[key - 1] = q.y;
            double* row = b.keyShape.data() + std::size_t(key - 1) * b.modes;
            for (int i = 0; i < b.modes; ++i) row[i] = norm * bridgeShape(p, b.mIndex[i], b.nIndex[i], q, tg);
            KeyZone& kz = b.keyZone[key - 1];
            if (nz == 1) {
                kz = KeyZone{zoneBase, 1.0f, 0.0f, 1.0f};
            } else {
                const double pos = frac * (nz - 1);
                const int z0 = std::min(static_cast<int>(pos), nz - 2);
                const double f1 = pos - z0;
                kz = KeyZone{zoneBase + z0, static_cast<float>(1.0 - f1), static_cast<float>(f1), 1.0f};
            }
        }
        zoneBase += nz;
    }
    b.zones = zoneBase;
    for (int k = 0; k < 2; ++k)
        for (int i = 0; i < b.modes; ++i)
            b.pickupShape.push_back(norm * plateShape(p, b.mIndex[i], b.nIndex[i], p.pickupX[k], p.pickupY[k]));

    // Asymptotic modal density: n = (A / (4 pi)) INT_0^2pi sqrt(rho_h / D(th)) dth.
    double integ = 0.0;
    const int nt = 720;
    for (int t = 0; t < nt; ++t) integ += std::sqrt(p.massPerArea / directionalD(p, 2.0 * kPi * (t + 0.5) / nt));
    integ *= 2.0 * kPi / nt;
    b.density = p.lengthX * p.widthY / (4.0 * kPi) * integ;
    b.meanConductance = b.density / (4.0 * b.mass);
    for (int i = 0; i < kTabN; ++i)
        b.bridgeTab.push_back(bridgeFactorRaw(p, kTabLo * std::pow(kTabHi / kTabLo, double(i) / (kTabN - 1))));

    // Continuation table (trapezoid in f', step proportional to the
    // resonance width, up to 16 times the evaluation frequency).
    const int ng = std::max(8, p.continuationPoints);
    const double fLo = 10.0, fHi = 40000.0;
    for (int g = 0; g < ng; ++g) {
        const double f = fLo * std::pow(fHi / fLo, double(g) / (ng - 1));
        const double wv = 2.0 * kPi * f;
        const double top = 8.0 * std::max(f, p.maxFreqHz);
        cd acc = 0.0;
        double fp = p.maxFreqHz;
        auto integrand = [&](double x) {
            const double wx = 2.0 * kPi * x;
            return b.bridgeFactor(x) / (wx * wx - wv * wv + cd(0.0, b.eta(x) * wx * wv));
        };
        cd prev = integrand(fp);
        while (fp < top) {
            const double h = 0.05 * b.eta(fp) * fp;
            const double nx = std::min(fp + h, top);
            const cd cur = integrand(nx);
            acc += 0.5 * (prev + cur) * (nx - fp);
            prev = cur;
            fp = nx;
        }
        b.contF.push_back(f);
        b.contY.push_back(cd(0.0, wv) * (b.density / b.mass) * acc);
    }
    return b;
}

const BoardModel& defaultBoard() {
    static const BoardModel board = buildPlateBoard(PlateParams{});
    return board;
}

cd boardModalAdmittance(const BoardModel& b, const double* sa, const double* sb, double f) noexcept {
    const double w = 2.0 * kPi * f;
    cd y = 0.0;
    for (int i = 0; i < b.modes; ++i) {
        const double wm = 2.0 * kPi * b.freq[i];
        y += sa[i] * sb[i] / cd(wm * wm - w * w, 2.0 * b.sigma[i] * w);
    }
    return cd(0.0, w) * y;
}

cd boardModalAdmittanceS(const BoardModel& b, const double* sa, const double* sb, cd s) noexcept {
    cd y = 0.0;
    for (int i = 0; i < b.modes; ++i) {
        const double wm = 2.0 * kPi * b.freq[i];
        y += sa[i] * sb[i] / (wm * wm + s * s + 2.0 * b.sigma[i] * s);
    }
    return s * y;
}

cd boardDrivingPointS(const BoardModel& b, int key, cd s) noexcept {
    const double* sh = b.shapeOfKey(key);
    return boardModalAdmittanceS(b, sh, sh, s) + b.continuation(s.imag() / (2.0 * kPi));
}

cd boardDrivingPoint(const BoardModel& b, int key, double f) noexcept {
    const double* s = b.shapeOfKey(key);
    return boardModalAdmittance(b, s, s, f) + b.continuation(f);
}

void keyLoopShape(const BoardModel& b, int key, double* psi) noexcept {
    const KeyZone& kz = b.keyZone[key - 1];
    const double* s0 = b.shapeOfZone(kz.z0);
    const double* s1 = kz.w1 != 0.0f ? b.shapeOfZone(kz.z0 + 1) : s0;
    for (int i = 0; i < b.modes; ++i) psi[i] = double(kz.w0) * s0[i] + double(kz.w1) * s1[i];
}

// The board mode runs p <- c p + b D (Soundboard::setMode); the zone reads
// Im p after the update and the strings receive it one sample later. The
// strings' force is read from their state after their own update; for a
// string mode at frequency f this loop acts like the continuous admittance
// fs (z - 1) L(z), L = z^-1 H, H(z) = (b z / (z - c) - conj(b) z / (z - conj c)) / 2j.
cd loopModeAdmittance(double freqHz, double sigma, double f, double fs) noexcept {
    const double w = 2.0 * kPi * freqHz;
    const double wd = std::sqrt(std::max(w * w - sigma * sigma, 1e-6 * w * w));
    const cd lambda(-sigma, wd);
    const cd c = std::exp(lambda / fs);
    const cd bb = (c - 1.0) / (lambda * wd);
    const cd z = std::polar(1.0, 2.0 * kPi * f / fs);
    const cd H = (bb * z / (z - c) - std::conj(bb) * z / (z - std::conj(c))) / cd(0.0, 2.0);
    return fs * (z - 1.0) * H / z;
}

cd loopModeAdmittanceS(double freqHz, double sigma, cd s, double fs) noexcept {
    const double w = 2.0 * kPi * freqHz;
    const double wd = std::sqrt(std::max(w * w - sigma * sigma, 1e-6 * w * w));
    const cd lambda(-sigma, wd);
    const cd c = std::exp(lambda / fs);
    const cd bb = (c - 1.0) / (lambda * wd);
    const cd z = std::exp(s / fs);
    const cd H = (bb * z / (z - c) - std::conj(bb) * z / (z - std::conj(c))) / cd(0.0, 2.0);
    return fs * (z - 1.0) * H / z;
}

cd loopSelfAdmittanceS(const BoardModel& b, int key, cd s, double fs, double g, cd D) noexcept {
    std::vector<double> psi(static_cast<std::size_t>(b.modes));
    keyLoopShape(b, key, psi.data());
    return loopSelfAdmittanceS(b, psi.data(), s, fs, g, D);
}

cd loopSelfAdmittanceS(const BoardModel& b, const double* psi, cd s, double fs, double g, cd D) noexcept {
    cd y = 0.0;
    for (int i = 0; i < b.modes; ++i) y += psi[i] * psi[i] * loopModeAdmittanceS(b.freq[i], b.sigma[i], s, fs);
    const cd z = std::exp(s / fs);
    const cd L = y / (fs * (z - 1.0));
    return fs * (z - 1.0) * g * L / (1.0 - g * L * D);
}

cd loopSelfAdmittance(const BoardModel& b, int key, double f, double fs) noexcept {
    std::vector<double> psi(static_cast<std::size_t>(b.modes));
    keyLoopShape(b, key, psi.data());
    cd y = 0.0;
    for (int i = 0; i < b.modes; ++i) y += psi[i] * psi[i] * loopModeAdmittance(b.freq[i], b.sigma[i], f, fs);
    return y;
}

cd loopSelfAdmittance(const BoardModel& b, int key, double f, double fs, double g, double K) noexcept {
    const cd z = std::polar(1.0, 2.0 * kPi * f / fs);
    const cd y = loopSelfAdmittance(b, key, f, fs);  // fs (z - 1) L
    const cd L = y / (fs * (z - 1.0));
    return fs * (z - 1.0) * g * L / (1.0 - g * L * K);
}

double stringStaticStiffness(const KeyParams& k, double fs, double cutoffHz, double r) noexcept {
    return stringResidualStiffness(k, fs, cutoffHz, r, 0.0);
}

double stringLoadStiffness(const KeyParams& k, double fs, double cutoffHz, double r, double f) noexcept {
    const double limit = std::min({cutoffHz, k.maxFreqHz, 0.45 * fs});
    const double m = 0.5 * k.linearDensity * k.length;
    const int S = std::clamp(k.strings, 1, 3);
    const double w = 2.0 * kPi * f;
    double D = 0.0;
    for (int n = 1; n <= k.maxPartials; ++n) {
        const double fn = n * k.f0 * std::sqrt(1.0 + k.B * n * n);
        if (fn >= limit) break;
        if (std::fabs(fn - f) < 0.02 * f) continue;  // resonant: sympathetic coupling, not loading
        const double o = k.tension * n * kPi / k.length * (1.0 + k.B * n * n);
        const double wn = 2.0 * kPi * fn;
        D += S * (1.0 + r * r) * o * o / (m * wn * wn) * w * w / (wn * wn - w * w);
    }
    return D;
}

cd stringResidualStiffnessS(const KeyParams& k, double fs, double cutoffHz, double r, cd s) noexcept {
    const double limit = std::min({cutoffHz, k.maxFreqHz, 0.45 * fs});
    const double m = 0.5 * k.linearDensity * k.length;
    const int S = std::clamp(k.strings, 1, 3);
    const double f = s.imag() / (2.0 * kPi);
    cd D = 0.0;
    double best = 1e300;
    int own = 0;
    for (int n = 1; n <= k.maxPartials; ++n) {
        const double fn = n * k.f0 * std::sqrt(1.0 + k.B * n * n);
        if (fn >= limit) break;
        if (std::fabs(fn - f) < best) {
            best = std::fabs(fn - f);
            own = n;
        }
    }
    for (int n = 1; n <= k.maxPartials; ++n) {
        const double fn = n * k.f0 * std::sqrt(1.0 + k.B * n * n);
        if (fn >= limit) break;
        const double o = k.tension * n * kPi / k.length * (1.0 + k.B * n * n);
        const double wn = 2.0 * kPi * fn;
        const double Kn = S * (1.0 + r * r) * o * o / (m * wn * wn);
        D += n == own ? cd(-Kn) : Kn * (-s * s) / (wn * wn + s * s);
    }
    return D;
}

double stringResidualStiffness(const KeyParams& k, double fs, double cutoffHz, double r, double f) noexcept {
    const double limit = std::min({cutoffHz, k.maxFreqHz, 0.45 * fs});
    const double m = 0.5 * k.linearDensity * k.length;
    const int S = std::clamp(k.strings, 1, 3);
    const double w = 2.0 * kPi * f;
    double D = 0.0, best = 1e300;
    int own = 0;
    for (int n = 1; n <= k.maxPartials; ++n) {
        const double fn = n * k.f0 * std::sqrt(1.0 + k.B * n * n);
        if (fn >= limit) break;
        if (std::fabs(fn - f) < best) {
            best = std::fabs(fn - f);
            own = n;
        }
    }
    for (int n = 1; n <= k.maxPartials; ++n) {
        const double fn = n * k.f0 * std::sqrt(1.0 + k.B * n * n);
        if (fn >= limit) break;
        const double o = k.tension * n * kPi / k.length * (1.0 + k.B * n * n);
        const double wn = 2.0 * kPi * fn;
        const double Kn = S * (1.0 + r * r) * o * o / (m * wn * wn);
        if (f <= 0.0) D += Kn;
        else D += n == own ? -Kn : Kn * w * w / (wn * wn - w * w);
    }
    return D;
}

SoundboardBridge::SoundboardBridge(const BoardModel& board, double fs, double loopGain, const PianoTable* table,
                                   double loopCutoffHz)
    : board_(board), fs_(fs), g_(std::max(loopGain, 0.0)),
      r_(board.params.cross / std::sqrt(board.params.horizontalRatio)),
      cutoff_(loopCutoffHz > 0.0 ? std::min(loopCutoffHz, board.params.maxFreqHz) : board.params.maxFreqHz) {
    if (table && g_ > 0.0)
        for (int k = 0; k < kNumKeys; ++k) keys_[k] = table->keys[k];
    hasKeys_ = table != nullptr && g_ > 0.0;
    psi_.resize(std::size_t(kNumKeys) * board_.modes);
    for (int k = 1; k <= kNumKeys; ++k) keyLoopShape(board_, k, psi_.data() + std::size_t(k - 1) * board_.modes);
}

cd SoundboardBridge::baked(int key, double f) const noexcept {
    cd y = boardDrivingPoint(board_, key, f);
    if (g_ > 0.0 && f < cutoff_) {
        const KeyParams& kp = keys_[key - 1];
        const double D = hasKeys_ && kp.bridgeCoupling ? stringResidualStiffness(kp, fs_, cutoff_, r_, f) : 0.0;
        y -= loopSelfAdmittance(board_, key, f, fs_, g_, D);
        // Interpolated loop shapes can exceed the true driving point between
        // zones; the bake must stay passive on its own.
        if (y.real() < 0.0) y = cd(0.0, y.imag());
    }
    return y;
}

// First-order loading of the board by the other keys' strings (their
// non-resonant end stiffness D_k'), as the run-time loop produces it:
// L_kk += g sum_k' D_k' (b_k'^T Z b_k)^2, Z the zone-port loop transfer.
cd SoundboardBridge::loadingShift(int key, double f) const noexcept {
    if (!hasKeys_ || f >= cutoff_) return 0.0;
    const int M = board_.modes, Z = board_.zones;
    std::vector<double> psi(static_cast<std::size_t>(M));
    keyLoopShape(board_, key, psi.data());
    const cd z = std::polar(1.0, 2.0 * kPi * f / fs_);
    std::vector<cd> h(static_cast<std::size_t>(M));
    for (int i = 0; i < M; ++i)
        h[i] = psi[i] * loopModeAdmittance(board_.freq[i], board_.sigma[i], f, fs_) / (fs_ * (z - 1.0));
    std::vector<cd> v(static_cast<std::size_t>(Z));  // displacement at each zone per newton at key
    for (int q = 0; q < Z; ++q) {
        const double* sh = board_.shapeOfZone(q);
        cd acc = 0.0;
        for (int i = 0; i < M; ++i) acc += sh[i] * h[i];
        v[q] = acc;
    }
    cd dL = 0.0;
    for (int k = 1; k <= kNumKeys; ++k) {
        if (k == key || !keys_[k - 1].bridgeCoupling) continue;
        const KeyZone& kz = board_.keyZone[k - 1];
        const cd t = double(kz.w0) * v[kz.z0] + (kz.w1 != 0.0f ? double(kz.w1) * v[kz.z0 + 1] : cd(0.0));
        dL += stringLoadStiffness(keys_[k - 1], fs_, cutoff_, r_, f) * t * t;
    }
    return fs_ * (z - 1.0) * g_ * g_ * dL;
}

cd SoundboardBridge::bakedS(int key, cd s) const noexcept {
    cd y = boardDrivingPointS(board_, key, s);
    if (g_ > 0.0 && s.imag() / (2.0 * kPi) < cutoff_) {
        const KeyParams& kp = keys_[key - 1];
        const cd D = hasKeys_ && kp.bridgeCoupling ? stringResidualStiffnessS(kp, fs_, cutoff_, r_, s) : cd(0.0);
        y -= loopSelfAdmittanceS(board_, psi_.data() + std::size_t(key - 1) * board_.modes, s, fs_, g_, D);
        if (y.real() < 0.0) y = cd(0.0, y.imag());
    }
    return y;
}

Admittance2 SoundboardBridge::atLambda(int key, cd lambda) const noexcept {
    const double f = lambda.imag() / (2.0 * kPi);
    const cd yb = bakedS(key, lambda);
    const double yh = board_.meanConductance * board_.bridgeFactor(f) / board_.params.horizontalRatio;
    return {yb, r_ * yb, r_ * r_ * yb + yh};
}

Admittance2 SoundboardBridge::totalLambda(int key, cd lambda) const noexcept {
    const double f = lambda.imag() / (2.0 * kPi);
    const cd yb = boardDrivingPointS(board_, key, lambda) + loadingShift(key, f);
    const double yh = board_.meanConductance * board_.bridgeFactor(f) / board_.params.horizontalRatio;
    return {yb, r_ * yb, r_ * r_ * yb + yh};
}

Admittance2 SoundboardBridge::total(int key, double f) const noexcept {
    const cd yb = boardDrivingPoint(board_, key, f) + loadingShift(key, f);
    const double yh = board_.meanConductance * board_.bridgeFactor(f) / board_.params.horizontalRatio;
    return {yb, r_ * yb, r_ * r_ * yb + yh};
}

Admittance2 SoundboardBridge::at(int key, double f) const noexcept {
    const cd yb = baked(key, f);
    const double yh = board_.meanConductance * board_.bridgeFactor(f) / board_.params.horizontalRatio;
    Admittance2 y;
    y.vv = yb;
    y.vh = r_ * yb;
    y.hh = r_ * r_ * yb + yh;
    return y;
}

// Board modes of the loop near f, strongest first by their admittance at f.
// Continuous-time approximation of each mode's loop term:
// g psi^2 s / (s^2 + 2 sigma s + w^2), positive-frequency part a / (s - p),
// p = -sigma + j w_d, a = g psi^2 p / (2 j w_d).
int SoundboardBridge::loopPoles(int key, double f, cd* pole, cd* residue, int max) const noexcept {
    if (g_ <= 0.0 || f >= cutoff_ || max <= 0) return 0;
    const double* psi = psi_.data() + std::size_t(key - 1) * board_.modes;
    struct Cand {
        double weight;
        int i;
    };
    std::vector<Cand> c;
    const double w = 2.0 * kPi * f;
    for (int i = 0; i < board_.modes; ++i) {
        if (std::fabs(board_.freq[i] - f) > 0.25 * f) continue;
        const double wi = 2.0 * kPi * board_.freq[i];
        c.push_back({g_ * psi[i] * psi[i] / (std::fabs(wi - w) + board_.sigma[i]), i});
    }
    std::sort(c.begin(), c.end(), [](const Cand& a, const Cand& b) { return a.weight > b.weight; });
    const int n = std::min(max, static_cast<int>(c.size()));
    for (int q = 0; q < n; ++q) {
        const int i = c[q].i;
        const double wm = 2.0 * kPi * board_.freq[i], sg = board_.sigma[i];
        const double wd = std::sqrt(std::max(wm * wm - sg * sg, 1e-6 * wm * wm));
        pole[q] = cd(-sg, wd);
        residue[q] = g_ * psi[i] * psi[i] * pole[q] / cd(0.0, 2.0 * wd);
    }
    return n;
}

Admittance2 SoundboardBridge::loopLambda(int key, cd lambda) const noexcept {
    if (g_ <= 0.0 || lambda.imag() / (2.0 * kPi) >= cutoff_) return {};
    const KeyParams& kp = keys_[key - 1];
    const cd D = hasKeys_ && kp.bridgeCoupling ? stringResidualStiffnessS(kp, fs_, cutoff_, r_, lambda) : cd(0.0);
    const cd yl = loopSelfAdmittanceS(board_, psi_.data() + std::size_t(key - 1) * board_.modes, lambda, fs_, g_, D);
    return {yl, r_ * yl, r_ * r_ * yl};
}

Admittance2 SoundboardBridge::loopLoading(int key, double f) const noexcept {
    const cd yl = loadingShift(key, f);
    return {yl, r_ * yl, r_ * r_ * yl};
}

bool loadSoundboard(Soundboard& sb, const BoardModel& b, double fs) {
    if (!sb.allocate(b.modes, b.zones, 2)) return false;
    for (int i = 0; i < b.modes; ++i) {
        sb.setMode(i, b.freq[i], b.sigma[i], fs);
        for (int z = 0; z < b.zones; ++z) sb.setZoneShape(z, i, b.shapeOfZone(z)[i]);
        for (int k = 0; k < 2; ++k) sb.setPickupShape(k, i, b.pickupShape[std::size_t(k) * b.modes + i]);
    }
    // Displacements below 1e-15 m reach no string audibly; velocities below
    // 1e-11 m/s are far below the output's noise floor.
    sb.setSleepThresholds(1e-15, 1e-11);
    return true;
}

}  // namespace pm
