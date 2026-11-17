#include "pm_room/room.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace pm {

namespace {

constexpr double kPi = 3.14159265358979323846;
// (x + c) - c is exactly x for |x| above about 1e-12 and exactly 0 for |x|
// below about 1e-27: a branchless flush that keeps the recursive states out
// of the denormal range without relying on FTZ/DAZ.
constexpr float kFlush = 1e-20f;
constexpr float kSilence = 1e-7f;  // -140 dBFS
constexpr double kMidHz = 707.0;   // geometric centre of the 500 Hz and 1 kHz octaves
constexpr double kHfHz = 8000.0;
constexpr uint32_t kLinePad = 16;  // floats (one cache line)

uint32_t nextPow2(uint32_t v) {
    uint32_t p = 1;
    while (p < v) p <<= 1;
    return p;
}

int nextPrime(int v) {
    for (int n = std::max(v, 2);; ++n) {
        bool prime = true;
        for (int d = 2; d * d <= n; ++d)
            if (n % d == 0) { prime = false; break; }
        if (prime) return n;
    }
}

// 16-point fast Walsh-Hadamard transform (unnormalised; H H^T = 16 I).
inline void fwht16(float* x) noexcept {
    for (int h = 1; h < 16; h <<= 1)
        for (int i = 0; i < 16; i += 2 * h)
            for (int j = i; j < i + h; ++j) {
                const float a = x[j], b = x[j + h];
                x[j] = a + b;
                x[j + h] = a - b;
            }
}

// Line input and output sign patterns. Left drives the even lines, right the
// odd ones; the outputs use two orthogonal +-1 vectors, so with mutually
// uncorrelated line signals the expected left/right correlation is zero.
constexpr float kInSign[16] = {1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1};
constexpr float kOutL[16] = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
constexpr float kOutR[16] = {1, 1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1};

}  // namespace

// ---------------------------------------------------------------------------
// Geometry. The room is scaled about its corner (x = y = z = 0): dimensions
// and the piano's position scale by `scale`, the microphones keep their
// offset from the piano (the pair stays where the engineer put it).
//
// Early reflections: shoebox image sources. Per axis the images are
//   x' = 2 q L + s      (q != 0: 2|q| reflections)
//   x' = 2 q L - s      (|q - 1| + |q| reflections)
// and an image's amplitude relative to the direct sound is
//   (d_direct / d_image) * prod_axes sqrt(1 - alpha_axis)^reflections.
// All images of total order 1..3 (62 of them) are ranked by arrival time and
// the earliest kTaps are kept per microphone. The direct sound itself is not
// produced: the dry input already is the microphone signal.
// ---------------------------------------------------------------------------
void Room::derive(double scale, Derived& d) const noexcept {
    double L[3], src[3], beta[3];
    for (int a = 0; a < 3; ++a) {
        L[a] = geom_.dims[a] * scale;
        src[a] = geom_.source[a] * scale;
        beta[a] = std::sqrt(std::max(0.0, 1.0 - geom_.absorption[a]));
    }
    // Per-axis image coordinates of order <= 3 (7 per axis).
    struct AxisImg {
        double c;
        int order;
    };
    double distSum = 0.0;
    for (int ch = 0; ch < 2; ++ch) {
        double mic[3];
        for (int a = 0; a < 3; ++a)
            mic[a] = std::clamp(src[a] + geom_.mic[ch][a] - geom_.source[a], 0.05, L[a] - 0.05);
        AxisImg ax[3][10];
        int nAx[3] = {0, 0, 0};
        for (int a = 0; a < 3; ++a)
            for (int q = -2; q <= 2; ++q)
                for (int p = 0; p < 2; ++p) {
                    const int order = p ? std::abs(q - 1) + std::abs(q) : 2 * std::abs(q);
                    if (order > 3) continue;
                    ax[a][nAx[a]++] = {2.0 * q * L[a] + (p ? -src[a] : src[a]), order};
                }
        const double d0 = std::sqrt((src[0] - mic[0]) * (src[0] - mic[0]) + (src[1] - mic[1]) * (src[1] - mic[1]) +
                                    (src[2] - mic[2]) * (src[2] - mic[2]));
        distSum += d0;
        struct Cand {
            double dist, gain;
        };
        Cand cand[64];
        int nc = 0;
        for (int i = 0; i < nAx[0]; ++i)
            for (int j = 0; j < nAx[1]; ++j)
                for (int k = 0; k < nAx[2]; ++k) {
                    const int order = ax[0][i].order + ax[1][j].order + ax[2][k].order;
                    if (order < 1 || order > 3 || nc == 64) continue;
                    const double dx = ax[0][i].c - mic[0], dy = ax[1][j].c - mic[1], dz = ax[2][k].c - mic[2];
                    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double g = (d0 / dist) * std::pow(beta[0], ax[0][i].order) *
                                     std::pow(beta[1], ax[1][j].order) * std::pow(beta[2], ax[2][k].order);
                    cand[nc++] = {dist, g};
                }
        // Insertion sort by arrival (62 entries, no allocation).
        for (int i = 1; i < nc; ++i) {
            const Cand c = cand[i];
            int j = i - 1;
            while (j >= 0 && cand[j].dist > c.dist) {
                cand[j + 1] = cand[j];
                --j;
            }
            cand[j + 1] = c;
        }
        for (int t = 0; t < kTaps; ++t) {
            const Cand& c = cand[std::min(t, nc - 1)];
            const int delay = int(std::lround((c.dist - d0) / kSpeedOfSound * fs_));
            d.taps[ch][t] = {std::max(1, delay), t < nc ? float(c.gain) : 0.0f};
        }
    }
    d.micDist = 0.5 * distSum;

    // Late field. Line lengths spread geometrically over 1x..4x the mean free
    // path 4V/S (about 20..80 ms for the default hall at size 1), each rounded
    // up to a prime so that the lengths are mutually prime.
    const double V = L[0] * L[1] * L[2];
    const double S = 2.0 * (L[0] * L[1] + L[0] * L[2] + L[1] * L[2]);
    d.volume = V;
    const double mfp = 4.0 * V / S / kSpeedOfSound * fs_;
    int prev = 0;
    for (int i = 0; i < kLines; ++i) {
        const int target = int(std::lround(mfp * std::pow(4.0, i / double(kLines - 1))));
        d.len[i] = nextPrime(std::max(target, prev + 1));
        prev = d.len[i];
    }
    // The late field starts at about the mixing time t_mix = sqrt(V) ms
    // (V in m^3): the line input is delayed so that the first line output
    // arrives then.
    const double tmix = std::sqrt(V) * 1e-3 * fs_;
    d.pre = std::max(0, int(std::lround(tmix)) - d.len[0]);
    d.lateStart = (d.pre + d.len[0]) / fs_;
}

void Room::applySize(double scale) noexcept {
    derive(scale, scratch_);
    std::memcpy(taps_, scratch_.taps, sizeof(taps_));
    std::memcpy(len_, scratch_.len, sizeof(len_));
    for (int i = 0; i < kLines; ++i) len_[i] = std::min<int>(len_[i], int(fdnMask_));
    preDelay_ = std::min<int>(scratch_.pre, int(inMask_));
    volume_ = scratch_.volume;
    micDist_ = scratch_.micDist;
    lateStart_ = scratch_.lateStart;
    size_ = scale;
}

// Per-line absorption filters. Jot's design (Jot & Chaigne 1991): a line of
// length L samples gets the attenuation k = 10^(-3 L / (fs T60)) and the
// one-pole H(z) = k (1 - b) / (1 - b z^-1) with
//   b = (ln 10 / 4) log10(k) (1 - 1 / alpha^2),   alpha = T60(Nyquist) / T60(0).
// Here the same filter is used, but its pole is solved exactly so that |H|
// meets 10^(-3 L / (fs T60)) at 707 Hz and 10^(-3 L / (fs hfRatio T60)) at
// 8 kHz: with r = |H(w1)| / |H(wm)|,
//   (1 - r^2) b^2 - 2 (cos wm - r^2 cos w1) b + (1 - r^2) = 0
// (the root in [0, 1)), and c = |H(wm)| sqrt(1 - 2 b cos wm + b^2) / (1 - b).
void Room::computeLineCoefficients() noexcept {
    const double T = std::exp(logT_);
    const double wm = 2.0 * kPi * kMidHz / fs_;
    const double w1 = 2.0 * kPi * std::min(kHfHz, 0.45 * fs_) / fs_;
    double g2 = 0.0;
    for (int i = 0; i < kLines; ++i) {
        const double gm = std::pow(10.0, -3.0 * len_[i] / (fs_ * T));
        const double g1 = std::pow(10.0, -3.0 * len_[i] / (fs_ * T * hf_));
        const double r2 = (g1 / gm) * (g1 / gm);
        const double a = 1.0 - r2, bq = std::cos(wm) - r2 * std::cos(w1);
        double p = 0.0;
        if (a > 1e-12) p = (bq - std::sqrt(std::max(0.0, bq * bq - a * a))) / a;
        p = std::clamp(p, 0.0, 0.99);
        // The DC gain c grows with the pole. For long lines with a short T60
        // and a small hfRatio the exact solution would put T60(DC) far above
        // T60(mid) (a booming low-frequency tail); cap T60(DC) at 2 T60(mid),
        // i.e. c <= sqrt(gm), by lowering the pole (bisection, c increases
        // with p). The 8 kHz target is then not met for those lines.
        auto dcGain = [&](double q) { return gm * std::sqrt(1.0 - 2.0 * q * std::cos(wm) + q * q) / (1.0 - q); };
        const double cap = std::sqrt(gm);
        if (dcGain(p) > cap) {
            double lo = 0.0, hi = p;
            for (int it = 0; it < 30; ++it) {
                const double mid = 0.5 * (lo + hi);
                (dcGain(mid) > cap ? hi : lo) = mid;
            }
            p = lo;
        }
        const double c = std::min(0.99999, dcGain(p));
        b0_[i] = float(0.25 * c * (1.0 - p));
        pole_[i] = float(p);
        g2 += gm * gm;
    }
    meanG2_ = g2 / kLines;
}

// Wet gains. Level 0 dB reproduces the diffuse-field reverberant-to-direct
// energy ratio at the microphones, (r / r_c)^2 with the critical distance
// r_c = 0.057 sqrt(V / T60) (V in m^3, T60 in s). The early taps already
// carry their physical 1/r gains; the late field supplies the reverberant
// energy arriving after lateStart, (r / r_c)^2 10^(-6 lateStart / T60).
// Its raw energy per unit impulse: 8 lines are fed, the loop is orthogonal
// (H / 4), so the line outputs carry 8 g^2 / (1 - g^2) in total (g^2 the
// mean per-pass energy gain at mid frequencies), and an output tap sums the
// 16 lines with +-1/4 (the 1/4 folded into b0): 0.5 g^2 / (1 - g^2).
void Room::computeGainTargets() noexcept {
    const double T = std::exp(logT_);
    const double rc2 = 0.057 * 0.057 * volume_ / T;
    const double eLate = micDist_ * micDist_ / rc2 * std::pow(10.0, -6.0 * lateStart_ / T);
    const double raw = 0.5 * meanG2_ / std::max(1e-9, 1.0 - meanG2_);
    const double lateNorm = std::sqrt(eLate / raw);
    const double earlyW = bal_ >= 1.0 ? 0.0 : std::cos(bal_ * kPi / 2) * std::sqrt(2.0);
    const double lateW = bal_ <= 0.0 ? 0.0 : std::sin(bal_ * kPi / 2) * std::sqrt(2.0);
    const double fade = double(fadeCnt_) / fadeBlocks_;
    gETarget_ = float(level_ * earlyW * fade);
    gLTarget_ = float(level_ * lateW * lateNorm * fade);
}

double Room::reverbToDirectDb() const noexcept {
    const double rc2 = 0.057 * 0.057 * volume_ / std::exp(logT_);
    return 10.0 * std::log10(micDist_ * micDist_ / rc2);
}

int Room::earlyTap(int ch, int k, float* gain) const noexcept {
    if (gain) *gain = taps_[ch][k].gain;
    return taps_[ch][k].delay;
}

// ---------------------------------------------------------------------------
bool Room::prepare(double sampleRate, int /*maxFrames*/) noexcept {
    // No scratch buffers: process() works sample by sample, any n is fine.
    fs_ = sampleRate;
    fadeBlocks_ = std::max(1, int(std::lround(0.02 * fs_ / kControlBlock)));
    alpha_ = 1.0 - std::exp(-kControlBlock / (0.05 * fs_));  // 50 ms smoothing
    return configure(geom_);
}

bool Room::configure(const RoomGeometry& g) noexcept {
    geom_ = g;
    // Allocate for the extremes of the size range.
    uint32_t inNeed = 1, lineNeed = 1;
    for (double s : {kMinSize, kMaxSize}) {
        derive(s, scratch_);
        for (int ch = 0; ch < 2; ++ch)
            for (int t = 0; t < kTaps; ++t) inNeed = std::max<uint32_t>(inNeed, scratch_.taps[ch][t].delay + 1);
        inNeed = std::max<uint32_t>(inNeed, scratch_.pre + 1);
        lineNeed = std::max<uint32_t>(lineNeed, scratch_.len[kLines - 1] + 1);
    }
    const uint32_t inSize = nextPow2(inNeed), fdnSize = nextPow2(lineNeed);
    if (inBuf_[0].size() != inSize) {
        if (!inBuf_[0].allocate(inSize) || !inBuf_[1].allocate(inSize)) return prepared_ = false;
    }
    // Lines are spaced one power of two plus one cache line apart: the 16
    // writes of a sample share one index, and with a power-of-two spacing
    // they would all fall into the same L1 set.
    if (fdnSize_ != fdnSize || lines_.size() != std::size_t(fdnSize + kLinePad) * kLines) {
        if (!lines_.allocate(std::size_t(fdnSize + kLinePad) * kLines)) return prepared_ = false;
    }
    inMask_ = inSize - 1;
    fdnSize_ = fdnSize;
    fdnMask_ = fdnSize - 1;
    lpA_ = float(1.0 - std::exp(-2.0 * kPi * std::min(geom_.earlyHfCutoffHz, 0.45 * fs_) / fs_));
    size_ = -1.0;  // force applySize in snapAll
    prepared_ = true;
    reset();
    return true;
}

void Room::clearLate() noexcept {
    std::memset(lines_.data(), 0, lines_.size() * sizeof(float));
    std::memset(st_, 0, sizeof(st_));
    lp_[0] = lp_[1] = 0.0f;
}

void Room::reset() noexcept {
    if (!prepared_) return;
    std::memset(inBuf_[0].data(), 0, inBuf_[0].size() * sizeof(float));
    std::memset(inBuf_[1].data(), 0, inBuf_[1].size() * sizeof(float));
    clearLate();
    pos_ = 0;
    zeroRun_ = uint64_t(inMask_) + 1;
    quietRun_ = 0;
    blockPeak_ = 0.0f;
    silent_ = true;
    snapAll();
}

// Latch every parameter and jump to it (used while silent: nothing sounds,
// so nothing can click).
void Room::snapAll() noexcept {
    const double sz = std::clamp(double(aSize_.load(std::memory_order_relaxed)), kMinSize, kMaxSize);
    if (sz != size_) applySize(sz);
    logT_ = std::log(std::clamp(double(aDecay_.load(std::memory_order_relaxed)), 0.3, 6.0));
    hf_ = std::clamp(double(aHf_.load(std::memory_order_relaxed)), 0.1, 1.0);
    level_ = std::pow(10.0, double(aLevelDb_.load(std::memory_order_relaxed)) / 20.0);
    bal_ = std::clamp(double(aBal_.load(std::memory_order_relaxed)), 0.0, 1.0);
    enabledNow_ = aEnabled_.load(std::memory_order_relaxed);
    fadeCnt_ = fadeBlocks_;
    computeLineCoefficients();
    computeGainTargets();
    gE_ = gETarget_;
    gL_ = gLTarget_;
    dgE_ = dgL_ = 0.0f;
}

namespace {
// One-pole smoothing step at control rate that lands exactly on the target.
inline bool smooth(double& v, double target, double alpha, double snap) {
    if (v == target) return false;
    v += (target - v) * alpha;
    if (std::fabs(target - v) < snap) v = target;
    return true;
}
}  // namespace

// Runs at every absolute stream position that is a multiple of kControlBlock.
void Room::control() noexcept {
    if (silent_) {
        snapAll();
        return;
    }
    // The previous block's ramps end exactly on their targets.
    gE_ = gETarget_;
    gL_ = gLTarget_;

    // Idle: input exactly zero for the whole input history and nothing above
    // -140 dBFS written into any line for a whole line buffer.
    quietRun_ = blockPeak_ < kSilence ? quietRun_ + kControlBlock : 0;
    blockPeak_ = 0.0f;
    if (zeroRun_ > inMask_ && quietRun_ > fdnSize_) {
        clearLate();
        silent_ = true;
        snapAll();
        return;
    }

    const double sz = std::clamp(double(aSize_.load(std::memory_order_relaxed)), kMinSize, kMaxSize);
    const bool en = aEnabled_.load(std::memory_order_relaxed);
    enabledNow_ = en;
    bool fadeOut = !en || sz != size_;
    if (fadeOut && fadeCnt_ == 0) {
        // Faded out completely (the last ramp reached zero at this boundary).
        if (!en) {
            std::memset(inBuf_[0].data(), 0, inBuf_[0].size() * sizeof(float));
            std::memset(inBuf_[1].data(), 0, inBuf_[1].size() * sizeof(float));
            clearLate();
            zeroRun_ = uint64_t(inMask_) + 1;
            silent_ = true;
            snapAll();
            return;
        }
        // Size change: new taps and lengths, empty lines, fade back in. The
        // input history is kept, so the early taps resume at once.
        applySize(sz);
        clearLate();
        fadeOut = false;
    }
    fadeCnt_ = fadeOut ? std::max(0, fadeCnt_ - 1) : std::min(fadeBlocks_, fadeCnt_ + 1);

    const double tT = std::log(std::clamp(double(aDecay_.load(std::memory_order_relaxed)), 0.3, 6.0));
    const double hfT = std::clamp(double(aHf_.load(std::memory_order_relaxed)), 0.1, 1.0);
    const double levT = std::pow(10.0, double(aLevelDb_.load(std::memory_order_relaxed)) / 20.0);
    const double balT = std::clamp(double(aBal_.load(std::memory_order_relaxed)), 0.0, 1.0);
    bool coef = smooth(logT_, tT, alpha_, 1e-4);
    coef |= smooth(hf_, hfT, alpha_, 1e-4);
    smooth(level_, levT, alpha_, 1e-5 * levT);
    smooth(bal_, balT, alpha_, 1e-4);
    if (coef) computeLineCoefficients();
    computeGainTargets();
    dgE_ = (gETarget_ - gE_) / float(kControlBlock);
    dgL_ = (gLTarget_ - gL_) / float(kControlBlock);
}

void Room::processRun(const float* inL, const float* inR, float* outL, float* outR, int n) noexcept {
    if (silent_) {
        bool zero = true;
        if (enabledNow_)
            for (int i = 0; i < n; ++i) zero &= (inL[i] == 0.0f) & (inR[i] == 0.0f);
        if (zero) {
            std::memset(outL, 0, sizeof(float) * n);
            std::memset(outR, 0, sizeof(float) * n);
            if (enabledNow_) zeroRun_ += uint64_t(n);
            return;
        }
        silent_ = false;
        quietRun_ = 0;
    }

    float* const bufL = inBuf_[0].data();
    float* const bufR = inBuf_[1].data();
    float* const lines = lines_.data();
    const uint32_t im = inMask_, fm = fdnMask_, stride = fdnSize_ + kLinePad;
    const uint32_t pre = uint32_t(preDelay_);
    uint32_t iw = inW_, w = w_;
    float lpL = lp_[0], lpR = lp_[1], gE = gE_, gL = gL_, peak = blockPeak_;
    const float lpA = lpA_, dgE = dgE_, dgL = dgL_;
    uint64_t zr = zeroRun_;
    // Local copies: the buffer stores could alias the members, which would
    // force reloads inside the loop.
    int d0[kTaps], d1[kTaps];
    float t0[kTaps], t1[kTaps];
    for (int t = 0; t < kTaps; ++t) {
        d0[t] = taps_[0][t].delay, t0[t] = taps_[0][t].gain;
        d1[t] = taps_[1][t].delay, t1[t] = taps_[1][t].gain;
    }
    alignas(64) float st[kLines], b0[kLines], pole[kLines];
    uint32_t len[kLines];
    for (int i = 0; i < kLines; ++i) st[i] = st_[i], b0[i] = b0_[i], pole[i] = pole_[i], len[i] = uint32_t(len_[i]);

    for (int s = 0; s < n; ++s) {
        float xL = inL[s], xR = inR[s];
        xL = std::fabs(xL) < 1e-15f ? 0.0f : xL;
        xR = std::fabs(xR) < 1e-15f ? 0.0f : xR;
        zr = (xL == 0.0f && xR == 0.0f) ? zr + 1 : 0;
        bufL[iw] = xL;
        bufR[iw] = xR;

        // Early reflections. Each microphone's taps read that microphone's own
        // input: the tap delays are relative to the direct sound at the same
        // microphone, and the dry signal of that channel is exactly that
        // direct sound. A mono sum would comb-filter at the spaced pair's
        // inter-channel delay and collapse the image.
        // Four partial sums per channel: a single chain of 32 dependent adds
        // would bound the loop by add latency.
        float aL[4] = {0, 0, 0, 0}, aR[4] = {0, 0, 0, 0};
        for (int t = 0; t < kTaps; t += 4)
            for (int k = 0; k < 4; ++k) {
                aL[k] += t0[t + k] * bufL[(iw - uint32_t(d0[t + k])) & im];
                aR[k] += t1[t + k] * bufR[(iw - uint32_t(d1[t + k])) & im];
            }
        const float eL = (aL[0] + aL[1]) + (aL[2] + aL[3]), eR = (aR[0] + aR[1]) + (aR[2] + aR[3]);
        lpL += lpA * (eL - lpL);
        lpR += lpA * (eR - lpR);
        lpL = (lpL + kFlush) - kFlush;
        lpR = (lpR + kFlush) - kFlush;

        // Late field.
        const float uL = bufL[(iw - pre) & im], uR = bufR[(iw - pre) & im];
        alignas(64) float f[kLines];
        for (int i = 0; i < kLines; ++i) {
            const float y = lines[i * stride + ((w - len[i]) & fm)];
            float v = b0[i] * y + pole[i] * st[i];
            v = (v + kFlush) - kFlush;
            st[i] = v;
            f[i] = v;
        }
        float pL[4] = {0, 0, 0, 0}, pR[4] = {0, 0, 0, 0};
        for (int i = 0; i < kLines; i += 4)
            for (int k = 0; k < 4; ++k) {
                pL[k] += kOutL[i + k] * f[i + k];
                pR[k] += kOutR[i + k] * f[i + k];
            }
        const float lateL = (pL[0] + pL[1]) + (pL[2] + pL[3]), lateR = (pR[0] + pR[1]) + (pR[2] + pR[3]);
        fwht16(f);
        for (int i = 0; i < kLines; i += 2) {
            const float v0 = f[i] + kInSign[i] * uL, v1 = f[i + 1] + kInSign[i + 1] * uR;
            lines[i * stride + w] = v0;
            lines[(i + 1) * stride + w] = v1;
            peak = std::max(peak, std::max(std::fabs(v0), std::fabs(v1)));  // not fmax: that is a libcall without -ffinite-math-only
        }

        outL[s] = gE * lpL + gL * lateL;
        outR[s] = gE * lpR + gL * lateR;
        gE += dgE;
        gL += dgL;
        iw = (iw + 1) & im;
        w = (w + 1) & fm;
    }
    for (int i = 0; i < kLines; ++i) st_[i] = st[i];
    inW_ = iw;
    w_ = w;
    lp_[0] = lpL;
    lp_[1] = lpR;
    gE_ = gE;
    gL_ = gL;
    blockPeak_ = peak;
    zeroRun_ = zr;
}

void Room::process(const float* inL, const float* inR, float* outL, float* outR, int n) noexcept {
    if (!prepared_) {
        std::memset(outL, 0, sizeof(float) * std::max(0, n));
        std::memset(outR, 0, sizeof(float) * std::max(0, n));
        return;
    }
    int done = 0;
    while (done < n) {
        const int phase = int(pos_ % kControlBlock);
        if (phase == 0) control();
        const int run = std::min(n - done, kControlBlock - phase);
        processRun(inL + done, inR + done, outL + done, outR + done, run);
        pos_ += uint64_t(run);
        done += run;
    }
}

void Room::setDecay(double s) noexcept { aDecay_.store(float(s), std::memory_order_relaxed); }
void Room::setHfRatio(double r) noexcept { aHf_.store(float(r), std::memory_order_relaxed); }
void Room::setLevelDb(double db) noexcept { aLevelDb_.store(float(db), std::memory_order_relaxed); }
void Room::setEarlyLateBalance(double b) noexcept { aBal_.store(float(b), std::memory_order_relaxed); }
void Room::setEnabled(bool on) noexcept { aEnabled_.store(on, std::memory_order_relaxed); }
void Room::setSize(double scale) noexcept { aSize_.store(float(scale), std::memory_order_relaxed); }

}  // namespace pm
