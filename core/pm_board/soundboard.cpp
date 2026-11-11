#include "pm_board/soundboard.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

#include "pm_base/simd.h"

namespace pm {

namespace {

int padTo(int n, int w) { return (n + w - 1) / w * w; }

// Both kernels: same per-mode operation order as the string bank kernel.
// Output sums differ only in summation order (lanes vs sequential).
void stepScalar(int padded, int zones, int pickups, const float* F, float* zr, float* zi,
                const float* dr, const float* ci, const float* br, const float* bi,
                const float* shape, const float* pRe, const float* pIm, float* disp, float* vel,
                const int* outZones, int nOut) noexcept {
    for (int i = 0; i < padded; ++i) {
        float d = 0.0f;
        for (int z = 0; z < zones; ++z) d += shape[z * padded + i] * F[z];
        const float ar = zr[i] + br[i] * d;
        const float ai = zi[i] + bi[i] * d;
        const float nr = (ar + dr[i] * zr[i]) - ci[i] * zi[i];
        const float ni = (ai + dr[i] * zi[i]) + ci[i] * zr[i];
        zr[i] = nr;
        zi[i] = ni;
    }
    for (int o = 0; o < nOut; ++o) {
        const int z = outZones ? outZones[o] : o;
        float s = 0.0f;
        for (int i = 0; i < padded; ++i) s += shape[z * padded + i] * zi[i];
        disp[z] = s;
    }
    for (int p = 0; p < pickups; ++p) {
        float s = 0.0f;
        for (int i = 0; i < padded; ++i) s += pRe[p * padded + i] * zr[i] + pIm[p * padded + i] * zi[i];
        vel[p] = s;
    }
}

void stepSimd(int padded, int zones, int pickups, const float* F, float* zr, float* zi,
              const float* dr, const float* ci, const float* br, const float* bi, const float* shape,
              const float* pRe, const float* pIm, float* disp, float* vel, const int* outZones, int nOut) noexcept {
    using namespace simd;
    // Only zones that carry force this sample contribute to the drive.
    int active[Soundboard::kMaxZones];
    vf fz[Soundboard::kMaxZones];
    int na = 0;
    for (int z = 0; z < zones; ++z)
        if (F[z] != 0.0f) {
            active[na] = z;
            fz[na++] = set1(F[z]);
        }
    for (int i = 0; i < padded; i += kWidth) {
        vf d = zero();
        for (int a = 0; a < na; ++a) d = fmadd(load(shape + active[a] * padded + i), fz[a], d);
        const vf r = load(zr + i), m = load(zi + i), c = load(ci + i), e = load(dr + i);
        const vf ar = fmadd(load(br + i), d, r);
        const vf ai = fmadd(load(bi + i), d, m);
        store(zr + i, fnmadd(c, m, fmadd(e, r, ar)));
        store(zi + i, fmadd(c, r, fmadd(e, m, ai)));
    }
    for (int o = 0; o < nOut; ++o) {
        const int z = outZones ? outZones[o] : o;
        const float* row = shape + z * padded;
        vf a0 = zero(), a1 = zero();
        int i = 0;
        for (; i + 2 * kWidth <= padded; i += 2 * kWidth) {
            a0 = fmadd(load(row + i), load(zi + i), a0);
            a1 = fmadd(load(row + i + kWidth), load(zi + i + kWidth), a1);
        }
        for (; i < padded; i += kWidth) a0 = fmadd(load(row + i), load(zi + i), a0);
        disp[z] = hsum(add(a0, a1));
    }
    for (int p = 0; p < pickups; ++p) {
        const float* re = pRe + p * padded;
        const float* im = pIm + p * padded;
        vf a0 = zero(), a1 = zero();
        for (int i = 0; i < padded; i += kWidth) {
            a0 = fmadd(load(re + i), load(zr + i), a0);
            a1 = fmadd(load(im + i), load(zi + i), a1);
        }
        vel[p] = hsum(add(a0, a1));
    }
}

}  // namespace

bool Soundboard::allocate(int modes, int zones, int pickups) noexcept {
    if (modes < 0 || zones < 1 || zones > kMaxZones || pickups < 0 || pickups > kMaxPickups) return false;
    modes_ = modes;
    // A whole number of 2 SIMD vectors (and at least one), so every kernel
    // loop runs over full vectors; padding modes have zero shapes.
    padded_ = std::max(padTo(modes, 2 * simd::kWidth), 2 * simd::kWidth);
    zones_ = zones;
    pickups_ = pickups;
    const std::size_t n = static_cast<std::size_t>(padded_);
    const bool ok = pr_.allocate(n) && pi_.allocate(n) && dr_.allocate(n) && ci_.allocate(n) &&
                    br_.allocate(n) && bi_.allocate(n) && vRe_.allocate(n) && vIm_.allocate(n) &&
                    zoneShape_.allocate(n * zones) && pickRe_.allocate(n * std::max(pickups, 1)) &&
                    pickIm_.allocate(n * std::max(pickups, 1)) && scratch_.allocate(n) &&
                    omega2_.allocate(n);
    if (!ok) {
        modes_ = padded_ = zones_ = pickups_ = 0;
        return false;
    }
    maxZoneShape_ = maxPickShape_ = 0.0f;
    asleep_ = true;
    inputSeen_ = false;
    sinceCheck_ = 0;
    steps_ = 0;
    return true;
}

void Soundboard::setMode(int i, double freqHz, double sigma, double fs) noexcept {
    const double w = 2.0 * std::numbers::pi * freqHz;
    const double wd = std::sqrt(std::max(w * w - sigma * sigma, 1e-6 * w * w));
    const std::complex<double> lambda(-sigma, wd);
    const double theta = wd / fs;
    const double rm1 = std::expm1(-sigma / fs);
    const double half = std::sin(0.5 * theta);
    const double cRe1 = rm1 * std::cos(theta) - 2.0 * half * half;
    const double cIm = (1.0 + rm1) * std::sin(theta);
    const std::complex<double> b = std::complex<double>(cRe1, cIm) / (lambda * wd);
    dr_[i] = static_cast<float>(cRe1);
    ci_[i] = static_cast<float>(cIm);
    br_[i] = static_cast<float>(b.real());
    bi_[i] = static_cast<float>(b.imag());
    vRe_[i] = static_cast<float>(wd);
    vIm_[i] = static_cast<float>(-sigma);
    omega2_[i] = w * w;
}

void Soundboard::setZoneShape(int z, int i, double phi) noexcept {
    zoneShape_[static_cast<std::size_t>(z) * padded_ + i] = static_cast<float>(phi);
    maxZoneShape_ = std::max(maxZoneShape_, static_cast<float>(std::fabs(phi)));
}

// Call after setMode(i).
void Soundboard::setPickupShape(int p, int i, double phi) noexcept {
    const std::size_t k = static_cast<std::size_t>(p) * padded_ + i;
    pickRe_[k] = static_cast<float>(phi * vRe_[i]);
    pickIm_[k] = static_cast<float>(phi * vIm_[i]);
    maxPickShape_ = std::max(maxPickShape_, static_cast<float>(std::fabs(phi) * std::hypot(vRe_[i], vIm_[i])));
}

void Soundboard::setSleepThresholds(double displacement, double velocity) noexcept {
    dispThreshold_ = displacement;
    velThreshold_ = velocity;
}

void Soundboard::clearState() noexcept {
    for (int i = 0; i < padded_; ++i) pr_[i] = pi_[i] = 0.0f;
    asleep_ = true;
    inputSeen_ = false;
    sinceCheck_ = 0;
}

void Soundboard::scaleState(float g) noexcept {
    if (asleep_) return;
    for (int i = 0; i < padded_; ++i) {
        pr_[i] *= g;
        pi_[i] *= g;
    }
}

void Soundboard::setState(int i, float re, float im) noexcept {
    pr_[i] = re;
    pi_[i] = im;
    if (re != 0.0f || im != 0.0f) asleep_ = false;
}

double Soundboard::energy() const noexcept {
    double e = 0.0;
    for (int i = 0; i < modes_; ++i) {
        const double q = pi_[i];
        const double qd = double(vRe_[i]) * pr_[i] + double(vIm_[i]) * pi_[i];
        e += 0.5 * (qd * qd + omega2_[i] * q * q);
    }
    return e;
}

void Soundboard::sleepCheck() noexcept {
    sinceCheck_ = 0;
    if (inputSeen_) {
        inputSeen_ = false;
        return;
    }
    // Bounds on every output from the state: sum |p| times the largest row
    // entry (displacement), and times the largest velocity weight (pickups).
    double amp = 0.0;
    for (int i = 0; i < modes_; ++i) amp += std::sqrt(double(pr_[i]) * pr_[i] + double(pi_[i]) * pi_[i]);
    if (amp * maxZoneShape_ < dispThreshold_ && amp * maxPickShape_ < velThreshold_) {
        for (int i = 0; i < padded_; ++i) pr_[i] = pi_[i] = 0.0f;
        asleep_ = true;
    }
}

void Soundboard::step(const float* F, float* disp, float* vel, Kernel kernel, const int* outZones,
                      int nOut) noexcept {
    if (nOut < 0 || !outZones) {
        outZones = nullptr;
        nOut = zones_;
    }
    bool input = false;
    for (int z = 0; z < zones_; ++z) input = input || F[z] != 0.0f;
    if (asleep_ && !input) {
        for (int z = 0; z < zones_; ++z) disp[z] = 0.0f;
        for (int p = 0; p < pickups_; ++p) vel[p] = 0.0f;
        return;
    }
    asleep_ = false;
    inputSeen_ = inputSeen_ || input;
    if (kernel == Kernel::Simd)
        stepSimd(padded_, zones_, pickups_, F, pr_.data(), pi_.data(), dr_.data(), ci_.data(), br_.data(),
                 bi_.data(), zoneShape_.data(), pickRe_.data(), pickIm_.data(), disp, vel, outZones, nOut);
    else
        stepScalar(padded_, zones_, pickups_, F, pr_.data(), pi_.data(), dr_.data(), ci_.data(), br_.data(),
                   bi_.data(), zoneShape_.data(), pickRe_.data(), pickIm_.data(), disp, vel, outZones, nOut);
    ++steps_;
    if (++sinceCheck_ == kSleepInterval) sleepCheck();
}

// ---------------------------------------------------------------- CouplingLoop

namespace {
constexpr int kZ = Soundboard::kMaxZones;
}  // namespace

bool CouplingLoop::configure(const KeyZone* kz, int keys, int zones, int taps) noexcept {
    if (keys < 1 || keys > kMaxKeys || zones < 1 || zones > kZ || taps < 0 || taps > kMaxTaps) return false;
    keys_ = keys;
    zones_ = zones;
    taps_ = taps;
    int ent[kZ * kZ];
    for (int& e : ent) e = -1;
    nEnt_ = 0;
    for (int k = 0; k < keys; ++k) {
        kz_[k] = kz[k];
        const int za[2] = {kz[k].z0, kz[k].z0 + 1};
        const float wa[2] = {kz[k].w0, kz[k].w1};
        const int nz = kz[k].w1 != 0.0f ? 2 : 1;
        if (kz[k].z0 < 0 || kz[k].z0 + nz > zones) return false;
        for (int q = 0; q < 4; ++q) {
            keyEnt_[k][q] = -1;
            keyEntW_[k][q] = 0.0f;
        }
        for (int a = 0; a < nz; ++a)
            for (int b = 0; b < nz; ++b) {
                int& e = ent[za[a] * kZ + za[b]];
                if (e < 0) {
                    e = nEnt_++;
                    entDst_[e] = za[a];
                    entSrc_[e] = za[b];
                }
                keyEnt_[k][2 * a + b] = e;
                keyEntW_[k][2 * a + b] = wa[a] * wa[b];
            }
    }
    // Tap t reads every zone some key uses.
    for (int t = 0; t < taps; ++t)
        for (int z = 0; z < kZ; ++z) tapEnt_[t][z] = -1;
    for (int t = 0; t < taps; ++t)
        for (int k = 0; k < keys; ++k)
            for (int a = 0; a < (kz[k].w1 != 0.0f ? 2 : 1); ++a) {
                int& e = tapEnt_[t][kz[k].z0 + a];
                if (e >= 0) continue;
                e = nEnt_++;
                entDst_[e] = zones + t;
                entSrc_[e] = kz[k].z0 + a;
            }
    entPad_ = (nEnt_ + simd::kWidth - 1) / simd::kWidth * simd::kWidth;
    const std::size_t ep = static_cast<std::size_t>(entPad_);
    if (!h_.allocate(std::size_t{kMaxKeys} * kMaxBlock) || !G_.allocate(kMaxBlock * ep) ||
        !dE_.allocate(kMaxBlock * ep) || !acc_.allocate(ep) || !d_.allocate((kMaxBlock + 1) * std::size_t{kZ}) ||
        !f0_.allocate(kMaxBlock * std::size_t{kZ + kMaxTaps}) || !u_.allocate(kMaxBlock))
        return false;
    reset();
    return true;
}

void CouplingLoop::reset() noexcept {
    for (int k = 0; k < kMaxKeys; ++k) {
        hasH_[k] = member_[k] = listen_[k] = false;
        ver_[k] = 0;
    }
    for (int z = 0; z < kZ; ++z) zoneOut_[z] = zoneOutPrev_[z] = false;
    for (int i = 0; i <= kMaxBlock; ++i)
        for (int z = 0; z < kZ; ++z) d_[i * kZ + z] = 0.0f;
    nMembers_ = nListen_ = nOut_ = 0;
    dirty_ = true;
}

void CouplingLoop::rebuildKernel() noexcept {
    const std::size_t ep = static_cast<std::size_t>(entPad_);
    for (std::size_t i = 0; i < kMaxBlock * ep; ++i) G_[i] = 0.0f;
    for (int m = 0; m < nMembers_; ++m) {
        const int k = members_[m];
        const float s = gain_ * kz_[k].forceScale;
        const float* h = h_.data() + std::size_t(k) * kMaxBlock;
        for (int q = 0; q < 4; ++q) {
            const int e = keyEnt_[k][q];
            if (e < 0) continue;
            const float w = s * keyEntW_[k][q];
            for (int j = 0; j < kMaxBlock; ++j) G_[j * ep + e] += w * h[j];
        }
        // Tap t: sum_k tapGain fs F_k with u_k = g (w0 d_z0 + w1 d_z1).
        for (int t = 0; t < taps_; ++t)
            for (int a = 0; a < (kz_[k].w1 != 0.0f ? 2 : 1); ++a) {
                const float w = s * tapGain_[t][k] * (a == 0 ? kz_[k].w0 : kz_[k].w1);
                const int e = tapEnt_[t][kz_[k].z0 + a];
                for (int j = 0; j < kMaxBlock; ++j) G_[j * ep + e] += w * h[j];
            }
    }
    dirty_ = false;
}

void CouplingLoop::begin(ModeBank& bank, const SegmentIO* io, int n, Kernel kernel) noexcept {
    // Membership and impulse responses.
    const bool loop = gain_ != 0.0f;
    int nm = 0, nl = 0;
    for (int k = 0; k < keys_; ++k) {
        bool mem = false, lis = false;
        if (loop) {
            if (io[k].skip) {
                lis = true;
            } else {
                const std::uint32_t v = bank.coupledVersion(k);
                if (v != ver_[k] || !hasH_[k]) {
                    hasH_[k] = bank.coupledResponse(k, h_.data() + std::size_t(k) * kMaxBlock, kMaxBlock, &k1_[k], kernel);
                    ver_[k] = v;
                    if (hasH_[k] || member_[k]) dirty_ = true;
                }
                mem = hasH_[k];
                lis = mem || bank.segmentWakeable(k);
            }
        }
        if (mem != member_[k]) dirty_ = true;
        member_[k] = mem;
        if (mem) members_[nm++] = k;
        // A key that stops listening restarts its static correction from rest.
        if (listen_[k] && !lis) bank.setCoupledInputHistory(k, 0.0f);
        listen_[k] = lis;
        if (lis) listenList_[nl++] = k;
    }
    nMembers_ = nm;
    nListen_ = nl;
    if (dirty_) rebuildKernel();

    // Zones read back; a zone not read in the last sub-block starts from 0.
    for (int z = 0; z < kZ; ++z) zoneOut_[z] = false;
    for (int l = 0; l < nl; ++l) {
        const KeyZone& z = kz_[listenList_[l]];
        zoneOut_[z.z0] = true;
        if (z.w1 != 0.0f) zoneOut_[z.z0 + 1] = true;
    }
    nOut_ = 0;
    for (int z = 0; z < zones_; ++z) {
        if (zoneOut_[z]) outZones_[nOut_++] = z;
        if (!zoneOutPrev_[z]) d_[z] = 0.0f;
    }

    // Free zone forces: every key's free output, and the static correction's
    // tap on the last input of the previous sub-block.
    constexpr int W = kZ + kMaxTaps;
    for (int i = 0; i < n * W; ++i) f0_[i] = 0.0f;
    for (int k = 0; k < keys_; ++k) {
        if (io[k].skip || !io[k].wrote) continue;
        const KeyZone& z = kz_[k];
        const float* y = io[k].out;
        const float a0 = z.w0 * z.forceScale, a1 = z.w1 * z.forceScale;
        float* f = f0_.data();
        if (a1 != 0.0f)
            for (int i = 0; i < n; ++i) {
                f[i * W + z.z0] += a0 * y[i];
                f[i * W + z.z0 + 1] += a1 * y[i];
            }
        else
            for (int i = 0; i < n; ++i) f[i * W + z.z0] += a0 * y[i];
        for (int t = 0; t < taps_; ++t) {
            const float a = tapGain_[t][k] * z.forceScale;
            for (int i = 0; i < n; ++i) f[i * W + kZ + t] += a * y[i];
        }
    }
    for (int m = 0; m < nm; ++m) {
        const int k = members_[m];
        const KeyZone& z = kz_[k];
        const float c = -k1_[k] * bank.coupledInputHistory(k) * z.forceScale;
        f0_[z.z0] += z.w0 * c;
        if (z.w1 != 0.0f) f0_[z.z0 + 1] += z.w1 * c;
        for (int t = 0; t < taps_; ++t) f0_[kZ + t] += tapGain_[t][k] * c;
    }
}

void CouplingLoop::forces(int i, float* zoneForce, float* tap, Kernel kernel) noexcept {
    const float* f = f0_.data() + i * (kZ + kMaxTaps);
    for (int z = 0; z < zones_; ++z) zoneForce[z] += f[z];
    for (int t = 0; t < taps_; ++t) tap[t] += f[kZ + t];
    if (nMembers_ == 0) return;
    const int ep = entPad_;
    const float* d = d_.data() + i * kZ;
    float* de = dE_.data() + std::size_t(i) * ep;
    for (int e = 0; e < nEnt_; ++e) de[e] = d[entSrc_[e]];
    for (int e = nEnt_; e < ep; ++e) de[e] = 0.0f;
    float* acc = acc_.data();
    if (kernel == Kernel::Simd) {
        using namespace simd;
        for (int e = 0; e < ep; e += kWidth) {
            vf a0 = zero(), a1 = zero();
            int j = 0;
            for (; j + 1 <= i; j += 2) {
                a0 = fmadd(load(G_.data() + j * ep + e), load(dE_.data() + (i - j) * ep + e), a0);
                a1 = fmadd(load(G_.data() + (j + 1) * ep + e), load(dE_.data() + (i - j - 1) * ep + e), a1);
            }
            if (j <= i) a0 = fmadd(load(G_.data() + j * ep + e), load(dE_.data() + (i - j) * ep + e), a0);
            store(acc + e, add(a0, a1));
        }
    } else {
        for (int e = 0; e < nEnt_; ++e) {
            float a = 0.0f;
            for (int j = 0; j <= i; ++j) a += G_[j * ep + e] * dE_[(i - j) * ep + e];
            acc[e] = a;
        }
    }
    for (int e = 0; e < nEnt_; ++e) {
        const int d = entDst_[e];
        if (d < zones_) zoneForce[d] += acc[e];
        else tap[d - zones_] += acc[e];
    }
}

void CouplingLoop::push(int i, const float* zoneDisp) noexcept {
    float* d = d_.data() + (i + 1) * kZ;
    for (int o = 0; o < nOut_; ++o) d[outZones_[o]] = zoneDisp[outZones_[o]];
}

void CouplingLoop::end(ModeBank& bank, const SegmentIO* io, int n, Kernel kernel) noexcept {
    float* u = u_.data();
    for (int l = 0; l < nListen_; ++l) {
        const int k = listenList_[l];
        for (int i = 0; i < n; ++i) u[i] = keyInput(k, i);
        if (io[k].skip) bank.setCoupledInputHistory(k, u[n - 1]);
        else bank.applyCoupledInput(k, u, n, kernel);
    }
    for (int z = 0; z < kZ; ++z) {
        d_[z] = d_[n * kZ + z];
        zoneOutPrev_[z] = zoneOut_[z];
    }
}

}  // namespace pm
