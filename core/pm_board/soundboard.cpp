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

}  // namespace pm
