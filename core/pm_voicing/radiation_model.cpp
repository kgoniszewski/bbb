#include "pm_voicing/radiation_model.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_board/soundboard.h"

namespace pm {

namespace {

using cd = std::complex<double>;
constexpr double kPi = std::numbers::pi;

struct Geometry {
    double mx, my, mz;                 // microphone
    bool image = false;                // lid image exists
    double ix, iy, iz;                 // image of the microphone in the lid plane
    double ny, nz, ey, ez, hz, len, lx;  // lid normal, lid direction, hinge height, length, x extent
    double r0;                         // bulk delay distance (board centre)
    double refl;
};

Geometry geometry(const BoardModel& b, const RadiationParams& r, int mic) {
    Geometry g{};
    g.mx = r.micX[mic];
    g.my = r.micY[mic];
    g.mz = r.micZ[mic];
    // One bulk delay for both microphones (mean distance to the board
    // centre): the inter-microphone time difference stays in the phases.
    g.r0 = 0.0;
    for (int q = 0; q < 2; ++q) {
        const double cx = 0.5 * b.params.lengthX - r.micX[q], cy = 0.5 * b.params.widthY - r.micY[q];
        const double d = std::sqrt(cx * cx + cy * cy + r.micZ[q] * r.micZ[q]);
        if (!r.commonBulkDelay) g.r0 += q == mic ? d : 0.0;
        else g.r0 += 0.5 * d;
    }
    const double a = r.lidAngleDeg * kPi / 180.0;
    g.ey = std::cos(a);
    g.ez = std::sin(a);
    g.ny = -std::sin(a);
    g.nz = std::cos(a);
    g.hz = r.lidHingeHeight;
    g.len = r.lidLength;
    g.lx = b.params.lengthX;
    g.refl = r.lidReflection;
    // Signed distance of the microphone from the lid plane (negative: under the lid).
    const double s = g.ny * g.my + g.nz * (g.mz - g.hz);
    if (r.lid && r.lidReflection != 0.0 && s < 0.0) {
        g.image = true;
        g.ix = g.mx;
        g.iy = g.my - 2.0 * s * g.ny;
        g.iz = g.mz - 2.0 * s * g.nz;
    }
    return g;
}

// Lid reflection from a board point (x, y, 0): valid when the specular
// point lies on the lid.
bool lidHit(const Geometry& g, double x, double y) {
    const double dy = g.iy - y, dz = g.iz, dx = g.ix - x;
    const double den = g.ny * dy + g.nz * dz;
    if (den == 0.0) return false;
    const double t = (g.ny * (0.0 - y) + g.nz * (g.hz - 0.0)) / den;
    if (t <= 0.0 || t >= 1.0) return false;
    const double qx = x + t * dx, qy = y + t * dy, qz = t * dz;
    const double along = g.ey * qy + g.ez * (qz - g.hz);
    return along >= 0.0 && along <= g.len && qx >= 0.0 && qx <= g.lx;
}

int gridCount(double extent, double hAir, int halfWaves, const RadiationParams& r) {
    const double nAir = extent / hAir;
    const double nShape = r.patchesPerHalfWave * std::max(halfWaves, 1);
    return static_cast<int>(std::ceil(r.gridScale * std::max({nAir, nShape, double(r.minPatches)})));
}

}  // namespace

std::complex<double> micModeGain(const BoardModel& b, const RadiationParams& r, int mic, int i) {
    const PlateParams& p = b.params;
    const double w = 2.0 * kPi * b.freq[i];
    const double wd = std::sqrt(std::max(w * w - b.sigma[i] * b.sigma[i], 1e-6 * w * w));
    const double k = wd / r.c;
    const double hAir = r.c / (r.patchesPerWavelength * std::max(b.freq[i], 1.0));
    const int nx = gridCount(p.lengthX, hAir, b.mIndex[i], r);
    const int ny = gridCount(p.widthY, hAir, b.nIndex[i], r);
    std::vector<double> xs(nx), ys(ny), fx(nx), fy(ny);
    const double hx = p.lengthX / nx, hy = p.widthY / ny;
    for (int a = 0; a < nx; ++a) xs[a] = (a + 0.5) * hx;
    for (int c = 0; c < ny; ++c) ys[c] = (c + 0.5) * hy;
    boardModeShapeAxes(b, i, xs.data(), nx, fx.data(), ys.data(), ny, fy.data());
    const Geometry g = geometry(b, r, mic);
    cd acc = 0.0;
    for (int c = 0; c < ny; ++c) {
        cd row = 0.0;
        const double dy = ys[c] - g.my, dy2 = dy * dy + g.mz * g.mz;
        const double iy = ys[c] - g.iy, iy2 = iy * iy + g.iz * g.iz;
        for (int a = 0; a < nx; ++a) {
            const double dx = xs[a] - g.mx;
            const double rr = std::sqrt(dx * dx + dy2);
            cd kern = std::polar(1.0 / rr, -k * (rr - g.r0));
            if (g.image && lidHit(g, xs[a], ys[c])) {
                const double ri = std::sqrt(dx * dx + iy2);
                kern += std::polar(g.refl / ri, -k * (ri - g.r0));
            }
            row += fx[a] * kern;
        }
        acc += fy[c] * row;
    }
    return cd(0.0, wd * r.rho / (2.0 * kPi)) * acc * (hx * hy);
}

MicGains computeMicGains(const BoardModel& b, const RadiationParams& r) {
    MicGains m;
    m.modes = b.modes;
    m.plate = b.params;
    m.params = r;
    m.gain.resize(std::size_t(2) * b.modes);
    for (int mic = 0; mic < 2; ++mic)
        for (int i = 0; i < b.modes; ++i) m.gain[std::size_t(mic) * b.modes + i] = micModeGain(b, r, mic, i);
    // Geometric means over the normalisation band; values far below the
    // band's rms (nodal lines) are floored so that a near-zero does not
    // dominate the log.
    auto logMean = [&](auto mag) {
        double ss = 0.0;
        int n = 0;
        for (int mic = 0; mic < 2; ++mic)
            for (int i = 0; i < b.modes; ++i)
                if (b.freq[i] >= r.normLoHz && b.freq[i] <= r.normHiHz) ss += mag(mic, i) * mag(mic, i), ++n;
        if (n == 0) return 0.0;
        const double floor = 1e-3 * std::sqrt(ss / n);
        double acc = 0.0;
        for (int mic = 0; mic < 2; ++mic)
            for (int i = 0; i < b.modes; ++i)
                if (b.freq[i] >= r.normLoHz && b.freq[i] <= r.normHiHz) acc += std::log(std::max(mag(mic, i), floor));
        return acc / n;
    };
    const double lg = logMean([&](int mic, int i) { return std::abs(m.gain[std::size_t(mic) * b.modes + i]); });
    const double lp = logMean([&](int mic, int i) { return std::fabs(b.pickupShape[std::size_t(mic) * b.modes + i]); });
    m.scale = r.levelTrim * std::exp(lp - lg);
    for (cd& g : m.gain) g *= m.scale;
    return m;
}

void loadMicGains(Soundboard& sb, const MicGains& g) noexcept {
    for (int mic = 0; mic < 2; ++mic)
        for (int i = 0; i < g.modes; ++i) {
            const cd v = g.row(mic)[i];
            sb.setPickupGain(mic, i, v.real(), v.imag());
        }
    sb.updatePickupBound();
}

std::complex<double> micModalTransfer(const BoardModel& b, const double* sa, const std::complex<double>* gain,
                                      double f) noexcept {
    const double w = 2.0 * kPi * f;
    cd y = 0.0;
    for (int i = 0; i < b.modes; ++i) {
        const double wm = 2.0 * kPi * b.freq[i];
        y += sa[i] * gain[i] / cd(wm * wm - w * w, 2.0 * b.sigma[i] * w);
    }
    return cd(0.0, w) * y;
}

}  // namespace pm
