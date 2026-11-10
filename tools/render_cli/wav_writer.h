// Minimal WAV writer: 32-bit IEEE float or 24-bit PCM, little-endian, any
// channel count (samples interleaved).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace pm_tools {

enum class WavFormat { Float32, Pcm24 };

namespace detail {
inline void put16(std::vector<std::uint8_t>& b, std::uint32_t v) {
    b.push_back(static_cast<std::uint8_t>(v));
    b.push_back(static_cast<std::uint8_t>(v >> 8));
}
inline void put32(std::vector<std::uint8_t>& b, std::uint32_t v) {
    put16(b, v & 0xFFFF);
    put16(b, v >> 16);
}
inline void putTag(std::vector<std::uint8_t>& b, const char* t) { b.insert(b.end(), t, t + 4); }
}  // namespace detail

// Returns false on I/O error. Pcm24 clips to [-1, 1); returns the number of
// clipped samples through 'clipped' if given. x holds interleaved frames.
inline bool writeWav(const char* path, const std::vector<float>& x, int sampleRate,
                     WavFormat fmt, std::size_t* clipped = nullptr, int channels = 1) {
    using namespace detail;
    const bool isFloat = fmt == WavFormat::Float32;
    const std::uint32_t bytesPerSample = isFloat ? 4 : 3;
    const std::uint32_t dataBytes = static_cast<std::uint32_t>(x.size()) * bytesPerSample;
    const std::uint32_t fmtBytes = isFloat ? 18 : 16;  // non-PCM fmt carries cbSize
    const std::uint32_t factBytes = isFloat ? 12 : 0;  // non-PCM files need a fact chunk
    const std::uint32_t pad = dataBytes & 1;

    std::vector<std::uint8_t> b;
    b.reserve(64 + dataBytes);
    putTag(b, "RIFF");
    put32(b, 4 + (8 + fmtBytes) + factBytes + (8 + dataBytes + pad));
    putTag(b, "WAVE");
    putTag(b, "fmt ");
    put32(b, fmtBytes);
    put16(b, isFloat ? 3 : 1);  // WAVE_FORMAT_IEEE_FLOAT / WAVE_FORMAT_PCM
    const std::uint32_t ch = static_cast<std::uint32_t>(channels);
    put16(b, ch);
    put32(b, static_cast<std::uint32_t>(sampleRate));
    put32(b, static_cast<std::uint32_t>(sampleRate) * bytesPerSample * ch);
    put16(b, bytesPerSample * ch);
    put16(b, bytesPerSample * 8);
    if (isFloat) {
        put16(b, 0);
        putTag(b, "fact");
        put32(b, 4);
        put32(b, static_cast<std::uint32_t>(x.size() / ch));
    }
    putTag(b, "data");
    put32(b, dataBytes);

    std::size_t nClip = 0;
    for (float s : x) {
        if (isFloat) {
            std::uint32_t u;
            static_assert(sizeof(u) == sizeof(s));
            std::memcpy(&u, &s, 4);
            put32(b, u);
        } else {
            const double scaled = std::round(static_cast<double>(s) * 8388608.0);
            const double c = std::clamp(scaled, -8388608.0, 8388607.0);
            nClip += (c != scaled);
            const std::int32_t v = static_cast<std::int32_t>(c);
            const std::uint32_t u = static_cast<std::uint32_t>(v);
            b.push_back(static_cast<std::uint8_t>(u));
            b.push_back(static_cast<std::uint8_t>(u >> 8));
            b.push_back(static_cast<std::uint8_t>(u >> 16));
        }
    }
    if (pad) b.push_back(0);
    if (clipped) *clipped = nClip;

    std::FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const bool ok = std::fwrite(b.data(), 1, b.size(), f) == b.size();
    return std::fclose(f) == 0 && ok;
}

}  // namespace pm_tools
