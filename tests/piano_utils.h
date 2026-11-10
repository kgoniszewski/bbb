// Helpers for hammer and engine tests: a single key (one bank segment plus
// its contact tables) struck in isolation, and engine rendering.
#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_engine/engine.h"
#include "pm_hammer/hammer.h"
#include "pm_voicing/key_loader.h"
#include "pm_voicing/piano_table.h"

namespace pm_test {

inline pm::KeyParams tableKey(int key) {
    static pm::PianoTable t = [] {
        pm::PianoTable x;
        pm::buildPianoTable(x);
        return x;
    }();
    return t.keys[key - 1];
}

// Hammer velocities used for pp / mf / ff.
inline constexpr double kPP = 0.8, kMF = 2.2, kFF = 5.0;

struct SingleKey {
    pm::KeyParams params;
    pm::KeyModeSet modes;
    pm::ModeBank bank;
    pm::ContactTables contact;
    pm::HammerParams hp;
    pm::HammerState hs;
    double fs;

    SingleKey(const pm::KeyParams& k, double sampleRate, int oversampling = 0) : params(k), fs(sampleRate) {
        if (oversampling > 0) params.contactOversampling = oversampling;
        modes = pm::computeKeyModes(params, fs);
        const int n = modes.count;
        bank.allocateSegments(&n, 1);
        contact.allocate(bank.capacity(), 1);
        pm::loadKey(bank, contact, 0, params, fs);
        bank.setSegmentDamper(0, 0.0f);
        hp = pm::hammerParams(params);
    }
    pm::HammerKeyModes view() { return contact.view(bank, 0); }
    double stringEnergy() const {
        double e = 0.0;
        for (int i = 0; i < modes.count; ++i)
            e += pm::modalEnergy(bank.stateRe(i), bank.stateIm(i), modes.freq[i], modes.sigma[i],
                                 modes.modalMass);
        return e;
    }
    double modeAmplitude(int i) const {  // sqrt of modal energy
        return std::sqrt(pm::modalEnergy(bank.stateRe(i), bank.stateIm(i), modes.freq[i],
                                         modes.sigma[i], modes.modalMass));
    }
    void strike(double v) { pm::launchHammer(hs, hp, view(), v); }
};

struct StrikeResult {
    double ke0 = 0, keEnd = 0, stringEnergy = 0, feltLoss = 0;  // at the end of the last contact
    double contactSpan = 0;  // s, first to last contact
    double contactTime = 0;  // s, total in contact
    int contacts = 0;
    double peakForce = 0;
    std::vector<float> out;  // key output, from launch to the hammer's release
    int samples = 0;
};

// Strikes and runs one base sample at a time until the hammer is finished.
inline StrikeResult runStrike(SingleKey& key, double velocity, int maxSamples = 48000) {
    pm::ScopedFlushDenormals ftz;
    StrikeResult r;
    key.strike(velocity);
    r.ke0 = key.hs.kineticEnergy0;
    const pm::HammerKeyModes m = key.view();
    bool wasIn = false;
    float y = 0.0f;
    for (int s = 0; s < maxSamples; ++s) {
        pm::runHammer(key.hs, key.hp, m, &y, 1);
        r.out.push_back(y);
        const bool in = key.hs.inContact;
        if (wasIn && !in) {
            r.keEnd = 0.5 * key.hp.mass * key.hs.v * key.hs.v;
            r.stringEnergy = key.stringEnergy();
            r.feltLoss = key.hs.feltLoss;
        }
        wasIn = in;
        if (pm::hammerFinished(key.hs, m)) break;
    }
    r.samples = static_cast<int>(r.out.size());
    r.contactSpan = (key.hs.lastContact - key.hs.firstContact + 1) * m.h;
    r.contactTime = key.hs.contactSubsteps * m.h;
    r.contacts = key.hs.contacts;
    r.peakForce = key.hs.peakForce;
    return r;
}

// Engine helpers.
inline pm::Event noteOn(std::uint64_t sample, int note, int vel7) {
    pm::Event e;
    e.time = sample;
    e.type = pm::EventType::NoteOn;
    e.timeBase = pm::TimeBase::SampleTime;
    e.note = static_cast<std::uint8_t>(note);
    e.value = pm::value16FromMidi7(vel7);
    return e;
}
inline pm::Event noteOff(std::uint64_t sample, int note) {
    pm::Event e = noteOn(sample, note, 0);
    e.type = pm::EventType::NoteOff;
    return e;
}
inline pm::Event sustain(std::uint64_t sample, bool down) {
    pm::Event e;
    e.time = sample;
    e.type = pm::EventType::Sustain;
    e.timeBase = pm::TimeBase::SampleTime;
    e.value = down ? 65535 : 0;
    return e;
}
inline pm::Event simple(std::uint64_t sample, pm::EventType t) {
    pm::Event e;
    e.time = sample;
    e.type = t;
    e.timeBase = pm::TimeBase::SampleTime;
    return e;
}

struct Stereo {
    std::vector<float> l, r;
};

// Renders nFrames in buffers of 'block' frames, pushing each event just
// before the buffer that contains it (the queue holds 1024).
inline Stereo renderEngine(pm::Engine& eng, const std::vector<pm::Event>& events, int nFrames,
                           int block = 64) {
    Stereo s;
    s.l.assign(static_cast<std::size_t>(nFrames), 0.0f);
    s.r.assign(static_cast<std::size_t>(nFrames), 0.0f);
    std::size_t next = 0;
    for (int pos = 0; pos < nFrames; pos += block) {
        const int n = std::min(block, nFrames - pos);
        while (next < events.size() && events[next].time < eng.sampleTime() + static_cast<std::uint64_t>(n))
            eng.pushEvent(events[next++]);
        eng.render(s.l.data() + pos, s.r.data() + pos, n, nullptr);
    }
    return s;
}

inline double peakAbs(const std::vector<float>& x, int from = 0, int to = -1) {
    if (to < 0) to = static_cast<int>(x.size());
    double m = 0.0;
    for (int i = from; i < to; ++i) m = std::max(m, double(std::fabs(x[i])));
    return m;
}

inline double rmsDb(const std::vector<float>& x, int from, int to) {
    double s = 0.0;
    for (int i = from; i < to; ++i) s += double(x[i]) * x[i];
    return 10.0 * std::log10(s / std::max(1, to - from) + 1e-30);
}

// Power-weighted mean frequency of x[from, from + len) (Hann window, DFT by
// direct evaluation on a grid of 'bins' frequencies up to fs/2).
inline double spectralCentroid(const std::vector<float>& x, int from, int len, double fs) {
    const int bins = 512;
    double num = 0.0, den = 0.0;
    for (int b = 1; b < bins; ++b) {
        const double f = 0.5 * fs * b / bins;
        double re = 0.0, im = 0.0;
        for (int i = 0; i < len; ++i) {
            const double w = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * i / (len - 1));
            const double ph = 2.0 * std::numbers::pi * f * i / fs;
            re += w * x[from + i] * std::cos(ph);
            im -= w * x[from + i] * std::sin(ph);
        }
        const double p = re * re + im * im;
        num += f * p;
        den += p;
    }
    return num / den;
}

}  // namespace pm_test
