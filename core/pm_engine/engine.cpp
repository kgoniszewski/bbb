#include "pm_engine/engine.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_voicing/key_loader.h"

namespace pm {

namespace {

constexpr double kPi = std::numbers::pi;

// Damper landing time (felt-contact curve, plan §1.9): 80 ms for a slow
// release down to 20 ms for the fastest.
double landingSeconds(float releaseVelocity01) noexcept {
    return 0.080 - 0.060 * std::clamp(static_cast<double>(releaseVelocity01), 0.0, 1.0);
}
constexpr double kDefaultReleaseVelocity = 64.0 / 127.0;
constexpr double kPedalLiftSeconds = 0.015;
constexpr double kPanicRampSeconds = 0.005;

// Soft clipper: identity below kClip, smooth saturation towards 1 above.
constexpr float kClip = 0.89f;
inline float softClip(float x, bool& limited) noexcept {
    const float a = std::fabs(x);
    if (a <= kClip) return x;
    limited = true;
    const float y = kClip + (1.0f - kClip) * std::tanh((a - kClip) / (1.0f - kClip));
    return std::copysign(std::min(y, 0.99999f), x);
}

}  // namespace

bool Engine::prepare(double sampleRate, int maxFrames) noexcept {
    PianoTable t;
    buildPianoTable(t);
    return prepare(sampleRate, maxFrames, t);
}

bool Engine::prepare(double sampleRate, int maxFrames, const PianoTable& table) noexcept {
    return prepare(sampleRate, maxFrames, table, defaultBridge());
}

bool Engine::prepare(double sampleRate, int maxFrames, const PianoTable& table,
                     const BridgeAdmittance& bridge) noexcept {
    prepared_ = false;
    if (!(sampleRate > 0.0) || maxFrames <= 0) return false;
    fs_ = sampleRate;
    maxFrames_ = maxFrames;
    table_ = table;

    // Normal modes of every key (non-real-time; the per-partial eigen-solves
    // take a few milliseconds in total).
    static const RigidBridge rigid;
    std::vector<KeyModeSet> sets(kNumKeys);
    totalModes_ = 0;
    perturbedPartials_ = uncoupledPartials_ = 0;
    for (int k = 0; k < kNumKeys; ++k) {
        const KeyParams& kp = table_.keys[k];
        sets[k] = computeKeyModes(kp, fs_, kp.bridgeCoupling ? bridge : static_cast<const BridgeAdmittance&>(rigid));
        keyModes_[k] = sets[k].count;
        totalModes_ += keyModes_[k];
        perturbedPartials_ += sets[k].perturbedPartials;
        uncoupledPartials_ += sets[k].uncoupledPartials;
        hammerParams_[k] = hammerParams(kp);
        unaString_[k] = kp.unaCordaString;
        unaMiss_[k] = kp.unaCordaMiss;
        unaSoften_[k] = std::clamp(kp.unaCordaSoften, 0.0, 0.9);
        hasDamper_[k] = kp.hasDamper;
        // Every partial's extra decay is at least damperSigma * damperFloor.
        const double minSigma = kp.damperSigma * kp.damperFloor;
        panicAmount_[k] = static_cast<float>(minSigma > 0.0 ? std::max(1.0, kPanicSigma / minSigma) : 1.0);
    }
    if (!bank_.allocateSegments(keyModes_, kNumKeys)) return false;
    if (!contact_.allocate(bank_.capacity(), kNumKeys)) return false;
    if (!io_.allocate(kNumKeys) || !keyBuf_.allocate(std::size_t{kNumKeys} * kSubBlock) ||
        !mixL_.allocate(kSubBlock) || !mixR_.allocate(kSubBlock) ||
        !groupModes_.allocate(static_cast<std::size_t>(bank_.groupCount())))
        return false;
    for (int k = 0; k < kNumKeys; ++k) {
        loadKeyModes(bank_, contact_, k, table_.keys[k], sets[k], fs_);
        io_[k].out = keyBuf_.data() + std::size_t(k) * kSubBlock;
        const int g0 = bank_.segmentFirstMode(k) / ModeBank::kGroupSize;
        for (int g = 0; g < bank_.segmentModeCount(k) / ModeBank::kGroupSize; ++g)
            groupModes_[g0 + g] = static_cast<std::uint8_t>(
                std::clamp(keyModes_[k] - g * ModeBank::kGroupSize, 0, ModeBank::kGroupSize));
    }
    appliedCullOn_ = cullOn_.load(std::memory_order_relaxed);
    appliedCullDb_ = cullDb_.load(std::memory_order_relaxed);
    bank_.setCulling(appliedCullOn_, appliedCullDb_);
    dcCoef_ = static_cast<float>(1.0 - 2.0 * kPi * 5.0 / fs_);  // 5 Hz DC blocker
    if (latency_.load(std::memory_order_relaxed) == 0)
        latency_.store(maxFrames, std::memory_order_relaxed);
    if (!velocitySet_) setVelocityFunction(velocityCurveFn, &defaultCurve_);

    Event drop;
    while (queue_.pop(drop)) {}
    resetState();
    prepared_ = true;
    return true;
}

void Engine::resetState() noexcept {
    bank_.clearState();
    for (int k = 0; k < kNumKeys; ++k) {
        keyDown_[k] = false;
        hammerOfKey_[k] = -1;
        const float d = hasDamper_[k] ? 1.0f : 0.0f;
        damp_[k] = Damper{d, d, d, 0, 0, false, false};
        bank_.setSegmentDamper(k, d);
        io_[k].skip = false;
    }
    sustain_ = 0.0f;
    sostenuto_ = false;
    for (int k = 0; k < kNumKeys; ++k) sostLatched_[k] = false;
    unaCorda_ = 0.0f;
    panicUntil_ = -1;
    freeCount_ = kHammerPool;
    for (int i = 0; i < kHammerPool; ++i) freeList_[i] = kHammerPool - 1 - i;
    activeCount_ = 0;
    pendingCount_ = 0;
    seq_ = 0;
    sampleTime_ = 0;
    appliedWidth_ = -1.0f;
    masterGain_ = masterTarget_ = std::pow(10.0f, masterGainDb_.load() / 20.0f);
    dcX_[0] = dcX_[1] = dcY_[0] = dcY_[1] = 0.0f;
    deferred_ = late_ = limited_ = nonFinite_ = 0;
    peakPre_ = 0.0f;
    peakHammers_ = 0;
}

void Engine::setVelocityFunction(VelocityFn fn, const void* user) noexcept {
    const int next = 1 - velActive_.load(std::memory_order_acquire);
    for (int i = 0; i < kVelTable; ++i)
        velTable_[next][i] = static_cast<float>(fn(double(i) / (kVelTable - 1), user));
    velActive_.store(next, std::memory_order_release);
    velocitySet_ = true;
}

double Engine::hammerVelocityFor(double x) const noexcept {
    const float* t = velTable_[velActive_.load(std::memory_order_acquire)];
    const double pos = std::clamp(x, 0.0, 1.0) * (kVelTable - 1);
    const int i = std::min(static_cast<int>(pos), kVelTable - 2);
    const double f = pos - i;
    return t[i] + f * (t[i + 1] - t[i]);
}

bool Engine::pushEvent(const Event& e) noexcept {
    if (queue_.push(e)) return true;
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
}

EngineStats Engine::stats() const noexcept {
    EngineStats s;
    s.totalModes = totalModes_;
    s.groups = bank_.groupCount();
    s.awakeGroups = pub_.awakeGroups.load(std::memory_order_relaxed);
    s.awakeModes = pub_.awakeModes.load(std::memory_order_relaxed);
    s.activeHammers = pub_.activeHammers.load(std::memory_order_relaxed);
    s.peakActiveHammers = pub_.peakHammers.load(std::memory_order_relaxed);
    s.droppedEvents = dropped_.load(std::memory_order_relaxed);
    s.deferredStrikes = pub_.deferred.load(std::memory_order_relaxed);
    s.lateEvents = pub_.late.load(std::memory_order_relaxed);
    s.limitedSamples = pub_.limited.load(std::memory_order_relaxed);
    s.nonFiniteSamples = pub_.nonFinite.load(std::memory_order_relaxed);
    s.peakPreLimiter = pub_.peakPre.load(std::memory_order_relaxed);
    return s;
}

// ---------------------------------------------------------------- events

bool Engine::insertPending(std::int64_t when, const Event& e) noexcept {
    if (pendingCount_ == kMaxPending) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    // Sorted by (when, arrival); events are few, insertion is cheap.
    int i = pendingCount_;
    while (i > 0 && pending_[i - 1].when > when) {
        pending_[i] = pending_[i - 1];
        --i;
    }
    pending_[i] = Pending{when, seq_++, e};
    ++pendingCount_;
    return true;
}

void Engine::drainQueue(const ClockMapping* clock) noexcept {
    const std::int64_t now = static_cast<std::int64_t>(sampleTime_);
    const bool constant = schedulingMode() == SchedulingMode::ConstantLatency;
    const std::int64_t latency = constant ? constantLatencyFrames() : 0;
    Event e;
    while (pendingCount_ < kMaxPending && queue_.pop(e)) {
        std::int64_t when = now;
        if (e.timeBase == TimeBase::SampleTime) {
            when = static_cast<std::int64_t>(e.time);
        } else if (clock && clock->hostTicksPerSample > 0.0) {
            // Signed difference: correct across counter wrap and for past events.
            const auto dt = static_cast<std::int64_t>(e.time - clock->hostTimeAtBufferStart);
            when = now + static_cast<std::int64_t>(
                             std::floor(static_cast<double>(dt) / clock->hostTicksPerSample)) +
                   latency;
        }
        if (when < now) {
            if (constant && e.timeBase == TimeBase::HostTime) ++late_;
            when = now;  // already due: start of this buffer
        }
        insertPending(when, e);
    }
}

void Engine::applyEvent(const Event& e, std::int64_t now) noexcept {
    const int k = static_cast<int>(e.note) - kFirstMidiNote;
    const bool validKey = k >= 0 && k < kNumKeys;
    switch (static_cast<std::uint8_t>(e.type)) {
        case static_cast<std::uint8_t>(EventType::NoteOn):
            if (!validKey) break;
            if (e.value == 0) {
                keyDown_[k] = false;
                retargetDamper(k, now, static_cast<float>(kDefaultReleaseVelocity));
            } else {
                noteOn(k, e.value, now);
            }
            break;
        case static_cast<std::uint8_t>(EventType::NoteOff):
            if (!validKey) break;
            keyDown_[k] = false;
            retargetDamper(k, now,
                           e.value ? e.value / 65535.0f : static_cast<float>(kDefaultReleaseVelocity));
            break;
        case static_cast<std::uint8_t>(EventType::Sustain): {
            const bool wasDown = sustain_ >= 0.5f;
            sustain_ = e.value / 65535.0f;
            if ((sustain_ >= 0.5f) != wasDown)
                for (int i = 0; i < kNumKeys; ++i)
                    retargetDamper(i, now, static_cast<float>(kDefaultReleaseVelocity));
            break;
        }
        case static_cast<std::uint8_t>(EventType::Sostenuto): {
            const bool down = e.value >= 32768;
            if (down && !sostenuto_) {
                // Latch the dampers that the keys themselves hold up now.
                for (int i = 0; i < kNumKeys; ++i) sostLatched_[i] = keyDown_[i] && hasDamper_[i];
            } else if (!down && sostenuto_) {
                for (int i = 0; i < kNumKeys; ++i) {
                    const bool was = sostLatched_[i];
                    sostLatched_[i] = false;
                    if (was) retargetDamper(i, now, static_cast<float>(kDefaultReleaseVelocity));
                }
            }
            sostenuto_ = down;
            break;
        }
        case static_cast<std::uint8_t>(EventType::UnaCorda):
            unaCorda_ = e.value / 65535.0f;
            break;
        case static_cast<std::uint8_t>(EventType::AllNotesOff):
            for (int i = 0; i < kNumKeys; ++i) {
                keyDown_[i] = false;
                retargetDamper(i, now, static_cast<float>(kDefaultReleaseVelocity));
            }
            break;
        case static_cast<std::uint8_t>(EventType::Panic): {
            while (activeCount_ > 0) releaseHammer(0, false);
            // Drop deferred strikes still waiting for a hammer.
            int j = 0;
            for (int i = 0; i < pendingCount_; ++i)
                if (static_cast<std::uint8_t>(pending_[i].e.type) != kStrikeOnly)
                    pending_[j++] = pending_[i];
            pendingCount_ = j;
            sustain_ = 0.0f;
            sostenuto_ = false;
            for (int i = 0; i < kNumKeys; ++i) sostLatched_[i] = false;
            unaCorda_ = 0.0f;
            panicUntil_ = now + static_cast<std::int64_t>(kPanicSeconds * fs_);
            for (int i = 0; i < kNumKeys; ++i) {
                keyDown_[i] = false;
                retargetDamper(i, now, 1.0f);
            }
            break;
        }
        case kStrikeOnly:
            if (!validKey) break;
            if (!strike(k, e.value / 1000.0, now))
                insertPending((now / kSubBlock + 1) * kSubBlock, e);
            break;
        default:
            break;
    }
}

void Engine::noteOn(int k, std::uint16_t vel, std::int64_t now) noexcept {
    keyDown_[k] = true;
    // The key lifts its damper before the hammer arrives: instant, and
    // click-free since only the decay rate changes.
    if (now >= panicUntil_) {
        damp_[k] = Damper{0.0f, 0.0f, 0.0f, now, 0, false, false};
        bank_.setSegmentDamper(k, 0.0f);
    }
    const double v = hammerVelocityFor(vel / 65535.0);
    if (!strike(k, v, now)) {
        // Pool exhausted and no hammer can be recycled: retry at the next
        // grid point (absolute, so the result does not depend on buffering).
        ++deferred_;
        Event e;
        e.type = static_cast<EventType>(kStrikeOnly);
        e.note = static_cast<std::uint8_t>(k + kFirstMidiNote);
        e.value = static_cast<std::uint16_t>(std::clamp(v * 1000.0, 0.0, 65535.0));  // mm/s
        insertPending((now / kSubBlock + 1) * kSubBlock, e);
    }
}

bool Engine::strike(int k, double velocity, std::int64_t now) noexcept {
    int slot = hammerOfKey_[k];
    if (slot < 0) {
        if (freeCount_ == 0) {
            // Recycle the oldest hammer that has already left its string.
            int best = -1;
            for (int a = 0; a < activeCount_; ++a) {
                const Hammer& h = hammers_[active_[a]];
                if (!h.state.inContact && h.state.lastContact >= 0 &&
                    (best < 0 || h.launchTime < hammers_[active_[best]].launchTime))
                    best = a;
            }
            if (best < 0) return false;
            releaseHammer(best, true);
        }
        slot = freeList_[--freeCount_];
        active_[activeCount_++] = slot;
        hammerOfKey_[k] = slot;
        hammers_[slot].key = k;
        peakHammers_ = std::max(peakHammers_, activeCount_);
    }
    Hammer& h = hammers_[slot];
    h.launchTime = static_cast<std::uint64_t>(now);
    h.unaCorda = unaCorda_;
    h.params = hammerParams_[k];
    if (unaCorda_ > 0.0f) {
        const double u = unaCorda_;
        if (unaString_[k] >= 0 && unaString_[k] < h.params.strings)
            h.params.offset[unaString_[k]] += u * u * unaMiss_[k];
        h.params.K *= 1.0 - u * unaSoften_[k];
    }
    const HammerKeyModes m = contact_.view(bank_, k);
    launchHammer(h.state, h.params, m, velocity);
    bank_.wakeSegment(k);
    return true;
}

void Engine::releaseHammer(int a, bool report) noexcept {
    const int slot = active_[a];
    Hammer& h = hammers_[slot];
    if (report) {
        const HammerKeyModes m = contact_.view(bank_, h.key);
        StrikeInfo info;
        info.sampleTime = h.launchTime;
        info.note = h.key + kFirstMidiNote;
        info.velocity = static_cast<float>(h.state.launchVelocity);
        if (h.state.firstContact >= 0)
            info.contactSpanMs = static_cast<float>(
                1e3 * m.h * double(h.state.lastContact - h.state.firstContact + 1));
        info.contactTimeMs = static_cast<float>(1e3 * m.h * double(h.state.contactSubsteps));
        info.contacts = h.state.contacts;
        info.peakForce = static_cast<float>(h.state.peakForce);
        info.unaCorda = h.unaCorda;
        info.fallbackSolves = h.state.fallbackSolves;
        if (h.state.kineticEnergy0 > 0.0)
            info.feltLossFraction = static_cast<float>(h.state.feltLoss / h.state.kineticEnergy0);
        strikeLog_.push(info);
    }
    hammerOfKey_[h.key] = -1;
    io_[h.key].skip = false;
    bank_.wakeSegment(h.key);
    active_[a] = active_[--activeCount_];
    freeList_[freeCount_++] = slot;
}

// ---------------------------------------------------------------- dampers

float Engine::damperTarget(int k, std::int64_t now) const noexcept {
    if (now < panicUntil_) return panicAmount_[k];
    if (!hasDamper_[k] || keyDown_[k] || sustain_ >= 0.5f || sostLatched_[k]) return 0.0f;
    return 1.0f;
}

void Engine::retargetDamper(int k, std::int64_t now, float releaseVelocity) noexcept {
    Damper& d = damp_[k];
    const float to = damperTarget(k, now);
    if (d.ramping ? d.to == to : d.cur == to) return;
    const bool landing = to > d.cur;
    double seconds;
    if (now < panicUntil_) seconds = kPanicRampSeconds;
    else if (landing) seconds = landingSeconds(releaseVelocity);
    else seconds = kPedalLiftSeconds;
    d.from = d.cur;
    d.to = to;
    d.start = now;
    d.len = std::max(1, static_cast<int>(seconds * fs_));
    d.ramping = true;
    d.landing = landing;
}

// Runs on the absolute kSubBlock grid. Landing follows a raised-cosine felt
// contact curve; lifting is linear.
void Engine::controlTick(std::int64_t now) noexcept {
    if (panicUntil_ >= 0 && now >= panicUntil_) {
        panicUntil_ = -1;
        for (int k = 0; k < kNumKeys; ++k) retargetDamper(k, now, 1.0f);
    }
    for (int k = 0; k < kNumKeys; ++k) {
        Damper& d = damp_[k];
        if (!d.ramping) continue;
        const double t = std::clamp(double(now - d.start) / d.len, 0.0, 1.0);
        const double s = d.landing ? 0.5 - 0.5 * std::cos(kPi * t) : t;
        d.cur = static_cast<float>(d.from + (d.to - d.from) * s);
        if (t >= 1.0) {
            d.cur = d.to;
            d.ramping = false;
        }
        bank_.setSegmentDamper(k, d.cur);
    }
}

// ---------------------------------------------------------------- render

void Engine::readParams() noexcept {
    const float w = std::clamp(width_.load(std::memory_order_relaxed), 0.0f, 1.0f);
    if (w != appliedWidth_) {
        appliedWidth_ = w;
        for (int k = 0; k < kNumKeys; ++k) {
            // Equal-power pan; bass to the left, as seen from the keyboard.
            const double pan = std::clamp(table_.keys[k].pan * w, -1.0, 1.0);
            const double th = 0.25 * kPi * (pan + 1.0);
            gainL_[k] = static_cast<float>(std::cos(th));
            gainR_[k] = static_cast<float>(std::sin(th));
        }
    }
    masterTarget_ = std::pow(10.0f, std::clamp(masterGainDb_.load(std::memory_order_relaxed),
                                               -96.0f, 24.0f) / 20.0f);
    const bool on = cullOn_.load(std::memory_order_relaxed);
    const float db = cullDb_.load(std::memory_order_relaxed);
    if (on != appliedCullOn_ || db != appliedCullDb_) {
        appliedCullOn_ = on;
        appliedCullDb_ = db;
        bank_.setCulling(on, db);
        for (int a = 0; a < activeCount_; ++a) bank_.wakeSegment(hammers_[active_[a]].key);
    }
}

void Engine::processSubBlock(float* left, float* right, int n, std::int64_t now) noexcept {
    for (int a = 0; a < activeCount_; ++a) {
        Hammer& h = hammers_[active_[a]];
        const HammerKeyModes m = contact_.view(bank_, h.key);
        runHammer(h.state, h.params, m, io_[h.key].out, n);
        io_[h.key].skip = true;
    }
    bank_.processSegments(io_.data(), n);

    float* mL = mixL_.data();
    float* mR = mixR_.data();
    std::fill(mL, mL + n, 0.0f);
    std::fill(mR, mR + n, 0.0f);
    for (int k = 0; k < kNumKeys; ++k) {
        if (!io_[k].skip && !io_[k].wrote) continue;
        const float* y = io_[k].out;
        const float gl = gainL_[k], gr = gainR_[k];
        for (int i = 0; i < n; ++i) {
            mL[i] += gl * y[i];
            mR[i] += gr * y[i];
        }
    }

    // Hammers are released only on a grid relative to their own launch, so
    // the hand-over back to the bank (which sums modes in another order)
    // happens at the same point of the note whatever the buffering.
    const std::int64_t end = now + n;
    for (int a = 0; a < activeCount_;) {
        Hammer& h = hammers_[active_[a]];
        if ((end - static_cast<std::int64_t>(h.launchTime)) % kSubBlock == 0 &&
            hammerFinished(h.state, contact_.view(bank_, h.key)))
            releaseHammer(a, true);
        else
            ++a;
    }

    const float gStep = 1.0f - std::exp(-1.0f / (0.01f * static_cast<float>(fs_)));
    for (int i = 0; i < n; ++i) {
        masterGain_ += (masterTarget_ - masterGain_) * gStep;
        float ch[2] = {mL[i], mR[i]};
        for (int c = 0; c < 2; ++c) {
            float x = ch[c];
            if (!std::isfinite(x)) {
                ++nonFinite_;
                x = 0.0f;
            }
            const float y = x - dcX_[c] + dcCoef_ * dcY_[c];
            dcX_[c] = x;
            dcY_[c] = y;
            const float v = y * masterGain_;
            peakPre_ = std::max(peakPre_, std::fabs(v));
            bool limited = false;
            ch[c] = softClip(v, limited);
            limited_ += limited;
        }
        left[i] = ch[0];
        right[i] = ch[1];
    }
}

void Engine::render(float* left, float* right, int nFrames, const ClockMapping* clock) noexcept {
    if (nFrames <= 0) return;
    if (!prepared_) {
        std::fill(left, left + nFrames, 0.0f);
        std::fill(right, right + nFrames, 0.0f);
        return;
    }
    ScopedFlushDenormals ftz;
    readParams();
    drainQueue(clock);

    int pos = 0;
    while (pos < nFrames) {
        const std::int64_t now = static_cast<std::int64_t>(sampleTime_) + pos;
        while (pendingCount_ > 0 && pending_[0].when <= now) {
            const Event e = pending_[0].e;
            --pendingCount_;
            for (int i = 0; i < pendingCount_; ++i) pending_[i] = pending_[i + 1];
            applyEvent(e, now);
        }
        if (now % kSubBlock == 0) controlTick(now);
        std::int64_t len = std::min<std::int64_t>(nFrames - pos, kSubBlock - now % kSubBlock);
        if (pendingCount_ > 0) len = std::min<std::int64_t>(len, pending_[0].when - now);
        for (int a = 0; a < activeCount_; ++a) {
            const std::int64_t age = now - static_cast<std::int64_t>(hammers_[active_[a]].launchTime);
            len = std::min<std::int64_t>(len, kSubBlock - age % kSubBlock);
        }
        processSubBlock(left + pos, right + pos, static_cast<int>(len), now);
        pos += len;
    }
    sampleTime_ += static_cast<std::uint64_t>(nFrames);

    int awakeGroups = 0, awakeModes = 0;
    for (int g = 0; g < bank_.groupCount(); ++g)
        if (bank_.groupActive(g)) {
            ++awakeGroups;
            awakeModes += groupModes_[g];
        }
    pub_.awakeGroups.store(awakeGroups, std::memory_order_relaxed);
    pub_.awakeModes.store(awakeModes, std::memory_order_relaxed);
    pub_.activeHammers.store(activeCount_, std::memory_order_relaxed);
    pub_.peakHammers.store(peakHammers_, std::memory_order_relaxed);
    pub_.deferred.store(deferred_, std::memory_order_relaxed);
    pub_.late.store(late_, std::memory_order_relaxed);
    pub_.limited.store(limited_, std::memory_order_relaxed);
    pub_.nonFinite.store(nonFinite_, std::memory_order_relaxed);
    pub_.peakPre.store(peakPre_, std::memory_order_relaxed);
}

}  // namespace pm
