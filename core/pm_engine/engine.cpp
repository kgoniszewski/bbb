#include "pm_engine/engine.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_voicing/key_loader.h"

// Per-phase timers (EngineProfile, engine.h): only in the profiling build
// pm_engine_profile; in the real-time library these expand to nothing.
#if defined(PM_PROFILE)
#include <chrono>
#define PM_PROF_START(t) auto t = std::chrono::steady_clock::now()
#define PM_PROF_LAP(phase, t)                                                                       \
    do {                                                                                          \
        const auto pmNow = std::chrono::steady_clock::now();                                      \
        prof_.us[EngineProfile::phase] += std::chrono::duration<double, std::micro>(pmNow - t).count(); \
        t = pmNow;                                                                                \
    } while (0)
#else
#define PM_PROF_START(t) ((void)0)
#define PM_PROF_LAP(phase, t) ((void)0)
#endif

namespace pm {

namespace {

constexpr double kPi = std::numbers::pi;
// Loop bands of the Balanced and Eco tiers (see engine.h).
constexpr double kBalancedLoopHz = 2000.0, kEcoLoopHz = 1000.0;

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

double loopCutoffForTier(QualityTier t, const PlateParams& plate) noexcept {
    switch (t) {
        case QualityTier::Balanced: return std::min(kBalancedLoopHz, plate.maxFreqHz);
        case QualityTier::Eco: return std::min(kEcoLoopHz, plate.maxFreqHz);
        default: return plate.maxFreqHz;
    }
}

bool Engine::prepare(double sampleRate, int maxFrames) noexcept {
    PianoTable t;
    buildPianoTable(t);
    return prepare(sampleRate, maxFrames, t);
}

bool Engine::prepare(double sampleRate, int maxFrames, const PianoTable& table) noexcept {
    if (!(sampleRate > 0.0) || maxFrames <= 0) return false;
    boardModel_ = buildPlateBoard(boardCfg_.plate);
    const double g = boardCfg_.enabled ? std::clamp(boardCfg_.sympathetic, 0.0, 1.5) : 0.0;
    const double cutoff = boardCfg_.loopCutoffHz > 0.0 ? boardCfg_.loopCutoffHz
                                                      : loopCutoffForTier(boardCfg_.tier, boardCfg_.plate);
    // The tuner matches octaves on the partials as they sound on this board
    // (bridge shifts, string-board hybrids); the bake then uses the result.
    PianoTable tuned = table;
    {
        const SoundboardBridge pre(boardModel_, sampleRate, g, &table, cutoff);
        tuneToSoundingPartials(tuned, pre);
    }
    const SoundboardBridge bridge(boardModel_, sampleRate, g, &tuned, cutoff);
    return prepareImpl(sampleRate, maxFrames, tuned, bridge, boardCfg_.enabled);
}

bool Engine::prepare(double sampleRate, int maxFrames, const PianoTable& table,
                     const BridgeAdmittance& bridge) noexcept {
    return prepareImpl(sampleRate, maxFrames, table, bridge, false);
}

bool Engine::prepareImpl(double sampleRate, int maxFrames, const PianoTable& table,
                         const BridgeAdmittance& bridge, bool board) noexcept {
    prepared_ = false;
    if (!(sampleRate > 0.0) || maxFrames <= 0) return false;
    fs_ = sampleRate;
    maxFrames_ = maxFrames;
    table_ = table;

    // Normal modes of every key (non-real-time; the per-partial eigen-solves
    // and admittance sums take a fraction of a second in total).
    static const RigidBridge rigid;
    std::vector<KeyModeSet> sets(kNumKeys);
    int bankModes[kNumKeys];
    totalModes_ = 0;
    perturbedPartials_ = uncoupledPartials_ = 0;
    for (int k = 0; k < kNumKeys; ++k) {
        const KeyParams& kp = table_.keys[k];
        sets[k] = computeKeyModes(kp, fs_, kp.bridgeCoupling ? bridge : static_cast<const BridgeAdmittance&>(rigid));
        keyModes_[k] = sets[k].count;
        keyCoupled_[k] = sets[k].coupledModes;
        bankModes[k] = keyBankModes(sets[k]);
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
    if (!bank_.allocateSegments(bankModes, kNumKeys)) return false;
    if (!contact_.allocate(bank_.capacity(), kNumKeys)) return false;
    if (!io_.allocate(kNumKeys) || !keyBuf_.allocate(std::size_t{kNumKeys} * kSubBlock) ||
        !mixL_.allocate(kSubBlock) || !mixR_.allocate(kSubBlock) || !hfL_.allocate(kSubBlock) ||
        !hfR_.allocate(kSubBlock) || !dampShape_.allocate(static_cast<std::size_t>(bank_.capacity())) ||
        !groupModes_.allocate(static_cast<std::size_t>(bank_.groupCount())))
        return false;
    for (int k = 0; k < kNumKeys; ++k) {
        loadKeyModes(bank_, contact_, k, table_.keys[k], sets[k], fs_);
        bank_.setSegmentCoupledGroups(k, keyCoupledGroups(sets[k]));
        io_[k].out = keyBuf_.data() + std::size_t(k) * kSubBlock;
        const int m0 = bank_.segmentFirstMode(k);
        for (int i = 0; i < sets[k].count; ++i)
            dampShape_[static_cast<std::size_t>(m0 + bankIndex(sets[k], i))] = static_cast<float>(sets[k].damperShape[i]);
        const int g0 = m0 / ModeBank::kGroupSize;
        const int cg = keyCoupledGroups(sets[k]);
        for (int g = 0; g < bank_.segmentModeCount(k) / ModeBank::kGroupSize; ++g) {
            const int used = g < cg ? sets[k].coupledModes - g * ModeBank::kGroupSize
                                    : (sets[k].count - sets[k].coupledModes) - (g - cg) * ModeBank::kGroupSize;
            groupModes_[g0 + g] = static_cast<std::uint8_t>(std::clamp(used, 0, ModeBank::kGroupSize));
        }
    }

    // Soundboard and coupling.
    boardOn_ = board;
    symGain_ = 0.0f;
    cullOffsetDb_ = 0.0f;
    if (board) {
        if (!loadSoundboard(board_, boardModel_, fs_)) return false;
        symGain_ = static_cast<float>(std::clamp(boardCfg_.sympathetic, 0.0, 1.5));
        for (int k = 0; k < kNumKeys; ++k) {
            keyZone_[k] = boardModel_.keyZone[k];
            const double gk = table_.keys[k].outputGain;
            keyZone_[k].forceScale = static_cast<float>(gk != 0.0 ? 1.0 / gk : 0.0);
        }
        // Static correction (soundboard_model.h): the bank applies it to its
        // awake coupled groups; a hammer owns all of its key's modes, so the
        // key's full taps apply while it does.
        for (int k = 0; k < kNumKeys; ++k) {
            bank_.computeStaticCorrection(k);
            statK0_[k] = bank_.staticCorrection0(k);
            statK1_[k] = bank_.staticCorrection1(k);
        }
        if (!loop_.configure(keyZone_, kNumKeys, board_.zoneCount(), 2)) return false;
        bank_.precomputeCoupledResponses();
        loop_.setGain(symGain_);
        // Peak gain from a key's bank output to the output, over every key,
        // pickup and board resonance (the modal sum evaluated at each mode's
        // frequency), for the culling threshold.
        double peak = 0.0;
        for (int k = 1; k <= kNumKeys; ++k)
            for (int p = 0; p < 2; ++p)
                for (int m = 0; m < boardModel_.modes; ++m) {
                    const double y = std::abs(boardModalAdmittance(
                        boardModel_, boardModel_.shapeOfKey(k),
                        boardModel_.pickupShape.data() + std::size_t(p) * boardModel_.modes, boardModel_.freq[m]));
                    peak = std::max(peak, y * keyZone_[k - 1].forceScale * kBoardOutputGain);
                }
        cullOffsetDb_ = peak > 1.0 ? static_cast<float>(20.0 * std::log10(peak)) : 0.0f;
        // Direct path gain: mean |transfer admittance| from the bridge points
        // to the pickups over the top fifth of the modal range.
        const double fMax = boardModel_.params.maxFreqHz;
        double acc = 0.0;
        int cnt = 0;
        for (int k = 1; k <= kNumKeys; k += 3)
            for (double f = 0.75 * fMax; f < 0.95 * fMax; f *= 1.002)
                for (int p = 0; p < 2; ++p) {
                    acc += std::abs(boardModalAdmittance(boardModel_, boardModel_.shapeOfKey(k),
                                                         boardModel_.pickupShape.data() + std::size_t(p) * boardModel_.modes, f));
                    ++cnt;
                }
        hfGain_ = static_cast<float>(cnt > 0 ? acc / cnt : 0.0);
        // Second-order Butterworth high-pass at 0.9 fMax (bilinear).
        const double wc = std::tan(kPi * std::min(0.9 * fMax, 0.45 * fs_) / fs_);
        const double k2 = wc * wc, q = std::sqrt(2.0), a0 = 1.0 + q * wc + k2;
        hpB_[0] = static_cast<float>(1.0 / a0);
        hpB_[1] = static_cast<float>(-2.0 / a0);
        hpB_[2] = static_cast<float>(1.0 / a0);
        hpA_[0] = static_cast<float>(2.0 * (k2 - 1.0) / a0);
        hpA_[1] = static_cast<float>((1.0 - q * wc + k2) / a0);
    }
    knockLen_ = std::max(2, static_cast<int>(0.0015 * fs_));
    noiseCoef_ = static_cast<float>(1.0 - std::exp(-2.0 * kPi * 2000.0 / fs_));
    pedalTau_ = 0.010 * fs_;

    appliedCullOn_ = cullOn_.load(std::memory_order_relaxed);
    appliedCullDb_ = cullDb_.load(std::memory_order_relaxed);
    bank_.setCulling(appliedCullOn_, appliedCullDb_ - cullOffsetDb_);
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
        applied_[k] = d;
        bank_.setSegmentDamper(k, d);
        cullOffsetApplied_[k] = 0.0f;
        bank_.setSegmentCullOffset(k, 0.0f, 0.0f);
        io_[k].skip = false;
        uPrev_[k] = 0.0f;
        bank_.setCoupledInputHistory(k, 0.0f);
    }
    board_.clearState();
    if (boardOn_) loop_.reset();
    for (int z = 0; z < Soundboard::kMaxZones; ++z) zoneF_[z] = zoneD_[z] = noiseLp_[z] = 0.0f;
    for (float& h : hpX_[0]) h = 0.0f;
    for (float& h : hpX_[1]) h = 0.0f;
    for (float& h : hpY_[0]) h = 0.0f;
    for (float& h : hpY_[1]) h = 0.0f;
    nKnocks_ = 0;
    rng_ = 0x9E3779B9u;
    noiseAmp_ = 0.0f;
    pedal_ = pedalPrev_ = pedalFrom_ = 0.0f;
    pedalStart_ = 0;
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
            when = now + hostTimeToFrameDelta(e.time, *clock) + latency;
        }
        if (when < now) {
            if (constant && e.timeBase == TimeBase::HostTime) ++late_;
            when = now;  // already due: start of this buffer
        }
        insertPending(when, e);
    }
}

void Engine::applyEvent(const Event& e, std::int64_t now) noexcept {
    ++info_.events;
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
        case static_cast<std::uint8_t>(EventType::KeyDownSilent):
            if (!validKey) break;
            keyDown_[k] = true;
            joinLoop(k);
            if (now >= panicUntil_) retargetDamper(k, now, static_cast<float>(kDefaultReleaseVelocity));
            break;
        case static_cast<std::uint8_t>(EventType::NoteOff):
            if (!validKey) break;
            keyDown_[k] = false;
            retargetDamper(k, now,
                           e.value ? e.value / 65535.0f : static_cast<float>(kDefaultReleaseVelocity));
            break;
        case static_cast<std::uint8_t>(EventType::Sustain): {
            // Continuous: the smoothed position restarts from its value at the
            // event's exact time (sampled on the control grid by controlTick).
            pedalFrom_ = pedalAt(now);
            pedalStart_ = now;
            const float next = e.value / 65535.0f;
            if (next > sustain_)
                for (int i = 0; i < kNumKeys; ++i) joinLoop(i);
            sustain_ = next;
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
            pedalFrom_ = pedalStart_ = 0;
            pedal_ = pedalPrev_ = sustain_ = 0.0f;
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
        applied_[k] = 0.0f;
        bank_.setSegmentDamper(k, 0.0f);
        joinLoop(k);
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
    h.contactMark = 0;
    bank_.wakeSegment(k);
    const float knock = knockLevel_.load(std::memory_order_relaxed);
    if (boardOn_ && knock > 0.0f && nKnocks_ < kMaxKnocks)
        knocks_[nKnocks_++] = Knock{k, knock * static_cast<float>(velocity), 0};
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
    flushHammer(h.state, contact_.view(bank_, h.key));
    hammerOfKey_[h.key] = -1;
    io_[h.key].skip = false;
    bank_.wakeSegment(h.key);
    active_[a] = active_[--activeCount_];
    freeList_[freeCount_++] = slot;
}

// ---------------------------------------------------------------- dampers

float Engine::damperTarget(int k, std::int64_t now) const noexcept {
    if (now < panicUntil_) return panicAmount_[k];
    if (!hasDamper_[k] || keyDown_[k] || sostLatched_[k]) return 0.0f;
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

// Displacement amplitude of key k's strings at the damper (peak estimate
// from the normal modes' magnitudes).
float Engine::keyAmplitude(int k) const noexcept {
    const int m0 = bank_.segmentFirstMode(k), n = bank_.segmentModeCount(k);
    const float* zr = bank_.stateReData() + m0;
    const float* zi = bank_.stateImData() + m0;
    const float* ds = dampShape_.data() + m0;
    // Squares only (no hypot): this runs for every key on every control tick
    // while the pedal holds the dampers off.
    // (n is a whole number of groups.)
    float s[4] = {};
    for (int i = 0; i < n; i += 4)
        for (int j = 0; j < 4; ++j) s[j] += (zr[i + j] * zr[i + j] + zi[i + j] * zi[i + j]) * (ds[i + j] * ds[i + j]);
    return std::sqrt(2.0f * ((s[0] + s[1]) + (s[2] + s[3])));
}

// Sustain-pedal factor of key k's damper amount (plan §1.9): felt offset e
// from pressed (-kFeltPress, full damping) through light contact (e = 0,
// kLightContact) to clear (+kFeltZone at the end of the transition zone,
// +kFeltGap at full pedal). With a gap e > 0 a vibration of amplitude A
// reaches the felt for acos(e / A) / pi of each cycle.
float Engine::pedalFactor(int k) noexcept {
    const float bass = 1.0f - k / float(kNumKeys - 1);
    const float p0T = hpP0T_.load(std::memory_order_relaxed), p0B = hpP0B_.load(std::memory_order_relaxed);
    const float w = hpW_.load(std::memory_order_relaxed);
    const float p0 = p0T + (p0B - p0T) * bass;
    const float t = (pedal_ - p0) / w;
    float e;
    if (t <= 1.0f) {
        float x = std::clamp(t, 0.0f, 1.0f);
        x = x * x * (3.0f - 2.0f * x);
        e = -kFeltPress + x * (kFeltPress + kFeltZone);
    } else {
        const float tMax = std::max((1.0f - p0) / w, 1.0001f);
        e = kFeltZone + (kFeltGap - kFeltZone) * std::min((t - 1.0f) / (tMax - 1.0f), 1.0f);
    }
    if (e <= 0.0f) return std::min(1.0f, kLightContact + (1.0f - kLightContact) * (-e / kFeltPress));
    if (damp_[k].cur <= 0.0f) return 0.0f;  // the key holds its damper up anyway
    const float a = keyAmplitude(k);
    if (a <= e) return 0.0f;
    return kLightContact * (2.0f / static_cast<float>(kPi)) * std::acos(e / a);
}

// A key whose damper starts to lift takes bridge input from this sample on.
void Engine::joinLoop(int k) noexcept {
    if (cullOffsetApplied_[k] != 0.0f) {
        cullOffsetApplied_[k] = 0.0f;
        bank_.setSegmentCullOffset(k, 0.0f, 0.0f);
    }
}

float Engine::pedalAt(std::int64_t t) const noexcept {
    const double x = double(t - pedalStart_) / pedalTau_;
    if (x > 40.0) return sustain_;
    return static_cast<float>(sustain_ + (pedalFrom_ - sustain_) * std::exp(-x));
}

// Runs on the absolute kSubBlock grid. Landing follows a raised-cosine felt
// contact curve; lifting is linear.
void Engine::controlTick(std::int64_t now) noexcept {
    if (panicUntil_ >= 0 && now >= panicUntil_) {
        panicUntil_ = -1;
        for (int k = 0; k < kNumKeys; ++k) retargetDamper(k, now, 1.0f);
    }
    // Panic also damps the board (its low modes ring for seconds).
    if (now < panicUntil_ && boardOn_)
        board_.scaleState(static_cast<float>(std::exp(-kPanicSigma * kSubBlock / fs_)));
    pedal_ = pedalAt(now);
    const float speed = std::fabs(pedal_ - pedalPrev_) * static_cast<float>(fs_ / kSubBlock);
    pedalPrev_ = pedal_;
    int moved = 0, withDamper = 0;
    const float dampedCull = dampedCullDb_.load(std::memory_order_relaxed);
    for (int k = 0; k < kNumKeys; ++k) {
        Damper& d = damp_[k];
        if (d.ramping) {
            const double t = std::clamp(double(now - d.start) / d.len, 0.0, 1.0);
            const double s = d.landing ? 0.5 - 0.5 * std::cos(kPi * t) : t;
            d.cur = static_cast<float>(d.from + (d.to - d.from) * s);
            if (t >= 1.0) {
                d.cur = d.to;
                d.ramping = false;
            }
        }
        float a;
        if (now < panicUntil_) a = d.cur;
        else if (!hasDamper_[k] || d.cur <= 0.0f) a = 0.0f;
        else a = d.cur * pedalFactor(k);
        if (hasDamper_[k]) {
            ++withDamper;
            moved += d.cur > 0.0f ? 1 : 0;
        }
        if (a != applied_[k]) {
            applied_[k] = a;
            bank_.setSegmentDamper(k, a);
        }
        // Out of the loop only once the damper is fully seated and still (a
        // string just being damped keeps listening while it rings down);
        // back in at the exact time of the event that lifts it.
        const float off = boardOn_ && a >= 1.0f && !d.ramping ? dampedCull : 0.0f;
        if (off != cullOffsetApplied_[k]) {
            cullOffsetApplied_[k] = off;
            bank_.setSegmentCullOffset(k, 0.0f, off);
        }
    }
    // Damper-lift noise: proportional to pedal speed and to the share of
    // dampers the pedal moves (those not already held up by their keys).
    const float level = liftNoiseLevel_.load(std::memory_order_relaxed);
    noiseAmp_ = boardOn_ && level > 0.0f && withDamper > 0 ? level * speed * float(moved) / float(withDamper) : 0.0f;
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
            if (boardOn_) {
                loop_.setTapGain(0, k, gainL_[k]);
                loop_.setTapGain(1, k, gainR_[k]);
            }
        }
    }
    masterTarget_ = std::pow(10.0f, std::clamp(masterGainDb_.load(std::memory_order_relaxed),
                                               -96.0f, 24.0f) / 20.0f);
    const bool on = cullOn_.load(std::memory_order_relaxed);
    const float db = cullDb_.load(std::memory_order_relaxed);
    if (on != appliedCullOn_ || db != appliedCullDb_) {
        appliedCullOn_ = on;
        appliedCullDb_ = db;
        bank_.setCulling(on, db - cullOffsetDb_);
        for (int a = 0; a < activeCount_; ++a) bank_.wakeSegment(hammers_[active_[a]].key);
    }
}

// One sub-block with the soundboard loop (see the header): every awake string
// mode advances freely as a block, then the hammers, the zone core of the
// loop and the board run sample by sample, then the loop's input is added
// to the strings' state (CouplingLoop, pm_board/soundboard.h). Output into
// mixL_/mixR_ (board) and hfL_/hfR_ (direct).
void Engine::processBoard(int n) noexcept {
    PM_PROF_START(tp);
    for (int a = 0; a < activeCount_; ++a) {
        const int k = hammers_[active_[a]].key;
        hView_[a] = contact_.view(bank_, k);
        io_[k].skip = true;
    }
    loop_.begin(bank_, board_, io_.data());
    for (int a = 0; a < activeCount_; ++a) {
        const int k = hammers_[active_[a]].key;
        uPrev_[k] = bank_.coupledInputHistory(k);
    }
    PM_PROF_LAP(Events, tp);
    bank_.processSegmentsFree(io_.data(), n);
    PM_PROF_LAP(FreePass, tp);
    loop_.collect(bank_, io_.data(), n);
    const int* outZones = nullptr;
    const int nOut = loop_.outZones(outZones);

    const int zones = board_.zoneCount();
    const float pi2 = 2.0f * static_cast<float>(kPi);
    for (int i = 0; i < n; ++i) {
        for (int z = 0; z < zones; ++z) zoneF_[z] = 0.0f;
        hfL_[i] = hfR_[i] = 0.0f;
        PM_PROF_LAP(Board, tp);
        for (int a = 0; a < activeCount_; ++a) {
            Hammer& h = hammers_[active_[a]];
            const int k = h.key;
            const float u = loop_.keyInput(k, i);
            float f = 0.0f;
            stepHammer(h.state, h.params, hView_[a], &u, &f, 1);
            // A hammer owns all of its key's modes: the key's full taps apply.
            f -= statK0_[k] * u + statK1_[k] * uPrev_[k];
            uPrev_[k] = u;
            if (f == 0.0f) continue;
            const KeyZone& kz = keyZone_[k];
            const float fn = f * kz.forceScale;
            hfL_[i] += gainL_[k] * fn;
            hfR_[i] += gainR_[k] * fn;
            zoneF_[kz.z0] += kz.w0 * fn;
            if (kz.w1 != 0.0f) zoneF_[kz.z0 + 1] += kz.w1 * fn;
        }
        PM_PROF_LAP(Hammers, tp);
        float tap[2] = {0.0f, 0.0f};
        loop_.forces(i, zoneF_, tap);
        hfL_[i] += tap[0];
        hfR_[i] += tap[1];

        if (noiseAmp_ > 0.0f) {
            for (int z = 0; z < zones; ++z) {
                rng_ ^= rng_ << 13;
                rng_ ^= rng_ >> 17;
                rng_ ^= rng_ << 5;
                // Uniform in [-1, 1): rms 1/sqrt(3).
                const float w = static_cast<float>(static_cast<std::int32_t>(rng_)) * (1.0f / 2147483648.0f);
                noiseLp_[z] += noiseCoef_ * (1.7320508f * w - noiseLp_[z]);
                zoneF_[z] += noiseAmp_ * noiseLp_[z];
            }
        }
        for (int j = 0; j < nKnocks_;) {
            Knock& kn = knocks_[j];
            const float f = kn.amp * 0.5f * (1.0f - std::cos(pi2 * (kn.age + 0.5f) / knockLen_));
            const KeyZone& kz = keyZone_[kn.key];
            zoneF_[kz.z0] += kz.w0 * f;
            if (kz.w1 != 0.0f) zoneF_[kz.z0 + 1] += kz.w1 * f;
            if (++kn.age >= knockLen_) knocks_[j] = knocks_[--nKnocks_];
            else ++j;
        }
        board_.step(zoneF_, zoneD_, pick_, Kernel::Simd, outZones, nOut);
        loop_.push(i, zoneD_);
        mixL_[i] = kBoardOutputGain * pick_[0];
        mixR_[i] = kBoardOutputGain * pick_[1];
    }
    PM_PROF_LAP(Board, tp);
    loop_.end(bank_, io_.data(), n);
    PM_PROF_LAP(ForcedPass, tp);
    bank_.endCoupled(io_.data(), n);
}

void Engine::processSubBlock(float* left, float* right, int n, std::int64_t now) noexcept {
    ++info_.subBlocks;
    float* mL = mixL_.data();
    float* mR = mixR_.data();
    if (boardOn_) {
        processBoard(n);
        // Direct path, high-passed.
        const float g = kBoardOutputGain * hfGain_;
        float* bus[2] = {hfL_.data(), hfR_.data()};
        float* mix[2] = {mL, mR};
        for (int c = 0; c < 2; ++c)
            for (int i = 0; i < n; ++i) {
                const float x = bus[c][i];
                const float y = hpB_[0] * x + hpB_[1] * hpX_[c][0] + hpB_[2] * hpX_[c][1] - hpA_[0] * hpY_[c][0] -
                                hpA_[1] * hpY_[c][1];
                hpX_[c][1] = hpX_[c][0];
                hpX_[c][0] = x;
                hpY_[c][1] = hpY_[c][0];
                hpY_[c][0] = y;
                mix[c][i] += g * y;
            }
        // Stereo width scales the side signal (1 at the default 0.35).
        const float side = appliedWidth_ / 0.35f;
        for (int i = 0; i < n; ++i) {
            const float mid = 0.5f * (mL[i] + mR[i]), sd = 0.5f * (mL[i] - mR[i]) * side;
            mL[i] = mid + sd;
            mR[i] = mid - sd;
        }
    } else {
        PM_PROF_START(tp);
        for (int a = 0; a < activeCount_; ++a) {
            Hammer& h = hammers_[active_[a]];
            const HammerKeyModes m = contact_.view(bank_, h.key);
            runHammer(h.state, h.params, m, io_[h.key].out, n);
            io_[h.key].skip = true;
        }
        PM_PROF_LAP(Hammers, tp);
        bank_.processSegments(io_.data(), n);
        PM_PROF_LAP(FreePass, tp);
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
    info_.frames = nFrames;
    info_.subBlocks = 0;
    info_.events = 0;
#if defined(PM_PROFILE)
    prof_ = EngineProfile{};
    const auto tRender = std::chrono::steady_clock::now();
#endif
    PM_PROF_START(te);
    readParams();
    drainQueue(clock);
    PM_PROF_LAP(Events, te);

    int pos = 0;
    while (pos < nFrames) {
        const std::int64_t now = static_cast<std::int64_t>(sampleTime_) + pos;
        PM_PROF_START(tev);
        while (pendingCount_ > 0 && pending_[0].when <= now) {
            const Event e = pending_[0].e;
            --pendingCount_;
            for (int i = 0; i < pendingCount_; ++i) pending_[i] = pending_[i + 1];
            applyEvent(e, now);
        }
        if (now % kSubBlock == 0) controlTick(now);
        PM_PROF_LAP(Events, tev);
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
    int inContact = 0;
    for (int a = 0; a < activeCount_; ++a) {
        Hammer& h = hammers_[active_[a]];
        inContact += h.state.contactSubsteps > h.contactMark ? 1 : 0;
        h.contactMark = h.state.contactSubsteps;
    }
    info_.hammersActive = activeCount_;
    info_.hammersInContact = inContact;
    info_.awakeModes = awakeModes;
    pub_.awakeGroups.store(awakeGroups, std::memory_order_relaxed);
    pub_.awakeModes.store(awakeModes, std::memory_order_relaxed);
    pub_.activeHammers.store(activeCount_, std::memory_order_relaxed);
    pub_.peakHammers.store(peakHammers_, std::memory_order_relaxed);
    pub_.deferred.store(deferred_, std::memory_order_relaxed);
    pub_.late.store(late_, std::memory_order_relaxed);
    pub_.limited.store(limited_, std::memory_order_relaxed);
    pub_.nonFinite.store(nonFinite_, std::memory_order_relaxed);
    pub_.peakPre.store(peakPre_, std::memory_order_relaxed);
#if defined(PM_PROFILE)
    prof_.totalUs = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tRender).count();
#endif
}

}  // namespace pm
