#include "pm_bench/bench.h"

#include <algorithm>
#include <chrono>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_voicing/key_loader.h"

namespace pm::bench {

namespace {

using Clock = std::chrono::steady_clock;

Event ev(std::uint64_t t, EventType type, int note, int v7) {
    Event e;
    e.time = t;
    e.type = type;
    e.timeBase = TimeBase::SampleTime;
    e.note = static_cast<std::uint8_t>(note);
    e.value = value16FromMidi7(v7);
    return e;
}

// The events of a scene due in [t0, t1).
void sceneEvents(Scene s, double fs, std::uint64_t t0, std::uint64_t t1, std::vector<Event>& out) {
    out.clear();
    const std::uint64_t period = static_cast<std::uint64_t>(0.5 * fs);
    switch (s) {
        case Scene::Worst:
            if (t0 == 0) out.push_back(ev(0, EventType::Sustain, 0, 127));
            for (int n = 21; n <= 108; ++n) {
                const std::uint64_t t = static_cast<std::uint64_t>(n - 21) * 48;  // 1 ms apart
                if (t >= t0 && t < t1) out.push_back(ev(t, EventType::NoteOn, n, 110));
            }
            break;
        case Scene::Typical: {
            const int notes[4] = {48, 60, 64, 67};
            for (std::uint64_t base = (t0 / period) * period; base < t1; base += period)
                for (int n : notes) {
                    if (base >= t0) out.push_back(ev(base, EventType::NoteOn, n, 80));
                    const std::uint64_t off = base + period * 3 / 4;
                    if (off >= t0 && off < t1) out.push_back(ev(off, EventType::NoteOff, n, 64));
                }
            break;
        }
        case Scene::Cluster: {
            const std::uint64_t cp = static_cast<std::uint64_t>(0.1 * fs);
            for (std::uint64_t base = (t0 / cp) * cp; base < t1; base += cp) {
                if (base < t0) continue;
                const int first = (base / cp) % 2 ? 72 : 48;
                for (int n = first; n < first + 12; ++n) out.push_back(ev(base, EventType::NoteOn, n, 120));
                for (int n = first; n < first + 12; ++n)
                    out.push_back(ev(base + cp / 2, EventType::NoteOff, n, 64));
            }
            break;
        }
        case Scene::Bloom: {
            if (t0 == 0) out.push_back(ev(0, EventType::Sustain, 0, 127));
            const int notes[4] = {48, 60, 64, 67};
            for (std::uint64_t base = (t0 / period) * period; base < t1; base += period)
                for (int n : notes) {
                    if (base >= t0) out.push_back(ev(base, EventType::NoteOn, n, 90));
                    const std::uint64_t off = base + period / 5;
                    if (off >= t0 && off < t1) out.push_back(ev(off, EventType::NoteOff, n, 64));
                }
            break;
        }
    }
}

#if defined(PM_PROFILE)
Phases phasesOf(const EngineProfile& p) {
    static_assert(EngineProfile::kPhases + 1 == kPhaseCount, "phase table out of date");
    Phases r;
    double sum = 0.0;
    for (int i = 0; i < EngineProfile::kPhases; ++i) {
        r.us[i] = p.us[i];
        sum += p.us[i];
    }
    r.us[kPhaseCount - 1] = std::max(0.0, p.totalUs - sum);
    r.total = p.totalUs;
    return r;
}
#endif

void accumulate(Phases& acc, const Phases& p, double w) {
    for (int i = 0; i < kPhaseCount; ++i) acc.us[i] += w * p.us[i];
    acc.total += w * p.total;
}

}  // namespace

int caseCount(const Config& c) noexcept { return c.allTiers ? 13 : 7; }

Case caseAt(const Config& c, int i) noexcept {
    const QualityTier t = c.tier;
    switch (i) {
        case 0: return {"worst (88 keys held, pedal)", true, Scene::Worst, true, t};
        case 1: return {"worst, culling off", false, Scene::Worst, true, t};
        case 2: return {"typical (4-note chord, pedal up)", true, Scene::Typical, true, t};
        case 3: return {"cluster (12 notes ff every 100 ms)", true, Scene::Cluster, true, t};
        case 4: return {"bloom (pedal down, staccato chords)", true, Scene::Bloom, true, t};
        case 5: return {"worst, board off (A/B)", true, Scene::Worst, false, t};
        case 6: return {"cluster, board off (A/B)", true, Scene::Cluster, false, t};
        case 7: return {"worst, balanced", true, Scene::Worst, true, QualityTier::Balanced};
        case 8: return {"cluster, balanced", true, Scene::Cluster, true, QualityTier::Balanced};
        case 9: return {"bloom, balanced", true, Scene::Bloom, true, QualityTier::Balanced};
        case 10: return {"worst, eco", true, Scene::Worst, true, QualityTier::Eco};
        case 11: return {"cluster, eco", true, Scene::Cluster, true, QualityTier::Eco};
        case 12: return {"bloom, eco", true, Scene::Bloom, true, QualityTier::Eco};
        default: return {"?", true, Scene::Worst, true, t};
    }
}

const char* hammerName(int index) noexcept {
    static const char* const kNames[kHammerKeyCount] = {
        "hammer, key 1",  "hammer, key 16", "hammer, key 28", "hammer, key 40",
        "hammer, key 52", "hammer, key 64", "hammer, key 76", "hammer, key 88"};
    return index >= 0 && index < kHammerKeyCount ? kNames[index] : "?";
}

bool runScenario(const Config& c, const Case& k, Engine& eng, ScenarioResult& res) {
    res = ScenarioResult{};
    const double fs = c.sampleRate;
    const int block = std::max(1, c.block);
    eng.setCulling(k.cull, -100.0f);
    SoundboardConfig bc;
    bc.enabled = k.board;
    bc.tier = k.tier;
    eng.setSoundboardConfig(bc);
    if (!eng.prepare(fs, block)) return false;
    // The worst cases are measured from the first strike, so the max
    // includes the burst of 88 hammers (1 ms apart, pool of 32).
    const double warm = k.scene == Scene::Worst ? 0.0 : 0.5;

    std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
    std::vector<Event> events;
    const int warmBlocks = static_cast<int>(warm * fs / block);
    const int blocks = std::max(1, static_cast<int>(c.seconds * fs / block));
    std::uint64_t t = 0;
    auto step = [&](double* dt) {
        sceneEvents(k.scene, fs, t, t + static_cast<std::uint64_t>(block), events);
        for (const Event& e : events) eng.pushEvent(e);
        const auto a = Clock::now();
        eng.render(l.data(), r.data(), block, nullptr);
        if (dt) *dt = std::chrono::duration<double>(Clock::now() - a).count();
        t += static_cast<std::uint64_t>(block);
    };
    for (int b = 0; b < warmBlocks; ++b) step(nullptr);
    const double period = block / fs;
    res.periodUs = 1e6 * period;
    res.buffers = blocks;
    const int firstBlocks = std::max(1, static_cast<int>(0.1 * fs / block));
    std::vector<double> dts(static_cast<std::size_t>(blocks));
    std::vector<Phases> ph(static_cast<std::size_t>(blocks));
    double total = 0.0, first = 0.0, awake = 0.0;
    for (int b = 0; b < blocks; ++b) {
        step(&dts[b]);
#if defined(PM_PROFILE)
        ph[b] = phasesOf(eng.profile());
#else
        ph[b].total = 1e6 * dts[b];
#endif
        total += dts[b];
        if (b < firstBlocks) first += dts[b];
        awake += eng.stats().awakeModes;
    }
    res.meanAwakeModes = awake / blocks;
    res.first100msPercent = 100.0 * first / (std::min(firstBlocks, blocks) * period);
#if defined(PM_PROFILE)
    // Breakdown, ranked by the engine's own render time (the same buffers
    // as the timing above to within the timer overhead).
    res.hasPhases = true;
    std::vector<int> order(static_cast<std::size_t>(blocks));
    for (int b = 0; b < blocks; ++b) order[b] = b;
    std::sort(order.begin(), order.end(), [&](int a, int b) { return ph[a].total > ph[b].total; });
    res.maxIndex = order[0];
    res.maxBuf = ph[order[0]];
    res.topCount = std::max(1, blocks - static_cast<int>(0.99 * (blocks - 1)));
    for (int i = 0; i < res.topCount; ++i) accumulate(res.top1, ph[order[i]], 1.0 / res.topCount);
    for (int b = 0; b < blocks; ++b) accumulate(res.all, ph[b], 1.0 / blocks);
#else
    (void)accumulate;
#endif
    std::sort(dts.begin(), dts.end());
    res.avgPercent = 100.0 * total / (blocks * period);
    res.p99Percent = 100.0 * dts[static_cast<std::size_t>(0.99 * (blocks - 1))] / period;
    res.maxPercent = 100.0 * dts.back() / period;
    res.meanUs = 1e6 * total / blocks;
    res.p99Us = 1e6 * dts[static_cast<std::size_t>(0.99 * (blocks - 1))];
    res.maxUs = 1e6 * dts.back();
    res.awakeModesEnd = eng.stats().awakeModes;
    return true;
}

bool runHammer(const Config& c, int key, HammerResult& out) {
    out = HammerResult{};
    if (key < 1 || key > kNumKeys) return false;
    const double fs = c.sampleRate;
    const int block = std::max(1, c.block);
    const int reps = std::max(1, c.hammerStrikes);
    PianoTable* table = new PianoTable;
    buildPianoTable(*table);
    const KeyParams kp = table->keys[key - 1];
    delete table;
    const KeyModeSet ms = computeKeyModes(kp, fs);
    ModeBank bank;
    ContactTables contact;
    const int n = keyBankModes(ms);
    if (!bank.allocateSegments(&n, 1) || !contact.allocate(bank.capacity(), 1)) return false;
    loadKeyModes(bank, contact, 0, kp, ms, fs);
    bank.setSegmentDamper(0, 0.0f);
    const HammerParams hp = hammerParams(kp);
    std::vector<double> firstBuf, life;
    std::vector<float> buf(static_cast<std::size_t>(block));
    ScopedFlushDenormals ftz;
    for (int rep = 0; rep < reps; ++rep) {
        bank.clearState();
        HammerState hs;
        const HammerKeyModes m = contact.view(bank, 0);
        launchHammer(hs, hp, m, 2.2);
        double tot = 0.0;
        int b = 0;
        while (!hammerFinished(hs, m)) {
            const auto a = Clock::now();
            runHammer(hs, hp, m, buf.data(), block);
            const double dt = std::chrono::duration<double>(Clock::now() - a).count();
            if (b == 0) firstBuf.push_back(dt);
            tot += dt;
            ++b;
        }
        life.push_back(tot / std::max(b, 1));
    }
    std::sort(firstBuf.begin(), firstBuf.end());
    std::sort(life.begin(), life.end());
    out.key = key;
    out.strings = ms.strings;
    out.modes = ms.count;
    out.oversampling = kp.contactOversampling;
    out.contactUs = firstBuf.empty() ? 0.0 : 1e6 * firstBuf[firstBuf.size() / 2];
    out.lifeUs = 1e6 * life[life.size() / 2];
    return true;
}

}  // namespace pm::bench
