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

}  // namespace

void sceneEvents(Scene s, double fs, std::uint64_t end, std::vector<Event>& out) {
    out.clear();
    auto at = [fs](double seconds) { return static_cast<std::uint64_t>(seconds * fs); };
    const std::uint64_t period = at(0.5);
    auto add = [&](std::uint64_t t, EventType type, int note, int v7) {
        if (t < end) out.push_back(ev(t, type, note, v7));
    };
    switch (s) {
        case Scene::Worst: {
            add(0, EventType::Sustain, 0, 127);
            const std::uint64_t ms = std::max<std::uint64_t>(1, at(0.001));
            for (int n = 21; n <= 108; ++n) add(static_cast<std::uint64_t>(n - 21) * ms, EventType::NoteOn, n, 110);
            break;
        }
        case Scene::Typical: {
            const int notes[4] = {48, 60, 64, 67};
            for (std::uint64_t base = 0; base < end; base += period)
                for (int n : notes) {
                    add(base, EventType::NoteOn, n, 80);
                    add(base + period * 3 / 4, EventType::NoteOff, n, 64);
                }
            break;
        }
        case Scene::Cluster: {
            const std::uint64_t cp = at(0.1);
            for (std::uint64_t base = 0; base < end; base += cp) {
                const int first = (base / cp) % 2 ? 72 : 48;
                for (int n = first; n < first + 12; ++n) add(base, EventType::NoteOn, n, 120);
                for (int n = first; n < first + 12; ++n) add(base + cp / 2, EventType::NoteOff, n, 64);
            }
            break;
        }
        case Scene::Bloom: {
            add(0, EventType::Sustain, 0, 127);
            const int notes[4] = {48, 60, 64, 67};
            for (std::uint64_t base = 0; base < end; base += period)
                for (int n : notes) {
                    add(base, EventType::NoteOn, n, 90);
                    add(base + period / 5, EventType::NoteOff, n, 64);
                }
            break;
        }
        case Scene::Legato: {
            // Right hand: a scale up and down, a note every 180 ms held for
            // 250 ms (overlapping the next by 70 ms); left hand: a bass note
            // every 720 ms held for 650 ms. Pedal up, every note released
            // while it sounds.
            const int melody[14] = {60, 62, 64, 65, 67, 69, 71, 72, 71, 69, 67, 65, 64, 62};
            const int bass[4] = {36, 41, 43, 45};
            const std::uint64_t ioi = at(0.18), hold = at(0.25);
            for (std::uint64_t i = 0; i * ioi < end; ++i) {
                const std::uint64_t t = i * ioi;
                const int n = melody[i % 14];
                add(t, EventType::NoteOn, n, 70 + static_cast<int>((i * 7) % 26));
                add(t + hold, EventType::NoteOff, n, 64);
                if (i % 4 == 0) {
                    const int b = bass[(i / 4) % 4];
                    add(t, EventType::NoteOn, b, 75);
                    add(t + at(0.65), EventType::NoteOff, b, 50);
                }
            }
            break;
        }
        case Scene::Pedalling: {
            // Legato pedalling over a chord progression: at each chord change
            // the pedal comes up, the new chord is struck 10 ms later, the
            // pedal goes down 80 ms after that and the keys are released
            // halfway through the chord (the pedal holds the strings).
            // Chord lengths vary between 400 and 800 ms. On even chords
            // CC64 arrives as a ramp of many values (as a continuous pedal
            // sends it: down over 60 ms in 16 steps, up over 40 ms in 12
            // steps); on odd chords as plain 0 / 127.
            const int chords[4][5] = {{36, 48, 55, 60, 64}, {41, 53, 57, 60, 65}, {43, 55, 59, 62, 67}, {45, 57, 60, 64, 69}};
            const double lens[6] = {0.45, 0.70, 0.55, 0.80, 0.40, 0.65};
            double t = 0.0;
            for (int c = 0; at(t) < end; ++c) {
                const double len = lens[c % 6];
                const bool ramp = c % 2 == 0;
                if (c > 0) {
                    if (ramp)
                        for (int j = 1; j <= 12; ++j) add(at(t + 0.040 * (j - 1) / 11.0), EventType::Sustain, 0, 127 - 127 * j / 12);
                    else
                        add(at(t), EventType::Sustain, 0, 0);
                }
                const double on = t + 0.010, down = on + 0.080;
                for (int n : chords[c % 4]) add(at(on), EventType::NoteOn, n, 72 + 6 * (c % 4));
                if (ramp)
                    for (int j = 1; j <= 16; ++j) add(at(down + 0.060 * (j - 1) / 15.0), EventType::Sustain, 0, 127 * j / 16);
                else
                    add(at(down), EventType::Sustain, 0, 127);
                for (int n : chords[c % 4]) add(at(t + 0.5 * len), EventType::NoteOff, n, 64);
                t += len;
            }
            break;
        }
        case Scene::ChordRelease: {
            // Ten notes struck together and released together 1 s later,
            // every 1.5 s, pedal up.
            const int notes[10] = {36, 43, 48, 52, 55, 60, 64, 67, 72, 76};
            const std::uint64_t cp = at(1.5);
            for (std::uint64_t base = 0; base < end; base += cp)
                for (int n : notes) {
                    add(base, EventType::NoteOn, n, 100);
                    add(base + at(1.0), EventType::NoteOff, n, 64);
                }
            break;
        }
        case Scene::Idle:
            break;
    }
    std::stable_sort(out.begin(), out.end(), [](const Event& a, const Event& b) { return a.time < b.time; });
}

namespace {

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

// The realistic scenes are appended after the tier cases, so that the
// earlier cases keep their indices and output order.
constexpr int kBaseCases = 7, kTierCases = 6, kLiveCases = 4;

int caseCount(const Config& c) noexcept { return kBaseCases + (c.allTiers ? kTierCases : 0) + kLiveCases; }

Case caseAt(const Config& c, int i) noexcept {
    const QualityTier t = c.tier;
    const int live = kBaseCases + (c.allTiers ? kTierCases : 0);
    if (i >= live) {
        switch (i - live) {
            case 0: return {"legato (melody, note-offs, pedal up)", true, Scene::Legato, true, t};
            case 1: return {"pedalling (chords, CC64 ramps / on-off)", true, Scene::Pedalling, true, t};
            case 2: return {"release of a 10-note chord", true, Scene::ChordRelease, true, t};
            case 3: return {"idle (nothing sounding)", true, Scene::Idle, true, t};
            default: return {"?", true, Scene::Worst, true, t};
        }
    }
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
    sceneEvents(k.scene, fs, static_cast<std::uint64_t>(warmBlocks + blocks) * static_cast<std::uint64_t>(block), events);
    std::size_t next = 0;
    std::uint64_t t = 0;
    auto step = [&](double* dt) {
        const std::uint64_t t1 = t + static_cast<std::uint64_t>(block);
        for (; next < events.size() && events[next].time < t1; ++next) eng.pushEvent(events[next]);
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
