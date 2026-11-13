#include "pm_bench/bench.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <new>
#include <thread>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_host/speed_probe.h"
#include "pm_voicing/key_loader.h"

namespace pm::bench {

namespace {

using Clock = std::chrono::steady_clock;
volatile unsigned gEvictSink = 0;

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
        case Scene::HammerBurst: {
            const int notes[10] = {24, 29, 33, 36, 40, 43, 48, 52, 55, 60};
            const std::uint64_t cp = at(1.0);
            for (std::uint64_t base = 0; base < end; base += cp)
                for (int n : notes) {
                    add(base, EventType::NoteOn, n, 127);
                    add(base + cp / 2, EventType::NoteOff, n, 64);
                }
            break;
        }
        case Scene::PedalChords: {
            // At each change: pedal up, the new chord 10 ms later, pedal
            // down 80 ms after that, keys released halfway through.
            const int chords[8][5] = {{28, 40, 47, 56, 64}, {33, 45, 52, 61, 69}, {26, 38, 45, 54, 62},
                                      {31, 43, 50, 59, 67}, {36, 48, 55, 64, 71}, {29, 41, 48, 57, 65},
                                      {34, 46, 53, 62, 70}, {24, 36, 43, 52, 60}};
            const double len = 0.7;
            for (int c = 0; at(c * len) < end; ++c) {
                const double t = c * len;
                if (c > 0) add(at(t), EventType::Sustain, 0, 0);
                for (int j = 0; j < 5; ++j) add(at(t + 0.010), EventType::NoteOn, chords[c % 8][j], 60 + 9 * ((c + j) % 5));
                add(at(t + 0.090), EventType::Sustain, 0, 127);
                for (int n : chords[c % 8]) add(at(t + 0.5 * len), EventType::NoteOff, n, 64);
            }
            break;
        }
        case Scene::FreshBurst:
        case Scene::SameBurst: {
            // Pedal down; a bass note and a chord held (keys down) throughout.
            // Every 0.6 s six notes at 100-127 in one sample, released
            // 0.25 s later, two of them (staccato) already after 30 ms.
            // Every 2.4 s a legato pedal change (up 10 ms before the burst,
            // down 80 ms after it): those bursts' staccato notes are damped
            // while their hammers are still active. Fresh: keys drawn
            // (xorshift, fixed seed) from those not struck for 6 s; same:
            // the first burst's keys every time.
            const int held[4] = {29, 53, 57, 60};
            add(0, EventType::Sustain, 0, 127);
            for (int n : held) add(0, EventType::NoteOn, n, 70);
            std::uint32_t rng = 0x2545F491u;
            auto next = [&rng]() {
                rng ^= rng << 13;
                rng ^= rng >> 17;
                rng ^= rng << 5;
                return rng;
            };
            double lastStrike[128];
            for (double& x : lastStrike) x = -1e9;
            int first[6] = {};
            const double burstPeriod = 0.6;
            for (int b = 0; at(0.3 + b * burstPeriod) < end; ++b) {
                const double t = 0.3 + b * burstPeriod;
                const bool pedalChange = b % 4 == 0 && b > 0;
                if (pedalChange) {
                    add(at(t - 0.010), EventType::Sustain, 0, 0);
                    add(at(t + 0.080), EventType::Sustain, 0, 127);
                }
                int keys[6];
                for (int j = 0; j < 6; ++j) {
                    if (s == Scene::SameBurst && b > 0) {
                        keys[j] = first[j];
                        continue;
                    }
                    int n;
                    for (int tries = 0;; ++tries) {
                        n = 21 + static_cast<int>(next() % 88u);
                        bool ok = t - lastStrike[n] >= 6.0 || tries > 1000;
                        for (int h : held) ok = ok && n != h;
                        for (int i = 0; i < j; ++i) ok = ok && n != keys[i];
                        if (ok) break;
                    }
                    keys[j] = n;
                }
                if (b == 0)
                    for (int j = 0; j < 6; ++j) first[j] = keys[j];
                for (int j = 0; j < 6; ++j) {
                    lastStrike[keys[j]] = t;
                    add(at(t), EventType::NoteOn, keys[j], 100 + static_cast<int>(next() % 28u));
                    add(at(t + (j < 2 ? 0.030 : 0.25)), EventType::NoteOff, keys[j], 64);
                }
            }
            break;
        }
        case Scene::PedalLift: {
            // Pedal down; four keys held throughout (re-struck every 2.4 s);
            // a 12-note chord over both hands every 1.2 s, released 0.2 s
            // later into the pedal. Every 0.6 s, from 0.9 s, the pedal comes
            // up for 150 ms and goes down again; three notes are struck 20 ms
            // after it starts down. Odd changes send CC64 as plain 0 / 127,
            // even ones as ramps (up over 40 ms in 12 steps, down over 60 ms
            // in 16 steps).
            const int held[4] = {33, 45, 52, 57};
            const int chord[12] = {28, 35, 40, 47, 55, 59, 64, 67, 71, 76, 79, 88};
            const int lead[6][3] = {{60, 72, 84}, {62, 74, 86}, {58, 70, 82}, {65, 77, 89}, {63, 75, 87}, {61, 73, 85}};
            add(0, EventType::Sustain, 0, 127);
            for (std::uint64_t b = 0; b < end; b += at(2.4))
                for (int n : held) add(b, EventType::NoteOn, n, 80);
            for (int c = 0; at(1.2 * c) < end; ++c)
                for (int j = 0; j < 12; ++j) {
                    const double t = 1.2 * c + 0.004 * j;
                    add(at(t), EventType::NoteOn, chord[j], 75 + (5 * j + 11 * c) % 40);
                    add(at(t + 0.2), EventType::NoteOff, chord[j], 64);
                }
            for (int c = 0; at(0.9 + 0.6 * c) < end; ++c) {
                const double up = 0.9 + 0.6 * c, down = up + 0.15;
                if (c % 2) {
                    add(at(up), EventType::Sustain, 0, 0);
                    add(at(down), EventType::Sustain, 0, 127);
                } else {
                    for (int j = 1; j <= 12; ++j) add(at(up + 0.040 * (j - 1) / 11.0), EventType::Sustain, 0, 127 - 127 * j / 12);
                    for (int j = 1; j <= 16; ++j) add(at(down + 0.060 * (j - 1) / 15.0), EventType::Sustain, 0, 127 * j / 16);
                }
                for (int n : lead[c % 6]) {
                    add(at(down + 0.020), EventType::NoteOn, n, 85 + 7 * (c % 4));
                    add(at(down + 0.35), EventType::NoteOff, n, 64);
                }
            }
            break;
        }
        case Scene::Idle:
            break;
    }
    std::stable_sort(out.begin(), out.end(), [](const Event& a, const Event& b) { return a.time < b.time; });
}

namespace {

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

BufferDetail detailOf(const RenderInfo& i) {
    BufferDetail d;
    d.hammersInContact = i.hammersInContact;
    d.maxHammersInContact = i.maxHammersInContact;
    d.hammerContactSamples = i.hammerContactSamples;
    d.newtonEvals = i.newtonEvals;
    d.fallbackSolves = i.fallbackSolves;
    d.kernelRebuilds = i.kernelRebuilds;
    d.kernelKeyUpdates = i.kernelKeyUpdates;
    d.responseGroups = i.responseGroups;
    d.damperUpdates = i.damperUpdates;
    d.approxResponses = i.approxResponses;
    d.hammerSamples = i.hammerSamples;
    d.hammerDampedSamples = i.hammerDampedSamples;
    d.awakeModes = i.awakeModes;
    d.freeGroupBlocks = i.freeGroupBlocks;
    d.forcedGroupBlocks = i.forcedGroupBlocks;
    d.loopMembers = i.loopMembers;
    d.loopListeners = i.loopListeners;
    d.loopEntries = i.loopEntries;
    d.outZones = i.outZones;
    d.noteOns = i.noteOns;
    d.noteOffs = i.noteOffs;
    d.pedalEvents = i.pedalEvents;
    return d;
}

void accumulate(Phases& acc, const Phases& p, double w) {
    for (int i = 0; i < kPhaseCount; ++i) acc.us[i] += w * p.us[i];
    acc.total += w * p.total;
}

}  // namespace

// The realistic scenes are appended after the tier cases, so that the
// earlier cases keep their indices and output order.
constexpr int kBaseCases = 7, kTierCases = 6, kLiveCases = 9;

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
            case 4: return {"hammer burst (10 notes at 127, one sample)", true, Scene::HammerBurst, true, t};
            case 5: return {"pedalled chords (8 roots, CC64 on/off)", true, Scene::PedalChords, true, t};
            case 6: return {"fresh-key burst (6 new keys, pedal)", true, Scene::FreshBurst, true, t};
            case 7: return {"same-key burst (6 same keys, pedal)", true, Scene::SameBurst, true, t};
            case 8: return {"pedal lift (pedal up/down, strings sounding)", true, Scene::PedalLift, true, t};
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

bool prepareForCase(const Case& k, Engine& eng, double sampleRate, int maxFrames, std::size_t* bytes) {
    eng.setCulling(k.cull, -100.0f);
    SoundboardConfig bc;
    bc.enabled = k.board;
    bc.tier = k.tier;
    eng.setSoundboardConfig(bc);
    const std::size_t before = alignedLiveBytes();
    const bool ok = eng.prepare(sampleRate, maxFrames);
    if (bytes) {
        const std::size_t after = alignedLiveBytes();
        *bytes = sizeof(Engine) + (after > before ? after - before : 0);
    }
    return ok;
}

bool runScenario(const Config& c, const Case& k, Engine& eng, ScenarioResult& res) {
    res = ScenarioResult{};
    const double fs = c.sampleRate;
    const int block = std::max(1, c.block);
    if (!prepareForCase(k, eng, fs, block)) return false;
    const bool timing = c.phaseTiming;
    eng.setPhaseTiming(timing);
    // Load multiplier: the other engines, prepared alike (same work, phase
    // timing included). The first of them gives the memory per engine (a
    // re-prepare of 'eng' frees its arrays as it goes, so it cannot).
    const int load = std::clamp(c.load, 1, kMaxLoad);
    res.load = load;
    std::vector<std::unique_ptr<Engine>> extra;
    for (int i = 1; i < load; ++i) {
        extra.emplace_back(new (std::nothrow) Engine);
        if (!extra.back() || !prepareForCase(k, *extra.back(), fs, block, i == 1 ? &res.engineBytes : nullptr))
            return false;
        extra.back()->setPhaseTiming(timing);
    }
    std::vector<float> xl(static_cast<std::size_t>(block)), xr(static_cast<std::size_t>(block));
    // The worst cases are measured from the first strike, so the max
    // includes the burst of 88 hammers (1 ms apart, pool of 32).
    const double warm = k.scene == Scene::Worst ? 0.0 : 0.5;

    std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
    std::vector<Event> events;
    res.paced = c.paced;
    res.spin = c.paced && c.spin;
    res.evictBytes = c.paced ? c.evictBytes : 0;
    std::vector<unsigned char> scratch(res.evictBytes);
    for (std::size_t i = 0; i < scratch.size(); i += 4096) scratch[i] = 1;  // fault the pages in now
    const int warmBlocks = static_cast<int>(warm * fs / block);
    const int blocks = std::max(1, static_cast<int>(c.seconds * fs / block));
    sceneEvents(k.scene, fs, static_cast<std::uint64_t>(warmBlocks + blocks) * static_cast<std::uint64_t>(block), events);
    std::size_t next = 0;
    std::uint64_t t = 0;
    // Paced: buffer b is due at start + (b + 1) * period (absolute, so an
    // overrun is not made up by stretching the schedule).
    const auto periodDur = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(block / fs));
    Clock::time_point deadline = Clock::now();
    unsigned evictSink = 0;
    std::vector<double> probes, probeLat, probeThr;
    int stepCount = 0;
    const bool hook = c.paced && c.paceHook;
    int cpu0 = -1, cpu1 = -1;
    auto step = [&](double* dt) {
        const std::uint64_t t1 = t + static_cast<std::uint64_t>(block);
        for (; next < events.size() && events[next].time < t1; ++next) {
            eng.pushEvent(events[next]);
            for (auto& e : extra) e->pushEvent(events[next]);
        }
        if (dt && c.probeEvery > 0 && ++stepCount % c.probeEvery == 0) {
            float sink = 0.0f;
            const int cpu = host::currentCpu();
            const host::ProbeTimes pt = host::timeSpeedProbeSplit(&sink);
            probes.push_back(pt.totalUs());
            probeLat.push_back(pt.latencyUs);
            probeThr.push_back(pt.throughputUs);
            res.cpu.addProbe(cpu, pt.latencyUs, pt.throughputUs);
            gEvictSink = gEvictSink + static_cast<unsigned>(sink > 0.0f);
        }
        if (hook)
            c.paceHook(c.paceCtx, 1,
                       std::chrono::duration_cast<std::chrono::nanoseconds>(deadline + periodDur - Clock::now()).count());
        if (dt) cpu0 = host::currentCpu();
        const auto a = Clock::now();
        eng.render(l.data(), r.data(), block, nullptr);
        for (auto& e : extra) e->render(xl.data(), xr.data(), block, nullptr);
        if (dt) {
            *dt = std::chrono::duration<double>(Clock::now() - a).count();
            cpu1 = host::currentCpu();
        }
        if (hook) c.paceHook(c.paceCtx, 2, 0);
        t += static_cast<std::uint64_t>(block);
        if (c.paced) {
            // Other work on this core between buffers: one read-modify-write
            // per cache line of a buffer larger than the caches in question.
            for (std::size_t i = 0; i < scratch.size(); i += 64) evictSink += ++scratch[i];
            deadline += periodDur;
            if (c.spin)
                while (Clock::now() < deadline) {}
            else
                std::this_thread::sleep_until(deadline);
        }
    };
    if (hook) {
        c.paceHook(c.paceCtx, 0, std::chrono::duration_cast<std::chrono::nanoseconds>(periodDur).count());
        deadline = Clock::now();  // the hook may take a while (thread policy, workgroup join)
    }
    for (int b = 0; b < warmBlocks; ++b) step(nullptr);
    const double period = block / fs;
    res.periodUs = 1e6 * period;
    res.buffers = blocks;
    const int firstBlocks = std::max(1, static_cast<int>(0.1 * fs / block));
    std::vector<double> dts(static_cast<std::size_t>(blocks));
    std::vector<Phases> ph(static_cast<std::size_t>(blocks));
    double total = 0.0, first = 0.0, awake = 0.0, worstDt = -1.0;
    long rebuilds = 0, keyUpdates = 0;
    double freeBlocks = 0.0, forcedBlocks = 0.0, members = 0.0, listeners = 0.0;
    for (int b = 0; b < blocks; ++b) {
        step(&dts[b]);
        res.cpu.addBuffer(cpu0, cpu1, 1e6 * dts[b]);
        if (dts[b] > period) ++res.overruns;
        if (timing) ph[b] = phasesOf(eng.profile());
        else ph[b].total = 1e6 * dts[b];
        const RenderInfo& info = eng.lastRender();
        rebuilds += info.kernelRebuilds;
        keyUpdates += info.kernelKeyUpdates;
        res.maxDamperUpdates = std::max(res.maxDamperUpdates, info.damperUpdates);
        res.maxResponseGroups = std::max(res.maxResponseGroups, info.responseGroups);
        res.maxApproxResponses = std::max(res.maxApproxResponses, info.approxResponses);
        freeBlocks += info.freeGroupBlocks;
        forcedBlocks += info.forcedGroupBlocks;
        members += info.loopMembers;
        listeners += info.loopListeners;
        res.hammerSamples += info.hammerSamples;
        res.hammerContactSamples += info.hammerContactSamples;
        res.hammerDampedSamples += info.hammerDampedSamples;
        if (timing) res.hammerPhaseUs += ph[b].us[EngineProfile::Hammers];
        if (dts[b] > worstDt) {
            worstDt = dts[b];
            res.maxDetail = detailOf(info);
            res.cpu.worstStart = cpu0;
            res.cpu.worstEnd = cpu1;
        }
        total += dts[b];
        if (b < firstBlocks) first += dts[b];
        awake += eng.stats().awakeModes;
    }
    res.meanAwakeModes = awake / blocks;
    res.freeGroupBlocksPerBuffer = freeBlocks / blocks;
    res.forcedGroupBlocksPerBuffer = forcedBlocks / blocks;
    res.loopMembersMean = members / blocks;
    res.loopListenersMean = listeners / blocks;
    if (res.hammerSamples > 0) res.hammerUsPerSample = res.hammerPhaseUs / static_cast<double>(res.hammerSamples);
    if (res.hammerContactSamples > 0)
        res.hammerUsPerContactSample = res.hammerPhaseUs / static_cast<double>(res.hammerContactSamples);
    gEvictSink = evictSink;  // keeps the eviction walk observable
    if (hook) c.paceHook(c.paceCtx, 3, 0);
    if (!probes.empty()) {
        res.probeRuns = static_cast<int>(probes.size());
        double sum = 0.0;
        for (double x : probes) sum += x;
        res.probeMeanUs = sum / probes.size();
        std::sort(probes.begin(), probes.end());
        res.probeMedianUs = probes[probes.size() / 2];
        auto stats = [](std::vector<double>& v, ScenarioResult::ProbeStats& o) {
            std::sort(v.begin(), v.end());
            double s = 0.0;
            for (double x : v) s += x;
            o.runs = static_cast<int>(v.size());
            o.minUs = v.front();
            o.maxUs = v.back();
            o.medianUs = v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
            o.meanUs = s / static_cast<double>(v.size());
        };
        stats(probeLat, res.probeLatency);
        stats(probeThr, res.probeThroughput);
    }
    res.kernelRebuildsPerBuffer = static_cast<double>(rebuilds) / blocks;
    res.kernelKeyUpdatesPerBuffer = static_cast<double>(keyUpdates) / blocks;
    res.first100msPercent = 100.0 * first / (std::min(firstBlocks, blocks) * period);
    if (timing) {
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
    }
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
