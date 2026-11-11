// Plain-text score for the piano mode of render_cli. One event per line:
//
//   <time s> note_on  <key> <velocity 1..127>
//   <time s> note_off <key> [release velocity 1..127]
//   <time s> key_down <key>             key pressed silently (damper lifted, no strike)
//   <time s> cc64     <value 0..127>      sustain
//   <time s> cc66     <value 0..127>      sostenuto
//   <time s> cc67     <value 0..127>      una corda (continuous)
//   <time s> all_notes_off
//   <time s> panic
//
// <key> is a MIDI note number (21..108) or a name such as A0, C4, F#3, Bb5.
// '#' starts a comment. Events are sorted by time (stable).
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "pm_engine/events.h"

namespace pm_tools {

struct ScoreEvent {
    double seconds;
    pm::Event event;
};

// MIDI note number from "60" or "C4"; -1 if invalid.
inline int parseNote(const std::string& s) {
    if (s.empty()) return -1;
    if (std::isdigit(static_cast<unsigned char>(s[0]))) {
        char* end = nullptr;
        const long v = std::strtol(s.c_str(), &end, 10);
        return (*end == '\0' && v >= 0 && v <= 127) ? static_cast<int>(v) : -1;
    }
    static const int base[7] = {9, 11, 0, 2, 4, 5, 7};  // A B C D E F G
    const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
    if (c < 'A' || c > 'G') return -1;
    int semi = base[c - 'A'];
    std::size_t i = 1;
    if (i < s.size() && (s[i] == '#' || s[i] == 'b')) semi += s[i++] == '#' ? 1 : -1;
    char* end = nullptr;
    const long oct = std::strtol(s.c_str() + i, &end, 10);
    if (end == s.c_str() + i || *end != '\0') return -1;
    const long n = 12 * (oct + 1) + semi;
    return (n >= 0 && n <= 127) ? static_cast<int>(n) : -1;
}

// Returns false and prints the offending line on a syntax error.
inline bool readScore(const char* path, double fs, std::vector<ScoreEvent>& out) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open score %s\n", path);
        return false;
    }
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        std::istringstream ls(line);
        double t;
        std::string kind;
        if (!(ls >> t)) {
            std::string rest;
            if (std::istringstream(line) >> rest) goto bad;
            continue;  // blank
        }
        if (!(ls >> kind) || t < 0.0) goto bad;
        {
            ScoreEvent se{t, pm::Event{}};
            pm::Event& e = se.event;
            e.timeBase = pm::TimeBase::SampleTime;
            e.time = static_cast<std::uint64_t>(std::llround(t * fs));
            std::string a;
            int b = -1;
            if (kind == "key_down") {
                if (!(ls >> a)) goto bad;
                const int note = parseNote(a);
                if (note < 0) goto bad;
                e.note = static_cast<std::uint8_t>(note);
                e.type = pm::EventType::KeyDownSilent;
            } else if (kind == "note_on" || kind == "note_off") {
                if (!(ls >> a)) goto bad;
                const int note = parseNote(a);
                if (note < 0) goto bad;
                e.note = static_cast<std::uint8_t>(note);
                e.type = kind == "note_on" ? pm::EventType::NoteOn : pm::EventType::NoteOff;
                if (ls >> b) {
                    if (b < 0 || b > 127) goto bad;
                    e.value = pm::value16FromMidi7(b);
                } else if (kind == "note_on") {
                    goto bad;
                }
            } else if (kind == "cc64" || kind == "cc66" || kind == "cc67") {
                if (!(ls >> b) || b < 0 || b > 127) goto bad;
                e.type = kind == "cc64" ? pm::EventType::Sustain
                         : kind == "cc66" ? pm::EventType::Sostenuto
                                          : pm::EventType::UnaCorda;
                e.value = pm::value16FromMidi7(b);
            } else if (kind == "all_notes_off") {
                e.type = pm::EventType::AllNotesOff;
            } else if (kind == "panic") {
                e.type = pm::EventType::Panic;
            } else {
                goto bad;
            }
            std::string extra;
            if (ls >> extra) goto bad;
            out.push_back(se);
        }
        continue;
    bad:
        std::fprintf(stderr, "%s:%d: cannot parse: %s\n", path, lineNo, line.c_str());
        return false;
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const ScoreEvent& x, const ScoreEvent& y) { return x.seconds < y.seconds; });
    return true;
}

}  // namespace pm_tools
