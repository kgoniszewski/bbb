#include "pm_api/preset.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "pm_api/pm_api.h"
#include "pm_api/pm_api_internal.h"
#include "pm_api/param_table.h"

namespace pm::json {

namespace {

// Length of the valid UTF-8 sequence at p (1..4), or 0 if it is invalid
// (overlong forms, surrogates and code points above U+10FFFF included).
int utf8Len(const unsigned char* p, const unsigned char* e) noexcept {
    const unsigned c = p[0];
    if (c < 0x80) return 1;
    int n = 0;
    unsigned lo = 0x80, hi = 0xBF;
    if (c >= 0xC2 && c <= 0xDF) n = 2;
    else if (c >= 0xE0 && c <= 0xEF) {
        n = 3;
        if (c == 0xE0) lo = 0xA0;
        if (c == 0xED) hi = 0x9F;
    } else if (c >= 0xF0 && c <= 0xF4) {
        n = 4;
        if (c == 0xF0) lo = 0x90;
        if (c == 0xF4) hi = 0x8F;
    } else
        return 0;
    if (e - p < n) return 0;
    if (p[1] < lo || p[1] > hi) return 0;
    for (int i = 2; i < n; ++i)
        if (p[i] < 0x80 || p[i] > 0xBF) return 0;
    return n;
}

void appendUtf8(std::string& s, unsigned cp) {
    if (cp < 0x80) {
        s += static_cast<char>(cp);
    } else if (cp < 0x800) {
        s += static_cast<char>(0xC0 | (cp >> 6));
        s += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        s += static_cast<char>(0xE0 | (cp >> 12));
        s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        s += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        s += static_cast<char>(0xF0 | (cp >> 18));
        s += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        s += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

struct Parser {
    const char* b;
    const char* p;
    const char* e;
    std::string error;
    std::size_t offset = 0;

    bool fail(const char* at, const char* msg) {
        if (error.empty()) {
            error = msg;
            offset = static_cast<std::size_t>(at - b);
        }
        return false;
    }
    void ws() {
        while (p < e && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) ++p;
    }
    bool hex4(unsigned& cp) {
        if (e - p < 4) return fail(p, "truncated \\u escape");
        cp = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = p[i];
            unsigned d;
            if (c >= '0' && c <= '9') d = static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') d = static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') d = static_cast<unsigned>(c - 'A' + 10);
            else return fail(p + i, "bad hex digit in \\u escape");
            cp = cp * 16 + d;
        }
        p += 4;
        return true;
    }
    bool string(std::string& s) {
        ++p;  // opening quote
        for (;;) {
            if (p >= e) return fail(p, "unterminated string");
            const unsigned char c = static_cast<unsigned char>(*p);
            if (c == '"') {
                ++p;
                return true;
            }
            if (c < 0x20) return fail(p, "control character in string");
            if (c == '\\') {
                const char* at = p;
                if (++p >= e) return fail(p, "unterminated escape");
                const char x = *p++;
                switch (x) {
                    case '"': s += '"'; break;
                    case '\\': s += '\\'; break;
                    case '/': s += '/'; break;
                    case 'b': s += '\b'; break;
                    case 'f': s += '\f'; break;
                    case 'n': s += '\n'; break;
                    case 'r': s += '\r'; break;
                    case 't': s += '\t'; break;
                    case 'u': {
                        unsigned cp = 0;
                        if (!hex4(cp)) return false;
                        if (cp >= 0xDC00 && cp <= 0xDFFF) return fail(at, "lone low surrogate");
                        if (cp >= 0xD800 && cp <= 0xDBFF) {
                            if (e - p < 6 || p[0] != '\\' || p[1] != 'u') return fail(at, "lone high surrogate");
                            p += 2;
                            unsigned lo = 0;
                            if (!hex4(lo)) return false;
                            if (lo < 0xDC00 || lo > 0xDFFF) return fail(at, "bad surrogate pair");
                            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                        }
                        if (cp == 0) return fail(at, "\\u0000 is not allowed");
                        appendUtf8(s, cp);
                        break;
                    }
                    default: return fail(at, "bad escape");
                }
                continue;
            }
            const int n = utf8Len(reinterpret_cast<const unsigned char*>(p), reinterpret_cast<const unsigned char*>(e));
            if (n == 0) return fail(p, "invalid UTF-8");
            s.append(p, static_cast<std::size_t>(n));
            p += n;
        }
    }
    static bool digit(char c) { return c >= '0' && c <= '9'; }
    bool number(double& v) {
        const char* start = p;
        if (p < e && *p == '-') ++p;
        if (p >= e || !digit(*p)) return fail(p, "bad number");
        if (*p == '0') ++p;
        else
            while (p < e && digit(*p)) ++p;
        if (p < e && *p == '.') {
            ++p;
            if (p >= e || !digit(*p)) return fail(p, "bad number");
            while (p < e && digit(*p)) ++p;
        }
        if (p < e && (*p == 'e' || *p == 'E')) {
            ++p;
            if (p < e && (*p == '+' || *p == '-')) ++p;
            if (p >= e || !digit(*p)) return fail(p, "bad number");
            while (p < e && digit(*p)) ++p;
        }
        // strtod needs a terminated copy (the input need not be terminated).
        const std::string text(start, static_cast<std::size_t>(p - start));
        v = std::strtod(text.c_str(), nullptr);
        if (!std::isfinite(v)) return fail(start, "number out of range");
        return true;
    }
    bool literal(const char* word) {
        const std::size_t n = std::strlen(word);
        if (static_cast<std::size_t>(e - p) < n || std::memcmp(p, word, n) != 0) return fail(p, "bad literal");
        p += n;
        return true;
    }
    bool value(Value& v, int depth) {
        if (depth > kMaxDepth) return fail(p, "nesting too deep");
        ws();
        if (p >= e) return fail(p, "unexpected end of input");
        switch (*p) {
            case '{': {
                v.type = Value::Object;
                ++p;
                ws();
                if (p < e && *p == '}') {
                    ++p;
                    return true;
                }
                for (;;) {
                    ws();
                    if (p >= e || *p != '"') return fail(p, "expected a key string");
                    Member m;
                    if (!string(m.first)) return false;
                    ws();
                    if (p >= e || *p != ':') return fail(p, "expected ':'");
                    ++p;
                    if (!value(m.second, depth + 1)) return false;
                    v.obj.push_back(std::move(m));
                    ws();
                    if (p < e && *p == ',') {
                        ++p;
                        continue;
                    }
                    if (p < e && *p == '}') {
                        ++p;
                        return true;
                    }
                    return fail(p, "expected ',' or '}'");
                }
            }
            case '[': {
                v.type = Value::Array;
                ++p;
                ws();
                if (p < e && *p == ']') {
                    ++p;
                    return true;
                }
                for (;;) {
                    v.arr.emplace_back();
                    if (!value(v.arr.back(), depth + 1)) return false;
                    ws();
                    if (p < e && *p == ',') {
                        ++p;
                        continue;
                    }
                    if (p < e && *p == ']') {
                        ++p;
                        return true;
                    }
                    return fail(p, "expected ',' or ']'");
                }
            }
            case '"': v.type = Value::String; return string(v.str);
            case 't': v.type = Value::Bool; v.b = true; return literal("true");
            case 'f': v.type = Value::Bool; v.b = false; return literal("false");
            case 'n': v.type = Value::Null; return literal("null");
            default:
                if (*p == '-' || digit(*p)) {
                    v.type = Value::Number;
                    return number(v.num);
                }
                return fail(p, "unexpected character");
        }
    }
};

}  // namespace

const Value* Value::find(const char* key) const noexcept {
    const Value* found = nullptr;
    for (const auto& m : obj)
        if (m.first == key) found = &m.second;
    return found;
}

bool parse(const char* text, std::size_t len, Value& out, std::string& error, std::size_t& offset) {
    out = Value{};
    if (!text) {
        error = "no input";
        offset = 0;
        return false;
    }
    Parser ps{text, text, text + len, {}, 0};
    bool ok = ps.value(out, 1);
    if (ok) {
        ps.ws();
        if (ps.p != ps.e) ok = ps.fail(ps.p, "trailing characters after the document");
    }
    if (!ok) {
        error = ps.error;
        offset = ps.offset;
    }
    return ok;
}

void appendNumber(std::string& s, double v) {
    char buf[40];
    // Integral values as integers (-100, not -1e+02); exact below 2^53.
    if (v == std::floor(v) && std::fabs(v) < 9007199254740992.0 && !(v == 0.0 && std::signbit(v))) {
        std::snprintf(buf, sizeof buf, "%lld", static_cast<long long>(v));
        s += buf;
        return;
    }
    for (int prec = 1; prec <= 17; ++prec) {
        std::snprintf(buf, sizeof buf, "%.*g", prec, v);
        if (std::strtod(buf, nullptr) == v) break;
    }
    s += buf;
}

void appendString(std::string& s, const char* text) {
    s += '"';
    const auto* p = reinterpret_cast<const unsigned char*>(text ? text : "");
    const unsigned char* e = p + std::strlen(reinterpret_cast<const char*>(p));
    while (p < e) {
        const unsigned c = *p;
        if (c == '"' || c == '\\') {
            s += '\\';
            s += static_cast<char>(c);
        } else if (c == '\n') s += "\\n";
        else if (c == '\t') s += "\\t";
        else if (c == '\r') s += "\\r";
        else if (c < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof buf, "\\u%04x", c);
            s += buf;
        } else {
            const int n = utf8Len(p, e);
            if (n == 0) {
                s += "\xEF\xBF\xBD";  // U+FFFD for an invalid byte
                ++p;
                continue;
            }
            s.append(reinterpret_cast<const char*>(p), static_cast<std::size_t>(n));
            p += n;
            continue;
        }
        ++p;
    }
    s += '"';
}

}  // namespace pm::json

namespace {

using pm::json::Value;

struct Report {
    pm_preset_result_t& r;
    void warn(const std::string& w) {
        const std::size_t cap = sizeof r.warnings - 1;
        std::size_t len = std::strlen(r.warnings);
        if (len >= cap - 3) return;
        std::string add = (len ? "; " : "") + w;
        if (len + add.size() > cap - 3) add = add.substr(0, cap - 3 - len) + "...";
        std::memcpy(r.warnings + len, add.data(), add.size());
        r.warnings[len + add.size()] = '\0';
    }
    bool error(const char* msg, long long offset = -1) {
        std::snprintf(r.error, sizeof r.error, "%s", msg);
        r.error_offset = offset;
        r.ok = 0;
        return false;
    }
};

// v0 (hypothetical pre-release format): "params":[{"id":n,"value":x},...].
// Ids are stable forever, so each maps to its key; an unknown or invalid id
// becomes a key no parameter has, and is reported as unknown when applied.
bool migrateV0toV1(Value& root, Report& rep) {
    Value* params = nullptr;
    for (auto& m : root.obj)
        if (m.first == "params") params = &m.second;
    if (params) {
        if (params->type != Value::Array) return rep.error("v0: params is not an array");
        Value obj;
        obj.type = Value::Object;
        for (const Value& entry : params->arr) {
            const Value* id = entry.type == Value::Object ? entry.find("id") : nullptr;
            const Value* val = entry.type == Value::Object ? entry.find("value") : nullptr;
            std::string key = "#invalid_v0_entry";
            if (id && id->type == Value::Number && id->num == std::floor(id->num) && std::fabs(id->num) < 1e9) {
                const auto n = static_cast<std::int32_t>(id->num);
                key = pm::paramDescriptor(n) ? pm::paramDescriptor(n)->key : "#v0_id_" + std::to_string(n);
            }
            obj.obj.push_back(pm::json::Member{std::move(key), val ? *val : Value{}});
        }
        *params = std::move(obj);
    }
    for (auto& m : root.obj)
        if (m.first == "version") m.second.num = 1.0;
    return true;
}

using Migration = bool (*)(Value&, Report&);
// kMigrations[v] turns version v into v + 1.
constexpr Migration kMigrations[] = {migrateV0toV1};
static_assert(sizeof(kMigrations) / sizeof(kMigrations[0]) == PM_PRESET_VERSION, "one migration per version");

std::string writePreset(const pm_engine* engine, const char* name, std::int32_t flags) {
    std::string s = "{\"format\":\"pm-preset\",\"version\":";
    s += std::to_string(PM_PRESET_VERSION);
    s += ",\"name\":";
    pm::json::appendString(s, name);
    s += ",\"api\":\"" + std::to_string(PM_API_VERSION_MAJOR) + "." + std::to_string(PM_API_VERSION_MINOR) + "." +
         std::to_string(PM_API_VERSION_PATCH) + "\",\"params\":{";
    bool first = true;
    for (std::int32_t i = 0; i < pm_param_count(); ++i) {
        pm_param_info_t d{};
        pm_param_info(i, &d);
        if ((d.flags & PM_PARAM_FLAG_HIDDEN) && !(flags & PM_PRESET_WRITE_FULL_STATE)) continue;
        double v = 0.0;
        pm_engine_get_param(engine, d.id, &v);
        if (!first) s += ',';
        first = false;
        pm::json::appendString(s, d.key);
        s += ':';
        if (d.curve == PM_PARAM_CURVE_INTEGER || d.curve == PM_PARAM_CURVE_BOOLEAN)
            s += std::to_string(static_cast<long long>(std::llround(v)));
        else
            pm::json::appendNumber(s, v);
    }
    s += "}}";
    return s;
}

std::size_t copyOut(const std::string& s, char* buf, std::size_t cap) {
    if (buf && cap > 0) {
        const std::size_t n = s.size() < cap - 1 ? s.size() : cap - 1;
        std::memcpy(buf, s.data(), n);
        buf[n] = '\0';
    }
    return s.size();
}

bool readPreset(pm_engine* engine, const char* json, std::size_t len, std::int32_t flags, pm_preset_result_t& r) {
    r = pm_preset_result_t{};
    r.error_offset = -1;
    r.version = -1;
    Report rep{r};
    if (!engine) return rep.error("no engine");
    Value root;
    std::string err;
    std::size_t off = 0;
    if (!pm::json::parse(json, len, root, err, off)) return rep.error(("syntax: " + err).c_str(), static_cast<long long>(off));
    if (root.type != Value::Object) return rep.error("the document is not a JSON object", 0);
    const Value* format = root.find("format");
    if (!format || format->type != Value::String || format->str != "pm-preset")
        return rep.error("not a pm-preset document (\"format\" must be \"pm-preset\")");
    const Value* version = root.find("version");
    if (!version || version->type != Value::Number || version->num != std::floor(version->num) || version->num < 0)
        return rep.error("missing or invalid \"version\"");
    if (version->num > PM_PRESET_VERSION) {
        char msg[96];
        std::snprintf(msg, sizeof msg, "preset version %.0f is newer than supported (%d)", version->num,
                      PM_PRESET_VERSION);
        return rep.error(msg);
    }
    r.version = static_cast<std::int32_t>(version->num);
    for (int v = r.version; v < PM_PRESET_VERSION; ++v)
        if (!kMigrations[v](root, rep)) return false;

    const Value* params = nullptr;
    for (const auto& m : root.obj) {
        if (m.first == "format" || m.first == "version" || m.first == "api") continue;
        if (m.first == "params") {
            params = &m.second;
        } else if (m.first == "name") {
            if (m.second.type != Value::String) {
                rep.warn("\"name\" is not a string");
                continue;
            }
            std::size_t n = m.second.str.size() < sizeof r.name - 1 ? m.second.str.size() : sizeof r.name - 1;
            // Truncate at a character boundary.
            while (n < m.second.str.size() && n > 0 && (static_cast<unsigned char>(m.second.str[n]) & 0xC0) == 0x80) --n;
            std::memcpy(r.name, m.second.str.data(), n);
            r.name[n] = '\0';
        } else {
            ++r.unknown;
            rep.warn("unknown field '" + m.first + "'");
        }
    }
    if (params && params->type != Value::Object) return rep.error("\"params\" is not an object");

    // Values by id (a duplicate key: the last valid one wins), applied in table
    // order. present: 0 absent, 1 only of a wrong type (keeps its value), 2 valid.
    const std::int32_t count = pm_param_count();
    std::vector<double> value(static_cast<std::size_t>(count), 0.0);
    std::vector<char> present(static_cast<std::size_t>(count), 0);
    if (params) {
        for (const auto& m : params->obj) {
            const std::int32_t id = pm_param_id_from_key(m.first.c_str());
            const pm_param_info_t* d = pm::paramDescriptor(id);
            if (!d) {
                ++r.unknown;
                rep.warn("unknown parameter '" + m.first + "'");
                continue;
            }
            double v;
            if (m.second.type == Value::Number) v = m.second.num;
            else if (m.second.type == Value::Bool) v = m.second.b ? 1.0 : 0.0;
            else {
                ++r.wrong_type;
                rep.warn("'" + m.first + "' is not a number");
                if (!present[static_cast<std::size_t>(id)]) present[static_cast<std::size_t>(id)] = 1;
                continue;
            }
            if (present[static_cast<std::size_t>(id)] == 2) rep.warn("duplicate parameter '" + m.first + "'");
            if (v < d->min || v > d->max) {
                v = v < d->min ? d->min : d->max;
                ++r.clamped;
                rep.warn("'" + m.first + "' clamped");
            }
            value[static_cast<std::size_t>(id)] = v;
            present[static_cast<std::size_t>(id)] = 2;
        }
    }

    std::vector<double> before(static_cast<std::size_t>(count), 0.0);
    for (std::int32_t i = 0; i < count; ++i) {
        pm_param_info_t d{};
        pm_param_info(i, &d);
        const auto k = static_cast<std::size_t>(d.id);
        pm_engine_get_param(engine, d.id, &before[k]);
        if (!present[k] && !(d.flags & PM_PARAM_FLAG_HIDDEN)) {
            ++r.missing;
            if (flags & PM_PRESET_READ_MISSING_TO_DEFAULT) {
                value[k] = d.default_value;
                present[k] = 2;
            }
        }
    }
    for (std::int32_t i = 0; i < count; ++i) {
        pm_param_info_t d{};
        pm_param_info(i, &d);
        const auto k = static_cast<std::size_t>(d.id);
        if (present[k] != 2) continue;
        if (pm_engine_set_param(engine, d.id, value[k])) ++r.applied;
        else rep.warn(std::string("'") + d.key + "' rejected");
        double after = before[k];
        pm_engine_get_param(engine, d.id, &after);
        if (d.cls == PM_PARAM_CLASS_PREPARE && after != before[k]) r.needs_prepare = 1;
    }
    r.ok = 1;
    return true;
}

}  // namespace

extern "C" {

size_t pm_preset_write(const pm_engine* engine, const char* name, int32_t flags, char* buf, size_t cap) {
    if (!engine) return 0;
    return copyOut(writePreset(engine, name, flags), buf, cap);
}

int pm_preset_read(pm_engine* engine, const char* json, size_t len, int32_t flags, pm_preset_result_t* out) {
    pm_preset_result_t r;
    readPreset(engine, json, len, flags, r);
    if (r.ok && r.needs_prepare && (flags & PM_PRESET_READ_REPREPARE)) {
        const pm::Engine* e = pmEngineOf(engine);
        if (e->prepared() && pm_engine_prepare(engine, e->sampleRate(), e->maxFrames())) {
            r.needs_prepare = 0;
            r.reprepared = 1;
        }
    }
    if (out) *out = r;
    return r.ok;
}

size_t pm_host_preset_write(pm_host* host, const char* name, int32_t flags, char* buf, size_t cap) {
    return pm_preset_write(pm_host_engine(host), name, flags, buf, cap);
}

int pm_host_preset_read(pm_host* host, const char* json, size_t len, int32_t flags, pm_preset_result_t* out) {
    pm_engine* engine = pm_host_engine(host);
    pm_preset_result_t r;
    readPreset(engine, json, len, flags, r);
    if (r.ok && r.needs_prepare && (flags & PM_PRESET_READ_REPREPARE) && pm_host_is_prepared(host)) {
        if (pm_host_prepare(host, pm_host_sample_rate(host), pmEngineOf(engine)->maxFrames())) {
            r.needs_prepare = 0;
            r.reprepared = 1;
        }
    }
    if (out) *out = r;
    return r.ok;
}

}  // extern "C"
