// Presets (plan §5.3): a minimal JSON reader and the preset document
// helpers behind pm_preset_* in pm_api.h. Non-real-time (allocates); built
// without exceptions: errors are returned, never thrown.
#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace pm::json {

struct Value {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<Value> arr;
    std::vector<std::pair<std::string, Value>> obj;  // in document order; duplicates kept

    const Value* find(const char* key) const noexcept;  // the last member with this key
};

constexpr int kMaxDepth = 32;

// Strict RFC 8259 syntax: one value, surrounded by whitespace only. Strings
// are UTF-8 checked; \u escapes (with surrogate pairs) become UTF-8; \u0000
// is refused. Numbers must be finite as doubles. Returns false with a message
// and the byte offset of the error.
bool parse(const char* text, std::size_t len, Value& out, std::string& error, std::size_t& offset);

// Shortest decimal form (%.{1..17}g) that reads back as exactly v.
void appendNumber(std::string& s, double v);
// A JSON string literal; s must be valid UTF-8.
void appendString(std::string& s, const char* text);

}  // namespace pm::json
