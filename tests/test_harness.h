// Tiny self-contained test harness. Each test executable includes this header
// once; it provides main(), which runs every TEST in the file and returns
// non-zero if any check failed.
#pragma once

#include <cmath>
#include <cstdio>
#include <vector>

namespace pm_test {

struct Case {
    const char* name;
    void (*fn)();
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}

inline int& failures() {
    static int f = 0;
    return f;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline void fail(const char* file, int line, const char* expr) {
    std::printf("  FAILED %s:%d: %s\n", file, line, expr);
    ++failures();
}

}  // namespace pm_test

#define TEST(name)                                                 \
    static void name();                                            \
    static const pm_test::Registrar name##_registrar(#name, name); \
    static void name()

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) pm_test::fail(__FILE__, __LINE__, #cond);         \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                 \
    do {                                                                                      \
        const double pm_a_ = (a), pm_b_ = (b), pm_t_ = (tol);                                 \
        if (!(std::fabs(pm_a_ - pm_b_) <= pm_t_)) {                                           \
            std::printf("  FAILED %s:%d: |%s - %s| <= %s  (%.9g vs %.9g, tol %.3g)\n",        \
                        __FILE__, __LINE__, #a, #b, #tol, pm_a_, pm_b_, pm_t_);               \
            ++pm_test::failures();                                                            \
        }                                                                                     \
    } while (0)

#define CHECK_LE(a, b)                                                                        \
    do {                                                                                      \
        const double pm_a_ = (a), pm_b_ = (b);                                                \
        if (!(pm_a_ <= pm_b_)) {                                                              \
            std::printf("  FAILED %s:%d: %s <= %s  (%.9g vs %.9g)\n", __FILE__, __LINE__, #a, \
                        #b, pm_a_, pm_b_);                                                    \
            ++pm_test::failures();                                                            \
        }                                                                                     \
    } while (0)

int main() {
    for (const auto& c : pm_test::registry()) {
        const int before = pm_test::failures();
        std::printf("[ RUN  ] %s\n", c.name);
        c.fn();
        std::printf("[ %s ] %s\n", pm_test::failures() == before ? " OK " : "FAIL", c.name);
    }
    std::printf("%d failure(s)\n", pm_test::failures());
    return pm_test::failures() == 0 ? 0 : 1;
}
