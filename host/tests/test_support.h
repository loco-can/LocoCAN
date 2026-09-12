#pragma once

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

struct TestCase {
    const char* name;
    void (*fn)();
};

inline std::vector<TestCase>& registered_tests() {
    static std::vector<TestCase> tests;
    return tests;
}

struct TestRegistrar {
    TestRegistrar(const char* name, void (*fn)()) { registered_tests().push_back({name, fn}); }
};

inline int g_failures = 0;

inline void check(bool cond, const char* expr, const char* file, int line) {
    if (!cond) {
        std::fprintf(stderr, "  CHECK failed: %s (%s:%d)\n", expr, file, line);
        ++g_failures;
    }
}

template <typename A, typename B>
inline void check_eq(const A& a, const B& b, const char* sa, const char* sb, const char* file, int line) {
    if (!(a == static_cast<A>(b))) {
        std::ostringstream out;
        out << a << " != " << b;
        std::fprintf(stderr, "  CHECK_EQ failed: %s == %s (%s) (%s:%d)\n", sa, sb, out.str().c_str(),
                     file, line);
        ++g_failures;
    }
}

#define TEST(name)                                                                                 \
    void test_##name();                                                                            \
    static TestRegistrar registrar_##name(#name, test_##name);                                     \
    void test_##name()

#define CHECK(expr) check(static_cast<bool>(expr), #expr, __FILE__, __LINE__)
#define CHECK_EQ(a, b) check_eq((a), (b), #a, #b, __FILE__, __LINE__)
