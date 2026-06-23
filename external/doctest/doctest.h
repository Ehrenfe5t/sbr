// Minimal doctest-compatible test framework
// Replace with full doctest.h from https://github.com/doctest/doctest when network is available
// The full header is ~700KB; this minimal version supports the macros used in Phase 0 tests.

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <cstdlib>

// ── Minimal test infrastructure ──

namespace doctest_mini {

struct TestCase {
    std::string name;
    void (*func)();
};

inline std::vector<TestCase>& Registry() {
    static std::vector<TestCase> reg;
    return reg;
}

struct Registrar {
    Registrar(const char* name, void (*func)()) {
        Registry().push_back({name, func});
    }
};

struct AssertInfo {
    const char* file;
    int line;
    std::string expr;
    bool ok;
};

inline int g_passed = 0;
inline int g_failed = 0;

inline void ReportAssert(const AssertInfo& info) {
    if (info.ok) {
        g_passed++;
        std::cout << "  [PASS] " << info.expr << std::endl;
    } else {
        g_failed++;
        std::cout << "  [FAIL] " << info.file << ":" << info.line
                  << "  " << info.expr << std::endl;
    }
}

inline int RunAllTests() {
    g_passed = 0;
    g_failed = 0;
    for (auto& tc : Registry()) {
        std::cout << std::endl << "[" << tc.name << "]" << std::endl;
        tc.func();
    }
    std::cout << std::endl
              << "========== " << g_passed << " passed, "
              << g_failed << " failed ==========" << std::endl;
    return g_failed > 0 ? 1 : 0;
}

} // namespace doctest_mini

// ── doctest-compatible macros ──

#define DOCTEST_CONCAT_IMPL(x, y) x##y
#define DOCTEST_CONCAT(x, y) DOCTEST_CONCAT_IMPL(x, y)

#define TEST_CASE(name) \
    static void DOCTEST_CONCAT(test_func_, __LINE__)(); \
    static doctest_mini::Registrar DOCTEST_CONCAT(reg_, __LINE__)(name, DOCTEST_CONCAT(test_func_, __LINE__)); \
    static void DOCTEST_CONCAT(test_func_, __LINE__)()

#define CHECK(expr) \
    do { \
        bool _ok = (expr); \
        doctest_mini::ReportAssert({__FILE__, __LINE__, #expr, _ok}); \
    } while(0)

#define CHECK_FALSE(expr) \
    do { \
        bool _ok = !(expr); \
        doctest_mini::ReportAssert({__FILE__, __LINE__, "!(" #expr ")", _ok}); \
    } while(0)

#define SUBCASE(name) if (true)

// If full doctest.h is available, use its main; otherwise use our minimal main
#ifdef DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#undef DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#endif

// Provide a main that runs all registered tests
// test_main.cpp can simply: int main() { return doctest_mini::RunAllTests(); }
