#include "test_support.h"

#include <cstdio>

int main() {
    int failed_tests = 0;
    for (const TestCase& test : registered_tests()) {
        const int before = g_failures;
        std::printf("RUN  %s\n", test.name);
        test.fn();
        if (g_failures > before) {
            std::printf("FAIL %s (%d checks)\n", test.name, g_failures - before);
            ++failed_tests;
        } else {
            std::printf("PASS %s\n", test.name);
        }
    }
    std::printf("%zu tests, %d failed, %d failed checks\n", registered_tests().size(), failed_tests,
                g_failures);
    return (failed_tests == 0) ? 0 : 1;
}
