#include "test_framework.h"

void run_test_suite(test_suite_t* suite) {
    suite->passed = 0;
    suite->failed = 0;

    for (int i = 0; i < suite->num_cases; i++) {
        test_case_t* test = &suite->cases[i];
        printf("Running test: %s...", test->name);
        test->run = true;
        test->passed = test->test_func();
        
        if (test->passed) {
            suite->passed++;
            printf("PASSED\n");
        } else {
            suite->failed++;
            printf("FAILED\n");
        }
    }
}

void print_test_results(const test_suite_t* suite) {
    printf("\nTest Suite: %s\n", suite->name);
    printf("Passed: %d\n", suite->passed);
    printf("Failed: %d\n", suite->failed);
    printf("Total:  %d\n\n", suite->num_cases);
}

