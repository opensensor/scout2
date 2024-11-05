#include "test_framework.h"
#include "pid_controller_tests.h"
#include "attitude_estimator_tests.h"

#ifndef HOST_BUILD
#include "pico/stdlib.h"
#endif

int main() {
    #ifndef HOST_BUILD
    stdio_init_all();  // Only for Pico
    #endif

    // Initialize test cases
    test_case_t test_cases[] = {
        {"PID Initialization", test_pid_initialization, false, false},
        {"PID Reset", test_pid_reset, false, false},
        {"PID Output Limits", test_pid_output_limits, false, false},
        {"Attitude Estimator Init", test_attitude_estimator_initialization, false, false},
        {"Attitude Estimator Level", test_attitude_estimator_level, false, false}
    };

    // Create test suite
    test_suite_t test_suite = {
        .name = "Flight Controller Tests",
        .cases = test_cases,
        .num_cases = sizeof(test_cases) / sizeof(test_case_t)
    };

    printf("\nRunning Flight Controller Tests...\n");
    run_test_suite(&test_suite);
    print_test_results(&test_suite);

    return test_suite.failed > 0 ? 1 : 0;
}
