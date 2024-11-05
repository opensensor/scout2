#include "test_framework.h"
#include "pid_controller_tests.h"
#include "attitude_estimator_tests.h"

int main() {
    // Initialize test cases for PID controller
    test_case_t pid_tests[] = {
        {"PID Initialization", test_pid_initialization, false, false},
        {"PID Reset", test_pid_reset, false, false},
        {"PID Output Limits", test_pid_output_limits, false, false}
    };

    // Initialize test cases for attitude estimator
    test_case_t attitude_tests[] = {
        {"Attitude Estimator Initialization", test_attitude_estimator_initialization, false, false},
        {"Attitude Estimator Level", test_attitude_estimator_level, false, false}
    };

    // Create test suites
    test_suite_t pid_suite = {
        .name = "PID Controller Tests",
        .cases = pid_tests,
        .num_cases = sizeof(pid_tests) / sizeof(test_case_t)
    };

    test_suite_t attitude_suite = {
        .name = "Attitude Estimator Tests",
        .cases = attitude_tests,
        .num_cases = sizeof(attitude_tests) / sizeof(test_case_t)
    };

    // Run test suites
    printf("\nRunning PID Controller Tests...\n");
    run_test_suite(&pid_suite);
    print_test_results(&pid_suite);

    printf("\nRunning Attitude Estimator Tests...\n");
    run_test_suite(&attitude_suite);
    print_test_results(&attitude_suite);

    return (pid_suite.failed + attitude_suite.failed) > 0 ? 1 : 0;
}