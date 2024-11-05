#include "test_framework.h"
#include "pid_controller_tests.h"
#include "attitude_estimator_tests.h"
#include "mpu6050_tests.h"

#ifndef HOST_BUILD
#include "pico/stdlib.h"

static void wait_for_usb() {
    stdio_init_all();
    // Turn on the LED while we're waiting for USB
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    // Wait for USB connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    // Blink LED once connected
    gpio_put(LED_PIN, 0);
    sleep_ms(250);
    gpio_put(LED_PIN, 1);
}

#endif

int main() {
    #ifndef HOST_BUILD    
    stdio_init_all();  // Only for Pico
    wait_for_usb();
    #endif

    // Initialize test cases
    test_case_t test_cases[] = {
        {"PID Initialization", test_pid_initialization, false, false},
        {"PID Reset", test_pid_reset, false, false},
        {"PID Output Limits", test_pid_output_limits, false, false},
        {"Attitude Estimator Init", test_attitude_estimator_initialization, false, false},
        {"Attitude Estimator Level", test_attitude_estimator_level, false, false},
	{"MPU6050 Initialization", test_mpu6050_initialization, false, false},
	{"MPU6050 Connection", test_mpu6050_connection, false, false},
	{"MPU6050 Calibration", test_mpu6050_calibration, false, false},
	{"MPU6050 Scaling", test_mpu6050_scaling, false, false},
	{"MPU6050 Read Operations", test_mpu6050_read_operations, false, false}
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
