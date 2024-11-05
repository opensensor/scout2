#include "unity.h"
#include "pid_controller_tests.h"
#include "attitude_estimator_tests.h"
#include "mpu6050_tests.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

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

// Test configuration for MPU6050
static const uint8_t TEST_SDA_PIN = 4;
static const uint8_t TEST_SCL_PIN = 5;

void setUp(void) {
}

void tearDown(void) {
}

// Declare all test functions
void test_pid_initialization(void);
void test_pid_reset(void);
void test_pid_output_limits(void);
void test_attitude_estimator_initialization(void);
void test_attitude_estimator_level(void);
void test_mpu6050_initialization(void);
void test_mpu6050_connection(void);
void test_mpu6050_calibration(void);
void test_mpu6050_scaling(void);
void test_mpu6050_read_operations(void);

int main(void) {
    #ifndef HOST_BUILD    
    stdio_init_all();  // Only for Pico
    wait_for_usb();
    #endif

    UNITY_BEGIN();

    // PID Controller Tests
    RUN_TEST(test_pid_initialization);
    RUN_TEST(test_pid_reset);
    RUN_TEST(test_pid_output_limits);

    // Attitude Estimator Tests
    RUN_TEST(test_attitude_estimator_initialization);
    RUN_TEST(test_attitude_estimator_level);

    // MPU6050 Tests - only run if hardware is available
    RUN_TEST(test_mpu6050_initialization);
    RUN_TEST(test_mpu6050_connection);
    RUN_TEST(test_mpu6050_calibration);
    RUN_TEST(test_mpu6050_scaling);
    RUN_TEST(test_mpu6050_read_operations);

    return UNITY_END();
}
