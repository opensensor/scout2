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
static bool mpu6050_available = false;

// Helper to safely initialize I2C with timeout
static bool init_i2c_with_timeout(void) {
    // Configure I2C pins
    gpio_set_function(TEST_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TEST_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(TEST_SDA_PIN);
    gpio_pull_up(TEST_SCL_PIN);

    // Try to initialize I2C
    if (!i2c_init(i2c0, 400000)) {  // 400 kHz
        return false;
    }

    return true;
}

// Helper function to check if MPU6050 is present without hanging
static bool is_mpu6050_present(void) {
    // First check if I2C can be initialized
    if (!init_i2c_with_timeout()) {
        return false;
    }

    // Now try to init the MPU6050 without hanging
    mpu6050_config_t config = {0};
    mpu6050_t* dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
    if (!dev) {
        return false;
    }

    // Quick connection test
    bool present = mpu6050_test_connection(dev);
    free(dev);
    return present;
}

void setUp(void) {
    // Check MPU6050 presence once at startup
    static bool hardware_checked = false;
    
    if (!hardware_checked) {
        mpu6050_available = is_mpu6050_present();
        hardware_checked = true;
        if (!mpu6050_available) {
            TEST_MESSAGE("MPU6050 not detected - hardware tests will be skipped");
        }
    }
}

void tearDown(void) {
    // Nothing to do here
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
    if (mpu6050_available) {
        RUN_TEST(test_mpu6050_initialization);
        RUN_TEST(test_mpu6050_connection);
        RUN_TEST(test_mpu6050_calibration);
        RUN_TEST(test_mpu6050_scaling);
        RUN_TEST(test_mpu6050_read_operations);
    }

    return UNITY_END();
}
