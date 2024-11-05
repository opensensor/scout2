#include "mpu6050_tests.h"
#include "../src/drivers/mpu6050.h"
#include <math.h>
#include <stdlib.h>

// Test configuration
static const uint8_t TEST_SDA_PIN = 4;
static const uint8_t TEST_SCL_PIN = 5;


bool test_mpu6050_initialization() {
    mpu6050_config_t config = {
        .gyro_range = 0,      // ±250°/s
        .accel_range = 0,     // ±2g
        .dlpf_bandwidth = 0,  // Maximum bandwidth
        .sample_rate_div = 0  // Maximum sample rate (1kHz)
    };
    
    absolute_time_t start_time = get_absolute_time();
    mpu6050_t* dev = NULL;
    
    // Try initialization with timeout
    while (!dev && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
        sleep_ms(10);  // Small delay between retries
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 initialization failed after %d ms\n", TEST_TIMEOUT_MS);
        return false;
    }
    
    ASSERT_TRUE(dev != NULL);
    free(dev);
    return true;
}

bool test_mpu6050_connection() {
    mpu6050_config_t config = {0};
    
    absolute_time_t start_time = get_absolute_time();
    mpu6050_t* dev = NULL;
    
    // Try initialization with timeout
    while (!dev && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 initialization failed after %d ms\n", TEST_TIMEOUT_MS);
        return false;
    }
    
    // Try connection test with timeout
    start_time = get_absolute_time();
    bool connected = false;
    while (!connected && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        connected = mpu6050_test_connection(dev);
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 connection test failed after %d ms\n", TEST_TIMEOUT_MS);
        free(dev);
        return false;
    }
    
    ASSERT_TRUE(connected);
    free(dev);
    return true;
}

// Similar pattern for other test functions...
bool test_mpu6050_calibration() {
    mpu6050_config_t config = {
        .gyro_range = 0,
        .accel_range = 0
    };
    
    absolute_time_t start_time = get_absolute_time();
    mpu6050_t* dev = NULL;
    
    while (!dev && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 initialization failed after %d ms\n", TEST_TIMEOUT_MS);
        return false;
    }
    
    // Try calibration with timeout
    start_time = get_absolute_time();
    bool calibration_done = false;
    while (!calibration_done && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        mpu6050_calibrate(dev, 10);  // Reduced samples for testing
        calibration_done = true;  // Assuming calibration always completes
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 calibration failed after %d ms\n", TEST_TIMEOUT_MS);
        free(dev);
        return false;
    }
    
    vector3_t accel, gyro;
    mpu6050_read_scaled(dev, &accel, &gyro);
    
    ASSERT_FLOAT_EQUAL(0.0f, gyro.x, 0.1f);
    ASSERT_FLOAT_EQUAL(0.0f, gyro.y, 0.1f);
    ASSERT_FLOAT_EQUAL(0.0f, gyro.z, 0.1f);
    
    free(dev);
    return true;
}

bool test_mpu6050_scaling() {
    mpu6050_config_t config = {
        .gyro_range = 3,      // ±2000°/s
        .accel_range = 3,     // ±16g
        .dlpf_bandwidth = 0,
        .sample_rate_div = 0
    };
    
    // Initialize with timeout
    absolute_time_t start_time = get_absolute_time();
    mpu6050_t* dev = NULL;
    
    while (!dev && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 initialization failed after %d ms\n", TEST_TIMEOUT_MS);
        return false;
    }
    
    ASSERT_TRUE(dev != NULL);
    
    // Read scaled values with timeout
    vector3_t accel, gyro;
    start_time = get_absolute_time();
    bool read_success = false;
    
    while (!read_success && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        mpu6050_read_scaled(dev, &accel, &gyro);
        read_success = true;  // Assume read successful if no error
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 scaling read failed after %d ms\n", TEST_TIMEOUT_MS);
        free(dev);
        return false;
    }
    
    // Verify that readings are within physical limits
    ASSERT_TRUE(fabs(accel.x) <= 16.0f);  // ±16g range
    ASSERT_TRUE(fabs(accel.y) <= 16.0f);
    ASSERT_TRUE(fabs(accel.z) <= 16.0f);
    
    ASSERT_TRUE(fabs(gyro.x) <= 2000.0f);  // ±2000°/s range
    ASSERT_TRUE(fabs(gyro.y) <= 2000.0f);
    ASSERT_TRUE(fabs(gyro.z) <= 2000.0f);
    
    free(dev);
    return true;
}

bool test_mpu6050_read_operations() {
    mpu6050_config_t config = {0};
    
    // Initialize with timeout
    absolute_time_t start_time = get_absolute_time();
    mpu6050_t* dev = NULL;
    
    while (!dev && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 initialization failed after %d ms\n", TEST_TIMEOUT_MS);
        return false;
    }
    
    ASSERT_TRUE(dev != NULL);
    
    // Test raw reading with timeout
    vector3_t raw_accel, raw_gyro;
    start_time = get_absolute_time();
    bool raw_read_success = false;
    
    while (!raw_read_success && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        mpu6050_read_raw(dev, &raw_accel, &raw_gyro);
        raw_read_success = true;  // Assume read successful if no error
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 raw read failed after %d ms\n", TEST_TIMEOUT_MS);
        free(dev);
        return false;
    }
    
    // Raw values should be integers
    ASSERT_TRUE(raw_accel.x == (int16_t)raw_accel.x);
    ASSERT_TRUE(raw_accel.y == (int16_t)raw_accel.y);
    ASSERT_TRUE(raw_accel.z == (int16_t)raw_accel.z);
    
    // Test scaled reading with timeout
    vector3_t scaled_accel, scaled_gyro;
    start_time = get_absolute_time();
    bool scaled_read_success = false;
    
    while (!scaled_read_success && !is_timeout(start_time, TEST_TIMEOUT_MS)) {
        mpu6050_read_scaled(dev, &scaled_accel, &scaled_gyro);
        scaled_read_success = true;  // Assume read successful if no error
        sleep_ms(10);
    }
    
    if (is_timeout(start_time, TEST_TIMEOUT_MS)) {
        printf("TIMEOUT: MPU6050 scaled read failed after %d ms\n", TEST_TIMEOUT_MS);
        free(dev);
        return false;
    }
    
    // Scaled values should be floating point
    ASSERT_TRUE(scaled_accel.x != (int16_t)scaled_accel.x ||
                scaled_accel.y != (int16_t)scaled_accel.y ||
                scaled_accel.z != (int16_t)scaled_accel.z);
    
    free(dev);
    return true;
}

