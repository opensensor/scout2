#include "unity.h"
#include "mpu6050_tests.h"
#include <math.h>
#include <stdlib.h>

// Test configuration
static const uint8_t TEST_SDA_PIN = 4;
static const uint8_t TEST_SCL_PIN = 5;

void test_mpu6050_initialization(void) {
    mpu6050_config_t config = {
        .gyro_range = 0,      // ±250°/s
        .accel_range = 0,     // ±2g
        .dlpf_bandwidth = 0,  // Maximum bandwidth
        .sample_rate_div = 0  // Maximum sample rate (1kHz)
    };
    
    mpu6050_t* dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
    TEST_ASSERT_NOT_NULL(dev);
    free(dev);
}

void test_mpu6050_connection(void) {
    mpu6050_config_t config = {0};
    mpu6050_t* dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
    TEST_ASSERT_NOT_NULL(dev);
    
    bool connected = mpu6050_test_connection(dev);
    TEST_ASSERT_TRUE(connected);
    
    free(dev);
}

void test_mpu6050_calibration(void) {
    mpu6050_config_t config = {
        .gyro_range = 0,
        .accel_range = 0,
        .dlpf_bandwidth = 0,
        .sample_rate_div = 0
    };
    
    mpu6050_t* dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
    TEST_ASSERT_NOT_NULL(dev);
    
    mpu6050_calibrate(dev, 10);  // Reduced samples for testing
    
    vector3_t accel, gyro;
    mpu6050_read_scaled(dev, &accel, &gyro);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, gyro.x);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, gyro.y);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, gyro.z);
    
    free(dev);
}

void test_mpu6050_scaling(void) {
    mpu6050_config_t config = {
        .gyro_range = 3,      // ±2000°/s
        .accel_range = 3,     // ±16g
        .dlpf_bandwidth = 0,
        .sample_rate_div = 0
    };
    
    mpu6050_t* dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
    TEST_ASSERT_NOT_NULL(dev);
    
    vector3_t accel, gyro;
    mpu6050_read_scaled(dev, &accel, &gyro);
    
    // Verify that readings are within physical limits
    TEST_ASSERT_TRUE(fabs(accel.x) <= 16.0f);  // ±16g range
    TEST_ASSERT_TRUE(fabs(accel.y) <= 16.0f);
    TEST_ASSERT_TRUE(fabs(accel.z) <= 16.0f);
    
    TEST_ASSERT_TRUE(fabs(gyro.x) <= 2000.0f);  // ±2000°/s range
    TEST_ASSERT_TRUE(fabs(gyro.y) <= 2000.0f);
    TEST_ASSERT_TRUE(fabs(gyro.z) <= 2000.0f);
    
    free(dev);
}

void test_mpu6050_read_operations(void) {
    mpu6050_config_t config = {0};
    mpu6050_t* dev = mpu6050_init(TEST_SDA_PIN, TEST_SCL_PIN, &config);
    TEST_ASSERT_NOT_NULL(dev);
    
    vector3_t raw_accel, raw_gyro;
    mpu6050_read_raw(dev, &raw_accel, &raw_gyro);
    
    // Raw values should be integers
    TEST_ASSERT_EQUAL_FLOAT(raw_accel.x, (int16_t)raw_accel.x);
    TEST_ASSERT_EQUAL_FLOAT(raw_accel.y, (int16_t)raw_accel.y);
    TEST_ASSERT_EQUAL_FLOAT(raw_accel.z, (int16_t)raw_accel.z);
    
    vector3_t scaled_accel, scaled_gyro;
    mpu6050_read_scaled(dev, &scaled_accel, &scaled_gyro);
    
    // At least one component should be non-integer (floating point)
    TEST_ASSERT(scaled_accel.x != (int16_t)scaled_accel.x ||
                scaled_accel.y != (int16_t)scaled_accel.y ||
                scaled_accel.z != (int16_t)scaled_accel.z);
    
    free(dev);
}
