#pragma once
#include "pico/stdlib.h"
#include "unity.h"
#include "../src/drivers/mpu6050.h"

#define TEST_TIMEOUT_MS 1000  // 1 second timeout

// Helper function for timeout checks
static inline bool is_timeout(absolute_time_t start_time, uint32_t timeout_ms) {
    return absolute_time_diff_us(start_time, get_absolute_time()) > (timeout_ms * 1000);
}

void test_mpu6050_initialization(void);
void test_mpu6050_connection(void);
void test_mpu6050_calibration(void);
void test_mpu6050_scaling(void);
void test_mpu6050_read_operations(void);
