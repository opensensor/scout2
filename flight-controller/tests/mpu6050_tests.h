#pragma once
#include "pico/stdlib.h"
#include "test_framework.h"

#define TEST_TIMEOUT_MS 1000  // 1 second timeout

// Helper function for timeout checks
static inline bool is_timeout(absolute_time_t start_time, uint32_t timeout_ms) {
    return absolute_time_diff_us(start_time, get_absolute_time()) > (timeout_ms * 1000);
}

bool test_mpu6050_initialization(void);
bool test_mpu6050_connection(void);
bool test_mpu6050_calibration(void);
bool test_mpu6050_scaling(void);
bool test_mpu6050_read_operations(void);
