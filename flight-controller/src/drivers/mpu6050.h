// flight-controller/src/drivers/mpu6050.h
#pragma once

#include <stdint.h>
#include "../include/types.h"
#include "system.h"

// MPU6050 registers
#define MPU6050_ADDR              0x68
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_FIFO_EN      0x23
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

typedef struct {
    uint8_t gyro_range;     // 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
    uint8_t accel_range;    // 0=±2g, 1=±4g, 2=±8g, 3=±16g
    uint8_t dlpf_bandwidth; // Digital Low Pass Filter bandwidth
    uint8_t sample_rate_div;// Sample Rate = 1kHz / (1 + sample_rate_div)
} mpu6050_config_t;

// Forward declaration
typedef struct mpu6050_dev mpu6050_t;

// Function declarations
mpu6050_t* mpu6050_init(uint8_t sda_pin, uint8_t scl_pin, const mpu6050_config_t* config);
bool mpu6050_test_connection(mpu6050_t* dev);
void mpu6050_calibrate(mpu6050_t* dev, int num_samples);
void mpu6050_read_raw(mpu6050_t* dev, vector3_t* accel, vector3_t* gyro);
void mpu6050_read_scaled(mpu6050_t* dev, vector3_t* accel, vector3_t* gyro);

