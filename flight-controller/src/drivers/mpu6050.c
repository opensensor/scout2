#include "mpu6050.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdlib.h>

#define I2C_FREQ 400000  // 400 kHz

// Scaling factors for raw values
static const float GYRO_SCALE_250  = 131.0f;    // LSB/(°/s) for ±250°/s
static const float GYRO_SCALE_500  = 65.5f;     // LSB/(°/s) for ±500°/s
static const float GYRO_SCALE_1000 = 32.8f;     // LSB/(°/s) for ±1000°/s
static const float GYRO_SCALE_2000 = 16.4f;     // LSB/(°/s) for ±2000°/s

static const float ACCEL_SCALE_2G  = 16384.0f;  // LSB/g for ±2g
static const float ACCEL_SCALE_4G  = 8192.0f;   // LSB/g for ±4g
static const float ACCEL_SCALE_8G  = 4096.0f;   // LSB/g for ±8g
static const float ACCEL_SCALE_16G = 2048.0f;   // LSB/g for ±16g

struct mpu6050_dev {
    i2c_inst_t* i2c;
    uint8_t addr;
    float gyro_scale;
    float accel_scale;
    vector3_t gyro_offset;
    vector3_t accel_offset;
};

static inline int16_t combine_bytes(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

static bool mpu6050_write_reg(mpu6050_t* dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_write_timeout_us(dev->i2c, dev->addr, buf, 2, false, 1000000) == 2;
}

static uint8_t mpu6050_read_reg(mpu6050_t* dev, uint8_t reg) {
    uint8_t data;
    i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, 1000000);
    i2c_read_timeout_us(dev->i2c, dev->addr, &data, 1, false, 1000000);
    return data;
}

mpu6050_t* mpu6050_init(uint8_t sda_pin, uint8_t scl_pin, const mpu6050_config_t* config) {
    mpu6050_t* dev = malloc(sizeof(mpu6050_t));
    if (dev == NULL) return NULL;

    // Initialize I2C
    dev->i2c = i2c0;  // Use I2C0 by default
    dev->addr = MPU6050_ADDR;
    
    i2c_init(dev->i2c, I2C_FREQ);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // Reset the device
    mpu6050_write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);  // Wait for reset
    
    // Wake up
    mpu6050_write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x00);
    
    // Configure sample rate divider
    mpu6050_write_reg(dev, MPU6050_REG_SMPLRT_DIV, config->sample_rate_div);
    
    // Configure DLPF
    mpu6050_write_reg(dev, MPU6050_REG_CONFIG, config->dlpf_bandwidth);
    
    // Configure gyroscope range
    mpu6050_write_reg(dev, MPU6050_REG_GYRO_CONFIG, config->gyro_range << 3);
    
    // Configure accelerometer range
    mpu6050_write_reg(dev, MPU6050_REG_ACCEL_CONFIG, config->accel_range << 3);
    
    // Set scaling factors based on range settings
    switch (config->gyro_range) {
        case 0: dev->gyro_scale = GYRO_SCALE_250; break;
        case 1: dev->gyro_scale = GYRO_SCALE_500; break;
        case 2: dev->gyro_scale = GYRO_SCALE_1000; break;
        case 3: dev->gyro_scale = GYRO_SCALE_2000; break;
    }
    
    switch (config->accel_range) {
        case 0: dev->accel_scale = ACCEL_SCALE_2G; break;
        case 1: dev->accel_scale = ACCEL_SCALE_4G; break;
        case 2: dev->accel_scale = ACCEL_SCALE_8G; break;
        case 3: dev->accel_scale = ACCEL_SCALE_16G; break;
    }
    
    // Initialize offsets to zero
    memset(&dev->gyro_offset, 0, sizeof(vector3_t));
    memset(&dev->accel_offset, 0, sizeof(vector3_t));
    
    return dev;
}

bool mpu6050_test_connection(mpu6050_t* dev) {
    return mpu6050_read_reg(dev, MPU6050_REG_WHO_AM_I) == 0x68;
}

void mpu6050_calibrate(mpu6050_t* dev, int num_samples) {
    vector3_t gyro_sum = {0};
    vector3_t accel_sum = {0};
    
    // Collect samples
    for (int i = 0; i < num_samples; i++) {
        vector3_t accel, gyro;
        mpu6050_read_raw(dev, &accel, &gyro);
        
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;
        
        accel_sum.x += accel.x;
        accel_sum.y += accel.y;
        accel_sum.z += accel.z;
        
        sleep_ms(2);  // Wait for next sample
    }
    
    // Calculate average offsets
    dev->gyro_offset.x = gyro_sum.x / num_samples;
    dev->gyro_offset.y = gyro_sum.y / num_samples;
    dev->gyro_offset.z = gyro_sum.z / num_samples;
    
    // For accelerometer, only remove X and Y offset, keep Z at 1g
    dev->accel_offset.x = accel_sum.x / num_samples;
    dev->accel_offset.y = accel_sum.y / num_samples;
    dev->accel_offset.z = (accel_sum.z / num_samples) - dev->accel_scale; // Remove 1g
}

void mpu6050_read_raw(mpu6050_t* dev, vector3_t* accel, vector3_t* gyro) {
    uint8_t buffer[14];
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    
    // Read all data at once (6 bytes accel, 2 bytes temp, 6 bytes gyro)
    i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, 1000000);
    i2c_read_timeout_us(dev->i2c, dev->addr, buffer, 14, false, 1000000);
    
    // Combine bytes and store in vectors
    accel->x = combine_bytes(buffer[0], buffer[1]);
    accel->y = combine_bytes(buffer[2], buffer[3]);
    accel->z = combine_bytes(buffer[4], buffer[5]);
    
    gyro->x = combine_bytes(buffer[8], buffer[9]);
    gyro->y = combine_bytes(buffer[10], buffer[11]);
    gyro->z = combine_bytes(buffer[12], buffer[13]);
}

void mpu6050_read_scaled(mpu6050_t* dev, vector3_t* accel, vector3_t* gyro) {
    vector3_t raw_accel, raw_gyro;
    mpu6050_read_raw(dev, &raw_accel, &raw_gyro);
    
    // Apply scaling and remove offsets
    accel->x = (raw_accel.x - dev->accel_offset.x) / dev->accel_scale;
    accel->y = (raw_accel.y - dev->accel_offset.y) / dev->accel_scale;
    accel->z = (raw_accel.z - dev->accel_offset.z) / dev->accel_scale;
    
    gyro->x = (raw_gyro.x - dev->gyro_offset.x) / dev->gyro_scale;
    gyro->y = (raw_gyro.y - dev->gyro_offset.y) / dev->gyro_scale;
    gyro->z = (raw_gyro.z - dev->gyro_offset.z) / dev->gyro_scale;
}
