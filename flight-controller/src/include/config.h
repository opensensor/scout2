#pragma once

// System configuration
#define CONTROL_LOOP_FREQ 500
#define IMU_UPDATE_FREQ  1000
#define TELEMETRY_FREQ   100
#define DT (1.0f / CONTROL_LOOP_FREQ)

// Hardware pins
#define PIN_MOTOR1      2
#define PIN_MOTOR2      3
#define PIN_MOTOR3      4
#define PIN_MOTOR4      5
#define PIN_I2C_SDA    12
#define PIN_I2C_SCL    13

// PID constants
#define PID_ROLL_KP    0.5f
#define PID_ROLL_KI    0.2f
#define PID_ROLL_KD    0.1f
#define PID_PITCH_KP   0.5f
#define PID_PITCH_KI   0.2f
#define PID_PITCH_KD   0.1f
#define PID_YAW_KP     0.85f
#define PID_YAW_KI     0.15f
#define PID_YAW_KD     0.0f

// Flight limits
#define MAX_ANGLE      45.0f    // degrees
#define MAX_RATE      500.0f    // degrees/second
#define MOTOR_MIN      0.0f
#define MOTOR_MAX      1.0f
