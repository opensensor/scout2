#pragma once

// System configuration
#define CONTROL_LOOP_FREQ     500
#define IMU_UPDATE_FREQ       1000
#define TELEMETRY_FREQ       100

// Hardware pins
#define PIN_MOTOR1           2
#define PIN_MOTOR2           3
#define PIN_MOTOR3           4
#define PIN_MOTOR4           5
#define PIN_I2C_SDA         12
#define PIN_I2C_SCL         13

// PID constants
#define PID_ROLL_KP         0.5f
#define PID_ROLL_KI         0.2f
#define PID_ROLL_KD         0.1f

// Control loop timing
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ)
#define IMU_UPDATE_PERIOD_US (1000000 / IMU_UPDATE_FREQ)
