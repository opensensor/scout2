#pragma once

#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

typedef enum {
    FLIGHT_MODE_DISARMED,
    FLIGHT_MODE_ARMED,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_ALTITUDE_HOLD,
    FLIGHT_MODE_POSITION_HOLD
} flight_mode_t;

// Motor control types
typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} control_inputs_t;

// Common status/error codes
typedef enum {
    STATUS_OK = 0,
    ERROR_INIT_FAILED = -1,
    ERROR_CALIBRATION_FAILED = -2,
    ERROR_SENSOR_TIMEOUT = -3,
    ERROR_INVALID_STATE = -4
} status_code_t;

// Configuration structure
typedef struct {
    float pid_roll_p;
    float pid_roll_i;
    float pid_roll_d;
    float pid_pitch_p;
    float pid_pitch_i;
    float pid_pitch_d;
    float pid_yaw_p;
    float pid_yaw_i;
    float pid_yaw_d;
    float max_angle;
    float max_rate;
    float motor_idle_throttle;
    float motor_max_throttle;
} flight_config_t;
