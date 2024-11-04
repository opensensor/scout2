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
