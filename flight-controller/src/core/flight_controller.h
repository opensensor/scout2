// flight-controller/src/core/flight_controller.h
#pragma once

#include "../include/types.h"
#include "attitude_estimator.h"
#include "pid_controller.h"
#include "../drivers/mpu6050.h"
#include "../drivers/esc.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} setpoint_t;

typedef struct {
    mpu6050_t* imu;
    attitude_estimator_t* attitude_estimator;
    pid_controller_t* pid_roll;
    pid_controller_t* pid_pitch;
    pid_controller_t* pid_yaw;
    esc_controller_t* esc;
    flight_mode_t current_mode;
    setpoint_t setpoint;
} flight_controller_t;

flight_controller_t* flight_controller_init(void);
void flight_controller_update(flight_controller_t* fc);
void flight_controller_cleanup(flight_controller_t* fc);
