#pragma once

typedef struct {
    // PID gains
    float kp;
    float ki;
    float kd;
    
    // Limits
    float output_limit;
    float integral_limit;
    
    // State variables
    float integral;
    float prev_error;
    float prev_measurement;  // For derivative on measurement
    
    // Derivative low-pass filter
    float derivative_lpf;
    float derivative_filter_alpha;
    
    // Delta time
    float prev_time;
} pid_controller_t;

pid_controller_t* pid_controller_init(float kp, float ki, float kd);
void pid_controller_reset(pid_controller_t* pid);
float pid_controller_update(pid_controller_t* pid, float error, float dt);
void pid_controller_set_gains(pid_controller_t* pid, float kp, float ki, float kd);
void pid_controller_set_limits(pid_controller_t* pid, float output_limit, float integral_limit);

