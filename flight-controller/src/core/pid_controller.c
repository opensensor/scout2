#include "pid_controller.h"
#include <stdlib.h>
#include <math.h>

#define DERIVATIVE_FILTER_ALPHA 0.1f  // Lower = more filtering

pid_controller_t* pid_controller_init(float kp, float ki, float kd) {
    pid_controller_t* pid = malloc(sizeof(pid_controller_t));
    if (pid == NULL) return NULL;
    
    // Initialize gains
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // Set default limits
    pid->output_limit = 1.0f;
    pid->integral_limit = 0.5f;
    
    // Initialize state
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->derivative_lpf = 0.0f;
    pid->derivative_filter_alpha = DERIVATIVE_FILTER_ALPHA;
    pid->prev_time = 0.0f;
    
    return pid;
}

void pid_controller_reset(pid_controller_t* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->derivative_lpf = 0.0f;
}

static float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float pid_controller_update(pid_controller_t* pid, float error, float dt) {
    if (dt <= 0.0f) return 0.0f;  // Prevent division by zero
    
    // Calculate P term
    float p_term = pid->kp * error;
    
    // Calculate I term with anti-windup
    pid->integral += (error * dt);
    pid->integral = constrain(pid->integral, -pid->integral_limit, pid->integral_limit);
    float i_term = pid->ki * pid->integral;
    
    // Calculate D term with derivative on error
    float error_derivative = (error - pid->prev_error) / dt;
    
    // Apply low-pass filter to derivative
    pid->derivative_lpf = (pid->derivative_filter_alpha * error_derivative) + 
                         ((1.0f - pid->derivative_filter_alpha) * pid->derivative_lpf);
    
    float d_term = pid->kd * pid->derivative_lpf;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    output = constrain(output, -pid->output_limit, pid->output_limit);
    
    // If output is saturated and the input error would drive it further into saturation,
    // don't update integral (anti-windup)
    if ((output >= pid->output_limit && error > 0.0f) ||
        (output <= -pid->output_limit && error < 0.0f)) {
        // Don't update integral
    } else {
        pid->integral += error * dt;
    }
    
    // Store error for next iteration
    pid->prev_error = error;
    
    return output;
}

// Alternative update function using derivative on measurement
float pid_controller_update_dom(pid_controller_t* pid, float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;
    
    float error = setpoint - measurement;
    
    // P term
    float p_term = pid->kp * error;
    
    // I term with anti-windup
    pid->integral += (error * dt);
    pid->integral = constrain(pid->integral, -pid->integral_limit, pid->integral_limit);
    float i_term = pid->ki * pid->integral;
    
    // D term with derivative on measurement
    float measurement_derivative = (measurement - pid->prev_measurement) / dt;
    
    // Apply low-pass filter to derivative
    pid->derivative_lpf = (pid->derivative_filter_alpha * measurement_derivative) + 
                         ((1.0f - pid->derivative_filter_alpha) * pid->derivative_lpf);
    
    // Note the negative sign because we want derivative of error
    float d_term = -pid->kd * pid->derivative_lpf;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    output = constrain(output, -pid->output_limit, pid->output_limit);
    
    // Anti-windup
    if ((output >= pid->output_limit && error > 0.0f) ||
        (output <= -pid->output_limit && error < 0.0f)) {
        // Don't update integral
    } else {
        pid->integral += error * dt;
    }
    
    // Store measurement for next iteration
    pid->prev_measurement = measurement;
    
    return output;
}

void pid_controller_set_gains(pid_controller_t* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // Reset state when gains change
    pid_controller_reset(pid);
}

void pid_controller_set_limits(pid_controller_t* pid, float output_limit, float integral_limit) {
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
}