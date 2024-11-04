#pragma once

#include <stdint.h>
#include "config.h"

typedef struct {
    uint8_t motor1_pin;
    uint8_t motor2_pin;
    uint8_t motor3_pin;
    uint8_t motor4_pin;
    float min_pulse_ms;
    float max_pulse_ms;
    uint32_t pwm_freq;
    uint32_t clock_div;
    uint16_t wrap_value;
} esc_config_t;

typedef struct esc_controller esc_controller_t;

esc_controller_t* esc_init(const esc_config_t* config);
void esc_calibrate(esc_controller_t* esc);
void esc_arm(esc_controller_t* esc);
void esc_disarm(esc_controller_t* esc);
void esc_set_output(esc_controller_t* esc, float m1, float m2, float m3, float m4);
void esc_emergency_stop(esc_controller_t* esc);
