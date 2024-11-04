#include "esc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <stdlib.h>

#define PWM_RESOLUTION 16
#define MIN_THROTTLE 0.0f
#define MAX_THROTTLE 1.0f

struct esc_controller {
    esc_config_t config;
    uint8_t slice_num[4];  // PWM slice numbers for each motor
    uint8_t channel[4];    // PWM channels for each motor
    bool is_armed;
};

static inline uint16_t throttle_to_duty(const esc_controller_t* esc, float throttle) {
    // Constrain throttle to valid range
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;
    
    // Convert throttle to pulse width
    float pulse_width = esc->config.min_pulse_ms + 
                       throttle * (esc->config.max_pulse_ms - esc->config.min_pulse_ms);
    
    // Convert to duty cycle value
    float duty_cycle = pulse_width / (1000.0f / esc->config.pwm_freq);
    return (uint16_t)(duty_cycle * esc->config.wrap_value);
}

esc_controller_t* esc_init(const esc_config_t* config) {
    esc_controller_t* esc = malloc(sizeof(esc_controller_t));
    if (esc == NULL) return NULL;
    
    // Store configuration
    esc->config = *config;
    esc->is_armed = false;
    
    // Configure PWM for each motor
    uint8_t pins[4] = {
        config->motor1_pin,
        config->motor2_pin,
        config->motor3_pin,
        config->motor4_pin
    };
    
    // Initialize PWM for each motor
    for (int i = 0; i < 4; i++) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        esc->slice_num[i] = pwm_gpio_to_slice_num(pins[i]);
        esc->channel[i] = pwm_gpio_to_channel(pins[i]);
        
        // Configure PWM
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, config->clock_div);
        pwm_config_set_wrap(&cfg, config->wrap_value);
        pwm_init(esc->slice_num[i], &cfg, true);
        
        // Start with motors off
        pwm_set_gpio_level(pins[i], 0);
    }
    
    return esc;
}

void esc_calibrate(esc_controller_t* esc) {
    // ESC calibration procedure
    // 1. Set maximum throttle
    for (int i = 0; i < 4; i++) {
        pwm_set_gpio_level(esc->config.motor1_pin + i, 
                          throttle_to_duty(esc, MAX_THROTTLE));
    }
    
    // 2. Wait for ESC to recognize max throttle
    sleep_ms(5000);
    
    // 3. Set minimum throttle
    for (int i = 0; i < 4; i++) {
        pwm_set_gpio_level(esc->config.motor1_pin + i, 
                          throttle_to_duty(esc, MIN_THROTTLE));
    }
    
    // 4. Wait for ESC to recognize min throttle
    sleep_ms(2000);
}

void esc_arm(esc_controller_t* esc) {
    // Send minimum throttle signal to arm ESCs
    for (int i = 0; i < 4; i++) {
        pwm_set_gpio_level(esc->config.motor1_pin + i, 
                          throttle_to_duty(esc, MIN_THROTTLE));
    }
    
    // Wait for ESCs to initialize
    sleep_ms(1000);
    
    esc->is_armed = true;
}

void esc_disarm(esc_controller_t* esc) {
    // Set all motors to zero
    for (int i = 0; i < 4; i++) {
        pwm_set_gpio_level(esc->config.motor1_pin + i, 0);
    }
    
    esc->is_armed = false;
}

void esc_set_output(esc_controller_t* esc, float m1, float m2, float m3, float m4) {
    if (!esc->is_armed) return;
    
    // Convert throttle values to PWM duty cycles
    uint16_t duty1 = throttle_to_duty(esc, m1);
    uint16_t duty2 = throttle_to_duty(esc, m2);
    uint16_t duty3 = throttle_to_duty(esc, m3);
    uint16_t duty4 = throttle_to_duty(esc, m4);
    
    // Set PWM levels
    pwm_set_gpio_level(esc->config.motor1_pin, duty1);
    pwm_set_gpio_level(esc->config.motor2_pin, duty2);
    pwm_set_gpio_level(esc->config.motor3_pin, duty3);
    pwm_set_gpio_level(esc->config.motor4_pin, duty4);
}

void esc_emergency_stop(esc_controller_t* esc) {
    // Immediately stop all motors
    for (int i = 0; i < 4; i++) {
        pwm_set_gpio_level(esc->config.motor1_pin + i, 0);
    }
    
    esc->is_armed = false;
}

// Example usage in main.c:
const esc_config_t DEFAULT_ESC_CONFIG = {
    .motor1_pin = PIN_MOTOR1,
    .motor2_pin = PIN_MOTOR2,
    .motor3_pin = PIN_MOTOR3,
    .motor4_pin = PIN_MOTOR4,
    .min_pulse_ms = 1.0f,    // 1ms minimum pulse
    .max_pulse_ms = 2.0f,    // 2ms maximum pulse
    .pwm_freq = 400,         // 400Hz update rate
    .clock_div = 64,         // Clock divider
    .wrap_value = 62500      // For 400Hz with system clock at 125MHz
};
