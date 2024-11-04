#include "flight_controller.h"
#include "drivers/mpu6050.h"
#include "drivers/esc.h"
#include "utils/logger.h"
#include "include/config.h"
#include <stdlib.h>

flight_controller_t* flight_controller_init(void) {
    flight_controller_t* fc = malloc(sizeof(flight_controller_t));
    if (fc == NULL) return NULL;

    // Initialize IMU
    mpu6050_config_t imu_config = {
        .gyro_range = 1,        // ±500°/s
        .accel_range = 1,       // ±4g
        .dlpf_bandwidth = 2,    // 92Hz bandwidth
        .sample_rate_div = 4    // 200Hz sample rate
    };
    fc->imu = mpu6050_init(PIN_I2C_SDA, PIN_I2C_SCL, &imu_config);
    if (fc->imu == NULL) {
        free(fc);
        return NULL;
    }

    fc->attitude_estimator = attitude_estimator_init();
    fc->pid_roll = pid_controller_init(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
    fc->pid_pitch = pid_controller_init(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
    fc->pid_yaw = pid_controller_init(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
    
    // Initialize setpoint to zero
    fc->setpoint.roll = 0.0f;
    fc->setpoint.pitch = 0.0f;
    fc->setpoint.yaw = 0.0f;
    fc->setpoint.throttle = 0.0f;
    
    return fc;
}


struct flight_controller {
    mpu6050_t* imu;
    attitude_estimator_t* attitude_estimator;
    pid_controller_t* pid_roll;
    pid_controller_t* pid_pitch;
    pid_controller_t* pid_yaw;
    esc_controller_t* esc;
    flight_mode_t current_mode;
    setpoint_t setpoint;
};


void flight_controller_update(flight_controller_t* fc) {
    vector3_t accel, gyro;
    mpu6050_read_scaled(fc->imu, &accel, &gyro);

    attitude_estimator_update(fc->attitude_estimator, &accel, &gyro, DT);
    attitude_t current_attitude = attitude_estimator_get_attitude(fc->attitude_estimator);

    float roll_output = pid_controller_update(fc->pid_roll, 
                                            fc->setpoint.roll - current_attitude.roll,
                                            DT);

    float pitch_output = pid_controller_update(fc->pid_pitch,
                                             fc->setpoint.pitch - current_attitude.pitch,
                                             DT);

    float yaw_output = pid_controller_update(fc->pid_yaw,
                                           fc->setpoint.yaw - current_attitude.yaw,
                                           DT);

    // Calculate motor outputs
    float m1 = fc->setpoint.throttle + roll_output + pitch_output + yaw_output;
    float m2 = fc->setpoint.throttle - roll_output + pitch_output - yaw_output;
    float m3 = fc->setpoint.throttle - roll_output - pitch_output + yaw_output;
    float m4 = fc->setpoint.throttle + roll_output - pitch_output - yaw_output;

    esc_set_output(fc->esc, m1, m2, m3, m4);
}

void flight_controller_cleanup(flight_controller_t* fc) {
    if (fc == NULL) return;
    
    // Clean up components
    if (fc->pid_roll) free(fc->pid_roll);
    if (fc->pid_pitch) free(fc->pid_pitch);
    if (fc->pid_yaw) free(fc->pid_yaw);
    if (fc->attitude_estimator) free(fc->attitude_estimator);
    if (fc->imu) free(fc->imu);
    if (fc->esc) free(fc->esc);
    
    free(fc);
}
