#include "flight_controller.h"
#include "drivers/mpu6050.h"
#include "drivers/esc.h"
#include "utils/logger.h"
#include <stdlib.h>

struct flight_controller {
    attitude_estimator_t* attitude_estimator;
    pid_controller_t* pid_roll;
    pid_controller_t* pid_pitch;
    pid_controller_t* pid_yaw;
    esc_controller_t* esc;
    flight_mode_t current_mode;
};

flight_controller_t* flight_controller_init(void) {
    flight_controller_t* fc = malloc(sizeof(flight_controller_t));
    if (fc == NULL) return NULL;

    fc->attitude_estimator = attitude_estimator_init();
    fc->pid_roll = pid_controller_init(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
    
    return fc;
}

void flight_controller_update(flight_controller_t* fc) {
    vector3_t accel, gyro;
    mpu6050_read(&accel, &gyro);

    attitude_estimator_update(fc->attitude_estimator, &accel, &gyro, DT);
    attitude_t current_attitude = attitude_estimator_get_attitude(fc->attitude_estimator);

    float roll_output = pid_controller_update(fc->pid_roll, 
                                            fc->setpoint.roll - current_attitude.roll,
                                            DT);
                                            
    esc_set_output(fc->esc, roll_output, pitch_output, yaw_output, throttle);
}
