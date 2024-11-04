#pragma once

#include "types.h"

typedef struct {
    quaternion_t quaternion;
    vector3_t gyro_bias;
    float filter_alpha;
} attitude_estimator_t;

attitude_estimator_t* attitude_estimator_init(void);
void attitude_estimator_update(attitude_estimator_t* estimator, 
                             const vector3_t* accel, 
                             const vector3_t* gyro, 
                             float dt);
attitude_t attitude_estimator_get_attitude(const attitude_estimator_t* estimator);
