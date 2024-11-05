#include "attitude_estimator_tests.h"
#include "../src/core/attitude_estimator.h"
#include <stdlib.h>
#include <math.h>

bool test_attitude_estimator_initialization() {
    attitude_estimator_t* estimator = attitude_estimator_init();
    
    ASSERT_TRUE(estimator != NULL);
    ASSERT_FLOAT_EQUAL(1.0f, estimator->quaternion.q0, 0.0001f);
    ASSERT_FLOAT_EQUAL(0.0f, estimator->quaternion.q1, 0.0001f);
    ASSERT_FLOAT_EQUAL(0.0f, estimator->quaternion.q2, 0.0001f);
    ASSERT_FLOAT_EQUAL(0.0f, estimator->quaternion.q3, 0.0001f);
    
    free(estimator);
    return true;
}

bool test_attitude_estimator_level() {
    attitude_estimator_t* estimator = attitude_estimator_init();
    
    vector3_t accel = {0.0f, 0.0f, 1.0f};
    vector3_t gyro = {0.0f, 0.0f, 0.0f};
    
    attitude_estimator_update(estimator, &accel, &gyro, 0.002f);
    attitude_t attitude = attitude_estimator_get_attitude(estimator);
    
    ASSERT_FLOAT_EQUAL(0.0f, attitude.roll, 0.1f);
    ASSERT_FLOAT_EQUAL(0.0f, attitude.pitch, 0.1f);
    
    free(estimator);
    return true;
}
