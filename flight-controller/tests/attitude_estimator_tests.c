#include "attitude_estimator_tests.h"
#include "../src/core/attitude_estimator.h"
#include <stdlib.h>
#include <math.h>

void test_attitude_estimator_initialization(void) {
    attitude_estimator_t* estimator = attitude_estimator_init();
    
    TEST_ASSERT_NOT_NULL(estimator);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, estimator->quaternion.q0);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, estimator->quaternion.q1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, estimator->quaternion.q2);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, estimator->quaternion.q3);
    
    free(estimator);
}

void test_attitude_estimator_level(void) {
    attitude_estimator_t* estimator = attitude_estimator_init();
    
    vector3_t accel = {0.0f, 0.0f, 1.0f};
    vector3_t gyro = {0.0f, 0.0f, 0.0f};
    
    attitude_estimator_update(estimator, &accel, &gyro, 0.002f);
    attitude_t attitude = attitude_estimator_get_attitude(estimator);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, attitude.pitch);
    
    free(estimator);
}
