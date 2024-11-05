#include "pid_controller_tests.h"
#include "../src/core/pid_controller.h"
#include <stdlib.h>
#include <math.h>

void test_pid_initialization(void) {
    float kp = 1.0f, ki = 0.1f, kd = 0.05f;
    pid_controller_t* pid = pid_controller_init(kp, ki, kd);
    
    TEST_ASSERT_NOT_NULL(pid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, kp, pid->kp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, ki, pid->ki);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, kd, pid->kd);
    
    free(pid);
}

void test_pid_reset(void) {
    pid_controller_t* pid = pid_controller_init(1.0f, 0.1f, 0.05f);
    
    // Run PID to accumulate some state
    pid_controller_update(pid, 1.0f, 0.1f);
    pid_controller_reset(pid);
    
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid->integral);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid->prev_error);
    
    free(pid);
}

void test_pid_output_limits(void) {
    pid_controller_t* pid = pid_controller_init(1.0f, 0.1f, 0.05f);
    pid_controller_set_limits(pid, 0.5f, 0.25f);
    
    float output = pid_controller_update(pid, 2.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, output);
    
    output = pid_controller_update(pid, -2.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.5f, output);
    
    free(pid);
}
