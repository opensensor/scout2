#include "pid_controller_tests.h"
#include "../src/core/pid_controller.h"
#include <stdlib.h>
#include <math.h>

bool test_pid_initialization() {
    float kp = 1.0f, ki = 0.1f, kd = 0.05f;
    pid_controller_t* pid = pid_controller_init(kp, ki, kd);
    
    ASSERT_TRUE(pid != NULL);
    ASSERT_FLOAT_EQUAL(kp, pid->kp, 0.0001f);
    ASSERT_FLOAT_EQUAL(ki, pid->ki, 0.0001f);
    ASSERT_FLOAT_EQUAL(kd, pid->kd, 0.0001f);
    
    free(pid);
    return true;
}

bool test_pid_reset() {
    pid_controller_t* pid = pid_controller_init(1.0f, 0.1f, 0.05f);
    
    // Run PID to accumulate some state
    pid_controller_update(pid, 1.0f, 0.1f);
    pid_controller_reset(pid);
    
    ASSERT_FLOAT_EQUAL(0.0f, pid->integral, 0.0001f);
    ASSERT_FLOAT_EQUAL(0.0f, pid->prev_error, 0.0001f);
    
    free(pid);
    return true;
}

bool test_pid_output_limits() {
    pid_controller_t* pid = pid_controller_init(1.0f, 0.1f, 0.05f);
    pid_controller_set_limits(pid, 0.5f, 0.25f);
    
    float output = pid_controller_update(pid, 2.0f, 0.1f);
    ASSERT_FLOAT_EQUAL(0.5f, output, 0.0001f);
    
    output = pid_controller_update(pid, -2.0f, 0.1f);
    ASSERT_FLOAT_EQUAL(-0.5f, output, 0.0001f);
    
    free(pid);
    return true;
}