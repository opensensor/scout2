// src/core/attitude_estimator.c
#include "attitude_estimator.h"
#include <math.h>
#include <stdlib.h>

#define COMPLEMENTARY_FILTER_ALPHA 0.96f
#define GYRO_SENSITIVITY 65.5f     // For ±500 deg/s range
#define ACCEL_SENSITIVITY 4096.0f  // For ±8g range
#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.0174532925f

static void normalize_quaternion(quaternion_t* q) {
    float norm = sqrtf(q->q0 * q->q0 +
                      q->q1 * q->q1 +
                      q->q2 * q->q2 +
                      q->q3 * q->q3);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q->q0 *= inv_norm;
        q->q1 *= inv_norm;
        q->q2 *= inv_norm;
        q->q3 *= inv_norm;
    }
}

attitude_estimator_t* attitude_estimator_init(void) {
    attitude_estimator_t* estimator = malloc(sizeof(attitude_estimator_t));
    if (estimator == NULL) return NULL;

    // Initialize quaternion to no rotation
    estimator->quaternion.q0 = 1.0f;
    estimator->quaternion.q1 = 0.0f;
    estimator->quaternion.q2 = 0.0f;
    estimator->quaternion.q3 = 0.0f;

    // Initialize gyro bias to zero
    estimator->gyro_bias.x = 0.0f;
    estimator->gyro_bias.y = 0.0f;
    estimator->gyro_bias.z = 0.0f;

    // Set default filter value
    estimator->filter_alpha = COMPLEMENTARY_FILTER_ALPHA;

    return estimator;
}

static void update_from_gyro(quaternion_t* q, const vector3_t* gyro, float dt) {
    // Convert gyro readings from deg/s to rad/s
    float gx = gyro->x * DEG_TO_RAD;
    float gy = gyro->y * DEG_TO_RAD;
    float gz = gyro->z * DEG_TO_RAD;

    // Quaternion derivative from angular velocity
    float q0_dot = 0.5f * (-q->q1 * gx - q->q2 * gy - q->q3 * gz);
    float q1_dot = 0.5f * (q->q0 * gx + q->q2 * gz - q->q3 * gy);
    float q2_dot = 0.5f * (q->q0 * gy - q->q1 * gz + q->q3 * gx);
    float q3_dot = 0.5f * (q->q0 * gz + q->q1 * gy - q->q2 * gx);

    // Integrate to get new quaternion
    q->q0 += q0_dot * dt;
    q->q1 += q1_dot * dt;
    q->q2 += q2_dot * dt;
    q->q3 += q3_dot * dt;

    normalize_quaternion(q);
}

static void estimate_from_accel(quaternion_t* q_acc, const vector3_t* accel) {
    float ax = accel->x / ACCEL_SENSITIVITY;
    float ay = accel->y / ACCEL_SENSITIVITY;
    float az = accel->z / ACCEL_SENSITIVITY;

    // Normalize acceleration vector
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.0001f) return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimate roll and pitch from accelerometer
    // Note: Accelerometer cannot detect yaw
    float roll = atan2f(ay, az);
    float pitch = -asinf(ax);

    // Convert to quaternion (assuming yaw = 0)
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    q_acc->q0 = cr * cp;
    q_acc->q1 = sr * cp;
    q_acc->q2 = cr * sp;
    q_acc->q3 = -sr * sp;
}

static void quaternion_slerp(quaternion_t* result,
                           const quaternion_t* q1,
                           const quaternion_t* q2,
                           float t) {
    float cos_omega = q1->q0 * q2->q0 + q1->q1 * q2->q1 +
                     q1->q2 * q2->q2 + q1->q3 * q2->q3;

    quaternion_t q2_adj = *q2;

    if (cos_omega < 0.0f) {
        cos_omega = -cos_omega;
        q2_adj.q0 = -q2->q0;
        q2_adj.q1 = -q2->q1;
        q2_adj.q2 = -q2->q2;
        q2_adj.q3 = -q2->q3;
    }

    float k0, k1;
    if (cos_omega > 0.9995f) {
        // Linear interpolation for small angles
        k0 = 1.0f - t;
        k1 = t;
    } else {
        float omega = acosf(cos_omega);
        float sin_omega = sinf(omega);
        k0 = sinf((1.0f - t) * omega) / sin_omega;
        k1 = sinf(t * omega) / sin_omega;
    }

    result->q0 = k0 * q1->q0 + k1 * q2_adj.q0;
    result->q1 = k0 * q1->q1 + k1 * q2_adj.q1;
    result->q2 = k0 * q1->q2 + k1 * q2_adj.q2;
    result->q3 = k0 * q1->q3 + k1 * q2_adj.q3;

    normalize_quaternion(result);
}

void attitude_estimator_update(attitude_estimator_t* estimator,
                             const vector3_t* accel,
                             const vector3_t* gyro,
                             float dt) {
    // Remove gyro bias
    vector3_t unbiased_gyro = {
        .x = gyro->x - estimator->gyro_bias.x,
        .y = gyro->y - estimator->gyro_bias.y,
        .z = gyro->z - estimator->gyro_bias.z
    };

    // Update quaternion from gyro data
    update_from_gyro(&estimator->quaternion, &unbiased_gyro, dt);

    // Get quaternion estimate from accelerometer
    quaternion_t q_acc;
    estimate_from_accel(&q_acc, accel);

    // Fuse using complementary filter
    quaternion_slerp(&estimator->quaternion,
                    &estimator->quaternion,
                    &q_acc,
                    1.0f - estimator->filter_alpha);
}

attitude_t attitude_estimator_get_attitude(const attitude_estimator_t* estimator) {
    attitude_t attitude;

    // Convert quaternion to Euler angles
    quaternion_t q = estimator->quaternion;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
    float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);
    attitude.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
    if (fabsf(sinp) >= 1.0f)
        attitude.pitch = copysignf(90.0f, sinp); // Use 90° if out of range
    else
        attitude.pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.q0 * q.q3 + q.q1 * q.q2);
    float cosy_cosp = 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3);
    attitude.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    return attitude;
}

void attitude_estimator_calibrate_gyro(attitude_estimator_t* estimator,
                                     const vector3_t* gyro_samples,
                                     int num_samples) {
    vector3_t sum = {0.0f, 0.0f, 0.0f};

    // Calculate average
    for (int i = 0; i < num_samples; i++) {
        sum.x += gyro_samples[i].x;
        sum.y += gyro_samples[i].y;
        sum.z += gyro_samples[i].z;
    }

    estimator->gyro_bias.x = sum.x / num_samples;
    estimator->gyro_bias.y = sum.y / num_samples;
    estimator->gyro_bias.z = sum.z / num_samples;
}