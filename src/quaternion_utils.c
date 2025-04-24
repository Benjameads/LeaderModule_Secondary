
#include "quaternion_utils.h"
#include <math.h>
#include <string.h> // for memcpy

void quaternion_to_euler(const float q[4], float* yaw, float* pitch, float* roll) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    *roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    *pitch = asinf(2.0f * (w * y - z * x));
    *yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void quaternion_difference(const float q1[4], const float q2[4], float q_diff[4]) {
    // Computes q_diff = q1⁻¹ * q2
    // q1⁻¹ = [w, -x, -y, -z] for unit quaternions
    float w1 = q1[0], x1 = -q1[1], y1 = -q1[2], z1 = -q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

    q_diff[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q_diff[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q_diff[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q_diff[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

void quaternion_to_axis_angle(const float q[4], float axis[3], float* angle) {
    // Normalize if needed
    float norm = sqrtf(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm < 1e-6f) {
        axis[0] = 1.0f; axis[1] = 0.0f; axis[2] = 0.0f;
        *angle = 0.0f;
        return;
    }

    *angle = 2.0f * acosf(q[0]); // angle = 2 * arccos(w)
    float sin_half_angle = sqrtf(1.0f - q[0]*q[0]);
    axis[0] = q[1] / sin_half_angle;
    axis[1] = q[2] / sin_half_angle;
    axis[2] = q[3] / sin_half_angle;
}

