
#include "quaternion_utils.h"
#include <math.h>

void quaternion_to_euler(const float q[4], float* yaw, float* pitch, float* roll) {
    float w = q[0], x = q[1], yv = q[2], z = q[3];
    *yaw   = atan2f(2.0f*(w*z + x*yv), 1.0f - 2.0f*(yv*yv + z*z));
    *pitch = asinf(2.0f*(w*yv - z*x));
    *roll  = atan2f(2.0f*(w*x + yv*z), 1.0f - 2.0f*(x*x + yv*yv));
}
