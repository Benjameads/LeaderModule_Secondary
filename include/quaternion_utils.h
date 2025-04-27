#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H

void quaternion_to_euler(const float q[4], float* yaw, float* pitch, float* roll);
void quaternion_difference(const float q1[4], const float q2[4], float q_diff[4]);
void quaternion_to_axis_angle(const float q[4], float axis[3], float* angle);

#endif
