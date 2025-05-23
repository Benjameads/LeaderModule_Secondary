#ifndef RELATIVE_ORIENTATION_H
#define RELATIVE_ORIENTATION_H

#include <math.h>

typedef struct {
    float angle;      // angle between vectors (radians)
    float axis[3];    // rotation axis
    float curl;       // X component
    float twist;      // Y component
    float spread;     // Z component
} RelativeRotation;

//const float reference_vector[3] = {0, 1, 0}; // Reference vector for comparison (Y+ or extended finger pointing forward)

void normalize(float v[3]);
float safe_dot(const float a[3], const float b[3]);
void quaternion_rotate_vector(const float q[4], const float v[3], float out[3]);
void compare_orientation(const float vec_a[3], const float vec_b[3], RelativeRotation* result);
void cross_product(const float a[3], const float b[3], float result[3]);
void project_onto_plane(const float v[3], const float normal[3], float out[3]);
float angle_between_projected_vectors(const float a[3], const float b[3], const float axis[3]);
float dot_product(const float a[3], const float b[3]);
float rotation_about_axis(const float q_start[4], const float q_current[4], const float axis_local[3], const float ref_local[3]);
void compute_relative_quaternion(const float q_ref[4], const float q_target[4], float q_relative[4]);

#endif // RELATIVE_ORIENTATION_H
