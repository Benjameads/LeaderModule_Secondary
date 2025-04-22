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

void normalize_vector(float v[3]);
float safe_dot(const float a[3], const float b[3]);
void quaternion_rotate_vector(const float q[4], const float v[3], float out[3]);
void compare_orientation(const float vec_a[3], const float vec_b[3], RelativeRotation* result);

#endif // RELATIVE_ORIENTATION_H
