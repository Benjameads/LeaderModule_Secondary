#include "relative_orientation.h"

void normalize_vector(float v[3]) {
    float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 1e-6f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
}

float safe_dot(const float a[3], const float b[3]) {
    float d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    return d;
}

void quaternion_rotate_vector(const float q[4], const float v[3], float out[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // Rotation matrix from quaternion
    float r00 = 1 - 2 * (y * y + z * z);
    float r01 = 2 * (x * y - w * z);
    float r02 = 2 * (x * z + w * y);

    float r10 = 2 * (x * y + w * z);
    float r11 = 1 - 2 * (x * x + z * z);
    float r12 = 2 * (y * z - w * x);

    float r20 = 2 * (x * z - w * y);
    float r21 = 2 * (y * z + w * x);
    float r22 = 1 - 2 * (x * x + y * y);

    out[0] = r00 * v[0] + r01 * v[1] + r02 * v[2];
    out[1] = r10 * v[0] + r11 * v[1] + r12 * v[2];
    out[2] = r20 * v[0] + r21 * v[1] + r22 * v[2];
}

void compare_orientation(const float vec_a[3], const float vec_b[3], RelativeRotation* result) {
    float a[3] = { vec_a[0], vec_a[1], vec_a[2] };
    float b[3] = { vec_b[0], vec_b[1], vec_b[2] };
    normalize_vector(a);
    normalize_vector(b);

    // Cross product = rotation axis
    result->axis[0] = a[1]*b[2] - a[2]*b[1];
    result->axis[1] = a[2]*b[0] - a[0]*b[2];
    result->axis[2] = a[0]*b[1] - a[1]*b[0];
    normalize_vector(result->axis);

    // Angle
    result->angle = acosf(safe_dot(a, b));

    // Decompose motion
    result->curl   = result->axis[0] * result->angle;
    result->twist  = result->axis[1] * result->angle;
    result->spread = result->axis[2] * result->angle;
}

// Computes the cross product: result = a × b
void cross_product(const float a[3], const float b[3], float result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1]; // X
    result[1] = a[2] * b[0] - a[0] * b[2]; // Y
    result[2] = a[0] * b[1] - a[1] * b[0]; // Z
}

// Projects vector v onto a plane orthogonal to normal n
void project_onto_plane(const float v[3], const float n[3], float out[3]) {
    float dot = v[0]*n[0] + v[1]*n[1] + v[2]*n[2];  // v ⋅ n
    out[0] = v[0] - dot * n[0];
    out[1] = v[1] - dot * n[1];
    out[2] = v[2] - dot * n[2];
}

float angle_between_projected_vectors(const float a[3], const float b[3], const float axis[3]) {
    float a_norm[3] = {a[0], a[1], a[2]};
    float b_norm[3] = {b[0], b[1], b[2]};
    normalize_vector(a_norm);
    normalize_vector(b_norm);

    float dot = safe_dot(a_norm, b_norm);
    float angle = acosf(dot);

    float cross[3];
    cross_product(a_norm, b_norm, cross);
    float sign = safe_dot(cross, axis);  // Signed rotation around the axis

    return angle * (sign >= 0 ? 1.0f : -1.0f); // Radians
}

