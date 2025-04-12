
#include "MadgwickAHRS.h"
#include <math.h>

#define INV_SQRT(x) (1.0f / sqrtf(x))

void MadgwickInit(Madgwick* state) {
    state->sampleFreq = 100.0f;
    state->beta = 0.1f;
    state->q[0] = 1.0f;
    state->q[1] = 0.0f;
    state->q[2] = 0.0f;
    state->q[3] = 0.0f;
}

void MadgwickAHRSupdate(Madgwick* state, float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz) {
    float q1 = state->q[0], q2 = state->q[1], q3 = state->q[2], q4 = state->q[3];
    float recipNorm, s1, s2, s3, s4;
    float hx, hy, _2bx, _2bz;
    float qDot1, qDot2, qDot3, qDot4;

    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) return;

    recipNorm = INV_SQRT(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = INV_SQRT(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    float _2q1mx = 2.0f * q1 * mx;
    float _2q1my = 2.0f * q1 * my;
    float _2q1mz = 2.0f * q1 * mz;
    float _2q2mx = 2.0f * q2 * mx;
    float hx1 = mx * q1*q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2*q2 +
                2.0f * q2 * my * q3 + 2.0f * q2 * mz * q4 - mx * q3*q3 - mx * q4*q4;
    float hy1 = 2.0f * q1 * mx * q4 + my * q1*q1 - _2q1mz * q2 + 2.0f * q2 * mx * q3 -
                my * q2*q2 + my * q3*q3 + 2.0f * q3 * mz * q4 - my * q4*q4;
    hx = hx1;
    hy = hy1;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -2.0f * q1 * mx * q3 + 2.0f * q1 * my * q2 + mz * q1*q1 + 2.0f * q2 * mx * q4 -
           mz * q2*q2 + 2.0f * q3 * my * q4 - mz * q3*q3 + mz * q4*q4;

    // Gradient descent algorithm corrective step (simplified version)
    s1 = -2.0f * (q3 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) + q4 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay));
    s2 = 2.0f * (q2 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) + q1 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay));
    s3 = -2.0f * q1 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) + 2.0f * q4 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay);
    s4 = 2.0f * q2 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) - 2.0f * q3 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay);

    recipNorm = INV_SQRT(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    s4 *= recipNorm;

    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - state->beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - state->beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - state->beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - state->beta * s4;

    q1 += qDot1 * (1.0f / state->sampleFreq);
    q2 += qDot2 * (1.0f / state->sampleFreq);
    q3 += qDot3 * (1.0f / state->sampleFreq);
    q4 += qDot4 * (1.0f / state->sampleFreq);

    recipNorm = INV_SQRT(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    state->q[0] = q1 * recipNorm;
    state->q[1] = q2 * recipNorm;
    state->q[2] = q3 * recipNorm;
    state->q[3] = q4 * recipNorm;
}
