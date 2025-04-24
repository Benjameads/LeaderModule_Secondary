
#include "MadgwickAHRS.h"
#include <math.h>

#define INV_SQRT(x) (1.0f / sqrtf(x))
static inline float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

void MadgwickInit(Madgwick* state) {
    state->sampleFreq = 100.0f;
    state->samplePeriod = 1.0f / state->sampleFreq;
    state->beta = 0.1f;
    state->q[0] = 1.0f;
    state->q[1] = 0.0f;
    state->q[2] = 0.0f;
    state->q[3] = 0.0f;
}

/******************************************************************************
 * MadgwickAHRSupdate
 * -------------------
 * This function updates the internal quaternion representing orientation 
 * using 9-axis IMU data (gyroscope, accelerometer, and magnetometer).
 *
 * INPUT PARAMETERS:
 *   gx, gy, gz - Gyroscope angular velocity in radians per second (rad/s)
 *   ax, ay, az - Accelerometer measurements in units of gravity (g)
 *   mx, my, mz - Magnetometer measurements in microtesla (uT) or any 
 *                consistent unit; values are normalized internally
 *
 * EXPECTED UNITS:
 *   - Gyroscope: radians/second (rad/s)
 *       > If your IMU gives degrees/second, convert using: radians = degrees * (π / 180)
 *   - Accelerometer: raw values should be scaled to g units
 *       > If using ±2g range and 16-bit output, scale: accel_g = raw / 16384.0
 *   - Magnetometer: typically in microtesla (µT), but relative values also work
 *
 * NOTES:
 *   - The function uses these inputs to perform sensor fusion and update a 
 *     unit quaternion representing the 3D orientation of the device.
 *   - This is part of a complementary filter: accelerometer/magnetometer 
 *     correct drift, while gyro integrates angular velocity.
 *
 * See: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 ******************************************************************************/
void MadgwickAHRSupdate(Madgwick* state, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-state->q[1] * gx - state->q[2] * gy - state->q[3] * gz);
	qDot2 = 0.5f * (state->q[0] * gx + state->q[2] * gz - state->q[3] * gy);
	qDot3 = 0.5f * (state->q[0] * gy - state->q[1] * gz + state->q[3] * gx);
	qDot4 = 0.5f * (state->q[0] * gz + state->q[1] * gy - state->q[2] * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * state->q[0] * mx;
		_2q0my = 2.0f * state->q[0] * my;
		_2q0mz = 2.0f * state->q[0] * mz;
		_2q1mx = 2.0f * state->q[1] * mx;
		_2q0 = 2.0f * state->q[0];
		_2q1 = 2.0f * state->q[1];
		_2q2 = 2.0f * state->q[2];
		_2q3 = 2.0f * state->q[3];
		_2q0q2 = 2.0f * state->q[0] * state->q[2];
		_2q2q3 = 2.0f * state->q[2] * state->q[3];
		q0q0 = state->q[0] * state->q[0];
		q0q1 = state->q[0] * state->q[1];
		q0q2 = state->q[0] * state->q[2];
		q0q3 = state->q[0] * state->q[3];
		q1q1 = state->q[1] * state->q[1];
		q1q2 = state->q[1] * state->q[2];
		q1q3 = state->q[1] * state->q[3];
		q2q2 = state->q[2] * state->q[2];
		q2q3 = state->q[2] * state->q[3];
		q3q3 = state->q[3] * state->q[3];

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * state->q[3] + _2q0mz * state->q[2] + mx * q1q1 + _2q1 * my * state->q[2] + _2q1 * mz * state->q[3] - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * state->q[3] + my * q0q0 - _2q0mz * state->q[1] + _2q1mx * state->q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * state->q[3] - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * state->q[2] + _2q0my * state->q[1] + mz * q0q0 + _2q1mx * state->q[3] - mz * q1q1 + _2q2 * my * state->q[3] - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * state->q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * state->q[3] + _2bz * state->q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * state->q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * state->q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * state->q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * state->q[2] + _2bz * state->q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * state->q[3] - _4bz * state->q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * state->q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * state->q[2] - _2bz * state->q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * state->q[1] + _2bz * state->q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * state->q[0] - _4bz * state->q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * state->q[3] + _2bz * state->q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * state->q[0] + _2bz * state->q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * state->q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= state->beta * s0;
		qDot2 -= state->beta * s1;
		qDot3 -= state->beta * s2;
		qDot4 -= state->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	state->q[0] += qDot1 * state->samplePeriod;
	state->q[1] += qDot2 * state->samplePeriod;
	state->q[2] += qDot3 * state->samplePeriod;
	state->q[3] += qDot4 * state->samplePeriod;

	// Normalise quaternion
	recipNorm = invSqrt(state->q[0] * state->q[0] + state->q[1] * state->q[1] + state->q[2] * state->q[2] + state->q[3] * state->q[3]);
	state->q[0] *= recipNorm;
	state->q[1] *= recipNorm;
	state->q[2] *= recipNorm;
	state->q[3] *= recipNorm;
}

