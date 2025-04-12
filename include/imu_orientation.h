
#ifndef IMU_ORIENTATION_H
#define IMU_ORIENTATION_H

#include "MadgwickAHRS.h"
#include "IMU_READ.h"  // Assumed user header with struct IMUData

#define GYRO_500_SCALE_RAD (GYRO_500_SCALE*(3.14159265f / 180.0f))

void update_imu_orientation_from_raw(
    Madgwick* filter,
    const struct IMUData* sample,
    float dt,
    float* yaw_deg, float* pitch_deg, float* roll_deg
);

#endif
