
#include "imu_orientation.h"
#include "MadgwickAHRS.h"
#include "quaternion_utils.h"
#include <math.h>

#define RAD2DEG(x) ((x) * 57.2957795f)

void update_imu_orientation_from_raw( Madgwick* filter, const struct IMUData* sample, float dt, float* yaw_deg, float* pitch_deg, float* roll_deg) {
    float ax = (float)sample->accelX * ACCEL_4G_SCALE;
    float ay = (float)sample->accelY * ACCEL_4G_SCALE;
    float az = (float)sample->accelZ * ACCEL_4G_SCALE;

    float gx = (float)sample->gyroX * GYRO_500_SCALE_RAD;
    float gy = (float)sample->gyroY * GYRO_500_SCALE_RAD;
    float gz = (float)sample->gyroZ * GYRO_500_SCALE_RAD;

    float mx = (float)sample->magX * MAG_SCALE;
    float my = (float)sample->magY * MAG_SCALE;
    float mz = (float)sample->magZ * MAG_SCALE;

    filter->sampleFreq = 1.0f / dt;

    MadgwickAHRSupdate(filter, gx, gy, gz, ax, ay, az, mx, my, mz);

    quaternion_to_euler(filter->q, yaw_deg, pitch_deg, roll_deg);

    *yaw_deg   = RAD2DEG(*yaw_deg);
    *pitch_deg = RAD2DEG(*pitch_deg);
    *roll_deg  = RAD2DEG(*roll_deg);
}
