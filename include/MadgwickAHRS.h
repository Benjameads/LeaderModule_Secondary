
#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

typedef struct {
    float sampleFreq;
    float beta;
    float q[4]; // quaternion of sensor frame relative to auxiliary frame
} Madgwick;

void MadgwickInit(Madgwick* state);
void MadgwickAHRSupdate(Madgwick* state, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif
