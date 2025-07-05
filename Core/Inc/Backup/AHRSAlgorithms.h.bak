#ifndef _AHRSALOGRITHMS_H_
#define _AHRSALOGRITHMS_H_
#include <stdint.h>

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MadgwickQuaternionUpdate_not_mag(float ax, float ay, float az, float gx, float gy, float gz, float deltat);

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
void MahonyQuaternionUpdate_not_mag(float ax, float ay, float az,
									float gx, float gy, float gz, float deltat);

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float deltat);

void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt);

void MahonyQuaternionUpdate_mag(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);

const float * getQ();

#endif // _AHRSALOGRITHMS_H_
