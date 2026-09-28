#ifndef _AHRSALOGRITHMS_H_
#define _AHRSALOGRITHMS_H_
#include <stdint.h>

void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt);

const float * getQ();

#endif // _AHRSALOGRITHMS_H_
