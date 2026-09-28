#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include "main.h"
#include "arm_math.h"

void calculateNotchFilterCoeffs(float sampleRate, float centerFreq, float bandwidth, float32_t* coeffs, float alpha_coeffs);

#endif // NOTCH_FILTER_H
