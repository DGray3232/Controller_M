
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"  // Для предопределенных структур БПФ

#ifndef VIBRATION_ANALYSIS_H
#define VIBRATION_ANALYSIS_H

// Объявление структуры параметров FFT как extern
extern arm_rfft_fast_instance_f32 fft_params;

float calculate_vibration_frequency(float *input_buffer, int fft_len, int f_sample, float *fft_output_buffer, float *fft_magnitude_buffer);

#endif // VIBRATION_ANALYSIS_H
