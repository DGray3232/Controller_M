
#include "vibration_analysis.h"
#include "main.h"
#include "arm_math.h"
#include <math.h>

/**
 * @brief Рассчитывает частоту вибрации в герцах
 * @param *input_buffer указатель на входной буфер данных
 * @param fft_len Длина FFT (должна быть степенью 2)
 * @param f_sample Частота дискретизации, Гц
 * @param fft_output_buffer буфер для выходных данных FFT
 * @param fft_magnitude_buffer буфер для амплитуд
 */
float calculate_vibration_frequency(float *input_buffer, int fft_len, int f_sample, float *fft_output_buffer, float *fft_magnitude_buffer) {
    // Применение оконной функции Ханна к буферу
    for (int i = 0; i < fft_len; i++) {
        float hann = 0.5f * (1 - arm_cos_f32(2 * PI * i / (fft_len - 1)));
        input_buffer[i] = input_buffer[i] * hann;
    }
    // Выполнение FFT
    arm_rfft_fast_f32(&fft_params, input_buffer, fft_output_buffer, 0);
    // Вычисление амплитуд (только первая половина)
    arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude_buffer, fft_len / 2);
    // Поиск максимального значения (пропускаем DC компоненту i=0)
    float max_magnitude = 0.0f;
    int max_index = 1; // Начинаем с 1, пропускаем DC
    for(int i = 1; i < fft_len/2; i++) {
        if(fft_magnitude_buffer[i] > max_magnitude) {
            max_magnitude = fft_magnitude_buffer[i];
            max_index = i;
        }
    }
    float noise_floor = 1.0f; // Это значение нужно подобрать экспериментально
    if (max_magnitude < noise_floor) {
        return 0.0f; // Вибрации нет
    } else {
        float frequency_bin = (float)f_sample / fft_len;
        float frequency = max_index * frequency_bin;
        return frequency;
    }
}
