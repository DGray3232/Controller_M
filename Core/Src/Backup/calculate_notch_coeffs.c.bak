
#include "main.h"
#include "arm_math.h"
#include <math.h>
#include "calculate_notch_coeffs.h"

/**
 * @brief Рассчитывает коэффициенты для режекторного фильтра 2-го порядка (2 каскада)
 * @param sampleRate Частота дискретизации в Гц
 * @param centerFreq Центральная частота подавления в Гц
 * @param bandwidth Полоса подавления в Гц (-3 dB)
 * @param coeffs Указатель на массив коэффициентов (10 элементов для 2 стадий)
 * @param alpha_coeffs роцент от расчитанных коэффицентов. Пример alpha_coeffs = 0.9f 90% нового значения, 10% старого
 */

void calculateNotchFilterCoeffs(float sampleRate, float centerFreq, float bandwidth, float32_t* coeffs, float alpha_coeffs) {
    // Если частота некорректна или полоса <= 0, создаем пропускной фильтр (байпас)
    if (centerFreq <= 0.0f || bandwidth <= 0.0f || centerFreq >= sampleRate / 2.0f) {
        for (int i = 0; i < 2; i++) {
            coeffs[i*5 + 0] = 1.0f; // b0
            coeffs[i*5 + 1] = 0.0f; // b1
            coeffs[i*5 + 2] = 0.0f; // b2
            coeffs[i*5 + 3] = 0.0f; // a1
            coeffs[i*5 + 4] = 0.0f; // a2
        }
        return;
    }
    float32_t temp_coeffs[10] = {0};
    // Нормализация частоты
    float omega = 2.0f * M_PI * centerFreq / sampleRate;
    float Q = centerFreq / bandwidth;
    float alpha = sinf(omega) / (2.0f * Q);
    // Проверка, чтобы alpha не был слишком большим, что может привести к нестабильности
    if (alpha >= 1.0f) {
        alpha = 0.999f;
    }
    // Расчет коэффициентов (до нормализации)
    float b0 = 1.0f;
    float b1 = -2.0f * cosf(omega);
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosf(omega);
    float a2 = 1.0f - alpha;
    // Нормализация коэффициентов для одной стадии
    temp_coeffs[0] = b0 / a0;      // b0
    temp_coeffs[1] = b1 / a0;      // b1
    temp_coeffs[2] = b2 / a0;      // b2
    temp_coeffs[3] = -a1 / a0;     // a1 (с обратным знаком для CMSIS)
    temp_coeffs[4] = -a2 / a0;     // a2 (с обратным знаком для CMSIS)
    // Копируем коэффициенты для второй стадии
    for (int i = 0; i < 5; i++) {
    	temp_coeffs[5 + i] = temp_coeffs[i];
    }
    for (int i = 0; i < 10; i++) {
        coeffs[i] = alpha_coeffs * temp_coeffs[i] + (1-alpha_coeffs) * coeffs[i];
    }
}
