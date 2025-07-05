#include "notch.h"
#include <math.h>

/**
  * @brief  Инициализирует структуру режекторного (notch) фильтра
  * @param  f Указатель на структуру NotchFilter
  * @param  center_freq Центральная частота подавления в герцах
  *         Диапазон: (0, sample_rate/2)
  * @param  bandwidth Ширина полосы подавления в герцах
  *         Определяет ширину области вокруг центральной частоты, которая будет подавляться
  * @param  sample_rate Частота дискретизации сигнала в герцах
  *         Должна быть как минимум в 2 раза выше center_freq (согласно теореме Найквиста)
  * @retval None
  */
void init_notch_filter(NotchFilter* f, float center_freq, float bandwidth, float sample_rate) {
    float omega = 2.0f * M_PI * center_freq / sample_rate;
    float alpha = sinf(omega) * sinhf(logf(2.0f) / 2.0f * bandwidth * omega / sinf(omega));
    f->b0 = 1.0f;
    f->b1 = -2.0f * cosf(omega);
    f->b2 = 1.0f;
    f->a0 = 1.0f + alpha;
    f->a1 = -2.0f * cosf(omega);
    f->a2 = 1.0f - alpha;
    // Нормализация
    f->b0 /= f->a0; f->b1 /= f->a0; f->b2 /= f->a0;
    f->a1 /= f->a0; f->a2 /= f->a0;
    // Сброс состояний
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}
/**
  * @brief  Применяет режекторный фильтр к входному сигналу
  * @param  f Указатель на инициализированную структуру NotchFilter
  * @param  input Входное значение сигнала
  * @retval Отфильтрованное выходное значение
  */
float apply_notch_filter(NotchFilter* f, float input) {
    float output = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    // Обновление состояний
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;
    return output;
}
