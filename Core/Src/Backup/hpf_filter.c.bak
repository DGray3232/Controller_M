#include "hpf_filter.h"


/**
  * @brief  Инициализирует структуру фильтра высоких частот (ФВЧ)
  * @param  filter Указатель на структуру HPF
  * @param  alpha Коэффициент фильтрации (0.0 - 1.0)
  *         Определяет степень фильтрации:
  *         - Ближе к 0.0: сильное подавление низких частот (более агрессивный фильтр)
  *         - Ближе к 1.0: слабое подавление низких частот (более мягкий фильтр)
  * @param  initial_value Начальное значение сигнала
  *         Используется для инициализации предыдущих состояний фильтра
  * @retval None
  */

void HPF_Init(HPF *filter, float alpha, float initial_value) {
    filter->alpha = alpha;
    filter->prev_x = initial_value;
    filter->prev_y = initial_value;
}
/**
  * @brief  Вычисляет отфильтрованное значение ФВЧ
  * @param  filter Указатель на инициализированную структуру HPF
  * @param  new_x Новое входное значение сигнала
  * @retval Отфильтрованное выходное значение
  */
float HPF_Update(HPF *filter, float new_x) {
    float filtered = filter->alpha * filter->prev_y +
                     filter->alpha * (new_x - filter->prev_x);
    filter->prev_x = new_x;
    filter->prev_y = filtered;
    return filtered;
}
