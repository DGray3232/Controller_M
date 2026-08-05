#include "median_moving_average_filter.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

 float median_filter(float new_sample, MedianFilter* filter) {
    // Добавляем новое значение в буфер
    filter->buffer[filter->idx++] = new_sample;
    if (filter->idx >= 5){
    	filter->idx = 0;
    }
    // Создаем временный массив для сортировки
    float temp[5];
    memcpy(temp, filter->buffer, sizeof(filter->buffer));
    // Сортировка пузырьком
    for (int i = 0; i < 5 - 1; i++) {
        for (int j = 0; j < 5 - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                // Меняем элементы местами
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    // Возвращаем медиану (средний элемент)
    return temp[2];
}

void low_pass_filter_init(Filter_lpf* fil, float alpha) {
     fil->lpf_output = 0.0; // �?нициализация предыдущего значения
     fil->alf_lpf = alpha;
 }
 float low_pass_filter(Filter_lpf* fil, float new_reading) {
     // Экспоненциальное сглаживание
     fil->lpf_output = (fil->alf_lpf * new_reading) + ((1 - fil->alf_lpf) * fil->lpf_output);
     return fil->lpf_output;
 }
