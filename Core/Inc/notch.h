#ifndef NOTCH_H_
#define NOTCH_H_

// Структура для notch-фильтра
typedef struct {
    float b0, b1, b2, a0, a1, a2;
    float x1, x2, y1, y2;
} NotchFilter;

// Инициализация notch-фильтра
void init_notch_filter(NotchFilter* f, float center_freq, float bandwidth, float sample_rate);
// Применение notch-фильтра
float apply_notch_filter(NotchFilter* f, float input);

#endif /* NOTCH_H_ */
