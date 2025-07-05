#ifndef KALMAN_H_
#define KALMAN_H_

// Структура для фильтра Калмана
typedef struct {
    float x_est; // Оценка состояния (угловая скорость)
    float P_est; // Ковариация ошибки состояния
    float Q;     // Шум процесса
    float R;     // Шум измерения
} KalmanFilter;

// Функция инициализации фильтра Калмана
void kalman_filter_init(KalmanFilter *kf, float q, float r, float initial_state_uncertainty, float state);
// Функция обновления фильтра Калмана
float kalman_filter_update(KalmanFilter *kf, float measurement);

#endif /* KALMAN_H_ */
