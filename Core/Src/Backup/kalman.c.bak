#include "kalman.h"

/**
  * @brief  Инициализирует структуру фильтра Калмана с заданными параметрами
  * @param  kf Указатель на структуру KalmanFilter
  * @param  q   Дисперсия процесса (Process noise covariance)
  *             Определяет, насколько быстро фильтр реагирует на изменения состояния.
  *             Большие значения - быстрее реакция, но больше шум.
  * @param  r   Дисперсия измерения (Measurement noise covariance)
  *             Отражает доверие к показаниям датчика.
  *             Большие значения - меньше доверия измерениям.
  * @param  initial_state_uncertainty Начальная неопределенность состояния
  *             (Initial estimation error covariance)
  * @param  state Начальное значение состояния
  * @retval None
  * @note   Рекомендуемые начальные значения:
  *         - Q: 0.01 - 0.0001 (зависит от изменчивости процесса)
  *         - R: 0.1 - 1.0 (зависит от точности датчика)
  *         - initial_state_uncertainty: 1.0 (можно начинать с высокой неопределенности)
  */
void kalman_filter_init(KalmanFilter *kf, float q, float r, float initial_state_uncertainty, float state) {
    kf->x_est = state;
    kf->P_est = initial_state_uncertainty;
    kf->Q = q;
    kf->R = r;
}

/**
  * @brief  Выполняет обновление фильтра Калмана на основе нового измерения
  * @param  kf Указатель на структуру KalmanFilter
  * @param  measurement Новое измерение от датчика
  * @retval Отфильтрованное (оцененное) значение состояния
  * @note   Алгоритм выполняет:
  *         1. Предсказание состояния (Prediction step)
  *         2. Коррекцию на основе измерения (Update step)
  *         3. Обновление ковариации ошибки оценки
  */
float kalman_filter_update(KalmanFilter *kf, float measurement) {
    float P_pred = kf->P_est + kf->Q;
    float K = P_pred / (P_pred + kf->R);
    kf->x_est = kf->x_est + K * (measurement - kf->x_est);
    kf->P_est = (1.0f - K) * P_pred;
    return kf->x_est;
}
