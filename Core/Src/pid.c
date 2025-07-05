#include "pid.h"
#include <math.h>
#include "limits.h"

#define CONTROL_LOOP_DT 0.001f // 1000 Гц

/**
  * @brief  Инициализирует структуру PID-контроллера с заданными параметрами
  * @param  pid Указатель на структуру PID_Controller
  * @param  Kp Коэффициент пропорциональной составляющей
  * @param  Ki Коэффициент интегральной составляющей
  * @param  Kd Коэффициент дифференциальной составляющей
  * @param  alpha Коэффициент фильтрации выходного сигнала (0.0 - 1.0)
  * @param  alpha_derivative Коэффициент фильтрации производной (0.0 - 1.0)
  * @param  integral_limit Лимит интегральной составляющей (анти-винт)
  * @param  scale_factor Масштабирующий коэффициент выходного сигнала
  * @retval None
  */
void PID_Init(PID_Controller *pid, double Kp, double Ki, double Kd, double alpha, double alpha_derivative, double integral_limit, double scale_factor) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->alpha = alpha;
    pid->alpha_derivative = alpha_derivative;
    pid->integral_limit = integral_limit;
    pid->scale_factor = scale_factor;
    pid->integral = 0.0;
    pid->error_previous = 0.0;
    pid->previous_output = 0.0;
    pid->previous_derivative = 0.0;
}

/**
  * @brief  Вычисляет управляющий сигнал PID-контроллера
  * @param  pid Указатель на структуру PID_Controller
  * @param  error Текущая ошибка регулирования (заданное значение - текущее значение)
  * @param  trim Корректирующее смещение для ошибки регулирования
  * @retval Отфильтрованный и масштабированный управляющий сигнал
  * @note   Функция включает:
  *         - ограничение интегральной составляющей
  *         - фильтрацию производной
  *         - фильтрацию выходного сигнала
  *         - защиту от переполнения
  *         - обработку NaN/INF значений
  */
double PID_Compute(PID_Controller *pid, double error, double trim) {
    double dt = CONTROL_LOOP_DT;
    // Проверка на NaN / INF
    if (isnan(error) || isinf(error)) {
        error = 0.0;
    }
    error += trim;  // Добавляем трим к ошибке
    // интегральная часть
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit; // ограничение интегральной суммы
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit; // ограничение интегральной суммы
    }
    // Производная часть с фильтрацией
    double raw_derivative = (error - pid->error_previous) / dt;
    double filtered_derivative = pid->alpha_derivative * raw_derivative + (1 - pid->alpha_derivative) * pid->previous_derivative;
    pid->previous_derivative = filtered_derivative;
    pid->error_previous = error;
    // Вычисление управляющего сигнала
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * filtered_derivative;
    double safe_output = output;
    if (output > INT_MAX) safe_output = INT_MAX;
    else if (output < -INT_MAX) safe_output = -INT_MAX;
    // Фильтрация выходного сигнала
    double filtered_output = pid->alpha * safe_output + (1 - pid->alpha) * pid->previous_output;
    // Проверка на NaN / INF
    if (isnan(filtered_output) || isinf(filtered_output)) {
        filtered_output = 0.0;
        pid->integral = 0.0;  // Сброс интегратора при ошибке
    }
    // Масштабирование выходного сигнала
    filtered_output *= pid->scale_factor;
    pid->previous_output = filtered_output; // сохранение текущего выходного сигнала для следующего шага
    return filtered_output; // возвращение отфильтрованного управляющего сигнала
}
