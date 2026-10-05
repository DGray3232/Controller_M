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
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float alpha, float alpha_derivative, float integral_limit, float scale_factor) {
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
float PID_Compute(PID_Controller *pid, float error, float dt) {
	//float dt = CONTROL_LOOP_DT;
    // Проверка на NaN / INF
    if (isnan(error) || isinf(error)) {
        error = 0.0;
    }
    // интегральная часть
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit; // ограничение интегральной суммы
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit; // ограничение интегральной суммы
    }
    // Производная часть с фильтрацией
    float raw_derivative = (error - pid->error_previous) / dt;
    float filtered_derivative = pid->alpha_derivative * raw_derivative + (1 - pid->alpha_derivative) * pid->previous_derivative;
    pid->previous_derivative = filtered_derivative;
    pid->error_previous = error;
    // Вычисление управляющего сигнала
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * filtered_derivative;
    float safe_output = output;
    if (output > INT_MAX) safe_output = INT_MAX;
    else if (output < -INT_MAX) safe_output = -INT_MAX;
    // Фильтрация выходного сигнала
    float filtered_output = pid->alpha * safe_output + (1 - pid->alpha) * pid->previous_output;
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


/**
 * @brief  Сбрасывает накопленные состояния PID-регулятора (для altitude_pid)
 */
void PID_Reset(PID_Controller *pid) {
    pid->integral = 0.0;
    pid->error_previous = 0.0;
    pid->previous_output = 0.0;
    pid->previous_derivative = 0.0;
}

/**
 * @brief  Сбрасывает накопленные состояния PID-2-регулятора
 */
void PID_2_Reset(PID_2_Controller *pid) {
    pid->integral = 0.0;               // Обнуление интегральной суммы
    pid->error_previous = 0.0;         // Сброс предыдущей ошибки
    pid->previous_output = 0.0;        // Сброс предыдущего выхода
    pid->previous_derivative = 0.0;    // Сброс отфильтрованной производной
}

/**
 * @brief  Инициализирует PID DoM регулятор
  */
void PID_DoM_Init(PID_DoM_Controller *pid, float Kp, float Ki, float Kd, float alpha, float alpha_derivative, float integral_limit, float scale_factor) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->alpha = alpha;
    pid->alpha_derivative = alpha_derivative;
    pid->integral_limit = integral_limit;
    pid->scale_factor = scale_factor;
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->previous_output = 0.0f;
    pid->previous_derivative = 0.0f;
}

/**
 * @brief  Сброс состояния регулятора
 */
void PID_DoM_Reset(PID_DoM_Controller *pid) {
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->previous_output = 0.0f;
    pid->previous_derivative = 0.0f;
}

/**
 * @brief  Вычисляет управляющий сигнал (Derivative on Measurement)
 * @param  error ошибка
 * @param  measurement Текущее значение с датчика
 */
float PID_DoM_Compute(PID_DoM_Controller *pid, float error, float measurement, float dt) {
    //float dt = CONTROL_LOOP_DT;
    // 1. Обработка входных данных
    if (isnan(error) || isinf(error)) {
        error = 0.0;
    }
    if (isnan(measurement) || isinf(measurement)) {
        measurement = pid->previous_measurement; // Используем предыдущее значение
    }
    // 2. Интегральная часть (классическая)
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit; // ограничение интегральной суммы
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit; // ограничение интегральной суммы
    }
    // 3. Дифференциальная часть (от ИЗМЕРЕНИЯ)
    // Важно: берем (current - previous), что дает положительную производную при росте измерения.
    // В итоговой формуле мы будем ВЫЧИТАТЬ эту часть.
    float raw_derivative = (measurement - pid->previous_measurement) / dt;
    // Фильтрация производной (D-фильтр)
    float filtered_derivative = pid->alpha_derivative * raw_derivative + (1.0f - pid->alpha_derivative) * pid->previous_derivative;
    pid->previous_derivative = filtered_derivative;
    pid->previous_measurement = measurement;
    // 4. Вычисление выхода
    // Формула: Out = Kp*error + Ki*integral - Kd*(d_measurement/dt)
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) - (pid->Kd * filtered_derivative);
    // 5. Ограничение и фильтрация выхода (как в ваших примерах)
    if (output > (float)INT_MAX) output = (float)INT_MAX;
    else if (output < -(float)INT_MAX) output = -(float)INT_MAX;
    float filtered_output = pid->alpha * output + (1.0f - pid->alpha) * pid->previous_output;
    if (isnan(filtered_output) || isinf(filtered_output)) {
        filtered_output = 0.0f;
        pid->integral = 0.0f;
    }
    filtered_output *= pid->scale_factor;
    pid->previous_output = filtered_output;
    return filtered_output;
}
