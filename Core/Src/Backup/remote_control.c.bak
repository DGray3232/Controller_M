/* remote_control.c */
#include "remote_control.h"
#include "globals.h"       // Доступ к глобальным переменным (joystick_x, buffer_message и т.д.)
#include "config_param.h"  // Доступ к настройкам (DATA_SIZE, CONNECTION_TIMEOUT_MS)

// Функция для экспоненциального преобразования входного сигнала пульта
float expo_curve(float input, float expo_factor) {
    return input * (1 - expo_factor) + input * input * input * expo_factor;
}

/**
* @brief  Прием и разбор данных с пульта управления
* @param  data_size Размер принятых данных в байтах
* @retval None
* @note   Функция выполняет:
*         1. Проверку минимального размера пакета (не менее 4 байт)
*         2. Расчет контрольной суммы (сумма всех байтов кроме последних двух)
*         3. Сравнение с принятой контрольной суммой (последние 2 байта)
*         4. При успешной проверке:
*            - Обновление таймстампа последнего соединения
*            - Установка флага connection_lost в false
*            - Разбор данных управления:
*              * joystick_x (2 байта)
*              * joystick_y (2 байта)
*              * potentiometer_value (2 байта)
*              * right_left (2 байта)
*              * button (2 байта)
*         Формат пакета: [X_high, X_low, Y_high, Y_low, Pot_high, Pot_low,
*                         RL_high, RL_low, Btn_high, Btn_low, CS_high, CS_low]
*/
void receive_and_parse_data(int data_size) {
    calculated_checksum = 0;
    // Проверяем минимальный размер данных (2 байта данных + 2 байта контрольной суммы)
    if (data_size < 4) {
        last_received_time = HAL_GetTick();
        connection_lost = true;
        return; // Недостаточно данных для разбора
    }
    // Вычисляем контрольную сумму (все байты, кроме последних двух)
    for (uint8_t i = 0; i < data_size - 2; i++) {
        calculated_checksum += buffer_message[i];
    }
    // Получаем контрольную сумму из последних двух байтов
    received_checksum = (buffer_message[data_size - 2] << 8) | buffer_message[data_size - 1];
    // Если контрольная сумма совпадает, разбираем данные
    if (received_checksum == calculated_checksum) {
        // Обновляем время последнего успешного получения данных
        last_received_time = HAL_GetTick();
        connection_lost = false;
        // Данные начинаются с начала буфера (без заголовков типа и размера)
        joystick_x = (int16_t)((buffer_message[0] << 8) | buffer_message[1]);
        joystick_y = (int16_t)((buffer_message[2] << 8) | buffer_message[3]);
        potentiometer_value = (buffer_message[4] << 8) | buffer_message[5];
        right_left = (int16_t)((buffer_message[6] << 8) | buffer_message[7]);
        button = (buffer_message[8] << 8) | buffer_message[9];
        button_2 = (buffer_message[10] << 8) | buffer_message[11];
    }
}

/**
  * @brief  Проверка потери связи с пультом управления
  * @param  None
  * @retval None
  * @note   Функция выполняет:
  *         1. Проверку времени с последнего успешного приема данных
  *         2. При превышении таймаута CONNECTION_TIMEOUT_MS:
  *            - Установка флага connection_lost в true
  *            - Сброс всех управляющих сигналов в ноль
  *            - Установка Ш�?М моторов в минимальное значение (MIN_PULSE_WIDTH)
  *         3. �?спользует системный таймер через HAL_GetTick()
  *         Таймаут задается в мс через #define CONNECTION_TIMEOUT_MS
  */
void check_connection_loss(void) {
    current_time = HAL_GetTick();
    // Если прошло больше таймаута с последнего получения данных
    if ((current_time - last_received_time) > CONNECTION_TIMEOUT_MS) {
        connection_lost = true;
        // Сбрасываем все управляющие сигналы
        joystick_x = 0;
        joystick_y = 0;
        right_left = 0;
        potentiometer_value = 0;
        button = 0;
        // Устанавливаем минимальные импульсы на моторах
        Motors_Set_Throttle(MIN_PULSE_WIDTH, MIN_PULSE_WIDTH, MIN_PULSE_WIDTH, MIN_PULSE_WIDTH);
    }
}
