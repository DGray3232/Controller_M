#include "remote_control_mavlink.h"
#include "usart.h"

// === КОНФИГУРАЦИЯ ===
// Буфер для DMA
static uint8_t mav_rx_buffer[MAV_RX_BUFFER_SIZE];
static uint16_t old_pos = 0;

// Переменные парсера
static mavlink_message_t msg;
static mavlink_status_t status;

// Время последнего валидного пакета (для внешнего мониторинга)
//uint32_t mav_last_packet_time = 0; // определенна в globals.c

// === ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (MAPPING) ===
/*
static int16_t map_stick(uint16_t pwm) {
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    return (int16_t)((int32_t)(pwm - 1500) / 5);
}

static uint16_t map_throttle(uint16_t pwm) {
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    return (pwm - 1000);
}

static uint16_t map_switch(uint16_t pwm) {
    return (pwm > 1500) ? 1 : 0;
}
*/
// === ОБРАБОТЧИКИ СООБЩЕНИЙ===

/**
 * @brief Обработчик Heartbeat (#0)
 */
static void handle_heartbeat(const mavlink_message_t* message) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(message, &hb);
    // Здесь можно проверить, заармлен дрон или нет, или режим полета
    // Например:
    // bool is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
    // Обновляем таймер, так как получили валидный пакет
    mav_last_packet_time = HAL_GetTick();
}

/**
 * @brief Обработчик каналов управления (#70)
 */
static void handle_rc_channels(const mavlink_message_t* message) {
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(message, &rc);
/*
    // Маппинг каналов
    joystick_y          = map_stick(rc.chan1_raw);      // Roll
    joystick_x          = map_stick(rc.chan2_raw);      // Pitch
    potentiometer_value = map_throttle(rc.chan3_raw);   // Throttle
    right_left          = map_stick(rc.chan4_raw);      // Yaw
    button              = map_switch(rc.chan5_raw);     // Aux 1
    button_2            = map_switch(rc.chan6_raw);     // Aux 2
*/
    joystick_y          = rc.chan1_raw;      // Roll
    joystick_x          = rc.chan2_raw;      // Pitch
    potentiometer_value = rc.chan3_raw;   // Throttle
    right_left          = rc.chan4_raw;      // Yaw
    button              = rc.chan5_raw;     // Aux 1
    button_2            = rc.chan6_raw;     // Aux 2

    // Обновляем таймер
    mav_last_packet_time = HAL_GetTick();
}

/**
 * @brief Маршрутизатор сообщений.
 * Сюда добавлять новые case для новых типов сообщений.
 */
static void MAV_Dispatch_Message(const mavlink_message_t* message) {
    switch (message->msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT: // ID 0
             handle_heartbeat(message);
             break;

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: // ID 70
             handle_rc_channels(message);
             break;

        //--- ПРИМЕР: КАК ДОБАВИТЬ НОВОЕ ---
        //case MAVLINK_MSG_ID_COMMAND_LONG:
               //handle_command_long(message);
               //break;

        default:
            // Неизвестное или ненужное сообщение — просто игнорируем
            break;
    }
}

// === ОСНОВНЫЕ ФУНКЦИИ ===
void MAV_Init(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_DMA(huart, mav_rx_buffer, MAV_RX_BUFFER_SIZE);
    mav_last_packet_time = HAL_GetTick();
}
/**
 * @brief Главный цикл парсинга.
 * Вызывать в while(1) как можно чаще.
 * Не содержит проверок таймаута (они вынесены отдельно).
 */
void MAV_Process(void) {
    uint16_t dma_curr_pos = MAV_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    while (old_pos != dma_curr_pos) {
        uint8_t byte = mav_rx_buffer[old_pos];
        // Парсим байт
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            // Если пакет собран (CRC OK) — отправляем диспетчеру
            MAV_Dispatch_Message(&msg);
        }
        old_pos++;
        if (old_pos >= MAV_RX_BUFFER_SIZE) {
            old_pos = 0;
        }
    }
}
/**
 * @brief Функция проверки связи (вызывать отдельно)
 */
/*
void MAV_Check_Connection(void) {
    if ((HAL_GetTick() - mav_last_packet_time) > MAV_TIMEOUT_MS) {
        mavlink_connection_active = false;
        joystick_x = 0;
        joystick_y = 0;
        right_left = 0;
        potentiometer_value = 0;
        button = 0;
        button_2 = 0;
    } else {
        mavlink_connection_active = true;
    }
}
*/
void MAV_Check_Connection(UART_HandleTypeDef *huart) {
    uint32_t now = HAL_GetTick();
    // Проверка таймаута
    if ((now - mav_last_packet_time) > MAV_TIMEOUT_MS) {
        if (mavlink_connection_active) {
            mavlink_connection_active = false;
        }
        // Обнуляем стики
        joystick_x = 0;          // Roll center
        joystick_y = 0;          // Pitch center
        right_left = 0;          // Yaw center
        potentiometer_value = 0; // Throttle min (или center, зависит от вашей логики!)
        button = 0;
        button_2 = 0;
        // Проверяем ошибки на шине (Overrun, Noise, Framing errors)
        uint32_t srflags = huart->Instance->SR;
        // Если установлен флаг ORE (Overrun) или другие ошибки.
        // Или если просто долго нет связи, попробуем перезапустить DMA для профилактики.
        if ((srflags & USART_SR_ORE) || (srflags & USART_SR_FE) || (srflags & USART_SR_NE)) {
             __HAL_UART_CLEAR_OREFLAG(huart);
             __HAL_UART_CLEAR_NEFLAG(huart);
             __HAL_UART_CLEAR_FEFLAG(huart);
             // Останавливаем текущий DMA
             HAL_UART_DMAStop(huart);
             // Перезапускаем прием
             HAL_UART_Receive_DMA(huart, mav_rx_buffer, MAV_RX_BUFFER_SIZE);
        }
    } else {
        // Связь есть
        mavlink_connection_active = true;
    }
}
