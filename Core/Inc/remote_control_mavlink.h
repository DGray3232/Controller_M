#ifndef REMOTE_CONTROL_MAVLINK_H
#define REMOTE_CONTROL_MAVLINK_H

#include "main.h"
#include <stdbool.h>
#include "mavlink/common/mavlink.h"

#define MAV_RX_BUFFER_SIZE 512
#define MAV_TIMEOUT_MS 1000

// Глобальные флаги
extern bool mavlink_connection_active;
extern uint32_t mav_last_packet_time; // Если нужно читать время из main.c

// Ссылки на переменные управления
extern int16_t joystick_x;
extern int16_t joystick_y;
extern int16_t right_left;
extern uint16_t potentiometer_value;
extern uint16_t button;
extern uint16_t button_2;

void MAV_Init(UART_HandleTypeDef *huart);
void MAV_Process(void);           // Только парсинг
//void MAV_Check_Connection(void);  // Только проверка таймаута
void MAV_Check_Connection(UART_HandleTypeDef *huart);

#endif
