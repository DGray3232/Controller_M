/* remote_control.h */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h" // Для доступа к HAL и типам

// Прототипы функций управления
void receive_and_parse_data(int data_size);
void check_connection_loss(void);
float expo_curve(float input, float expo_factor);

// Прототип функции управления моторами 
void Motors_Set_Throttle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif /* REMOTE_CONTROL_H */