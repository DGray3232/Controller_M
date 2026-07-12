#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>
#include <stdbool.h>
#include "config_param.h"

// Размер кольцевого буфера (кол-во записей)
#define BLACKBOX_BUFFER_SIZE  1024

// Частота записи: 1 / BLACKBOX_DECIMATION от control loop (1000 Гц)
#define BLACKBOX_DECIMATION   10   // каждые 10 итераций → 100 Гц
                                   // 1024 записей × 100 Гц → ~10 секунд полёта

// Структура одной записи (упакована для компактности)
typedef struct __attribute__((packed)) {
    uint16_t timestamp;             // мс от старта
    int16_t  gyro[3];               // filtered_Gx,y,z [град/с * 10]
    int16_t  accel[3];              // filtered_Ax,y,z [g * 1000]
    int16_t  angles[2];             // pitch, roll [град * 10]
    uint16_t distance;              // мм
    int16_t  errors[3];             // error_pitch_rate, roll_rate, yaw_rate
    int16_t  pid_out[3];            // forse_pitch_rate, roll_rate, yaw_rate
    uint16_t motors[4];             // total_power_1-4 [мкс]
    uint16_t throttle;              // throttle_mshot [мкс]
    int16_t  altitude_correction;   // [мкс добавки к газу]
    uint8_t  flight_mode;           // FLIGHT_MODE_ACRO/ANGLE/MTF
    uint8_t  armed;                 // 1 = моторы крутятся
    // --- MTF / Optical Flow данные ---
    int16_t  mtf_flow[2];           // flow_velocity_x,y (сырой, пиксели/с)
    int16_t  mtf_speed[2];          // speed_cm_s_x,y после компенсации [см/с * 10]
    int16_t  mtf_target_angle[2];   // target_angle_pitch/roll_mtf [град * 10]
    uint8_t  mtf_flow_quality;      // flow_quality (0-255)
    uint8_t  mtf_distance_strength; // distance_strength (0-255)
} BlackboxSample_t;                  // 60 байт на запись → 60 КБ на буфер

// Инициализация blackbox
void Blackbox_Init(void);

// Запись лога — вызывать из run_control_loop.
// Запись идёт ТОЛЬКО в полёте (distance > 100) И с включёнными моторами.
void Blackbox_Write(void);

// Выгрузка RAM-буфера через USART2 в CSV формате
void Blackbox_Dump(void);

// Проверка — идёт ли выгрузка (буфер заморожен)
bool Blackbox_IsFrozen(void);

#endif // BLACKBOX_H
