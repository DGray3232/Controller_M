#ifndef OPTICAL_FLOW_COMPENSATION_H
#define OPTICAL_FLOW_COMPENSATION_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "median_moving_average_filter.h"

// Кольцевой буфер накоплений гироскопа (по одной записи на каждый вызов 1 кГц = 1 мс).
// Позволяет брать накопление ровно за окно [t-20мс, t], привязанное к моменту прихода
// свежего кадра потока, а не к локальному таймеру обработки.
#define GYRO_HISTORY_SIZE 64   // 64 записи = 64 мс истории (запас на задержку датчика)

typedef struct {
    float     dangle_x[GYRO_HISTORY_SIZE];  // накопленный угол по X за этот 1 мс (град)
    float     dangle_y[GYRO_HISTORY_SIZE];  // накопленный угол по Y за этот 1 мс (град)
    uint32_t  timestamp_us[GYRO_HISTORY_SIZE]; // время записи (мкс)
    uint8_t   head;          // индекс следующей записи
    uint8_t   count;         // сколько записей уже накоплено (до заполнения)
} GyroIntegration_t;

// Структура для хранения результатов вычислений
typedef struct {
    float speed_cm_s_x;
    float speed_cm_s_y;
    float distance_m;
    uint32_t last_optical_flow_update; // Время последнего обновления оптического потока
    bool new_optical_data_available;   // Флаг новых данных оптического потока
} OpticalFlowResults_t;

// Функции для работы с гироскопом (вызываются на каждой итерации 1000 Гц)
void gyro_integration_update(GyroIntegration_t* gyro_int, float filtered_Gx, float filtered_Gy);

// Суммирует накопление гироскопа за окно window_ms, сдвинутое на latency_ms назад
// (учёт задержки датчика). Возвращает накопленный угол (град) в out_x/out_y.
void gyro_get_window_sum(GyroIntegration_t* gyro_int, uint32_t window_ms, uint32_t latency_ms, float* out_x, float* out_y);

// Функции для вычисления скорости и компенсации (вызываются реже)

//void calculate_linear_velocity(GyroIntegration_t* gyro_int, OpticalFlowResults_t* results, uint32_t distance);

void compensate_rotation_for_optical_flow(OpticalFlowResults_t* results,
                                         Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                                         int16_t flow_velocity_x, int16_t flow_velocity_y);

// Вспомогательные функции
void gyro_integration_reset(GyroIntegration_t* gyro_int);
void optical_flow_results_init(OpticalFlowResults_t* results);

void process_optical_flow_data(OpticalFlowResults_t* results,
                              GyroIntegration_t* gyro_int,
                              Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                              uint32_t distance,
                              int16_t flow_velocity_x, int16_t flow_velocity_y,
                              uint8_t flow_quality,
                              float pitch_deg, float roll_deg);

void calculate_linear_velocity_from_saved_data(OpticalFlowResults_t* results,
                                               uint32_t distance,
                                               float pitch_deg, float roll_deg,
                                               float saved_acc_x, float saved_acc_y,
                                               uint32_t saved_time_us);                              

#endif
