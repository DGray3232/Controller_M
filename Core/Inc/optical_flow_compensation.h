#ifndef OPTICAL_FLOW_COMPENSATION_H
#define OPTICAL_FLOW_COMPENSATION_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "median_moving_average_filter.h"

// Структура для накопления данных гироскопа
typedef struct {
	float acc_x;      // Накопленная скорость по X (град/сек * время)
	float acc_y;      // Накопленная скорость по Y (град/сек * время)
    uint32_t time_us;     // Накопленное время в микросекундах
    uint32_t last_update; // Время последнего обновления
    uint32_t update_count;// Счетчик обновлений для децимации
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

// Функции для вычисления скорости и компенсации (вызываются реже)

//void calculate_linear_velocity(GyroIntegration_t* gyro_int, OpticalFlowResults_t* results, uint32_t distance);

void compensate_rotation_for_optical_flow(OpticalFlowResults_t* results,
                                         Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                                         int16_t flow_velocity_x, int16_t flow_velocity_y);

// Функция для обработки оптического потока (вызывается когда есть новые данные)
/*
void process_optical_flow_data(OpticalFlowResults_t* results,
                              GyroIntegration_t* gyro_int,
                              Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                              uint32_t distance,
                              int16_t flow_velocity_x, int16_t flow_velocity_y);
void process_optical_flow_data_v2(OpticalFlowResults_t* results,
                              GyroIntegration_t* gyro_int,
							  MedianFilter* mf_x, MedianFilter* mf_y,MovingAverageFilter* maf_x, MovingAverageFilter* maf_y,
                              uint32_t distance,
                              int16_t flow_velocity_x, int16_t flow_velocity_y);
*/
// Функция для обработки оптического потока v2 (вызывается когда есть новые данные)
void compensate_rotation_for_optical_flow_v2(OpticalFlowResults_t* results,
										  MedianFilter* mf_x, MedianFilter* mf_y,MovingAverageFilter* maf_x, MovingAverageFilter* maf_y,
                                          int16_t flow_velocity_x, int16_t flow_velocity_y);

// Вспомогательные функции
void gyro_integration_reset(GyroIntegration_t* gyro_int);
void optical_flow_results_init(OpticalFlowResults_t* results);
bool should_process_optical_flow(uint32_t current_time, uint32_t last_processing_time);

void calculate_linear_velocity(GyroIntegration_t* gyro_int, OpticalFlowResults_t* results, uint32_t distance, float pitch_deg, float roll_deg);

void process_optical_flow_data(OpticalFlowResults_t* results,
                              GyroIntegration_t* gyro_int,
                              Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                              uint32_t distance,
                              int16_t flow_velocity_x, int16_t flow_velocity_y,
                              float pitch_deg, float roll_deg);

void calculate_linear_velocity_from_saved_data(OpticalFlowResults_t* results,
                                               uint32_t distance,
                                               float pitch_deg, float roll_deg,
                                               float saved_acc_x, float saved_acc_y,
                                               uint32_t saved_time_us);                              

#endif
