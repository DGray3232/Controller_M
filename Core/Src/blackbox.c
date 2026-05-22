#include "blackbox.h"
#include "globals.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

// Кольцевой буфер
static BlackboxSample_t buffer[BLACKBOX_BUFFER_SIZE];
static volatile uint16_t write_idx = 0;
static volatile uint16_t sample_count = 0;
static volatile bool frozen = false;          // true — во время выгрузки запись останавливается
static volatile uint32_t start_time_ms = 0;

// Флаг: был ли уже взлёт с последней инициализации
static bool was_airborne = false;

// Счётчик для децимации (1000 Гц → 100 Гц)
static uint16_t decimation_counter = 0;

void Blackbox_Init(void) {
    write_idx = 0;
    sample_count = 0;
    frozen = false;
    was_airborne = false;
    decimation_counter = 0;
    start_time_ms = HAL_GetTick();
    memset(buffer, 0, sizeof(buffer));
}

void Blackbox_Write(void) {
    // Заморожено (идет выгрузка) — ничего не пишем
    if (frozen) return;

    // Пишем ТОЛЬКО в полёте (distance > 100) И с включёнными моторами
    if (distance <= 100 || m1_pulse[0] <= MIN_PULSE_WIDTH) {
        was_airborne = false;  // сброс — при следующем взлёте init
        return;
    }

    // Первый взлёт с моторами — переинициализируем буфер
    if (!was_airborne) {
        was_airborne = true;
        write_idx = 0;
        sample_count = 0;
        decimation_counter = 0;
        start_time_ms = HAL_GetTick();
        memset(buffer, 0, sizeof(buffer));
    }

    // Децимация: 1000 Гц → 100 Гц
    decimation_counter++;
    if (decimation_counter < BLACKBOX_DECIMATION) return;
    decimation_counter = 0;

    BlackboxSample_t *s = &buffer[write_idx];

    s->timestamp = (uint16_t)(HAL_GetTick() - start_time_ms);

    // Гироскоп: град/с × 10
    s->gyro[0] = (int16_t)(filtered_Gx * 10.0f);
    s->gyro[1] = (int16_t)(filtered_Gy * 10.0f);
    s->gyro[2] = (int16_t)(filtered_Gz * 10.0f);

    // Акселерометр: g × 1000
    s->accel[0] = (int16_t)(filtered_Ax * 1000.0f);
    s->accel[1] = (int16_t)(filtered_Ay * 1000.0f);
    s->accel[2] = (int16_t)(filtered_Az * 1000.0f);

    // Углы: град × 10
    s->angles[0] = (int16_t)(pitch * 10.0f);
    s->angles[1] = (int16_t)(roll * 10.0f);

    // Дальномер
    s->distance = (uint16_t)distance;

    // Ошибки PID (rate)
    s->errors[0] = (int16_t)error_pitch_rate;
    s->errors[1] = (int16_t)error_roll_rate;
    s->errors[2] = (int16_t)error_yaw_rate;

    // Выходы PID
    s->pid_out[0] = (int16_t)forse_pitch_rate;
    s->pid_out[1] = (int16_t)forse_roll_rate;
    s->pid_out[2] = (int16_t)forse_yaw_rate;

    // Моторы
    s->motors[0] = (uint16_t)total_power_1;
    s->motors[1] = (uint16_t)total_power_2;
    s->motors[2] = (uint16_t)total_power_3;
    s->motors[3] = (uint16_t)total_power_4;

    // Газ
    s->throttle = throttle_mshot;

    // Коррекция высоты
    s->altitude_correction = (int16_t)throttle_altitude_correction;

    // Режим и ARM (по фактическому состоянию моторов)
    s->flight_mode = active_mode;
    s->armed = (m1_pulse[0] > MIN_PULSE_WIDTH) ? 1 : 0;

    // --- MTF / Optical Flow данные ---
    s->mtf_flow[0] = flow_velocity_x;
    s->mtf_flow[1] = flow_velocity_y;
    s->mtf_speed[0] = (int16_t)(optical_flow_results.speed_cm_s_x * 10.0f);
    s->mtf_speed[1] = (int16_t)(optical_flow_results.speed_cm_s_y * 10.0f);
    s->mtf_target_angle[0] = (int16_t)(target_angle_pitch_mtf * 10.0f);
    s->mtf_target_angle[1] = (int16_t)(target_angle_roll_mtf * 10.0f);
    s->mtf_flow_quality = flow_quality;
    s->mtf_distance_strength = distance_strength;

    // Продвигаем указатель
    write_idx = (write_idx + 1) % BLACKBOX_BUFFER_SIZE;
    if (sample_count < BLACKBOX_BUFFER_SIZE) {
        sample_count++;
    }
}

void Blackbox_Dump(void) {
    frozen = true;
    HAL_Delay(10);  // дождаться завершения текущей итерации

    uint16_t total = (sample_count < BLACKBOX_BUFFER_SIZE) ? sample_count : BLACKBOX_BUFFER_SIZE;
    if (total == 0) {
        char *msg = "BLACKBOX: empty\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        frozen = false;
        return;
    }

    uint16_t start = 0;
    if (sample_count >= BLACKBOX_BUFFER_SIZE) {
        start = write_idx;  // самая старая запись
    }

    char header[] = "t_ms,gx,gy,gz,ax,ay,az,pitch,roll,dist,erP,erR,erY,pidP,pidR,pidY,m1,m2,m3,m4,thr,altCorr,mode,arm,"
                    "fvx,fvy,spdX,spdY,tgP,tgR,fQ,dStr\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)header, strlen(header), HAL_MAX_DELAY);

    char line[192];
    for (uint16_t i = 0; i < total; i++) {
        uint16_t idx = (start + i) % BLACKBOX_BUFFER_SIZE;
        BlackboxSample_t *s = &buffer[idx];

        int len = snprintf(line, sizeof(line),
            "%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,"
            "%u,%u,%u,%u,%u,%d,%u,%u,"
            "%d,%d,%d,%d,%d,%d,%u,%u\r\n",
            s->timestamp,
            s->gyro[0], s->gyro[1], s->gyro[2],
            s->accel[0], s->accel[1], s->accel[2],
            s->angles[0], s->angles[1],
            s->distance,
            s->errors[0], s->errors[1], s->errors[2],
            s->pid_out[0], s->pid_out[1], s->pid_out[2],
            s->motors[0], s->motors[1], s->motors[2], s->motors[3],
            s->throttle,
            s->altitude_correction,
            (unsigned int)s->flight_mode,
            (unsigned int)s->armed,
            // MTF данные
            s->mtf_flow[0], s->mtf_flow[1],
            s->mtf_speed[0], s->mtf_speed[1],
            s->mtf_target_angle[0], s->mtf_target_angle[1],
            (unsigned int)s->mtf_flow_quality,
            (unsigned int)s->mtf_distance_strength);

        if (len > 0) {
            HAL_UART_Transmit(&huart2, (uint8_t*)line, len, HAL_MAX_DELAY);
        }
    }

    char *footer = "BLACKBOX: end\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);

    // Разморозить и подготовить к новому полёту
    Blackbox_Init();
}

bool Blackbox_IsFrozen(void) {
    return frozen;
}
