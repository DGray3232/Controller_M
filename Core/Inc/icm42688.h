#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* ============================================================
 * Драйвер датчика ICM-42688-P (TDK InvenSense)
 * 6-осевой IMU: гироскоп + акселерометр, интерфейс SPI
 *
 * Подключение (SPI2):
 *   PB14 = MISO
 *   PB15 = MOSI
 *   PB10 = SCK
 *   PB6  = CS  (Chip Select, управляется программно)
 * ============================================================ */

/* --- Пин Chip Select --- */
#define ICM42688_CS_PORT     GPIOB
#define ICM42688_CS_PIN      GPIO_PIN_6

/* --- Регистры (Bank 0) --- */
#define ICM42688_REG_DEVICE_CONFIG    0x11   // bit0 = soft reset
#define ICM42688_REG_PWR_MGMT0        0x4E   // режимы питания гиро/аксель
#define ICM42688_REG_WHO_AM_I         0x75   // идентификатор чипа
#define ICM42688_REG_GYRO_CONFIG0     0x4F   // диапазон + ODR гироскопа
#define ICM42688_REG_ACCEL_CONFIG0    0x50   // диапазон + ODR акселерометра
#define ICM42688_REG_ACCEL_DATA_X1    0x1F   // начало блока данных (accel 6 + gyro 6)

/* --- Значения конфигурации --- */
#define ICM42688_WHO_AM_I_VALUE       0x47   // ожидаемое значение для ICM-42688-P

// PWR_MGMT0: гиро и аксель в режиме Low Noise (наименьший шум)
#define ICM42688_PWR_LOW_NOISE        0x0F

// GYRO_CONFIG0: FS_SEL=0 (±2000 dps), ODR=0x06 (1 кГц)
#define ICM42688_GYRO_2000DPS_1KHZ    0x06

// ACCEL_CONFIG0: FS_SEL=0 (±16 g), ODR=0x06 (1 кГц)
#define ICM42688_ACCEL_16G_1KHZ       0x06

/* ============================================================
 * Конфигурации для БОЛЕЕ ВЫСОКИХ частот (ODR)
 * ------------------------------------------------------------
 * Формат регистров GYRO_CONFIG0 / ACCEL_CONFIG0 (Bank 0):
 *   [7:6] = резерв (0)
 *   [5:4] = FS_SEL  (диапазон)
 *   [3:0] = ODR     (частота выдачи данных)
 *
 * ODR кодировка (одинакова для гиро и аксель):
 *   0x03 = 8 кГц     0x05 = 2 кГц     0x07 = 200 Гц
 *   0x04 = 4 кГц     0x06 = 1 кГц     0x08 = 100 Гц
 *
 * Полное значение = (FS_SEL << 4) | ODR
 * Здесь FS_SEL = 0 (гиро ±2000 dps, аксель ±16 g)
 * ============================================================ */
#define ICM42688_GYRO_2000DPS_8KHZ    0x03   // ±2000 dps @ 8 кГц
#define ICM42688_GYRO_2000DPS_4KHZ    0x04   // ±2000 dps @ 4 кГц
#define ICM42688_GYRO_2000DPS_2KHZ    0x05   // ±2000 dps @ 2 кГц

#define ICM42688_ACCEL_16G_8KHZ       0x03   // ±16 g @ 8 кГц
#define ICM42688_ACCEL_16G_4KHZ       0x04   // ±16 g @ 4 кГц
#define ICM42688_ACCEL_16G_2KHZ       0x05   // ±16 g @ 2 кГц

/* ============================================================
 * Частота управляющего цикла (таймер TIM11)
 * ------------------------------------------------------------
 * TIM11 тактируется от APB2 = 100 МГц (APB2 prescaler = 1).
 * С Prescaler = 99 (деление на 100) получаем счёт 1 МГц.
 *
 *   Частота   | Prescaler | Counter Period
 *   ----------|-----------|---------------
 *   1 кГц     | 99        | 999
 *   2 кГц     | 99        | 499
 *   4 кГц     | 99        | 249
 *   8 кГц     | 99        | 124
 *
 * В CubeMX значения задаются как "Period - 1":
 *   htim11.Init.Prescaler = 100 - 1;    (деление на 100)
 *   htim11.Init.Period    = 1000 - 1;   (для 1 кГц)
 * ============================================================ */

/* --- Коэффициенты пересчёта сырых значений в физические --- */
#define ICM42688_GYRO_SENSITIVITY     0.060976f        // 1/16.4 LSB/(°/s) при ±2000 dps
#define ICM42688_ACCEL_SENSITIVITY    0.00048828125f   // 1/2048 LSB/g при ±16 g

/* --- Инициализация датчика --- */
// Возвращает true при успехе (WHO_AM_I совпал)
bool ICM42688_Init(SPI_HandleTypeDef* hspi);

/* --- Чтение одного байта из регистра (для диагностики) --- */
uint8_t ICM42688_ReadReg(SPI_HandleTypeDef* hspi, uint8_t reg);

/* --- Чтение свежих данных --- */
// Читает гироскоп и акселерометр одной burst-транзакцией (~5 мкс)
// и раскладывает в глобальные imu_gx/gy/gz и imu_ax/ay/az.
// Вызывать из HAL_TIM_PeriodElapsedCallback перед run_control_loop().
void ICM42688_ReadAll(SPI_HandleTypeDef* hspi);

#endif // ICM42688_H
