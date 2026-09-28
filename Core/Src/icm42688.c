#include "icm42688.h"
#include "globals.h"
#include <string.h>

/* ============================================================
 * Драйвер датчика ICM-42688-P (TDK InvenSense)
 *
 * SPI протокол:
 *   - CS = LOW  → начало транзакции
 *   - 1-й байт  : адрес регистра (bit7=1 → чтение, bit7=0 → запись)
 *   - далее     : данные (при чтении адрес автоинкрементируется)
 *   - CS = HIGH → конец транзакции
 * ============================================================ */

/* --- Управление Chip Select --- */
static void ICM42688_CS_Low(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
}

static void ICM42688_CS_High(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
}

/* --- Запись одного байта в регистр --- */
static void ICM42688_WriteReg(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value };

    ICM42688_CS_Low();
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    ICM42688_CS_High();
}

/* --- Чтение одного байта из регистра --- */
uint8_t ICM42688_ReadReg(SPI_HandleTypeDef* hspi, uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = { 0 };

    ICM42688_CS_Low();
    HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    ICM42688_CS_High();

    return rx[1];
}

/**
  * @brief  Инициализация ICM-42688-P: сброс, режим Low Noise,
  *         диапазоны ±2000 dps / ±16g, ODR 1 кГц
  * @param  hspi: Указатель на обработчик SPI (SPI2)
  * @retval true если WHO_AM_I совпал (датчик отвечает), иначе false
  */
bool ICM42688_Init(SPI_HandleTypeDef* hspi)
{
    // CS = LOW: удерживаем, чтобы датчик гарантированно был в SPI-режиме
    ICM42688_CS_Low();
    HAL_Delay(10);   // пауза после подачи питания

    // Soft reset (DEVICE_CONFIG, bit0 = 1)
    ICM42688_WriteReg(hspi, ICM42688_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);   // ждём завершения сброса

    // Сброс бита reset + явный SPI Mode 0 (bit4 = 0)
    ICM42688_WriteReg(hspi, ICM42688_REG_DEVICE_CONFIG, 0x00);
    HAL_Delay(1);

    // Проверка идентификатора чипа
    uint8_t who_am_i = ICM42688_ReadReg(hspi, ICM42688_REG_WHO_AM_I);
    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
        return false;   // датчик не отвечает или не тот чип
    }

    // Режим Low Noise для гироскопа и акселерометра
    ICM42688_WriteReg(hspi, ICM42688_REG_PWR_MGMT0, ICM42688_PWR_LOW_NOISE);

    // Гироскоп: ±2000 dps, ODR = 1 кГц
    ICM42688_WriteReg(hspi, ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_2000DPS_1KHZ);

    // Акселерометр: ±16 g, ODR = 1 кГц
    ICM42688_WriteReg(hspi, ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_16G_1KHZ);

    return true;
}

/**
  * @brief  Чтение свежих данных гироскопа и акселерометра
  * @param  hspi: Указатель на обработчик SPI (SPI2)
  * @retval None
  * @note   Читает блок 12 байт начиная с ACCEL_DATA_X1 (0x1F):
  *         accel X/Y/Z (6 байт) + gyro X/Y/Z (6 байт).
  *         Результат пишется в глобальные imu_ax/ay/az и imu_gx/gy/gz.
  */
void ICM42688_ReadAll(SPI_HandleTypeDef* hspi)
{
    // tx[0] = адрес с битом чтения, остальные — dummy для приёма
    uint8_t tx[13] = { ICM42688_REG_ACCEL_DATA_X1 | 0x80, 0 };
    uint8_t rx[13] = { 0 };

    ICM42688_CS_Low();
    HAL_SPI_TransmitReceive(hspi, tx, rx, 13, HAL_MAX_DELAY);
    ICM42688_CS_High();

    // Сырые 16-битные значения (старший байт первым)
    int16_t raw_ax = (int16_t)((rx[1]  << 8) | rx[2]);
    int16_t raw_ay = (int16_t)((rx[3]  << 8) | rx[4]);
    int16_t raw_az = (int16_t)((rx[5]  << 8) | rx[6]);
    int16_t raw_gx = (int16_t)((rx[7]  << 8) | rx[8]);
    int16_t raw_gy = (int16_t)((rx[9]  << 8) | rx[10]);
    int16_t raw_gz = (int16_t)((rx[11] << 8) | rx[12]);

    // Пересчёт в физические величины [g] и [град/с]
    imu_ax = (float)raw_ax * ICM42688_ACCEL_SENSITIVITY;
    imu_ay = (float)raw_ay * ICM42688_ACCEL_SENSITIVITY;
    imu_az = (float)raw_az * ICM42688_ACCEL_SENSITIVITY;
    imu_gx = (float)raw_gx * ICM42688_GYRO_SENSITIVITY;
    imu_gy = (float)raw_gy * ICM42688_GYRO_SENSITIVITY;
    imu_gz = (float)raw_gz * ICM42688_GYRO_SENSITIVITY;
}
