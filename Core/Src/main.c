/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "limits.h"
#include "arm_const_structs.h"  // Для предопределенных структур БПФ
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "bmx055.h"
#include "AHRSAlgorithms.h"
#include "kalman.h"
#include "notch.h"
#include "hpf_filter.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void normalizeQuaternion(float q[4]);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG (180.0f / M_PI)

#define NUM_STAGES 1 // Для 2-го порядка фильтра
#define BLOCK_SIZE 1 // Размер блока данных для обработки

// Параметры протокола Multishot
#define MULTISHOT_MIN 500      // Минимальная длительность импульса (мкс)
#define MULTISHOT_MAX 2500     // Максимальная длительность импульса (мкс)

#define CONTROL_LOOP_DT 0.001f // 1000 Гц

#define DATA_SIZE             12
#define CONNECTION_TIMEOUT_MS 500    // Таймаут потери связи (мс)
#define MIN_PULSE_WIDTH       500    // Минимальная ширина импульса
#define MAX_PULSE_WIDTH       2500   // Максимальная ширина импульса
#define MAX_CORRECTION        500    //Ограничение пид-коррекции
#define MAX_PID               1500   //Ограничение пид-сигнала

#define TRIM_ROLL_ERROR  0.0f
#define TRIM_PITCH_ERROR 0.0f
#define TRIM_YAW_ERROR   0.0f

#define TRIM_FRONT_LEFT  25
#define TRIM_FRONT_RIGHT -10
#define TRIM_REAR_LEFT   55
#define TRIM_REAR_RIGHT  5

// Параметры фильтра Калмана
#define GYRO_PROCESS_NOISE      0.0005f
#define GYRO_MEASUREMENT_NOISE  0.5f
#define GYRO_ESTIMATION_ERROR   0.1f
#define GYRO_INITIAL_VALUE      0.0f

#define ACCEL_PROCESS_NOISE     0.1f
#define ACCEL_MEASUREMENT_NOISE 1.0f
#define ACCEL_ESTIMATION_ERROR  1.0f
#define ACCEL_INITIAL_VALUE     0.0f
#define ACCEL_Z_INITIAL_VALUE   1.0f  // для гравитации

//  Параметры режекторного фильтра
#define NOTCH_CENTER_FREQ       60.0f
#define NOTCH_WIDTH             10.0f
#define NOTCH_SAMPLE_RATE       1000.0f
#define ATTITUDE_NOTCH_WIDTH    5.0f

// Параметры П�?Д-регулятора
#define PITCH_PID_KP          350.0
#define PITCH_PID_KI          1.0
#define PITCH_PID_KD          55.0
#define ROLL_PID_KP           365.0
#define ROLL_PID_KI           1.0
#define ROLL_PID_KD           55.0
#define YAW_PID_KP            1000.0
#define YAW_PID_KI            1.0
#define YAW_PID_KD            200.0
#define ALPHA                 1.0
#define ALPHA_DERIVATIVE      0.1
#define INTEGRAL_LIMIT        10.0
#define SCALE_FACTOR          1.0

// Параметры фильтра верхних частот
#define HPF_ALPHA             0.99999

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

BMX055_t BMX055;

KalmanFilter gyro_filter_x;
KalmanFilter gyro_filter_y;
KalmanFilter gyro_filter_z;

KalmanFilter accel_filter_x;
KalmanFilter accel_filter_y;
KalmanFilter accel_filter_z;

NotchFilter notch_ax, notch_ay, notch_az;
NotchFilter notch_gx, notch_gy, notch_gz;
NotchFilter notch_roll, notch_pitch, notch_yaw;

HPF hpf_x;
HPF hpf_y;
HPF hpf_z;

PID_Controller pitch_pid;
PID_Controller roll_pid;
PID_Controller yaw_pid;

arm_biquad_cascade_df2T_instance_f32 lpf_Gx; // Экземпляр фильтра
arm_biquad_cascade_df2T_instance_f32 lpf_Gy;
arm_biquad_cascade_df2T_instance_f32 lpf_Gz;

arm_biquad_cascade_df2T_instance_f32 lpf_Ax; // Экземпляр фильтра
arm_biquad_cascade_df2T_instance_f32 lpf_Ay;
arm_biquad_cascade_df2T_instance_f32 lpf_Az;

arm_biquad_cascade_df2T_instance_f32 lpf_Errorx; // Экземпляр фильтра
arm_biquad_cascade_df2T_instance_f32 lpf_Errory;
arm_biquad_cascade_df2T_instance_f32 lpf_Errorz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char buf[250]; // буффер для отладки

int joystick_x = 0;
int joystick_y = 0;
int potentiometer_value = 0;// общий газ
int right_left = 0;
int button = 0;
int received_checksum, calculated_checksum = 0;
uint8_t buffer_message[DATA_SIZE] = {0}; // беффер для приема типа и сообщения

uint32_t last_received_time = 0;
bool connection_lost = false;

float32_t Ax[1] = {0}; // акселерометр g
float32_t Ay[1] = {0};
float32_t Az[1] = {0};
float32_t filter_Ax[1] = {0};
float32_t filter_Ay[1] = {0};
float32_t filter_Az[1] = {0};
float32_t offset_Ax = {0};
float32_t offset_Ay = {0};
float32_t offset_Az = {0};

float32_t Gx[1] = {0}; // гироскоп в градусах в секунду
float32_t Gy[1] = {0};
float32_t Gz[1] = {0};
float32_t bias_Gx = {0};
float32_t bias_Gy = {0};
float32_t bias_Gz = {0};
float32_t filter_Gx[1] = {0};
float32_t filter_Gy[1] = {0};
float32_t filter_Gz[1] = {0};

float32_t temp_x[1] = {0};
float32_t temp_y[1] = {0};
float32_t temp_z[1] = {0};
float32_t temp_forse_pitch[1] = {0};
float32_t temp_forse_roll[1] = {0};
float32_t temp_forse_yaw[1] = {0};

int forse_pitch = 0;
int forse_roll = 0;
int forse_yaw = 0;

float temp = 0.0f; // температура
float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
float roll_comp = 0.0f;   // Угол крена (вращение вокруг оси X)
float pitch_comp = 0.0f;  // Угол тангажа (вращение вокруг оси Y)
float Quat_actual[4] = {0}; // кватернион
float Quat_target[4] = {0}; // целевой кватернион
float Quat_actual_inv[4] = {0}; // инвертированный результирующий кватернион
float Quat_error[4] = {0}; // кватернион ошибок
float Quat_previous[4] = {0};

// Состояния фильтра
static float32_t filterState_lpf_Gx[NUM_STAGES * 4] = {0};
static float32_t filterState_lpf_Gy[NUM_STAGES * 4] = {0};
static float32_t filterState_lpf_Gz[NUM_STAGES * 4] = {0};

static float32_t filterState_lpf_Ax[NUM_STAGES * 4] = {0};
static float32_t filterState_lpf_Ay[NUM_STAGES * 4] = {0};
static float32_t filterState_lpf_Az[NUM_STAGES * 4] = {0};

static float32_t filterState_lpf_Errorx[NUM_STAGES * 4] = {0};
static float32_t filterState_lpf_Errory[NUM_STAGES * 4] = {0};
static float32_t filterState_lpf_Errorz[NUM_STAGES * 4] = {0};

static float32_t Coeffs_lpf_accel[NUM_STAGES * 5] = {0.04613180, 0.09226360, 0.04613180, 1.30728503, -0.49181224};
static float32_t Coeffs_lpf_Error[NUM_STAGES * 5] = {0.04613180, 0.09226360, 0.04613180, 1.30728503, -0.49181224};
static float32_t Coeffs_lpf[NUM_STAGES * 5] = {0.04613180, 0.09226360, 0.04613180, 1.30728503, -0.49181224};

uint32_t last_time = 0;
float freq = 0;

double errorpitch = 0;
double errorroll = 0;
double erroryaw = 0;

//////////////////////////////////////////////////////////////////////////Multishot/////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Устанавливает значение тяги для мотора в формате Multishot
  * @param  throttle Значение тяги в микросекундах (мкс)
  *         Должно быть в диапазоне [MULTISHOT_MIN, MULTISHOT_MAX] (500-2500 мкс)
  * @param  channel Канал таймера для управления мотором
  *         Допустимые значения:
  *         @arg TIM_CHANNEL_1
  *         @arg TIM_CHANNEL_2
  *         @arg TIM_CHANNEL_3
  *         @arg TIM_CHANNEL_4
  * @retval None
  * @note   Функция выполняет:
  *         1. Остановку текущего DMA-канала для указанного канала таймера
  *         2. Запуск нового DMA-передачи с заданным значением throttle
  *         3. Использует таймер htim1 (должен быть предварительно инициализирован)
  * @warning Перед использованием необходимо:
  *         - Настроить таймер htim1 в режиме PWM
  *         - Инициализировать DMA для работы с таймером
  *         - Убедиться, что throttle находится в допустимом диапазоне
  */
void Multishot_SetThrottle(uint16_t throttle, uint32_t channel)
{
  uint32_t M_pwm_data[1] = {0};
  M_pwm_data[0] = throttle;
  HAL_TIM_PWM_Stop_DMA(&htim1, channel);
  HAL_TIM_PWM_Start_DMA(&htim1, channel, M_pwm_data, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////// прием команд с пульта ////////////////////////////////////////////////////////////////////////////
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
  *            - Установка ШИМ моторов в минимальное значение (MIN_PULSE_WIDTH)
  *         3. Использует системный таймер через HAL_GetTick()
  *         Таймаут задается в мс через #define CONNECTION_TIMEOUT_MS
  */
void check_connection_loss(void) {
    uint32_t current_time = HAL_GetTick();
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
        Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_1);
        Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_2);
        Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_3);
        Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_4);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Ограничивает значение с плавающей точкой заданными границами
  * @param  value Значение для ограничения
  * @param  min_val Минимально допустимое значение (нижняя граница)
  * @param  max_val Максимально допустимое значение (верхняя граница)
  * @retval Ограниченное значение:
  *         - min_val, если value < min_val
  *         - max_val, если value > max_val
  *         - value, если в пределах границ
  */
float constrain_float(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
/**
  * @brief  Ограничивает целочисленное значение заданными границами
  * @param  value Значение для ограничения
  * @param  min_val Минимально допустимое значение (нижняя граница)
  * @param  max_val Максимально допустимое значение (верхняя граница)
  * @retval Ограниченное значение:
  *         - min_val, если value < min_val
  *         - max_val, если value > max_val
  *         - value, если в пределах границ
  */
int constrain(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
/**
  * @brief  Проверяет и ограничивает значение PID-регулятора
  * @param  value Значение PID для проверки
  * @param  max_pid_output Максимальное абсолютное значение выхода PID
  *         Должно быть положительным числом
  * @retval Безопасное значение PID:
  *         - 0, если value выходит за пределы [-10000, 10000]
  *         - Ограниченное значение в пределах [-max_pid_output, max_pid_output]
  * @note   Функция обеспечивает защиту от переполнения и аномальных значений
  */
int safe_pid_value(int value, int max_pid_output) {
    if (value < -10000 || value > 10000) {
        return 0;
    }
    return constrain(value, -max_pid_output, max_pid_output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////// фильтр махони ///////////////////////////////////////////////////////
float round_to(float number) {
    return round(number * 100) / 100;
}
float round_to_dec(float number) {
    return round(number * 10) / 10;
}
double invert(double x){
	return 0 - x;
}
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
void get_angle_mahony(void){

	  float filterAx = filter_Ax[0]*9.81;
	  float filterAy = filter_Ay[0]*9.81;
	  float filterAz = filter_Az[0]*9.81;
	  float filterGx = filter_Gx[0]*DEG_TO_RAD;
	  float filterGy = filter_Gy[0]*DEG_TO_RAD;
	  float filterGz = filter_Gz[0]*DEG_TO_RAD;

	  filterAx = round_to(filterAx);
	  filterAy = round_to(filterAy);
	  filterAz = round_to(filterAz);
	  filterGx = round_to(filterGx);
	  filterGy = round_to(filterGy);
	  filterGz = round_to(filterGz);

	  MahonyAHRSupdateIMU(filterAx, filterAy, filterAz, filterGx, filterGy, filterGz, CONTROL_LOOP_DT);

	  Quat_actual[0] = (*(getQ()));
	  Quat_actual[1] = (*(getQ()+1));
	  Quat_actual[2] = (*(getQ()+2));
	  Quat_actual[3] = (*(getQ()+3));

	  float alpha = 0.5; // Коэффициент сглаживания (0 < alpha < 1)
	  float dot = Quat_previous[0]*(*(getQ())) + Quat_previous[1]*(*(getQ()+1)) +
	              Quat_previous[2]*(*(getQ()+2)) + Quat_previous[3]*(*(getQ()+3));
	  if (dot < 0) {
	      // Если скалярное произведение отрицательное, инвертируем новый кватернион
	      Quat_actual[0] = alpha * (-*(getQ()))   + (1 - alpha) * Quat_previous[0];
	      Quat_actual[1] = alpha * (-*(getQ()+1)) + (1 - alpha) * Quat_previous[1];
	      Quat_actual[2] = alpha * (-*(getQ()+2)) + (1 - alpha) * Quat_previous[2];
	      Quat_actual[3] = alpha * (-*(getQ()+3)) + (1 - alpha) * Quat_previous[3];
	  }
	  else {
	      // иначе фильтруем как обычно
	      Quat_actual[0] = alpha * (*(getQ()))   + (1 - alpha) * Quat_previous[0];
	      Quat_actual[1] = alpha * (*(getQ()+1)) + (1 - alpha) * Quat_previous[1];
	      Quat_actual[2] = alpha * (*(getQ()+2)) + (1 - alpha) * Quat_previous[2];
	      Quat_actual[3] = alpha * (*(getQ()+3)) + (1 - alpha) * Quat_previous[3];
	  }
	  normalizeQuaternion(Quat_actual);
	  Quat_previous[0] = Quat_actual[0];
	  Quat_previous[1] = Quat_actual[1];
	  Quat_previous[2] = Quat_actual[2];
	  Quat_previous[3] = Quat_actual[3];

	  float q0 = Quat_actual[0];
	  float q1 = Quat_actual[1];
	  float q2 = Quat_actual[2];
	  float q3 = Quat_actual[3];

	  float q0q0 = q0 * q0;
	  float q1q1 = q1 * q1;
	  float q2q2 = q2 * q2;
	  float q3q3 = q3 * q3;

	  float q0q1 = q0 * q1;
	  float q0q2 = q0 * q2;
	  float q0q3 = q0 * q3;
	  float q1q2 = q1 * q2;
	  float q1q3 = q1 * q3;
	  float q2q3 = q2 * q3;

	  yaw = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
	  pitch = asinf(2.0f * (q1q3 - q0q2));
	  roll = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);

	  pitch *= RAD_TO_DEG;
	  roll *= RAD_TO_DEG;
	  yaw   *= RAD_TO_DEG;

	  roll = apply_notch_filter(&notch_roll, roll);
	  pitch = apply_notch_filter(&notch_pitch, pitch);
	  yaw = apply_notch_filter(&notch_yaw, yaw);

}
// Функция для нормализации кватерниона
void normalizeQuaternion(float q[4]) {
    // Вычисление нормы (длины) кватерниона
    float norm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    // Проверка на ноль перед делением
    if (norm != 0.0f) {
        // Деление компонентов кватерниона на его норму
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////преобразование углов с пульта в целевой кватернион////////////////////////////////////////////////////////////

void eulerToQuaternion(int joystick_x, int joystick_y, int right_left, float q[4]) {
    // Преобразуем значения из диапазона 0-100 в радианы
    float roll = (joystick_y / 100.0f) * M_PI;   // Поворот вокруг оси X
    float pitch = (joystick_x / 100.0f) * M_PI;  // Поворот вокруг оси Y
    float yaw = (right_left / 100.0f) * M_PI; // Поворот вокруг оси Z

    // Вычисление половин углов
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    // Вычисление компонентов кватерниона и сохранение в массив
    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy;// - 0.02; // x
    q[2] = cr * sp * cy + sr * cp * sy;// - 0.02; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z

    normalizeQuaternion(q);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////вычисления кватерниона ошибки/////////////////////////////////////////////////////////////
// Функция для вычисления обратного кватерниона
void quat_inverse(float quat[4], float quat_inv[4]) {
    // Обратный кватернион: [w, -x, -y, -z]
    quat_inv[0] = quat[0];  // w
    quat_inv[1] = -quat[1]; // -x
    quat_inv[2] = -quat[2]; // -y
    quat_inv[3] = -quat[3]; // -z
}
// Функция для умножения двух кватернионов
void quat_multiply(float q1[4], float q2[4], float result[4]) {
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]; // w
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]; // x
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]; // y
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]; // z
}
// Функция для вычисления ошибки между двумя кватернионами
void quat_error(float  q_actual[4], float  q_actual_inv[4], float  q_target[4], float q_error[4]) {
    // Вычисляем обратный кватернион
    quat_inverse(q_actual, q_actual_inv);
    // Вычисляем ошибку: q_error = Quat_target * Quat_actual^-1
    quat_multiply(q_target, q_actual_inv, q_error);

    normalizeQuaternion(q_error);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////// фильтрованные данные///////////////////////////////////////////////////////////////
void get_filter_data_accel() {

	BMX055_Read_Accel(&hi2c1, &BMX055);

	//Ax[0] = BMX055.Ax;
	Ax[0] = round_to(BMX055.Ax);
	Ax[0] = Ax[0] - offset_Ax;
    Ax[0] = apply_notch_filter(&notch_ax, Ax[0]);
    arm_biquad_cascade_df2T_f32 (&lpf_Ax, Ax, filter_Ax, BLOCK_SIZE);
    filter_Ax[0] = kalman_filter_update(&accel_filter_x, filter_Ax[0]);

    //Ay[0] = BMX055.Ay;
    Ay[0] = round_to(BMX055.Ay);
	Ay[0] = Ay[0] - offset_Ay;
    Ay[0] = apply_notch_filter(&notch_ay, Ay[0]);
   	arm_biquad_cascade_df2T_f32 (&lpf_Ay, Ay, filter_Ay, BLOCK_SIZE);
   	filter_Ay[0] = kalman_filter_update(&accel_filter_y, filter_Ay[0]);

   	//Az[0] = BMX055.Az;
   	Az[0] = round_to(BMX055.Az);
	Az[0] = Az[0] - offset_Az;
    Az[0] = apply_notch_filter(&notch_az, Az[0]);
    arm_biquad_cascade_df2T_f32 (&lpf_Az, Az, filter_Az, BLOCK_SIZE);
    filter_Az[0] = kalman_filter_update(&accel_filter_z, filter_Az[0]);
}
void get_filter_data_gyro() {

	BMX055_Read_Gyro(&hi2c1, &BMX055);

	//Gx[0] = BMX055.Gx;
	Gx[0] = round_to(BMX055.Gx);
	Gx[0] = Gx[0] - bias_Gx;
    Gx[0] = apply_notch_filter(&notch_gx, Gx[0]);
	Gx[0] = HPF_Update(&hpf_x, Gx[0]);
    arm_biquad_cascade_df2T_f32 (&lpf_Gx, Gx, filter_Gx, BLOCK_SIZE);
    filter_Gx[0] = kalman_filter_update(&gyro_filter_x, filter_Gx[0]);

    //Gy[0] = BMX055.Gy;
    Gy[0] = round_to(BMX055.Gy);
	Gy[0] = Gy[0] - bias_Gy;
    Gy[0] = apply_notch_filter(&notch_gy, Gy[0]);
	Gy[0] = HPF_Update(&hpf_y, Gy[0]);
   	arm_biquad_cascade_df2T_f32 (&lpf_Gy, Gy, filter_Gy, BLOCK_SIZE);
   	filter_Gy[0] = kalman_filter_update(&gyro_filter_y, filter_Gy[0]);

   	//Gz[0] = BMX055.Gz;
   	Gz[0] = round_to(BMX055.Gz);
    Gz[0] = Gz[0] - bias_Gz;
    Gz[0] = apply_notch_filter(&notch_gz, Gz[0]);
    Gz[0] = HPF_Update(&hpf_z, Gz[0]);
   	arm_biquad_cascade_df2T_f32 (&lpf_Gz, Gz, filter_Gz, BLOCK_SIZE);
   	filter_Gz[0] = kalman_filter_update(&gyro_filter_z, filter_Gz[0]);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////смещение imu///////////////////////////////////////////////////////////////

void bias() {
	for (int i = 0; i < 512; i++) {
		get_filter_data_gyro();
		bias_Gx += Gx[0];
		bias_Gy += Gy[0];
		bias_Gz += Gz[0];
		offset_Ax += Ax[0];
		offset_Ay += Ay[0];
		offset_Az += Az[0];
    }
 bias_Gx = bias_Gx / 512;
 bias_Gy = bias_Gy / 512;
 bias_Gz = bias_Gz / 512;
 offset_Ax = offset_Ax / 512;
 offset_Ay = offset_Ay / 512;
 offset_Az = offset_Az / 512;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////инициализация imu//////////////////////////////////////////////////////////////////////
void InitBMX055() {
    int retriesA = 0;
    int retriesG = 0;
    //int retriesM = 0;
    while (BMX055_BMA_Init(&hi2c1) == 1) {
    	retriesA++;
        if (retriesA >= 10) {
        }
    }
    while (BMX055_BMG_Init(&hi2c1) == 1) {
  	  retriesG++;
        if (retriesG >= 10) {
        }
    }
/*
    while (BMX055_BMM_Init(&hi2c1) == 1) {
  	  retriesM++;
        if (retriesM >= 10) {
        }
    }
*/

    setFastOffset_BMA(&hi2c1);
    setFastOffset_BMG(&hi2c1);
    //setFastOffset_BMM(&hi2c1, 1);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void run_control_loop(){

    // Проверяем потерю связи перед выполнением основного цикла
    check_connection_loss();
    // Если связь потеряна, не выполняем основной цикл управления
    if (connection_lost == true) {
    	HAL_UART_Receive_DMA(&huart6, buffer_message, DATA_SIZE);
    	receive_and_parse_data(DATA_SIZE);
    	return;
    }
	receive_and_parse_data(DATA_SIZE);
	get_filter_data_accel();
	get_filter_data_gyro();
	get_angle_mahony();
    eulerToQuaternion(joystick_x, joystick_y, right_left, Quat_target); // функция преобразования углов в кватернион
	quat_error(Quat_actual, Quat_actual_inv, Quat_target, Quat_error); // ошибка между текущим и целевым кватерниолном

	uint16_t throttle_mshot = ((potentiometer_value / 1000.0) * 2000.0) + 500;
	float max_pid_correction_mshot = MAX_CORRECTION;  //Ограничение пид-коррекции

	double error_pitch = Quat_error[2] * (-1); // ошибка по тангажу знак ошибки изменен
	double error_roll = Quat_error[1]; // ошибка по крену
	double error_yaw = Quat_error[3]; // ошибка по рысканью

	error_pitch = round_to(error_pitch);
	error_roll = round_to(error_roll);
	error_yaw = round_to(error_yaw);

	errorpitch = error_pitch;
	errorroll = error_roll;
	erroryaw = error_yaw;

	forse_pitch = PID_Compute(&pitch_pid, error_pitch, TRIM_PITCH_ERROR);
	forse_roll = PID_Compute(&roll_pid, error_roll, TRIM_ROLL_ERROR);
	forse_yaw = PID_Compute(&yaw_pid, error_yaw, TRIM_YAW_ERROR);

	// Проверяем на NaN, Inf и ограничиваем
	forse_pitch = safe_pid_value(forse_pitch, MAX_PID);
	forse_roll = safe_pid_value(forse_roll, MAX_PID);
	forse_yaw = safe_pid_value(forse_yaw, MAX_PID);

	temp_x[0] = forse_pitch;
	temp_y[0] = forse_roll;
	temp_z[0] = forse_yaw;

	arm_biquad_cascade_df2T_f32 (&lpf_Errorx, temp_x, temp_forse_pitch, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Errory, temp_y, temp_forse_roll, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Errorz, temp_z, temp_forse_yaw, BLOCK_SIZE);

	forse_pitch = temp_forse_pitch[0];
	forse_roll = temp_forse_roll[0];
	forse_yaw = temp_forse_yaw[0];

	if(button == 1 && potentiometer_value > 0){
	   float pid_correction_1 = forse_roll + forse_pitch + forse_yaw;
	   pid_correction_1 = constrain_float(pid_correction_1, -max_pid_correction_mshot, max_pid_correction_mshot);
	   int total_power_1 = throttle_mshot + pid_correction_1;// итоговая мощность левый передний мотор
	   total_power_1 = constrain(total_power_1 + TRIM_FRONT_LEFT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   float pid_correction_2 = -forse_roll + forse_pitch - forse_yaw;
	   pid_correction_2 = constrain_float(pid_correction_2, -max_pid_correction_mshot, max_pid_correction_mshot);
	   int total_power_2 = throttle_mshot + pid_correction_2;// итоговая мощность правый передний мотор
	   total_power_2 = constrain(total_power_2 + TRIM_FRONT_RIGHT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   float pid_correction_3 = forse_roll - forse_pitch - forse_yaw;
	   pid_correction_3 = constrain_float(pid_correction_3, -max_pid_correction_mshot, max_pid_correction_mshot);
	   int total_power_3 = throttle_mshot + pid_correction_3;// итоговая мощность левый задний мотор
	   total_power_3 = constrain(total_power_3 + TRIM_REAR_LEFT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   float pid_correction_4 = -forse_roll - forse_pitch + forse_yaw;
	   pid_correction_4 = constrain_float(pid_correction_4, -max_pid_correction_mshot, max_pid_correction_mshot);
	   int total_power_4 = throttle_mshot + pid_correction_4;// итоговая мощность правый задний мотор
	   total_power_4 = constrain(total_power_4 + TRIM_REAR_RIGHT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   Multishot_SetThrottle(total_power_1, TIM_CHANNEL_1);
	   Multishot_SetThrottle(total_power_2, TIM_CHANNEL_2);
	   Multishot_SetThrottle(total_power_3, TIM_CHANNEL_3);
	   Multishot_SetThrottle(total_power_4, TIM_CHANNEL_4);

	  } else {

	   Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_1);
	   Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_2);
	   Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_3);
	   Multishot_SetThrottle(MIN_PULSE_WIDTH, TIM_CHANNEL_4);

	  }

	  uint32_t dt = HAL_GetTick() - last_time;
	  freq = 1000.0f / dt;  // Частота в Гц
	  last_time = HAL_GetTick();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  InitBMX055();// инициализация imu

  arm_biquad_cascade_df2T_init_f32(&lpf_Gx, NUM_STAGES, Coeffs_lpf, filterState_lpf_Gx); // инициализация фильтра
  arm_biquad_cascade_df2T_init_f32(&lpf_Gy, NUM_STAGES, Coeffs_lpf, filterState_lpf_Gy);
  arm_biquad_cascade_df2T_init_f32(&lpf_Gz, NUM_STAGES, Coeffs_lpf, filterState_lpf_Gz);

  arm_biquad_cascade_df2T_init_f32(&lpf_Ax, NUM_STAGES, Coeffs_lpf_accel, filterState_lpf_Ax); // инициализация фильтра
  arm_biquad_cascade_df2T_init_f32(&lpf_Ay, NUM_STAGES, Coeffs_lpf_accel, filterState_lpf_Ay);
  arm_biquad_cascade_df2T_init_f32(&lpf_Az, NUM_STAGES, Coeffs_lpf_accel, filterState_lpf_Az);

  arm_biquad_cascade_df2T_init_f32(&lpf_Errorx, NUM_STAGES, Coeffs_lpf_Error, filterState_lpf_Errorx); // инициализация фильтра
  arm_biquad_cascade_df2T_init_f32(&lpf_Errory, NUM_STAGES, Coeffs_lpf_Error, filterState_lpf_Errory);
  arm_biquad_cascade_df2T_init_f32(&lpf_Errorz, NUM_STAGES, Coeffs_lpf_Error, filterState_lpf_Errorz);

  kalman_filter_init(&gyro_filter_x, GYRO_PROCESS_NOISE, GYRO_MEASUREMENT_NOISE, GYRO_ESTIMATION_ERROR, GYRO_INITIAL_VALUE);
  kalman_filter_init(&gyro_filter_y, GYRO_PROCESS_NOISE, GYRO_MEASUREMENT_NOISE, GYRO_ESTIMATION_ERROR, GYRO_INITIAL_VALUE);
  kalman_filter_init(&gyro_filter_z, GYRO_PROCESS_NOISE, GYRO_MEASUREMENT_NOISE, GYRO_ESTIMATION_ERROR, GYRO_INITIAL_VALUE);

  kalman_filter_init(&accel_filter_x, ACCEL_PROCESS_NOISE, ACCEL_MEASUREMENT_NOISE, ACCEL_ESTIMATION_ERROR, ACCEL_INITIAL_VALUE);
  kalman_filter_init(&accel_filter_y, ACCEL_PROCESS_NOISE, ACCEL_MEASUREMENT_NOISE, ACCEL_ESTIMATION_ERROR, ACCEL_INITIAL_VALUE);
  kalman_filter_init(&accel_filter_z, ACCEL_PROCESS_NOISE, ACCEL_MEASUREMENT_NOISE, ACCEL_ESTIMATION_ERROR, ACCEL_Z_INITIAL_VALUE);

  init_notch_filter(&notch_ax, NOTCH_CENTER_FREQ, NOTCH_WIDTH, NOTCH_SAMPLE_RATE);
  init_notch_filter(&notch_ay, NOTCH_CENTER_FREQ, NOTCH_WIDTH, NOTCH_SAMPLE_RATE);
  init_notch_filter(&notch_az, NOTCH_CENTER_FREQ, NOTCH_WIDTH, NOTCH_SAMPLE_RATE);

  init_notch_filter(&notch_gx, NOTCH_CENTER_FREQ, NOTCH_WIDTH, NOTCH_SAMPLE_RATE);
  init_notch_filter(&notch_gy, NOTCH_CENTER_FREQ, NOTCH_WIDTH, NOTCH_SAMPLE_RATE);
  init_notch_filter(&notch_gz, NOTCH_CENTER_FREQ, NOTCH_WIDTH, NOTCH_SAMPLE_RATE);

  init_notch_filter(&notch_roll, NOTCH_CENTER_FREQ, ATTITUDE_NOTCH_WIDTH, NOTCH_SAMPLE_RATE);
  init_notch_filter(&notch_pitch, NOTCH_CENTER_FREQ, ATTITUDE_NOTCH_WIDTH, NOTCH_SAMPLE_RATE);
  init_notch_filter(&notch_yaw, NOTCH_CENTER_FREQ, ATTITUDE_NOTCH_WIDTH, NOTCH_SAMPLE_RATE);

  HPF_Init(&hpf_x, HPF_ALPHA, Gx[0]);
  HPF_Init(&hpf_y, HPF_ALPHA, Gy[0]);
  HPF_Init(&hpf_z, HPF_ALPHA, Gz[0]);

  PID_Init(&pitch_pid, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, ALPHA, ALPHA_DERIVATIVE, INTEGRAL_LIMIT, SCALE_FACTOR);
  PID_Init(&roll_pid, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, ALPHA, ALPHA_DERIVATIVE, INTEGRAL_LIMIT, SCALE_FACTOR);
  PID_Init(&yaw_pid, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, ALPHA, ALPHA_DERIVATIVE, INTEGRAL_LIMIT, SCALE_FACTOR);

  bias();

  last_time = HAL_GetTick();

  //HPF_Init(&hpf_x, HPF_ALPHA, Gx[0]);
  //HPF_Init(&hpf_y, HPF_ALPHA, Gy[0]);
  //HPF_Init(&hpf_z, HPF_ALPHA, Gz[0]);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim11);
  HAL_UART_Receive_DMA(&huart6, buffer_message, DATA_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //snprintf(buf, sizeof(buf),"freq %f,button %d,potentiometer_value %d,joystick_x %d,joystick_y %d,right_left %d,pitch %f,roll %f,yaw %f\n",
	  //freq,button,potentiometer_value,joystick_x,joystick_y,right_left,pitch,roll,yaw);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim11) {
        run_control_loop(); // Вызов всей логики управления
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
