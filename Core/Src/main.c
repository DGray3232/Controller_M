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
#include "remote_control.h"
#include "remote_control_mavlink.h"
#include "config_param.h"
#include "globals.h"
#include "arm_math.h"
#include "limits.h"
#include "arm_const_structs.h"  // Для предопределенных структур БПФ
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "calculate_notch_coeffs.h"
#include "vibration_analysis.h"
#include "optical_flow_compensation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void normalizeQuaternion(float q[4]);
int constrain(int value, int min_val, int max_val);
void micolink_decode(uint8_t data, uint32_t* distance, uint8_t* distance_strength,
                     uint8_t* distance_precision, uint8_t* distance_status,
                     int16_t* flow_velocity_x, int16_t* flow_velocity_y,
                     uint8_t* flow_quality, uint8_t* flow_status);
void uart6_transmit_dma_with_rx_pause(uint8_t* data, uint16_t size);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция для добавления значения в кольцевой буфер x
void add_to_x(float32_t value) {
	Ax_fft[x_index] = value;
    x_index = (x_index + 1) % FFT_LEN;
}
// Функция для добавления значения в кольцевой буфер y
void add_to_y(float32_t value) {
	Ay_fft[y_index] = value;
    y_index = (y_index + 1) % FFT_LEN;
}
// Функция для добавления значения в кольцевой буфер z
void add_to_z(float32_t value) {
	Az_fft[z_index] = value;
    z_index = (z_index + 1) % FFT_LEN;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////Обновление коэффициенты для режекторного фильтра////////////////////////////////////////////////////

void update_coeff() {
		 vibration_frequency_x = calculate_vibration_frequency(Ax_fft, FFT_LEN, F_SAMPLE, fft_output_buffer, fft_magnitude_buffer);
    	 if (vibration_frequency_x == 0) {
    		 calculateNotchFilterCoeffs(1000, 0, 5, Coeffs_notch_x, 1.0);
    	 }
    	 if (fabs(vibration_frequency_x - previous_freq_x) > FREQ_HYSTERESIS) {
    	     calculateNotchFilterCoeffs(1000, vibration_frequency_x, 5, Coeffs_notch_x, 0.9f);
    	     previous_freq_x = vibration_frequency_x;
    	     memset(filterState_Gx_notch, 0, sizeof(filterState_Gx_notch));
    	     memset(filterState_Ax_notch, 0, sizeof(filterState_Ax_notch));
    	     arm_biquad_cascade_df2T_init_f32(&imu_Gx_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_x, filterState_Gx_notch);
    	     arm_biquad_cascade_df2T_init_f32(&imu_Ax_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_x, filterState_Ax_notch);
    	 }
    	 vibration_frequency_y = calculate_vibration_frequency(Ay_fft, FFT_LEN, F_SAMPLE, fft_output_buffer, fft_magnitude_buffer);
   	 if (vibration_frequency_y == 0) {
   		 calculateNotchFilterCoeffs(1000, 0, 5, Coeffs_notch_y, 1.0);
   	 }
	 if (fabs(vibration_frequency_y - previous_freq_y) > FREQ_HYSTERESIS) {
	     calculateNotchFilterCoeffs(1000, vibration_frequency_y, 5, Coeffs_notch_y, 0.9f);
	     previous_freq_y = vibration_frequency_y;
	     memset(filterState_Gy_notch, 0, sizeof(filterState_Gy_notch));
	     memset(filterState_Ay_notch, 0, sizeof(filterState_Ay_notch));
	     arm_biquad_cascade_df2T_init_f32(&imu_Gy_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_y, filterState_Gy_notch);
	     arm_biquad_cascade_df2T_init_f32(&imu_Ay_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_y, filterState_Ay_notch);
	 }
	 vibration_frequency_z = calculate_vibration_frequency(Az_fft, FFT_LEN, F_SAMPLE, fft_output_buffer, fft_magnitude_buffer);
      	 if (vibration_frequency_z == 0) {
      		 calculateNotchFilterCoeffs(1000, 0, 5, Coeffs_notch_z, 1.0);
      	 }
    	 if (fabs(vibration_frequency_z - previous_freq_z) > FREQ_HYSTERESIS) {
    	     calculateNotchFilterCoeffs(1000, vibration_frequency_z, 5, Coeffs_notch_z, 0.9f);
    	     previous_freq_z = vibration_frequency_z;
    	     memset(filterState_Gz_notch, 0, sizeof(filterState_Gz_notch));
    	     memset(filterState_Az_notch, 0, sizeof(filterState_Az_notch));
    	     arm_biquad_cascade_df2T_init_f32(&imu_Gz_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_z, filterState_Gz_notch);
    	     arm_biquad_cascade_df2T_init_f32(&imu_Az_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_z, filterState_Az_notch);
    	 }
    	count_calculate_frequency = 0;
    	count_calculate_frequency_flag = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////Multishot/////////////////////////////////////////////////////////////////////////////////

/**
  * @brief  Устанавливает значение тяги для всех моторов.
  * @param  m1, m2, m3, m4 Значения тяги в микросекундах (мкс) для моторов 1-4.
  *         Должны быть в диапазоне [MIN_PULSE_WIDTH, MAX_PULSE_WIDTH] (500-2500 мкс)
  * @retval None
  * @note   Функция просто записывает новые значения в глобальные буферы.
  *         DMA автоматически, в фоновом режиме, перенесет эти значения в таймер.
  */
void Motors_Set_Throttle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
  // Записываем новые значения напрямую в буферы, которые уже читаются DMA.
  // Ограничиваем значения.
  m1_pulse[0] = constrain(m1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  m2_pulse[0] = constrain(m2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  m3_pulse[0] = constrain(m3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  m4_pulse[0] = constrain(m4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////�?нициализация DMA/////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  �?нициализация DMA для работы с моторами в циклическом режиме.
  * @note   Настраивает DMA для непрерывной передачи данных из буферов mX_pulse
  *         в регистры сравнения таймера TIM1. Вызывается один раз при старте.
  * @retval None
  */
void Motors_DMA_Init(void)
{
  // Останавливаем DMA
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_4);
  // Запускаем DMA в циклицеском режиме для каждого канала.
  // Теперь DMA будет бесконечно отправлять значения из буферов mX_pulse
  // на соответствующие регистры CCR таймера.
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)&m1_pulse, 1);
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)&m2_pulse, 1);
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)&m3_pulse, 1);
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t*)&m4_pulse, 1);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

/**
 * @brief  Применяет процентное изменение к целочисленному значению.
 * @param  base_value  Исходное значение.
 * @param  percent     Процент изменения. Положительное число увеличивает,
 *                     отрицательное – уменьшает. 0 оставляет без изменений.
 * @retval Результат с учётом процента. 
 */
int32_t trim_percent(int32_t base_value, int8_t percent)
{
    return base_value + (base_value * percent) / 100;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////// фильтр махони ///////////////////////////////////////////////////////
/*
float round_to(float number) {
    return round(number * 100) / 100;
}
*/
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

	  float filterAx = filtered_Ax*9.81;
	  float filterAy = filtered_Ay*9.81;
	  float filterAz = filtered_Az*9.81;
	  float filterGx = filtered_Gx*DEG_TO_RAD;
	  float filterGy = filtered_Gy*DEG_TO_RAD;
	  float filterGz = filtered_Gz*DEG_TO_RAD;

	  //filterAx = round_to(filterAx);
	  //filterAy = round_to(filterAy);
	  //filterAz = round_to(filterAz);
	  //filterGx = round_to(filterGx);
	  //filterGy = round_to(filterGy);
	  //filterGz = round_to(filterGz);

	  MahonyAHRSupdateIMU(filterAx, filterAy, filterAz, filterGx, filterGy, filterGz, CONTROL_LOOP_DT);

	  Quat_actual[0] = (*(getQ()));
	  Quat_actual[1] = (*(getQ()+1));
	  Quat_actual[2] = (*(getQ()+2));
	  Quat_actual[3] = (*(getQ()+3));

	  float alpha = 1.0; // Коэффициент сглаживания (0 < alpha < 1)
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
/*
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
*/
////////////////////////////////////////////////////////////////// фильтрованные данные///////////////////////////////////////////////////////////////
void get_filter_data_accel() {

	Ax[0] = BMX055.Ax;
	Ax[0] = Ax[0] - offset_Ax;

	Ax_for_fft[0] = Ax[0];
	arm_biquad_cascade_df2T_f32 (&hpf_Ax, Ax_for_fft, filter_hpf_Ax_for_fft, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Ax, filter_hpf_Ax_for_fft, filter_lpf_Ax_for_fft, BLOCK_SIZE);
	filter_Ax_for_fft = filter_lpf_Ax_for_fft[0];
	add_to_x(filter_Ax_for_fft);

	Ax[0] = median_filter(Ax[0], &Accel_x);
    arm_biquad_cascade_df2T_f32 (&imu_Ax_notch, Ax, filter_Ax_notch, BLOCK_SIZE);
    arm_biquad_cascade_df2T_f32 (&imu_Ax_lpf, filter_Ax_notch, filter_Ax_lpf, BLOCK_SIZE);
    filtered_Ax = filter_Ax_lpf[0];
    if (isnan(filtered_Ax) || isinf(filtered_Ax)) {
    	filtered_Ax = prev_filtered_Ax;
    }
    prev_filtered_Ax = filtered_Ax;

    Ay[0] = BMX055.Ay;
	Ay[0] = Ay[0] - offset_Ay;

	Ay_for_fft[0] = Ay[0];
	arm_biquad_cascade_df2T_f32 (&hpf_Ay, Ay_for_fft, filter_hpf_Ay_for_fft, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Ay, filter_hpf_Ay_for_fft, filter_lpf_Ay_for_fft, BLOCK_SIZE);
	filter_Ay_for_fft = filter_lpf_Ay_for_fft[0];
	add_to_y(filter_Ay_for_fft);

	Ay[0] = median_filter(Ay[0], &Accel_y);
    arm_biquad_cascade_df2T_f32 (&imu_Ay_notch, Ay, filter_Ay_notch, BLOCK_SIZE);
    arm_biquad_cascade_df2T_f32 (&imu_Ay_lpf, filter_Ay_notch, filter_Ay_lpf, BLOCK_SIZE);
    filtered_Ay = filter_Ay_lpf[0];
    if (isnan(filtered_Ay) || isinf(filtered_Ay)) {
    	filtered_Ay = prev_filtered_Ay;
    }
    prev_filtered_Ay = filtered_Ay;

   	Az[0] = BMX055.Az;
	Az[0] = Az[0] - offset_Az;

	Az_for_fft[0] = Az[0];
	arm_biquad_cascade_df2T_f32 (&hpf_Az, Az_for_fft, filter_hpf_Az_for_fft, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Az, filter_hpf_Az_for_fft, filter_lpf_Az_for_fft, BLOCK_SIZE);
	filter_Az_for_fft = filter_lpf_Az_for_fft[0];
	add_to_z(filter_Az_for_fft);

	Az[0] = median_filter(Az[0], &Accel_z);
    arm_biquad_cascade_df2T_f32 (&imu_Az_notch, Az, filter_Az_notch, BLOCK_SIZE);
    arm_biquad_cascade_df2T_f32 (&imu_Az_lpf, filter_Az_notch, filter_Az_lpf, BLOCK_SIZE);
    filtered_Az = filter_Az_lpf[0];
    if (isnan(filtered_Az) || isinf(filtered_Az)) {
    	filtered_Az = prev_filtered_Az;
    }
    prev_filtered_Az = filtered_Az;

}

void get_filter_data_gyro() {

	Gx[0] = BMX055.Gx;
	Gx[0] = Gx[0] - bias_Gx;

	arm_biquad_cascade_df2T_f32 (&imu_Gx_D_lpf, Gx, filter_Gx_D_lpf, BLOCK_SIZE);
	filtered_Gx_D = filter_Gx_D_lpf[0];

	Gx[0] = median_filter(Gx[0], &Gyro_x);
	arm_biquad_cascade_df2T_f32 (&imu_Gx_notch, Gx, filter_Gx_notch, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&imu_Gx_lpf, filter_Gx_notch, filter_Gx_lpf, BLOCK_SIZE);
    filtered_Gx = filter_Gx_lpf[0];
    if (isnan(filtered_Gx) || isinf(filtered_Gx)) {
    	filtered_Gx = prev_filtered_Gx;
    }
    prev_filtered_Gx = filtered_Gx;

    Gy[0] = BMX055.Gy;
	Gy[0] = Gy[0] - bias_Gy;

	arm_biquad_cascade_df2T_f32 (&imu_Gy_D_lpf, Gy, filter_Gy_D_lpf, BLOCK_SIZE);
	filtered_Gy_D = filter_Gy_D_lpf[0];

	Gy[0] = median_filter(Gy[0], &Gyro_y);
	arm_biquad_cascade_df2T_f32 (&imu_Gy_notch, Gy, filter_Gy_notch, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&imu_Gy_lpf, filter_Gy_notch, filter_Gy_lpf, BLOCK_SIZE);
   	filtered_Gy = filter_Gy_lpf[0];
    if (isnan(filtered_Gy) || isinf(filtered_Gy)) {
    	filtered_Gy = prev_filtered_Gy;
    }
    prev_filtered_Gy = filtered_Gy;

   	Gz[0] = BMX055.Gz;
    Gz[0] = Gz[0] - bias_Gz;

	arm_biquad_cascade_df2T_f32 (&imu_Gz_D_lpf, Gz, filter_Gz_D_lpf, BLOCK_SIZE);
	filtered_Gz_D = filter_Gz_D_lpf[0];

	Gz[0] = median_filter(Gz[0], &Gyro_z);
	arm_biquad_cascade_df2T_f32 (&imu_Gz_notch, Gz, filter_Gz_notch, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&imu_Gz_lpf, filter_Gz_notch, filter_Gz_lpf, BLOCK_SIZE);
   	filtered_Gz = filter_Gz_lpf[0];
    if (isnan(filtered_Gz) || isinf(filtered_Gz)) {
    	filtered_Gz = prev_filtered_Gz;
    }
    prev_filtered_Gz = filtered_Gz;
}

void get_mtf_data() {
    for(int i = 0; i < MIKOLINL; i++) {
        micolink_decode(buffer_message_mtf02[i],
                       &distance, &distance_strength, &distance_precision, &distance_status,
                       &flow_velocity_x, &flow_velocity_y, &flow_quality, &flow_status);
    }
}
HAL_StatusTypeDef I2C_Start_Read_gyro(void) {
    current_device = 0;
    return BMX055_Read_Gyro_DMA(&hi2c1, dma_gyro_buffer);
}

HAL_StatusTypeDef I2C_Start_Read_accel(void) {
    current_device = 1;
    return BMX055_Read_Accel_DMA(&hi2c1, dma_accel_buffer);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////смещение imu///////////////////////////////////////////////////////////////

void bias() {
	for (int i = 0; i < BIAS_OFSET; i++) {
		BMX055_Read_Gyro(&hi2c1, &BMX055);
		float32_t gx = median_filter(BMX055.Gx, &Gyro_x_bias);
		float32_t gy = median_filter(BMX055.Gy, &Gyro_y_bias);
		float32_t gz = median_filter(BMX055.Gz, &Gyro_z_bias);
		bias_Gx += gx;
		bias_Gy += gy;
		bias_Gz += gz;
		BMX055_Read_Accel(&hi2c1, &BMX055);
		float32_t ax = median_filter(BMX055.Ax, &Accel_x_ofset);
		float32_t ay = median_filter(BMX055.Ay, &Accel_y_ofset);
		float32_t az = median_filter(BMX055.Az, &Accel_z_ofset);
		offset_Ax += ax;
		offset_Ay += ay;
		offset_Az += az;
		HAL_Delay(1);
	}
 bias_Gx = bias_Gx / BIAS_OFSET;
 bias_Gy = bias_Gy / BIAS_OFSET;
 bias_Gz = bias_Gz / BIAS_OFSET;
 offset_Ax = offset_Ax / BIAS_OFSET;
 offset_Ay = offset_Ay / BIAS_OFSET;
 offset_Az = (offset_Az / BIAS_OFSET) - 1;
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
        	break;
    }
    }
    while (BMX055_BMG_Init(&hi2c1) == 1) {
  	  retriesG++;
        if (retriesG >= 10) {
        	break;
        }
    }
/*
    while (BMX055_BMM_Init(&hi2c1) == 1) {
  	  retriesM++;
        if (retriesM >= 10) {
        }
    }
*/

    //setFastOffset_BMA(&hi2c1);
    setFastOffset_BMG(&hi2c1);
    //setFastOffset_BMM(&hi2c1, 1);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void run_control_loop(){

	data_ready_gyro = 0;
	data_ready_accel = 0;

	count_calculate_frequency++;
	if (count_calculate_frequency == 1000) {
		count_calculate_frequency_flag = 1;
	}
    // Определяем режим полета по кнопке
    if (button_mode == 0) {
        flight_mode = FLIGHT_MODE_ACRO;
    } else if (button_mode == 1) {
        flight_mode = FLIGHT_MODE_ANGLE;
    }
	active_mode = flight_mode; // Определяем активный режим с приоритетом
	get_filter_data_accel(); // получение отфильтрованных данных акселерометра
	get_filter_data_gyro(); // получение отфильтрованных данных гироскопа
	get_mtf_data();   // Обработка буфера c данными датчика mtf
    gyro_integration_update(&gyro_integration, filtered_Gx, filtered_Gy); // обновление гпроскопа - 1000 Гц
    process_optical_flow_data(&optical_flow_results,&gyro_integration,&lpf_mtf_x, &lpf_mtf_y,distance,flow_velocity_x, flow_velocity_y, pitch, roll); // обработка оптического потока 50 Гц
	get_angle_mahony();  // получение кватернионов и углов
    //eulerToQuaternion(joystick_x, joystick_y, right_left, Quat_target); // функция преобразования углов в кватернион
	//quat_error(Quat_actual, Quat_actual_inv, Quat_target, Quat_error); // ошибка между текущим и целевым кватерниолном
	throttle_mshot = ((potentiometer_value / 1000.0) * 2000.0) + 500;
	max_pid_correction_mshot = MAX_CORRECTION;  //Ограничение пид-коррекции
	//error_pitch_angle = round_to(Quat_error[2] * (-1)); // ошибка по углу тангажа знак ошибки изменен
	//error_roll_angle = round_to(Quat_error[1]); // ошибка по углу крена
	error_pitch_angle = (Quat_error[2] * (-1)); // ошибка по углу тангажа знак ошибки изменен
	error_roll_angle = (Quat_error[1]); // ошибка по углу крена
	//текущая угловая скорость
	actual_velocity_pitch = (-filtered_Gy);
	actual_velocity_roll = filtered_Gx;
	actual_velocity_yaw = filtered_Gz;
	//текущая угловая скорость для д компоненты пид
	actual_velocity_pitch_D = (-filtered_Gy_D);
	actual_velocity_roll_D = filtered_Gx_D;
	actual_velocity_yaw_D = filtered_Gz_D;
    // MTF имеет высший приоритет при выполнении условий
	if (distance > 100) {
	    active_mode = FLIGHT_MODE_MTF;
	}
	// Вычисляем target_velocity на основе активного режима
    switch(active_mode) {
	    case FLIGHT_MODE_MTF: {
	        // 1. Читаем стики как целевую скорость (например, макс 50 см/с)
	        float target_speed_x = expo_curve(joystick_y, 0.01f) * 50.0f;
	        float target_speed_y = expo_curve(joystick_x, 0.01f) * 50.0f;

	        static float target_angle_pitch_mtf = 0.0f;
	        static float target_angle_roll_mtf = 0.0f;

	        // Работает на частоте 50 Гц
	        if (optical_flow_results.new_optical_data_available) {
	            error_pitch_mtf = target_speed_y - optical_flow_results.speed_cm_s_y;
	            error_roll_mtf  = target_speed_x - optical_flow_results.speed_cm_s_x;

	            // ПИД скорости вычисляет ТРЕБУЕМЫЙ УГОЛ НАКЛОНА (в градусах)
	            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, optical_flow_results.speed_cm_s_y, 0.020f);
	            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, optical_flow_results.speed_cm_s_x, 0.020f);

	            // Ограничиваем максимальный наклон для безопасности (макс 15 градусов)
	            target_angle_pitch_mtf = constrain_float(target_angle_pitch_mtf, -15.0f, 15.0f);
	            target_angle_roll_mtf = constrain_float(target_angle_roll_mtf, -15.0f, 15.0f);

			// ----- Удержание высоты по button_2 -----
			// Детектируем фронт включения кнопки для захвата текущей высоты
			if (button_2 == 1 && last_button_2_state == false) {
    			// Компенсация наклона при захвате высоты
                target_altitude_mm = (float)distance * cosf(pitch * DEG_TO_RAD) * cosf(roll  * DEG_TO_RAD);
    			altitude_hold_active = true;
			}
			last_button_2_state = (button_2 == 1);
			// Если кнопка отпущена, выключаем удержание и сбрасываем интегратор
			if (button_2 == 0) {
    			altitude_hold_active = false;
    			PID_Reset(&altitude_pid); 
    			throttle_altitude_correction = 0.0f;
			}
			// Вычисление коррекции тяги (работает с частотой получения distance ~50 Гц)
			if (altitude_hold_active) {
				float current_altitude_mm = (float)distance * cosf(pitch * DEG_TO_RAD) * cosf(roll  * DEG_TO_RAD);
    			altitude_error_mm = target_altitude_mm - current_altitude_mm;
				// Ограничиваем ошибку, чтобы избежать резких скачков
                altitude_error_mm = constrain_float(altitude_error_mm, -500.0f, 500.0f);
				// Вычисляем ПИД-выход (можно использовать PID_Compute или PID_DoM_Compute)
    			throttle_altitude_correction = PID_Compute(&altitude_pid, altitude_error_mm,0.020f);
                // Ограничиваем коррекцию
                throttle_altitude_correction = constrain_float(throttle_altitude_correction,-ALTITUDE_MAX_CORRECTION,ALTITUDE_MAX_CORRECTION);
			}	  
			// ----- Конец удержание высоты по button_2 -----

	            optical_flow_results.new_optical_data_available = false;
	        }

	        // 2. Внутренний контур ANGLE (1000 Гц)
	        // Вычисляем ошибку угла в градусах (целевой угол от MTF минус текущий угол)
	        error_pitch_angle = target_angle_pitch_mtf - pitch;
	        error_roll_angle = target_angle_roll_mtf - roll;

	        // ANGLE ПИД вычисляет требуемую угловую скорость (Rate)
	        target_velocity_pitch = PID_DoM_Compute(&pitch_pid_angle_DoM, error_pitch_angle, pitch, 0.001f);
	        target_velocity_roll = PID_DoM_Compute(&roll_pid_angle_DoM, error_roll_angle, roll, 0.001f);
	        target_velocity_yaw = expo_curve(right_left, 0.01f) * 0.5f;

	        target_velocity_pitch = safe_pid_value(target_velocity_pitch, MAX_P);
	        target_velocity_roll = safe_pid_value(target_velocity_roll, MAX_P);

	        // 3. Вычисляем ошибку для контура RATE
	        error_pitch_rate = target_velocity_pitch - actual_velocity_pitch;
	        error_roll_rate = target_velocity_roll - actual_velocity_roll;
	        error_yaw_rate = target_velocity_yaw - actual_velocity_yaw;

	        error_pitch_rate_D = target_velocity_pitch - actual_velocity_pitch_D;
	        error_roll_rate_D = target_velocity_roll - actual_velocity_roll_D;
	        error_yaw_rate_D = target_velocity_yaw - actual_velocity_yaw_D;
	        break;
	    }

	    case FLIGHT_MODE_ANGLE: {

			float target_angle_pitch_rc = (joystick_x / 100.0f) * 45.0f;
        	float target_angle_roll_rc  = (joystick_y / 100.0f) * 45.0f;

        	error_pitch_angle = target_angle_pitch_rc - pitch;
        	error_roll_angle  = target_angle_roll_rc - roll;

	        target_velocity_pitch = PID_DoM_Compute(&pitch_pid_angle_DoM, error_pitch_angle, pitch, 0.001f);
	        target_velocity_roll = PID_DoM_Compute(&roll_pid_angle_DoM, error_roll_angle, roll, 0.001f);
	        target_velocity_yaw = expo_curve(right_left, 0.01f) * 0.5f;

	        target_velocity_pitch = safe_pid_value(target_velocity_pitch, MAX_P);
	        target_velocity_roll = safe_pid_value(target_velocity_roll, MAX_P);

	        // Вычисляем ошибку для контура RATE
	        error_pitch_rate = target_velocity_pitch - actual_velocity_pitch;
	        error_roll_rate = target_velocity_roll - actual_velocity_roll;
	        error_yaw_rate = target_velocity_yaw - actual_velocity_yaw;

	        error_pitch_rate_D = target_velocity_pitch - actual_velocity_pitch_D;
	        error_roll_rate_D = target_velocity_roll - actual_velocity_roll_D;
	        error_yaw_rate_D = target_velocity_yaw - actual_velocity_yaw_D;
	        break;
	    }

	    case FLIGHT_MODE_ACRO:
	    default: {
	        target_velocity_roll = expo_curve(joystick_y, 0.01f);
	        target_velocity_pitch = expo_curve(joystick_x, 0.01f);
	        target_velocity_yaw = expo_curve(right_left, 0.01f);

	        error_pitch_rate = target_velocity_pitch - actual_velocity_pitch;
	        error_roll_rate = target_velocity_roll - actual_velocity_roll;
	        error_yaw_rate = target_velocity_yaw - actual_velocity_yaw;

	        error_pitch_rate_D = target_velocity_pitch - actual_velocity_pitch_D;
	        error_roll_rate_D = target_velocity_roll - actual_velocity_roll_D;
	        error_yaw_rate_D = target_velocity_yaw - actual_velocity_yaw_D;
	        flight_mode = FLIGHT_MODE_ACRO;
	        break;
	    }
	}

	// Итоговая тяга с учётом удержания высоты
    final_throttle = throttle_mshot;
    if (altitude_hold_active) {
        final_throttle += throttle_altitude_correction;
        final_throttle = constrain(final_throttle, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	}

	///////компенсация потери вертикальной тяги при наклоне//////////////////////////////////////////
    float pitch_rad = pitch * DEG_TO_RAD;
    float roll_rad  = roll * DEG_TO_RAD;
	// Коэффициент компенсации: 1 / (cos(pitch) * cos(roll))
	float tilt_comp = 1.0f / (cosf(pitch_rad) * cosf(roll_rad));
	// Защита от слишком больших углов (чтобы не было деления на ноль или большой мощности)
	if (tilt_comp > 2.0f) {
    	tilt_comp = 2.0f;   // максимум двойной газ
	}
	if (tilt_comp < 1.0f) {
    	tilt_comp = 1.0f;   // при углах, близких к нулю, не уменьшаем газ меньше исходного
	}
	// Применяем компенсацию к итоговой мощности моторов
	final_throttle = (uint16_t)((float)final_throttle * tilt_comp);
	// Финальное ограничение диапазоном ШИМ
	final_throttle = constrain(final_throttle, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    ///////конец компенсации потери вертикальной тяги при наклоне//////////////////////////////////////////

	if(button == 1 && potentiometer_value > 0){

		//forse_pitch_rate = PID_2_Compute(&pitch_pid_rate, error_pitch_rate, error_pitch_rate_D);
		//forse_roll_rate = PID_2_Compute(&roll_pid_rate, error_roll_rate, error_roll_rate_D);
		//forse_yaw_rate = PID_2_Compute(&yaw_pid_rate, error_yaw_rate, error_yaw_rate_D);

	   forse_pitch_rate = PID_DoM_Compute(&pitch_pid_rate_DoM, error_pitch_rate, actual_velocity_pitch_D, 0.001f);
	   forse_roll_rate = PID_DoM_Compute(&roll_pid_rate_DoM, error_roll_rate, actual_velocity_roll_D, 0.001f);
	   forse_yaw_rate = PID_DoM_Compute(&yaw_pid_rate_DoM, error_yaw_rate, actual_velocity_yaw_D, 0.001f);

	   // Проверяем на NaN, Inf и ограничиваем
	   forse_pitch_rate = safe_pid_value(forse_pitch_rate, MAX_PID_RATE);
	   forse_roll_rate = safe_pid_value(forse_roll_rate, MAX_PID_RATE);
	   forse_yaw_rate = safe_pid_value(forse_yaw_rate, MAX_PID_RATE);

	   pid_correction_1 = forse_roll_rate + forse_pitch_rate + forse_yaw_rate;
	   pid_correction_1 = constrain_float(pid_correction_1, -max_pid_correction_mshot, max_pid_correction_mshot);
	   //total_power_1 = throttle_mshot + pid_correction_1;// итоговая мощность левый передний мотор
	   total_power_1 = final_throttle + pid_correction_1;// итоговая мощность левый передний мотор
	   //total_power_1 = constrain(total_power_1 + TRIM_FRONT_LEFT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   total_power_1 = constrain(trim_percent(total_power_1, TRIM_PERCENT_FRONT_LEFT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_2 = -forse_roll_rate + forse_pitch_rate - forse_yaw_rate;
	   pid_correction_2 = constrain_float(pid_correction_2, -max_pid_correction_mshot, max_pid_correction_mshot);
	   //total_power_2 = throttle_mshot + pid_correction_2;// итоговая мощность правый передний мотор
	   total_power_2 = final_throttle + pid_correction_2;// итоговая мощность правый передний мотор
	   //total_power_2 = constrain(total_power_2 + TRIM_FRONT_RIGHT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   total_power_2 = constrain(trim_percent(total_power_2, TRIM_PERCENT_FRONT_RIGHT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_3 = forse_roll_rate - forse_pitch_rate - forse_yaw_rate;
	   pid_correction_3 = constrain_float(pid_correction_3, -max_pid_correction_mshot, max_pid_correction_mshot);
	   //total_power_3 = throttle_mshot + pid_correction_3;// итоговая мощность левый задний мотор
	   total_power_3 = final_throttle + pid_correction_3;// итоговая мощность левый задний мотор
	   //total_power_3 = constrain(total_power_3 + TRIM_REAR_LEFT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   total_power_3 = constrain(trim_percent(total_power_3, TRIM_PERCENT_REAR_LEFT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_4 = -forse_roll_rate - forse_pitch_rate + forse_yaw_rate;
	   pid_correction_4 = constrain_float(pid_correction_4, -max_pid_correction_mshot, max_pid_correction_mshot);
	   //total_power_4 = throttle_mshot + pid_correction_4;// итоговая мощность правый задний мотор
	   total_power_4 = final_throttle + pid_correction_4;// итоговая мощность правый задний мотор
	   //total_power_4 = constrain(total_power_4 + TRIM_REAR_RIGHT, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   total_power_4 = constrain(trim_percent(total_power_4, TRIM_PERCENT_REAR_RIGHT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   
       // Применяем экспонинциального сглаживания к каждому сигналу мотора
       filtered_power_1 = MOTOR_OUTPUT_FILTER_ALPHA * (int)total_power_1 + (1.0f - MOTOR_OUTPUT_FILTER_ALPHA) * filtered_power_1;
       filtered_power_2 = MOTOR_OUTPUT_FILTER_ALPHA * (int)total_power_2 + (1.0f - MOTOR_OUTPUT_FILTER_ALPHA) * filtered_power_2;
       filtered_power_3 = MOTOR_OUTPUT_FILTER_ALPHA * (int)total_power_3 + (1.0f - MOTOR_OUTPUT_FILTER_ALPHA) * filtered_power_3;
       filtered_power_4 = MOTOR_OUTPUT_FILTER_ALPHA * (int)total_power_4 + (1.0f - MOTOR_OUTPUT_FILTER_ALPHA) * filtered_power_4;
       filtered_power_1 = constrain_float(filtered_power_1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
       filtered_power_2 = constrain_float(filtered_power_2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
       filtered_power_3 = constrain_float(filtered_power_3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
       filtered_power_4 = constrain_float(filtered_power_4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   // Передаем на моторы уже отфильтрованные значения
	   Motors_Set_Throttle((uint16_t)filtered_power_1,(uint16_t)filtered_power_2,(uint16_t)filtered_power_3,(uint16_t)filtered_power_4);
	   
	   /*
       total_power_1 = constrain_float(total_power_1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
       total_power_2 = constrain_float(total_power_2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
       total_power_3 = constrain_float(total_power_3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
       total_power_4 = constrain_float(total_power_4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	   Motors_Set_Throttle((uint16_t)total_power_1,(uint16_t)total_power_2,(uint16_t)total_power_3,(uint16_t)total_power_4);
	   */
	   //Motors_Set_Throttle(MAX_PULSE_WIDTH, MAX_PULSE_WIDTH, MAX_PULSE_WIDTH, MAX_PULSE_WIDTH);

	  } else {

	   // Сбрасываем состояния фильтра на минимальное значение
	   filtered_power_1 = MIN_PULSE_WIDTH;
	   filtered_power_2 = MIN_PULSE_WIDTH;
	   filtered_power_3 = MIN_PULSE_WIDTH;
	   filtered_power_4 = MIN_PULSE_WIDTH;

	   total_power_1 = 500;
	   total_power_2 = 500;
	   total_power_3 = 500;
	   total_power_4 = 500;

	   PID_2_Reset(&pitch_pid_rate);
	   PID_2_Reset(&roll_pid_rate);
	   PID_2_Reset(&yaw_pid_rate);

	   Motors_Set_Throttle(MIN_PULSE_WIDTH, MIN_PULSE_WIDTH, MIN_PULSE_WIDTH, MIN_PULSE_WIDTH);

	  }
/*
	  uint32_t current_cycle_time = DWT->CYCCNT;
	  uint32_t dt_cycles = current_cycle_time - last_cycle_time;
	  last_cycle_time = current_cycle_time;
	  HAL_RCC_GetHCLKFreq(); // вернет частоту ядра
	  if (dt_cycles > 0) {
	      freq = (float)HAL_RCC_GetHCLKFreq() / (float)dt_cycles;
	  }
*/
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
  HAL_Delay(2000);// Даем питанию стабилизироваться, а датчикам загрузиться
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart6, buffer_message_mtf02, MIKOLINL);
  //HAL_UART_Receive_DMA(&huart1, buffer_message, DATA_SIZE); //для кастомного протокола remote_control
  MAV_Init(&huart1); //инициализация для протокола mavlink

  I2C_Bus_Reset(&hi2c1);
  InitBMX055();// инициализация imu

  arm_rfft_fast_init_f32(&fft_params, FFT_LEN);

  arm_biquad_cascade_df2T_init_f32(&imu_Gx_lpf, NUM_STAGES_GYRO_LPF, Coeffs_gyro_lpf, filterState_Gx_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Gx_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_x, filterState_Gx_notch);
  arm_biquad_cascade_df2T_init_f32(&imu_Gy_lpf, NUM_STAGES_GYRO_LPF, Coeffs_gyro_lpf, filterState_Gy_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Gy_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_y, filterState_Gy_notch);
  arm_biquad_cascade_df2T_init_f32(&imu_Gz_lpf, NUM_STAGES_GYRO_LPF, Coeffs_gyro_lpf, filterState_Gz_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Gz_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_z, filterState_Gz_notch);

  arm_biquad_cascade_df2T_init_f32(&imu_Ax_lpf, NUM_STAGES_ACCEL_LPF, Coeffs_accel_lpf, filterState_Ax_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Ax_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_x, filterState_Ax_notch);
  arm_biquad_cascade_df2T_init_f32(&imu_Ay_lpf, NUM_STAGES_ACCEL_LPF, Coeffs_accel_lpf, filterState_Ay_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Ay_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_y, filterState_Ay_notch);
  arm_biquad_cascade_df2T_init_f32(&imu_Az_lpf, NUM_STAGES_ACCEL_LPF, Coeffs_accel_lpf, filterState_Az_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Az_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_z, filterState_Az_notch);

  arm_biquad_cascade_df2T_init_f32(&imu_Gx_D_lpf, NUM_STAGES_D_GYRO_LPF, Coeffs_D_Gyro_lpf, filterState_Gx_D_lpf); // инициализация фильтра
  arm_biquad_cascade_df2T_init_f32(&imu_Gy_D_lpf, NUM_STAGES_D_GYRO_LPF, Coeffs_D_Gyro_lpf, filterState_Gy_D_lpf);
  arm_biquad_cascade_df2T_init_f32(&imu_Gz_D_lpf, NUM_STAGES_D_GYRO_LPF, Coeffs_D_Gyro_lpf, filterState_Gz_D_lpf);

  arm_biquad_cascade_df2T_init_f32(&lpf_Ax, NUM_STAGES_ACCEL_FFT, Coeffs_accel_lpf_fft, filterState_Ax_lpf_fft); // инициализация фильтра
  arm_biquad_cascade_df2T_init_f32(&lpf_Ay, NUM_STAGES_ACCEL_FFT, Coeffs_accel_lpf_fft, filterState_Ay_lpf_fft);
  arm_biquad_cascade_df2T_init_f32(&lpf_Az, NUM_STAGES_ACCEL_FFT, Coeffs_accel_lpf_fft, filterState_Az_lpf_fft);

  arm_biquad_cascade_df2T_init_f32(&hpf_Ax, NUM_STAGES_ACCEL_FFT, Coeffs_accel_hpf_fft, filterState_Ax_hpf_fft); // инициализация фильтра
  arm_biquad_cascade_df2T_init_f32(&hpf_Ay, NUM_STAGES_ACCEL_FFT, Coeffs_accel_hpf_fft, filterState_Ay_hpf_fft);
  arm_biquad_cascade_df2T_init_f32(&hpf_Az, NUM_STAGES_ACCEL_FFT, Coeffs_accel_hpf_fft, filterState_Az_hpf_fft);


  //PID_2_Init(&pitch_pid_rate, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, ALPHA, ALPHA_DERIVATIVE, INTEGRAL_LIMIT, SCALE_FACTOR);
  //PID_2_Init(&roll_pid_rate, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, ALPHA, ALPHA_DERIVATIVE, INTEGRAL_LIMIT, SCALE_FACTOR);
  //PID_2_Init(&yaw_pid_rate, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, ALPHA, ALPHA_DERIVATIVE, INTEGRAL_LIMIT, SCALE_FACTOR);

  //PID_Init(&pitch_pid_angle, PITCH_PID_KP_RATE, PITCH_PID_KI_RATE, PITCH_PID_KD_RATE, ALPHA_RATE, ALPHA_DERIVATIVE_RATE, INTEGRAL_LIMIT_RATE, SCALE_FACTOR_RATE);
  //PID_Init(&roll_pid_angle, ROLL_PID_KP_RATE, ROLL_PID_KI_RATE, ROLL_PID_KD_RATE, ALPHA_RATE, ALPHA_DERIVATIVE_RATE, INTEGRAL_LIMIT_RATE, SCALE_FACTOR_RATE);

  //PID_Init(&pitch_pid_mtf, PITCH_PID_KP_MTF, PITCH_PID_KI_MTF, PITCH_PID_KD_MTF, ALPHA_MTF, ALPHA_DERIVATIVE_MTF, INTEGRAL_LIMIT_MTF, SCALE_FACTOR_MTF);
  //PID_Init(&roll_pid_mtf, ROLL_PID_KP_MTF, ROLL_PID_KI_MTF, ROLL_PID_KD_MTF, ALPHA_MTF, ALPHA_DERIVATIVE_MTF, INTEGRAL_LIMIT_MTF, SCALE_FACTOR_MTF);

  //PID_DoM_Init(&pitch_pid_rate_DoM, PITCH_PID_KP_DoM, PITCH_PID_KI_DoM, PITCH_PID_KD_DoM, ALPHA_DoM, ALPHA_DERIVATIVE_DoM, INTEGRAL_LIMIT_DoM, SCALE_FACTOR_DoM);
  //PID_DoM_Init(&roll_pid_rate_DoM, ROLL_PID_KP_DoM, ROLL_PID_KI_DoM, ROLL_PID_KD_DoM, ALPHA_DoM, ALPHA_DERIVATIVE_DoM, INTEGRAL_LIMIT_DoM, SCALE_FACTOR_DoM);
  //PID_DoM_Init(&yaw_pid_rate_DoM, YAW_PID_KP_DoM, YAW_PID_KI_DoM, YAW_PID_KD_DoM, ALPHA_DoM, ALPHA_DERIVATIVE_DoM, INTEGRAL_LIMIT_DoM, SCALE_FACTOR_DoM);

  //PID_DoM_Init(&pitch_pid_angle_DoM, PITCH_PID_KP_RATE_DoM, PITCH_PID_KI_RATE_DoM, PITCH_PID_KD_RATE_DoM, ALPHA_RATE_DoM, ALPHA_DERIVATIVE_RATE_DoM, INTEGRAL_LIMIT_RATE_DoM, SCALE_FACTOR_RATE_DoM);
  //PID_DoM_Init(&roll_pid_angle_DoM, ROLL_PID_KP_RATE_DoM, ROLL_PID_KI_RATE_DoM, ROLL_PID_KD_RATE_DoM, ALPHA_RATE_DoM, ALPHA_DERIVATIVE_RATE_DoM, INTEGRAL_LIMIT_RATE_DoM, SCALE_FACTOR_RATE_DoM);

  PID_DoM_Init(&pitch_pid_mtf_DoM, PITCH_PID_KP_MTF_DoM, PITCH_PID_KI_MTF_DoM, PITCH_PID_KD_MTF_DoM, ALPHA_MTF_DoM, ALPHA_DERIVATIVE_MTF_DoM, INTEGRAL_LIMIT_MTF_DoM, SCALE_FACTOR_MTF_DoM);
  PID_DoM_Init(&roll_pid_mtf_DoM, ROLL_PID_KP_MTF_DoM, ROLL_PID_KI_MTF_DoM, ROLL_PID_KD_MTF_DoM, ALPHA_MTF_DoM, ALPHA_DERIVATIVE_MTF_DoM, INTEGRAL_LIMIT_MTF_DoM, SCALE_FACTOR_MTF_DoM);

  low_pass_filter_init(&lpf_mtf_y, 0.9f);
  low_pass_filter_init(&lpf_mtf_x, 0.9f);

  // Инициализация внутреннего контура (RATE)  
  PID_DoM_Init(&pitch_pid_rate_DoM, PITCH_PID_KP_RATE_DoM, PITCH_PID_KI_RATE_DoM, PITCH_PID_KD_RATE_DoM, ALPHA_RATE_DoM, ALPHA_DERIVATIVE_RATE_DoM, INTEGRAL_LIMIT_RATE_DoM, SCALE_FACTOR_RATE_DoM);
  PID_DoM_Init(&roll_pid_rate_DoM, ROLL_PID_KP_RATE_DoM, ROLL_PID_KI_RATE_DoM, ROLL_PID_KD_RATE_DoM, ALPHA_RATE_DoM, ALPHA_DERIVATIVE_RATE_DoM, INTEGRAL_LIMIT_RATE_DoM, SCALE_FACTOR_RATE_DoM);
  PID_DoM_Init(&yaw_pid_rate_DoM, YAW_PID_KP_RATE_DoM, YAW_PID_KI_RATE_DoM, YAW_PID_KD_RATE_DoM, ALPHA_RATE_DoM, ALPHA_DERIVATIVE_RATE_DoM, INTEGRAL_LIMIT_RATE_DoM, SCALE_FACTOR_RATE_DoM);

  // Инициализация внешнего контура (ANGLE) 
  PID_DoM_Init(&pitch_pid_angle_DoM, PITCH_PID_KP_DoM, PITCH_PID_KI_DoM, PITCH_PID_KD_DoM, ALPHA_DoM, ALPHA_DERIVATIVE_DoM, INTEGRAL_LIMIT_DoM, SCALE_FACTOR_DoM);
  PID_DoM_Init(&roll_pid_angle_DoM, ROLL_PID_KP_DoM, ROLL_PID_KI_DoM, ROLL_PID_KD_DoM, ALPHA_DoM, ALPHA_DERIVATIVE_DoM, INTEGRAL_LIMIT_DoM, SCALE_FACTOR_DoM);

  PID_Init(&altitude_pid, ALTITUDE_PID_KP, ALTITUDE_PID_KI, ALTITUDE_PID_KD,ALTITUDE_ALPHA, ALTITUDE_ALPHA_DERIVATIVE, ALTITUDE_INTEGRAL_LIMIT, ALTITUDE_SCALE_FACTOR);
  
  bias();

  HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();

  last_time = HAL_GetTick();

  Motors_DMA_Init(); // инициализация DMA для моторов

  optical_flow_results_init(&optical_flow_results);
  memset(&gyro_integration, 0, sizeof(gyro_integration));

  HAL_TIM_Base_Start_IT(&htim11);

  //CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  //DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	MAV_Process();
	MAV_Check_Connection(&huart1);
	///////////////для кастомного протокола remote_control/////////////////////////////////
	/*
    check_connection_loss();
    if (connection_lost == true) {
    	connection_lost_flag = 1;
    }
	if (data_ready_flag == 1) {
		receive_and_parse_data(DATA_SIZE); // получение сигналов управления
		data_ready_flag = 0;
	}
	*/
	////////////////////////////////////////////////////////////////////////////////////////
	if (count_calculate_frequency_flag == 1) {
		update_coeff();//Обновление коэффициентов для режекторного фильтра 1 раз в секунду
	}
	///////////////для кастомного протокола remote_control/////////////////////////////////
	/*
    if (connection_lost_flag == 1) {
        HAL_UART_AbortReceive(&huart1);  // Принудительно остановить DMA
    	MX_USART1_UART_Init();
    	HAL_UART_Receive_DMA(&huart1, buffer_message, DATA_SIZE);
    	receive_and_parse_data(DATA_SIZE);
    	connection_lost_flag = 0;
    }
    */
	/////////////////////////////////////////////////////////////////////////////////////////
	  //snprintf(buf, sizeof(buf),"freq %f\n",freq);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"distance %lu,distance_strength %d,distance_precision %d,distance_status %d,flow_velocity_x %d,flow_velocity_y %d,flow_quality %d,flow_status %d\n",
	  //distance,distance_strength,distance_precision,distance_status,flow_velocity_x,flow_velocity_y,flow_quality,flow_status);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

 //snprintf(buf, sizeof(buf),"button_2 %d,button %d,potentiometer_value %d,joystick_x %d,joystick_y %d,right_left %d,pitch %f,roll %f,distance %lu\n",
 //button_2,button,potentiometer_value,joystick_x,joystick_y,right_left,pitch,roll,distance);
 //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    //snprintf(buf, sizeof(buf),"current_time %d,last_received_time %d,potentiometer_value %d\n",current_time,last_received_time,potentiometer_value);
    //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"distance_metr %f\n",distance_metr);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"throttle_altitude_correction %f\n",throttle_altitude_correction);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
	  
	  //snprintf(buf, sizeof(buf),"gyro_x %f,gyro_y %f,mtf_x %f,mtf_y %f\n",gyro_x,gyro_y,mtf_x,mtf_y);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

 //snprintf(buf, sizeof(buf),"vibration_frequency_x %f,vibration_frequency_y %f,vibration_frequency_z %f\n",vibration_frequency_x,vibration_frequency_y,vibration_frequency_z);
 //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"freq %f\n",freq);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"final_throttle %f\n",final_throttle);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"target_velocity_pitch %f,target_velocity_roll %f,error_pitch_rate %f,error_roll_rate %f\n",
	  //target_velocity_pitch,target_velocity_roll,error_pitch_rate,error_roll_rate);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"flow_velocity_x %d,flow_velocity_y %d\n",flow_velocity_x,flow_velocity_y);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"flow_velocity_y %d,speed_cm_s_y %f\n",flow_velocity_y,optical_flow_results.speed_cm_s_y);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"speed_cm_s_y %f,speed_cm_s_x %f\n",optical_flow_results.speed_cm_s_y,optical_flow_results.speed_cm_s_x);
  	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"error_pitch_rate %f,error_roll_rate %f\n",error_pitch_rate,error_roll_rate);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"error_pitch_mtf %f,error_roll_mtf %f\n",error_pitch_mtf,error_roll_mtf);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"V_linear_y_gyro %f,V_linear_x_gyro %f\n",V_linear_y_gyro,V_linear_x_gyro);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"V_linear_y_gyro %f,speed_cm_s_y %f\n",V_linear_y_gyro,speed_cm_s_y);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

  	  //snprintf(buf, sizeof(buf),"speed_cm_s_y %f,speed_cm_s_x %f\n",optical_flow_results.speed_cm_s_y,optical_flow_results.speed_cm_s_x);
  	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"V_linear_x_gyro %f,V_linear_x_mtf %f\n",V_linear_x_gyro,V_linear_x_mtf);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"speed_cm_s_y %f,speed_cm_s_x %f\n",speed_cm_s_y,speed_cm_s_x);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"speed_cm_s_y %f,speed_cm_s_x %f\n",optical_flow_results.speed_cm_s_y,optical_flow_results.speed_cm_s_x);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"actual_velocity_roll %f,speed_cm_s_x %f\n",actual_velocity_roll,speed_cm_s_x);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"actual_velocity_pitch %f,error_pitch_mtf %f\n",actual_velocity_pitch,error_pitch_mtf);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

//snprintf(buf, sizeof(buf),"target_velocity_pitch %f,target_velocity_roll %f,error_pitch_mtf %f,error_roll_mtf %f\n",
//target_velocity_pitch,target_velocity_roll,error_pitch_mtf,error_roll_mtf);
//HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"total_power_1 %d,total_power_2 %d,total_power_3 %d,total_power_4 %d\n",total_power_1,total_power_2,total_power_3,total_power_4);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"error_pitch_mtf %f,error_roll_mtf %f\n",error_pitch_mtf,error_roll_mtf);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"button %d,potentiometer_value %d,joystick_x %d,joystick_y %d,right_left %d,pitch %f,roll %f,yaw %f\n",
	  //button,potentiometer_value,joystick_x,joystick_y,right_left,pitch,roll,yaw);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

  //snprintf(buf, sizeof(buf),"button_2 %d,button %d,potentiometer_value %d,joystick_x %d,joystick_y %d,right_left %d,pitch %f,roll %f,yaw %f\n",
  //button_2,button,potentiometer_value,joystick_x,joystick_y,right_left,pitch,roll,yaw);
  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"filter_Gx %f,filter_Gy %f,filter_Gz %f\n",filtered_Gx,filtered_Gy,filtered_Gz);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"filter_Gx_final %f,filter_Gy_final %f,filter_Gz_final %f\n",filter_Gx_final,filter_Gy_final,filter_Gz_final);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"target_velocity_pitch %f,target_velocity_roll %f,target_velocity_yaw %f\n",target_velocity_pitch,target_velocity_roll,target_velocity_yaw);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"Ax_temp %f,filter_Ax_temp %f\n",Gx_temp[0],filter_Gx_temp[0]);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"Ax %f,filter_Ax %f\n",Ax[0],filter_Ax[0]);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"throttle_mshot %d\n",throttle_mshot);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"Gx %f,filter_Gx %f\n",Gx[0],filtered_Gx);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"Gx %f,filter_Gx_final %f\n",Gx[0],filter_Gx_final);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"filter_Gx_final %f,filter_Gx %f\n",filter_Gx_final,filtered_Gx);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"Gx %f,filter_Gx %f\n",Gx[0],filtered_Gx_D);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"error_pitch_rate %f,error_roll_rate %f,error_yaw_rate %f\n",error_pitch_rate,error_roll_rate,error_yaw_rate);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"actual_velocity_pitch %f,actual_velocity_roll %f,actual_velocity_yaw %f,error_pitch_rate %f,error_roll_rate %f,error_yaw_rate %f\n",
	  //actual_velocity_pitch,actual_velocity_roll,actual_velocity_yaw,error_pitch_rate,error_roll_rate,error_yaw_rate);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"actual_velocity_pitch %f,actual_velocity_roll %f,optical_flow_results.speed_cm_s_y %f,optical_flow_results.speed_cm_s_x %f\n",
	  //actual_velocity_pitch,actual_velocity_roll,optical_flow_results.speed_cm_s_y,optical_flow_results.speed_cm_s_x);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"error_pitch_rate %f,error_roll_rate %f,error_yaw_rate %f\n",error_pitch_rate,error_roll_rate,error_yaw_rate);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"button %d,forse_pitch_rate %d,forse_roll_rate %d,forse_yaw_rate %d\n",button,forse_pitch_rate,forse_roll_rate,forse_yaw_rate);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //nprintf(buf, sizeof(buf),"button %d,potentiometer_value %d,filtered_power_1 %d,filtered_power_2 %d,filtered_power_3 %d,filtered_power_4 %d\n",
	  //button,potentiometer_value,filtered_power_1,filtered_power_2,filtered_power_3,filtered_power_4);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"target_velocity_pitch %f,target_velocity_roll %f,target_velocity_yaw %f\n",target_velocity_pitch,target_velocity_roll,target_velocity_yaw);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	  //snprintf(buf, sizeof(buf),"target_velocity_yaw %f\n",target_velocity_yaw);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);


	  //snprintf(buf, sizeof(buf),"flow_velocity_x %d,flow_velocity_y %d\n",flow_velocity_x,flow_velocity_y);
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

/////////////////////////////////////////////////////////////////////////// вариан запуска из прерывания i2c dma /////////////////////////////////////////////////////////////////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
   if (htim == &htim11) {
       // 1. Проверяем состояние HAL
       if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
           // Сбрасываем счетчик таймаута, так как шина жива
           i2c_timeout_counter = 0;
           // Пытаемся запустить чтение
           if (I2C_Start_Read_gyro() != HAL_OK) {
           }
       }
       else {
           // 2. Шина ЗАНЯТА (BUSY)
           i2c_timeout_counter++;
           // Если шина занята более 5 циклов (5 мс) - это зависание.
           // При 400кГц транзакция длится < 1мс.
           if (i2c_timeout_counter > 5) {
               // Принудительный сброс I2C
               HAL_I2C_Init(&hi2c1);
               i2c_timeout_counter = 0;
           }
       }
   }
}
// Колбэк завершения приема DMA
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
   if (hi2c->Instance == hi2c1.Instance) {
       if (current_device == 0) {
           // 1. Данные гироскопа получены
           BMX055_Process_Gyro_Raw(&BMX055, dma_gyro_buffer);
           data_ready_gyro = 1;
           // 2. Запускаем акселерометр
           if (I2C_Start_Read_accel() != HAL_OK) {
               // Если запуск не удался, цепочка разорвана.
           }
       }
       else if (current_device == 1) {
           // 3. Данные акселерометра получены
           BMX055_Process_Accel_Raw(&BMX055, dma_accel_buffer);
           data_ready_accel = 1;
           // 4. Запускаем расчеты
           run_control_loop();
       }
   }
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
   if (hi2c->Instance == hi2c1.Instance) {
       current_device = 0;
   }
}
///////////////для кастомного протокола remote_control/////////////////////////////////
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart1) {
    	data_ready_flag = 1;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart == &huart1) {
    	connection_lost_flag = 1;
    }
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
#ifdef USE_FULL_ASSERT
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
