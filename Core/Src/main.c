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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control_mavlink.h"
#include "blackbox.h"
#include "config_param.h"
#include "globals.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "calculate_notch_coeffs.h"
#include "vibration_analysis.h"
#include "optical_flow_compensation.h"
#include "icm42688.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int constrain(int value, int min_val, int max_val);
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

// Экспоненциальная кривая для сигнала пульта (из remote_control.c)
static float expo_curve(float input, float expo_factor) {
    return input * (1 - expo_factor) + input * input * input * expo_factor;
}

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
    	     calculateNotchFilterCoeffs(1000, vibration_frequency_x, 25, Coeffs_notch_x, 0.9f);
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
	     calculateNotchFilterCoeffs(1000, vibration_frequency_y, 25, Coeffs_notch_y, 0.9f);
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
    	     calculateNotchFilterCoeffs(1000, vibration_frequency_z, 25, Coeffs_notch_z, 0.9f);
    	     previous_freq_z = vibration_frequency_z;
    	     memset(filterState_Gz_notch, 0, sizeof(filterState_Gz_notch));
    	     memset(filterState_Az_notch, 0, sizeof(filterState_Az_notch));
    	     arm_biquad_cascade_df2T_init_f32(&imu_Gz_notch, NUM_STAGES_GYRO_NOTCH, Coeffs_notch_z, filterState_Gz_notch);
    	     arm_biquad_cascade_df2T_init_f32(&imu_Az_notch, NUM_STAGES_ACCEL_NOTCH, Coeffs_notch_z, filterState_Az_notch);
	}
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
void get_angle_mahony(void){

	  float filterAx = filtered_Ax*9.81;
	  float filterAy = filtered_Ay*9.81;
	  float filterAz = filtered_Az*9.81;
	  float filterGx = filtered_Gx*DEG_TO_RAD;
	  float filterGy = filtered_Gy*DEG_TO_RAD;
	  float filterGz = filtered_Gz*DEG_TO_RAD;

	  MahonyAHRSupdateIMU(filterAx, filterAy, filterAz, filterGx, filterGy, filterGz, CONTROL_LOOP_DT);

	  Quat_actual[0] = (*(getQ()));
	  Quat_actual[1] = (*(getQ()+1));
	  Quat_actual[2] = (*(getQ()+2));
	  Quat_actual[3] = (*(getQ()+3));

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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////// фильтрованные данные///////////////////////////////////////////////////////////////
void get_filter_data_accel() {

	Ax[0] = imu_ax;
	Ax[0] = Ax[0] - offset_Ax;

	Ax_for_fft[0] = Ax[0];
	arm_biquad_cascade_df2T_f32 (&hpf_Ax, Ax_for_fft, filter_hpf_Ax_for_fft, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Ax, filter_hpf_Ax_for_fft, filter_lpf_Ax_for_fft, BLOCK_SIZE);
	filter_Ax_for_fft = filter_lpf_Ax_for_fft[0];

	Ax[0] = median_filter(Ax[0], &Accel_x);
    arm_biquad_cascade_df2T_f32 (&imu_Ax_notch, Ax, filter_Ax_notch, BLOCK_SIZE);
    arm_biquad_cascade_df2T_f32 (&imu_Ax_lpf, filter_Ax_notch, filter_Ax_lpf, BLOCK_SIZE);
    filtered_Ax = filter_Ax_lpf[0];
    if (isnan(filtered_Ax) || isinf(filtered_Ax)) {
    	filtered_Ax = prev_filtered_Ax;
    }
    prev_filtered_Ax = filtered_Ax;

    Ay[0] = imu_ay;
	Ay[0] = Ay[0] - offset_Ay;

	Ay_for_fft[0] = Ay[0];
	arm_biquad_cascade_df2T_f32 (&hpf_Ay, Ay_for_fft, filter_hpf_Ay_for_fft, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Ay, filter_hpf_Ay_for_fft, filter_lpf_Ay_for_fft, BLOCK_SIZE);
	filter_Ay_for_fft = filter_lpf_Ay_for_fft[0];

	Ay[0] = median_filter(Ay[0], &Accel_y);
    arm_biquad_cascade_df2T_f32 (&imu_Ay_notch, Ay, filter_Ay_notch, BLOCK_SIZE);
    arm_biquad_cascade_df2T_f32 (&imu_Ay_lpf, filter_Ay_notch, filter_Ay_lpf, BLOCK_SIZE);
    filtered_Ay = filter_Ay_lpf[0];
    if (isnan(filtered_Ay) || isinf(filtered_Ay)) {
    	filtered_Ay = prev_filtered_Ay;
    }
    prev_filtered_Ay = filtered_Ay;

  	Az[0] = imu_az;
	Az[0] = Az[0] - offset_Az;

	Az_for_fft[0] = Az[0];
	arm_biquad_cascade_df2T_f32 (&hpf_Az, Az_for_fft, filter_hpf_Az_for_fft, BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32 (&lpf_Az, filter_hpf_Az_for_fft, filter_lpf_Az_for_fft, BLOCK_SIZE);
	filter_Az_for_fft = filter_lpf_Az_for_fft[0];

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

	Gx[0] = imu_gx;
	Gx[0] = Gx[0] - bias_Gx;
	add_to_x(Gx[0]);

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

    Gy[0] = imu_gy;
	Gy[0] = Gy[0] - bias_Gy;
	add_to_y(Gy[0]);

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

  	Gz[0] = imu_gz;
    Gz[0] = Gz[0] - bias_Gz;
    add_to_z(Gz[0]);

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

// Трекинг новых байт в кольцевом DMA-буфере MTF-02 (USART6)
static uint32_t mtf_dma_counter_prev = 0;
static uint8_t  mtf_dma_first = 1;

void get_mtf_data() {
    // counter уменьшается по мере приёма байт; при переполнении кольца перезагружается
    uint32_t ndtr    = MTF_DMA_BUFFER_SIZE;
    uint32_t counter = __HAL_DMA_GET_COUNTER(huart6.hdmarx);

    if (mtf_dma_first) {
        mtf_dma_counter_prev = counter;
        mtf_dma_first = 0;
        return;   // пропускаем первый (возможно неполный) кадр
    }

    // Сколько новых байт пришло с прошлого вызова
    uint32_t new_bytes = (mtf_dma_counter_prev - counter + ndtr) % ndtr;
    // Позиция, куда запишется следующий байт
    uint32_t write_pos = (ndtr - counter) % ndtr;

    for (uint32_t i = 0; i < new_bytes; i++) {
        uint32_t idx = (write_pos - new_bytes + i + ndtr) % ndtr;
        micolink_decode(buffer_message_mtf02[idx],
                        &distance, &distance_strength, &distance_precision, &distance_status,
                        &flow_velocity_x, &flow_velocity_y, &flow_quality, &flow_status,
                        &flow_frame_received);
    }
    mtf_dma_counter_prev = counter;
}
void IMU_Init(void) {
    // Инициализация ICM-42688-P (SPI2)
    bool imu_ok = ICM42688_Init(&hspi2);

    // Обнуление калибровочных значений (bias вычислит bias() при старте)
    bias_Gx = 0.0f; bias_Gy = 0.0f; bias_Gz = 0.0f;
    offset_Ax = 0.0f; offset_Ay = 0.0f; offset_Az = 0.0f;

    // Если датчик не отвечает — мигаем светодиодом (индикация ошибки)
    if (!imu_ok) {
        while (1) {
            HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
            HAL_Delay(200);
        }
    }

    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int cmp_float(const void *a, const void *b) {
    float fa = *(const float*)a, fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

void bias() {
    static float gx_buf[BIAS_OFSET], gy_buf[BIAS_OFSET], gz_buf[BIAS_OFSET];
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    int valid_samples = 0;

    for (int i = 0; i < BIAS_OFSET; i++) {
        // Читаем свежие данные IMU перед каждым сэмплом
        ICM42688_ReadAll(&hspi2);

        // Гироскоп из глобальных переменных IMU (медианный фильтр убирает одиночные выбросы)
        gx_buf[i] = median_filter(imu_gx, &Gyro_x_bias);
        gy_buf[i] = median_filter(imu_gy, &Gyro_y_bias);
        gz_buf[i] = median_filter(imu_gz, &Gyro_z_bias);

        // Акселерометр из глобальных переменных IMU
        float ax = median_filter(imu_ax, &Accel_x_ofset);
        float ay = median_filter(imu_ay, &Accel_y_ofset);
        float az = median_filter(imu_az, &Accel_z_ofset);

        float mag = sqrtf(ax*ax + ay*ay + az*az);
        if (mag > 0.95f && mag < 1.05f) {
            sum_ax += ax;
            sum_ay += ay;
            sum_az += az;
            valid_samples++;
        }
        HAL_Delay(1);
    }

    // Медиана гиро — устойчива к выбросам (движение дрона, одиночные скачки),
    // в отличие от среднего арифметического.
    qsort(gx_buf, BIAS_OFSET, sizeof(float), cmp_float);
    qsort(gy_buf, BIAS_OFSET, sizeof(float), cmp_float);
    qsort(gz_buf, BIAS_OFSET, sizeof(float), cmp_float);
    bias_Gx = gx_buf[BIAS_OFSET/2];
    bias_Gy = gy_buf[BIAS_OFSET/2];
    bias_Gz = gz_buf[BIAS_OFSET/2];

    // Усреднение акселерометра — только если достаточно валидных сэмплов
    if (valid_samples > BIAS_OFSET / 2) {
        // Калибруем только масштаб гравитации по Z. X/Y НЕ калибруем: наклон
        // стартовой площадки не должен попадать в offset, иначе дрон держит
        // наклон поверхности как горизонталь и его сносит. Дрон увидит реальный
        // наклон и angle-контур выровняет его в полёте.
        offset_Ax = 0.0f;
        offset_Ay = 0.0f;
        offset_Az = (sum_az / valid_samples) - 1.0f;  // вычитаем 1g
    } else {
        // Слишком много отбракованных сэмплов — не калибруем акселерометр
        offset_Ax = 0.0f;
        offset_Ay = 0.0f;
        offset_Az = 0.0f;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// инициализация imu удалена — используется IMU_Init() напрямую

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void run_control_loop(){
	static uint8_t prev_active_mode = FLIGHT_MODE_ACRO;

	count_calculate_frequency++;
	if (count_calculate_frequency >= FFT_LEN) {
		count_calculate_frequency = 0;
		count_calculate_frequency_flag = 1;
	}
    // Кнопка Circle (button_mode): 0=ACRO, 1=ANGLE — базовый ручной режим
    if (button_mode == 1) {
        flight_mode = FLIGHT_MODE_ANGLE;
    } else {
        flight_mode = FLIGHT_MODE_ACRO;
    }

    // Приоритет режимов (каждая кнопка — тумблер пульта, читаем состояние напрямую):
    //   Triangle (button_2) — MTF (горизонталь по оптическому потоку, газ прямой)
    //   X (button_alt_hold) — ALT_HOLD (только внутри MTF): захват высоты, газ/стики игнорируются
    //   иначе — ACRO/ANGLE по Circle
    if (button_2 == 1 && distance <= MTF_MAX_ALTITUDE_MM) {
        // MTF включён. Если ещё и X — удержание высоты поверх MTF.
        if (button_alt_hold == 1) {
            active_mode = FLIGHT_MODE_ALT_HOLD;
        } else {
            active_mode = FLIGHT_MODE_MTF;
        }
    } else {
        active_mode = flight_mode;   // ACRO или ANGLE по Circle
    }
	get_filter_data_accel(); // получение отфильтрованных данных акселерометра
	get_filter_data_gyro(); // получение отфильтрованных данных гироскопа
	get_mtf_data();   // Обработка буфера c данными датчика mtf
    gyro_integration_update(&gyro_integration, filtered_Gx, filtered_Gy); // обновление гпроскопа - 1000 Гц
    uint8_t flow_frame_now = flow_frame_received; // снять до process_optical_flow_data (там сбрасывается)
    process_optical_flow_data(&optical_flow_results,&gyro_integration,&lpf_mtf_x, &lpf_mtf_y,distance,flow_velocity_x, flow_velocity_y, flow_quality, pitch, roll); // обработка оптического потока 50 Гц
    get_angle_mahony();  // получение кватернионов и углов

    // --- Взлёт/посадка: детект перехода высоты через порог валидности MTF ---
    static uint8_t prev_grounded = 1;     // стартуем «на земле»
    static uint16_t grounded_cycles = 0;  // подряд циклов «на земле» (дебаунс)

    if (distance < MTF_MIN_VALID_ALTITUDE_MM) {
        if (grounded_cycles < 0xFFFF) grounded_cycles++;
    } else {
        grounded_cycles = 0;
    }
    // «На земле» считаем только после устойчивых MTF_GROUND_DEBOUNCE_CYCLES
    // циклов (1 цикл = 1 мс). Иначе кратковременный провал LiDAR до 0 даст
    // ложный «взлёт» и сброс EKF в полёте.
    uint8_t grounded = (grounded_cycles >= MTF_GROUND_DEBOUNCE_CYCLES);

    // Взлёт (был на земле -> поднялся выше порога): обнуляем EKF (скорость/позицию),
    // чтобы дрейф, накопленный на земле, не дал ложную скорость при взлёте.
    if (prev_grounded && !grounded) {
        ekf3_reset(&ekf3);
        // На земле angle-контур копил интеграл против наклона площадки (дрон стоял
        // на ножках и не мог выровняться). При отрыве этот момент «выстреливает»
        // наклоном и сносом — обнуляем интегралы, выравнивание произойдёт в воздухе.
        PID_DoM_Reset(&pitch_pid_angle_DoM);
        PID_DoM_Reset(&roll_pid_angle_DoM);
        PID_DoM_Reset(&pitch_pid_rate_DoM);
        PID_DoM_Reset(&roll_pid_rate_DoM);
        PID_DoM_Reset(&yaw_pid_rate_DoM);
    }
    prev_grounded = grounded;

    // EKF3 считается ТОЛЬКО в полёте (для A/B в blackbox и для PID при USE_EKF3).
    // На земле predict не вызываем: random-walk скорости копит дрейф позиции,
    // который потоком уже не обнулить (поток мерит скорость, а не позицию).
    if (!grounded) {
        // Компенсация g·sin(наклон) ДО EKF: акселерометр в body-фрейме видит
        // проекцию гравитации на горизонтальные оси (g_body_x = -sin(pitch),
        // g_body_y = -sin(roll) в соглашении кватерниона Mahony). Вычитаем её,
        // чтобы EKF не интегрировал наклон в ложную горизонтальную скорость
        // (g·sin-протечка: Mahony наклоняется от вибраций → g протекает в EKF).
        float ax_comp = filtered_Ax + sinf(pitch * DEG_TO_RAD);
        float ay_comp = filtered_Ay + sinf(roll * DEG_TO_RAD);
        ekf3_predict(&ekf3,
                     ax_comp * 9.81f, ay_comp * 9.81f, filtered_Az * 9.81f,
                     Quat_actual, CONTROL_LOOP_DT);
    }

    // Коррекция по потоку — ровно один раз на кадр (~50 Гц), тоже только в полёте.
    if (flow_frame_now && !grounded) {
        float h_m = ((float)distance / 1000.0f)
                    * cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD);
        // Средняя угловая скорость за окно измерения потока [t-40мс, t-20мс],
        // выровненная с моментом кадра MTF-02. Мгновенный гироскоп «сейчас»
        // рассинхронизирован с кадром на ~20 мс: в переходных процессах это даёт
        // ложную остаточную скорость от вращения, и дрон перелетает/описывает круг.
        float w_win_x = 0.0f, w_win_y = 0.0f;
        gyro_get_window_sum(&gyro_integration, OPTICAL_FLOW_UPDATE_PERIOD_MS,
                            OF_SENSOR_LATENCY_MS, &w_win_x, &w_win_y);
        float wdt = OPTICAL_FLOW_UPDATE_PERIOD_MS * 0.001f;   // время окна, с
        float gx_avg = (-w_win_x / wdt) * DEG_TO_RAD;         // в буфере X хранится с инверсией знака
        float gy_avg = ( w_win_y / wdt) * DEG_TO_RAD;
        ekf3_update_flow(&ekf3, flow_velocity_x, flow_velocity_y, h_m,
                         Quat_actual,
                         gx_avg, gy_avg, filtered_Gz * DEG_TO_RAD,
                         flow_quality);
    }

    throttle_mshot = ((potentiometer_value / 1000.0) * 2000.0) + 500;
	max_pid_correction_mshot = MAX_CORRECTION;  //Ограничение пид-коррекции
	//текущая угловая скорость
	actual_velocity_pitch = (-filtered_Gy);
	actual_velocity_roll = filtered_Gx;
	actual_velocity_yaw = filtered_Gz;
	//текущая угловая скорость для д компоненты пид
	actual_velocity_pitch_D = (-filtered_Gy_D);
	actual_velocity_roll_D = filtered_Gx_D;
	actual_velocity_yaw_D = filtered_Gz_D;
    // Режим уже определён выше: ACRO/ANGLE по Circle, MTF по тумблеру Triangle

    // Вошли в MTF: сбрасываем EKF (скорость стартует с нуля).
    if (active_mode == FLIGHT_MODE_MTF && prev_active_mode != FLIGHT_MODE_MTF) {
        ekf3_reset(&ekf3);
    }
    // Вошли в ALT_HOLD: захват текущей высоты, базовой тяги и фиксация точки
    if (active_mode == FLIGHT_MODE_ALT_HOLD && prev_active_mode != FLIGHT_MODE_ALT_HOLD) {
        // Взлёт с земли vs вход в висе.
        if (distance < GROUND_DISTANCE_MM) {
            // С земли: база тяги = взлётная, цель = высота взлёта. target_altitude_mm
            // остаётся текущей (≈0), а плавный рост к цели обеспечит climb-rate limit.
            hover_throttle_base = ALT_HOLD_TAKEOFF_THROTTLE;
            alt_hold_capture_alt_mm = ALT_HOLD_TAKEOFF_ALTITUDE;
        } else {
            // В висе: база = текущая тяга, цель = текущая высота (нет скачка).
            hover_throttle_base = (float)throttle_mshot;
            alt_hold_capture_alt_mm = (float)distance;
        }
        target_altitude_mm = (float)distance;
        smooth_altitude_mm = (float)distance;
        // Сброс PID высоты на входе в ALT_HOLD; засекаем текущую высоту (без выброса D)
        PID_DoM_Reset(&altitude_pid);
        altitude_pid.previous_measurement = smooth_altitude_mm;
        altitude_control = 0.0f;
        // Захват крестовины: дальше целевая высота смещается от pot (D-pad вверх/вниз)
        alt_hold_capture_pot = (float)potentiometer_value;
        yaw_capture = yaw;   // запоминаем курс для yaw-удержания
        ekf3_reset(&ekf3);
    }
    prev_active_mode = active_mode;

	// Вычисляем target_velocity на основе активного режима
    switch(active_mode) {
	    case FLIGHT_MODE_MTF: {
	        // 1. Читаем стики как целевую скорость (например, макс 50 см/с)
        float stick_norm_x = (fabsf(joystick_y) < STICK_DEADZONE) ? 0.0f : (joystick_y / 25.0f);
        float stick_norm_y = (fabsf(joystick_x) < STICK_DEADZONE) ? 0.0f : (joystick_x / 25.0f);
        float stick_speed_x = stick_norm_x * 50.0f;   // ±50 см/с
        float stick_speed_y = stick_norm_y * 50.0f;

	        // target_angle_pitch/roll_mtf объявлены в globals.c

#if USE_EKF3
        // Вариант A: скоростное удержание (без возврата в точку).
        // Стик в центре → целевая скорость 0 → контур гасит скорость, дрон висит
        // на месте. Стик отклонён → целевая скорость = стик. Позиционный ПИД
        // отключён: позиция ненаблюдаема (поток меряет только скорость, GPS нет),
        // и дрейф скорости превращался в ложную цель, насыщавшую контур (15°) и
        // реально разгонявшую дрон.
        float meas_speed_x = ekf3.vel_body_cms[0];
        float meas_speed_y = ekf3.vel_body_cms[1];
        float target_speed_x = stick_speed_x;   // стик — уже связанная СК (body)
        float target_speed_y = stick_speed_y;

        // Ошибки скорости: продольная (X) → наклон pitch, боковая (Y) → наклон roll.
        // (Было перепутано: pitch управлялся боковой скоростью, roll — продольной,
        //  из-за чего контур наклонял дрон в неверной плоскости и не гасил скорость.)
        error_pitch_mtf = meas_speed_x - target_speed_x;
        error_roll_mtf  = target_speed_y - meas_speed_y;

            // ПИД скорости вычисляет ТРЕБУЕМЫЙ УГОЛ НАКЛОНА (в градусах) — 1 кГц
            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_x, CONTROL_LOOP_DT);
            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_y, CONTROL_LOOP_DT);

            // D-член от акселерометра (горизонтальное ускорение) — демпфирование (оси: roll←Ay, pitch←Ax).
            target_angle_roll_mtf  += MTF_D_KD * (filtered_Ay * 9.81f - MTF_D_ACC_SIGN_Y * 9.81f * sinf(roll  * DEG_TO_RAD)) * 100.0f;
            target_angle_pitch_mtf += MTF_D_KD * (filtered_Ax * 9.81f - MTF_D_ACC_SIGN_X * 9.81f * sinf(pitch * DEG_TO_RAD)) * 100.0f;
#else
        // Старая схема: горизонталь привязана к кадру потока (~50 Гц), скоростное удержание.
        if (optical_flow_results.new_optical_data_available) {
            float meas_speed_x = optical_flow_results.speed_cm_s_x;
            float meas_speed_y = optical_flow_results.speed_cm_s_y;

            float target_speed_x = stick_speed_x;
            float target_speed_y = stick_speed_y;

        error_pitch_mtf = meas_speed_x - target_speed_x;
        error_roll_mtf  = target_speed_y - meas_speed_y;

	            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_x, 0.020f);
	            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_y, 0.020f);

            optical_flow_results.new_optical_data_available = false;
        }
#endif

        // Ограничиваем максимальный наклон для безопасности 
        target_angle_pitch_mtf = constrain_float(target_angle_pitch_mtf, -MAX_TILT_MTF, MAX_TILT_MTF);
        target_angle_roll_mtf = constrain_float(target_angle_roll_mtf, -MAX_TILT_MTF, MAX_TILT_MTF);

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

    case FLIGHT_MODE_ALT_HOLD: {
        // Удержание высоты поверх MTF: газ и стики игнорируются.
        // Горизонталь держит точку по оптическому потоку (как MTF),
        // высота держится P-регулятором вокруг захваченной target_altitude_mm.

#if USE_EKF3
        // Вариант A: удержание скорости (без возврата в точку). ALT_HOLD держит
        // высоту, горизонталь — нулевую скорость. Позиция ненаблюдаема без GPS,
        // поэтому позиционный ПИД убран: он превращал дрейф скорости в ложную цель.
        float meas_speed_x = ekf3.vel_body_cms[0];
        float meas_speed_y = ekf3.vel_body_cms[1];
        float target_speed_x = 0.0f;
        float target_speed_y = 0.0f;

        // Ошибки скорости: продольная (X) → pitch, боковая (Y) → roll.
        error_pitch_mtf = meas_speed_x - target_speed_x;
        error_roll_mtf  = target_speed_y - meas_speed_y;

        target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_x, CONTROL_LOOP_DT);
        target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_y, CONTROL_LOOP_DT);

        // D-член от акселерометра (горизонтальное ускорение) — демпфирование (оси: roll←Ay, pitch←Ax).
        target_angle_roll_mtf  += MTF_D_KD * (filtered_Ay * 9.81f - MTF_D_ACC_SIGN_Y * 9.81f * sinf(roll  * DEG_TO_RAD)) * 100.0f;
        target_angle_pitch_mtf += MTF_D_KD * (filtered_Ax * 9.81f - MTF_D_ACC_SIGN_X * 9.81f * sinf(pitch * DEG_TO_RAD)) * 100.0f;
#else
        // Старая схема: горизонталь привязана к кадру потока (~50 Гц), удержание скорости.
        if (optical_flow_results.new_optical_data_available) {
            float meas_speed_x = optical_flow_results.speed_cm_s_x;
            float meas_speed_y = optical_flow_results.speed_cm_s_y;

            error_pitch_mtf = meas_speed_x;
            error_roll_mtf  = -meas_speed_y;

            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_x, 0.020f);
            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_y, 0.020f);

            optical_flow_results.new_optical_data_available = false;
        }
#endif

        // Ограничиваем максимальный наклон для безопасности
        target_angle_pitch_mtf = constrain_float(target_angle_pitch_mtf, -MAX_TILT_MTF, MAX_TILT_MTF);
        target_angle_roll_mtf = constrain_float(target_angle_roll_mtf, -MAX_TILT_MTF, MAX_TILT_MTF);

        // Взлётная фаза: пока высота мала, держим дрон ровно (вертикальный взлёт).
        // На малой высоте поток мусорный → горизонтальный контур даёт ложный наклон и заваливает дрон.
        static uint8_t pos_hold_was_disabled = 0;
        if (smooth_altitude_mm < ALT_HOLD_POS_HOLD_MIN_ALTITUDE) {
            target_angle_pitch_mtf = 0.0f;
            target_angle_roll_mtf = 0.0f;
            pos_hold_was_disabled = 1;
        } else if (pos_hold_was_disabled) {
            // Контур только что включился (переход через порог высоты). За вертикальный
            // взлёт EKF накопил дрейф скорости (bias аксела), и если не сбросить — контур
            // «выстрелит» ложной скоростью и резко наклонит дрон. Сбрасываем скорость/позицию
            // EKF (bias сохраняем) и интеграл скоростного PID.
            ekf3_reset(&ekf3);
            PID_DoM_Reset(&pitch_pid_mtf_DoM);
            PID_DoM_Reset(&roll_pid_mtf_DoM);
            pos_hold_was_disabled = 0;
        }

        // Высота: сглаживание LiDAR + PID высоты (на частоте кадра ~50 Гц)
        if (optical_flow_results.new_optical_data_available) {
            float raw_alt = (float)distance * cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD);
            smooth_altitude_mm = smooth_altitude_mm * 0.7f + raw_alt * 0.3f;
            optical_flow_results.new_optical_data_available = false;

            // PID высоты (DoM): D от производной измерения = демпфирование вертикальной
            // скорости; dt = фактический период кадра потока (~20 мс).
            float alt_err = target_altitude_mm - smooth_altitude_mm;
            altitude_control = PID_DoM_Compute(&altitude_pid, alt_err, smooth_altitude_mm,
                                               OPTICAL_FLOW_UPDATE_PERIOD_MS * 0.001f);
        }

        // Крестовина (pot) плавно смещает целевую высоту с лимитом вертикальной скорости.
        // pot (0..1000) инкрементируется D-pad'ом на пульте: вверх/вниз меняет целевую высоту.
        float desired_alt = alt_hold_capture_alt_mm
                          + ((float)potentiometer_value - alt_hold_capture_pot) * ALT_HOLD_POT_TO_MM;
        if (desired_alt < MTF_MIN_VALID_ALTITUDE_MM) desired_alt = MTF_MIN_VALID_ALTITUDE_MM;
        if (desired_alt > MTF_MAX_ALTITUDE_MM) desired_alt = MTF_MAX_ALTITUDE_MM;
        float dalt = desired_alt - target_altitude_mm;
        float alt_step_max = ALT_HOLD_CLIMB_RATE_MAX * CONTROL_LOOP_DT;
        if (dalt > alt_step_max) dalt = alt_step_max;
        else if (dalt < -alt_step_max) dalt = -alt_step_max;
        target_altitude_mm += dalt;

        // Внутренний контур ANGLE (1000 Гц)
        error_pitch_angle = target_angle_pitch_mtf - pitch;
        error_roll_angle = target_angle_roll_mtf - roll;

        target_velocity_pitch = PID_DoM_Compute(&pitch_pid_angle_DoM, error_pitch_angle, pitch, 0.001f);
        target_velocity_roll = PID_DoM_Compute(&roll_pid_angle_DoM, error_roll_angle, roll, 0.001f);
        // Yaw-удержание: P-регулятор по ошибке курса возвращает дрон к захваченному курсу.
        float yaw_err = yaw_capture - yaw;
        while (yaw_err > 180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;
        target_velocity_yaw = YAW_HOLD_KP * yaw_err;

        target_velocity_pitch = safe_pid_value(target_velocity_pitch, MAX_P);
        target_velocity_roll = safe_pid_value(target_velocity_roll, MAX_P);
        target_velocity_yaw = safe_pid_value(target_velocity_yaw, MAX_P);

        // Вычисляем ошибку для контура RATE
        error_pitch_rate = target_velocity_pitch - actual_velocity_pitch;
        error_roll_rate = target_velocity_roll - actual_velocity_roll;
        error_yaw_rate = target_velocity_yaw - actual_velocity_yaw;

        error_pitch_rate_D = target_velocity_pitch - actual_velocity_pitch_D;
        error_roll_rate_D = target_velocity_roll - actual_velocity_roll_D;
        error_yaw_rate_D = target_velocity_yaw - actual_velocity_yaw_D;
        break;
    }

    case FLIGHT_MODE_ANGLE: {

			float target_angle_pitch_rc = (joystick_x / 25.0f) * 45.0f;
        	float target_angle_roll_rc  = (joystick_y / 25.0f) * 45.0f;

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
	        break;
	    }
	}

    // Итоговая тяга
    final_throttle = throttle_mshot;   // прямой газ (ACRO/ANGLE/MTF)
    if (active_mode == FLIGHT_MODE_ALT_HOLD) {
        // Газ игнорируется — тягу задаёт PID высоты (P+I+D) вокруг
        // захваченной базы тяги (мощность в момент включения удержания).
        final_throttle = hover_throttle_base + altitude_control;
    }
    final_throttle = constrain(final_throttle, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

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

	if(button == 1){

       // Перед вызовом PID — deadband не реагировать на малые отклонения
       if (fabsf(error_pitch_rate) < 1.0f) error_pitch_rate = 0.0f; // 10.0f
       if (fabsf(error_roll_rate)  < 1.0f) error_roll_rate  = 0.0f; // 10.0f
       if (fabsf(error_yaw_rate)   < 1.0f)  error_yaw_rate   = 0.0f; // 5.0f

	   forse_pitch_rate = PID_DoM_Compute(&pitch_pid_rate_DoM, error_pitch_rate, actual_velocity_pitch_D, 0.001f);
	   forse_roll_rate = PID_DoM_Compute(&roll_pid_rate_DoM, error_roll_rate, actual_velocity_roll_D, 0.001f);
	   forse_yaw_rate = PID_DoM_Compute(&yaw_pid_rate_DoM, error_yaw_rate, actual_velocity_yaw_D, 0.001f);

	   // Проверяем на NaN, Inf и ограничиваем
	   forse_pitch_rate = safe_pid_value(forse_pitch_rate, MAX_PID_RATE);
	   forse_roll_rate = safe_pid_value(forse_roll_rate, MAX_PID_RATE);
	   forse_yaw_rate = safe_pid_value(forse_yaw_rate, MAX_PID_RATE);

	   pid_correction_1 = forse_roll_rate + forse_pitch_rate + forse_yaw_rate;
	   pid_correction_1 = constrain_float(pid_correction_1, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_1 = trim_percent(final_throttle, TRIM_PERCENT_FRONT_LEFT) + pid_correction_1;
	   total_power_1 = constrain(total_power_1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_2 = -forse_roll_rate + forse_pitch_rate - forse_yaw_rate;
	   pid_correction_2 = constrain_float(pid_correction_2, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_2 = trim_percent(final_throttle, TRIM_PERCENT_FRONT_RIGHT) + pid_correction_2;
	   total_power_2 = constrain(total_power_2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_3 = forse_roll_rate - forse_pitch_rate - forse_yaw_rate;
	   pid_correction_3 = constrain_float(pid_correction_3, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_3 = trim_percent(final_throttle, TRIM_PERCENT_REAR_LEFT) + pid_correction_3;
	   total_power_3 = constrain(total_power_3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_4 = -forse_roll_rate - forse_pitch_rate + forse_yaw_rate;
	   pid_correction_4 = constrain_float(pid_correction_4, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_4 = trim_percent(final_throttle, TRIM_PERCENT_REAR_RIGHT) + pid_correction_4;
	   total_power_4 = constrain(total_power_4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

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
	Blackbox_Write();
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart6, buffer_message_mtf02, MTF_DMA_BUFFER_SIZE);
  MAV_Init(&huart1); //инициализация для протокола mavlink

  IMU_Init();// инициализация imu

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



  // PID высоты (DoM): P+I+D, D от измерения (демпфирование вертикальной скорости)
  PID_DoM_Init(&altitude_pid, ALTITUDE_HOLD_KP, ALTITUDE_HOLD_KI, ALTITUDE_HOLD_KD,
               ALTITUDE_HOLD_ALPHA, ALTITUDE_HOLD_ALPHA_D, ALTITUDE_HOLD_INTEGRAL_LIMIT, ALTITUDE_HOLD_SCALE);
  altitude_pid.output_limit = ALTITUDE_HOLD_MAX_OUTPUT;  // анти-виндап выхода PID высоты
  
  // Термопрогрев IMU: читаем данные впустую ~10 секунд,
  // пока гироскоп и акселерометр не стабилизируются после включения
  // (bias гиро плывёт ещё ~10-15 с после подачи питания — по полётным логам)
  for (int i = 0; i < 10000; i++) {
       ICM42688_ReadAll(&hspi2);
       HAL_Delay(1);
  }

  // Калибровка IMU (bias вычисляется из глобальных переменных imu_*)
  bias();
          // Диагностика калибровки
        char dbg[128];
        int len = snprintf(dbg, sizeof(dbg),
            "BIAS: Gx=%.2f Gy=%.2f Gz=%.2f | Ax=%.3f Ay=%.3f Az=%.3f\r\n",
            (double)bias_Gx, (double)bias_Gy, (double)bias_Gz,
            (double)offset_Ax, (double)offset_Ay, (double)offset_Az);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbg, len, HAL_MAX_DELAY);
        
  HAL_Delay(100);

  Motors_DMA_Init(); // инициализация DMA для моторов

  optical_flow_results_init(&optical_flow_results);
  memset(&gyro_integration, 0, sizeof(gyro_integration));
  ekf3_init(&ekf3);

  HAL_TIM_Base_Start_IT(&htim11);

  Blackbox_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	MAV_Process();
	MAV_Check_Connection(&huart1);
	if (count_calculate_frequency_flag == 1) {
		update_coeff();//Обновление коэффициентов режекторного фильтра каждые FFT_LEN сэмплов (~512 мс)
	}

	// Blackbox: по 'd' через USART2 — выгрузка лога в CSV
	uint8_t cmd;
	if (HAL_UART_Receive(&huart2, &cmd, 1, 1) == HAL_OK) {
	    if (cmd == 'd' || cmd == 'D') {
	        Blackbox_Dump();
	    }
	}

    //snprintf(buf, sizeof(buf),"pitch %f,roll %f,yaw %f,filter_Gx %f,filter_Gy %f,filter_Gz %f\n",pitch,roll,yaw,filtered_Gx,filtered_Gy,filtered_Gz);
	//HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    // --- старый вывод (без температуры) закомментирован ---
    
    snprintf(buf, sizeof(buf),
        "t=%lu | m1=%lu m2=%lu m3=%lu m4=%lu thr=%u | stk=%u %d %d %d | mode=%d altH=%u | "
        "roll=%.1f pitch=%.1f yaw=%.1f | "
        "imuA=%.2f %.2f %.2f | imuG=%.2f %.2f %.2f | "
        "fA=%.2f %.2f %.2f | fG=%.2f %.2f %.2f | "
        "dist=%lu str=%u | fvx=%d fvy=%d fq=%u | "
        "EKFv=%.2f %.2f | innov=%.2f %.2f\n",
        (unsigned long)HAL_GetTick(),
        (unsigned long)m1_pulse[0], (unsigned long)m2_pulse[0],
        (unsigned long)m3_pulse[0], (unsigned long)m4_pulse[0],
        (unsigned)throttle_mshot,
        (unsigned)potentiometer_value, joystick_x, joystick_y, right_left,
        button_mode, (unsigned)button_alt_hold,
        (double)roll, (double)pitch, (double)yaw,
        (double)imu_ax, (double)imu_ay, (double)imu_az,
        (double)imu_gx, (double)imu_gy, (double)imu_gz,
        (double)filtered_Ax, (double)filtered_Ay, (double)filtered_Az,
        (double)filtered_Gx, (double)filtered_Gy, (double)filtered_Gz,
        (unsigned long)distance, (unsigned)distance_strength,
        flow_velocity_x, flow_velocity_y, (unsigned)flow_quality,
        ekf3.vel_body_cms[0], ekf3.vel_body_cms[1],
        ekf3.innovation[0] * 100.0f, ekf3.innovation[1] * 100.0f);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    
/*
    // Вывод для сбора зависимости bias(T): добавлено поле температуры кристалла T.
    // Для полиномной компенсации нужны пары «T -> bias гиро» (в покое bias = imuG).
    snprintf(buf, sizeof(buf),
        "t=%lu | m1=%lu m2=%lu m3=%lu m4=%lu thr=%u | stk=%u %d %d %d | mode=%d altH=%u | "
        "roll=%.1f pitch=%.1f yaw=%.1f | "
        "imuA=%.2f %.2f %.2f | imuG=%.2f %.2f %.2f | T=%.2f | vibF=%.0f %.0f %.0f | "
        "fA=%.2f %.2f %.2f | fG=%.2f %.2f %.2f | "
        "dist=%lu str=%u | fvx=%d fvy=%d fq=%u | "
        "EKFv=%.2f %.2f | innov=%.2f %.2f\n",
        (unsigned long)HAL_GetTick(),
        (unsigned long)m1_pulse[0], (unsigned long)m2_pulse[0],
        (unsigned long)m3_pulse[0], (unsigned long)m4_pulse[0],
        (unsigned)throttle_mshot,
        (unsigned)potentiometer_value, joystick_x, joystick_y, right_left,
        button_mode, (unsigned)button_alt_hold,
        (double)roll, (double)pitch, (double)yaw,
        (double)imu_ax, (double)imu_ay, (double)imu_az,
        (double)imu_gx, (double)imu_gy, (double)imu_gz,
        (double)imu_temp_degc,
        (double)vibration_frequency_x, (double)vibration_frequency_y, (double)vibration_frequency_z,
        (double)filtered_Ax, (double)filtered_Ay, (double)filtered_Az,
        (double)filtered_Gx, (double)filtered_Gy, (double)filtered_Gz,
        (unsigned long)distance, (unsigned)distance_strength,
        flow_velocity_x, flow_velocity_y, (unsigned)flow_quality,
        ekf3.vel_body_cms[0], ekf3.vel_body_cms[1],
        ekf3.innovation[0] * 100.0f, ekf3.innovation[1] * 100.0f);
 	HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    */
/*
    // Отладочный вывод значений (раз в 100 мс)
    static uint32_t last_dbg_time = 0;
    if (HAL_GetTick() - last_dbg_time >= 100) {
        last_dbg_time = HAL_GetTick();
        snprintf(buf, sizeof(buf),
                 "btn=%u btn2=%u mode=%d act=%u altH=%u pot=%u joyX=%d joyY=%d yawR=%d pitch=%.1f roll=%.1f dist=%lu\r\n",
                 (unsigned)button, (unsigned)button_2, button_mode, (unsigned)active_mode,
                 (unsigned)button_alt_hold,
                 (unsigned)potentiometer_value, joystick_x, joystick_y, right_left,
                 (double)pitch, (double)roll, (unsigned long)distance);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    }
*/        
   /*
       snprintf(buf, sizeof(buf),
        "btn2=%u btn=%u pot=%u joyX=%d joyY=%d yawR=%d | "
        "pitch=%7.2f roll=%7.2f yaw=%7.2f | "
        "Gx=%7.2f Gy=%7.2f Gz=%7.2f | "
        "Ax=%7.2f Ay=%7.2f Az=%7.2f | "
        "imuGx=%7.2f imuGy=%7.2f imuGz=%7.2f | "
        "imuAx=%7.2f imuAy=%7.2f imuAz=%7.2f | "
        "dist=%lu str=%u fvx=%d fvy=%d fq=%u | "
        "m1=%lu m2=%lu m3=%lu m4=%lu thr=%u\n",
        (unsigned)button_2, (unsigned)button, (unsigned)potentiometer_value,
        joystick_x, joystick_y, right_left,
        (double)pitch, (double)roll, (double)yaw,
        (double)filtered_Gx, (double)filtered_Gy, (double)filtered_Gz,
        (double)filtered_Ax, (double)filtered_Ay, (double)filtered_Az,
        (double)imu_gx, (double)imu_gy, (double)imu_gz,
        (double)imu_ax, (double)imu_ay, (double)imu_az,
        (unsigned long)distance, (unsigned)distance_strength,
        flow_velocity_x, flow_velocity_y, (unsigned)flow_quality,
        (unsigned long)m1_pulse[0], (unsigned long)m2_pulse[0],
        (unsigned long)m3_pulse[0], (unsigned long)m4_pulse[0],
        (unsigned)throttle_mshot);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    */
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
       // Чтение свежих данных IMU (~5 мкс) и запуск цикла управления
       ICM42688_ReadAll(&hspi2);
       run_control_loop();
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
