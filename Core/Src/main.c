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
	add_to_x(filter_Ax_for_fft);

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
	add_to_y(filter_Ay_for_fft);

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

	Gx[0] = imu_gx;
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

    Gy[0] = imu_gy;
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

  	Gz[0] = imu_gz;
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

void bias() {
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    int valid_samples = 0;

    for (int i = 0; i < BIAS_OFSET; i++) {
        // Читаем свежие данные IMU перед каждым сэмплом
        ICM42688_ReadAll(&hspi2);

        // Гироскоп из глобальных переменных IMU
        sum_gx += median_filter(imu_gx, &Gyro_x_bias);
        sum_gy += median_filter(imu_gy, &Gyro_y_bias);
        sum_gz += median_filter(imu_gz, &Gyro_z_bias);

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

    // Усреднение гироскопа — всегда
    bias_Gx = sum_gx / BIAS_OFSET;
    bias_Gy = sum_gy / BIAS_OFSET;
    bias_Gz = sum_gz / BIAS_OFSET;

    // Усреднение акселерометра — только если достаточно валидных сэмплов
    if (valid_samples > BIAS_OFSET / 2) {
        offset_Ax = sum_ax / valid_samples;
        offset_Ay = sum_ay / valid_samples;
        offset_Az = (sum_az / valid_samples) - 1.0f;  // вычитаем 1g

        // Проверка: дрон должен стоять достаточно ровно (X и Y ~ 0)
        // Если offset_Ax или offset_Ay > 0.1g — значит дрон наклонён,
        // и калибровка акселерометра будет неверной.
        if (fabsf(offset_Ax) > 0.25f || fabsf(offset_Ay) > 0.25f) {
            offset_Ax = 0.0f;
            offset_Ay = 0.0f;
            offset_Az = 0.0f;
        }
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

    // EKF3 всегда считается (для A/B сравнения в blackbox), а USE_EKF3
    // ниже выбирает, какой источник скорости идёт в PID.
    ekf3_predict(&ekf3,
                 filtered_Ax * 9.81f, filtered_Ay * 9.81f, filtered_Az * 9.81f,
                 Quat_actual, CONTROL_LOOP_DT);

    // Коррекция по потоку — ровно один раз на кадр (~50 Гц)
    if (flow_frame_now) {
        float h_m = ((float)distance / 1000.0f)
                    * cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD);
        ekf3_update_flow(&ekf3, flow_velocity_x, flow_velocity_y, h_m,
                         Quat_actual,
                         filtered_Gx * DEG_TO_RAD, filtered_Gy * DEG_TO_RAD, filtered_Gz * DEG_TO_RAD,
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

    // Вошли в MTF: захват позиции (только горизонталь)
    if (active_mode == FLIGHT_MODE_MTF && prev_active_mode != FLIGHT_MODE_MTF) {
        target_pos_x = 0.0f; target_pos_y = 0.0f;
        pos_x = 0.0f; pos_y = 0.0f;
        position_hold_active = true;
        PID_Reset(&position_pid_x);
        PID_Reset(&position_pid_y);
        ekf3_reset(&ekf3);
    }
    // Вошли в ALT_HOLD: захват текущей высоты, базовой тяги и фиксация точки
    if (active_mode == FLIGHT_MODE_ALT_HOLD && prev_active_mode != FLIGHT_MODE_ALT_HOLD) {
        target_altitude_mm = (distance > GROUND_DISTANCE_MM) ? (float)distance : 0.0f;
        smooth_altitude_mm = target_altitude_mm;
        // База тяги = фактическая тяга на моторах в момент включения удержания.
        // Она уже соответствует текущей высоте → нет скачка при входе в режим.
        hover_throttle_base = (float)throttle_mshot;
        target_pos_x = pos_x;
        target_pos_y = pos_y;
        position_hold_active = true;
        PID_Reset(&position_pid_x);
        PID_Reset(&position_pid_y);
        ekf3_reset(&ekf3);
    }
    prev_active_mode = active_mode;

    // Сброс удержания вне MTF/ALT_HOLD
    if (active_mode != FLIGHT_MODE_MTF && active_mode != FLIGHT_MODE_ALT_HOLD) {
        position_hold_active = false;
        PID_Reset(&position_pid_x);
        PID_Reset(&position_pid_y);
    }
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
        // EKF3: позиция и скорость предсказываются на 1 кГц (ekf3_predict),
        // коррекция по потоку идёт асинхронно на частоте кадра (~50 Гц).
        // Поэтому позиционный контур работает на 1 кГц и не ждёт кадр датчика.
        pos_x = ekf3.pos_earth_m[0] * 100.0f;
        pos_y = ekf3.pos_earth_m[1] * 100.0f;
        float meas_speed_x = ekf3.vel_body_cms[0];
        float meas_speed_y = ekf3.vel_body_cms[1];

        // Целевая скорость (как DJI/Betaflight-hold):
        //   стик в центре → позиционный ПИД держит точку
        //   стик отклонён → точка смещается за дроном, скорость задаёт стик
        float target_speed_x, target_speed_y;
        bool stick_moving = (fabsf(stick_norm_x) > 0.001f) || (fabsf(stick_norm_y) > 0.001f);

        if (position_hold_active) {
            if (stick_moving) {
                // смещаем точку удержания к текущей позиции, чтобы не тянуло назад
                target_pos_x = pos_x;
                target_pos_y = pos_y;
                PID_Reset(&position_pid_x);
                PID_Reset(&position_pid_y);
                target_speed_x = stick_speed_x;
                target_speed_y = stick_speed_y;
            } else {
                target_speed_x = PID_Compute(&position_pid_x, target_pos_x - pos_x, CONTROL_LOOP_DT);
                target_speed_y = PID_Compute(&position_pid_y, target_pos_y - pos_y, CONTROL_LOOP_DT);
            }
            target_speed_x = constrain_float(target_speed_x, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);
            target_speed_y = constrain_float(target_speed_y, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);
        } else {
            target_speed_x = stick_speed_x;
            target_speed_y = stick_speed_y;
        }

        error_pitch_mtf = target_speed_y - meas_speed_y;
        error_roll_mtf  = target_speed_x - meas_speed_x;

            // ПИД скорости вычисляет ТРЕБУЕМЫЙ УГОЛ НАКЛОНА (в градусах) — 1 кГц
            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_y, CONTROL_LOOP_DT);
            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_x, CONTROL_LOOP_DT);

            // D-член от акселерометра (горизонтальное ускорение, без интеграции) — демпфирование
            target_angle_roll_mtf  -= MTF_D_KD * (filtered_Ax * 9.81f - MTF_D_ACC_SIGN_X * 9.81f * sinf(roll  * DEG_TO_RAD)) * 100.0f;
            target_angle_pitch_mtf -= MTF_D_KD * (filtered_Ay * 9.81f - MTF_D_ACC_SIGN_Y * 9.81f * sinf(pitch * DEG_TO_RAD)) * 100.0f;
#else
        // Старая схема: контур жёстко привязан к кадру потока (~50 Гц)
        if (optical_flow_results.new_optical_data_available) {
            // Интегрируем скорость потока в позицию (50 Гц, dt = 0.02 с)
            pos_x += optical_flow_results.speed_cm_s_x * 0.020f;
            pos_y += optical_flow_results.speed_cm_s_y * 0.020f;
            float meas_speed_x = optical_flow_results.speed_cm_s_x;
            float meas_speed_y = optical_flow_results.speed_cm_s_y;

            // Целевая скорость (как DJI/Betaflight-hold):
            //   стик в центре → позиционный ПИД держит точку
            //   стик отклонён → точка смещается за дроном, скорость задаёт стик
            float target_speed_x, target_speed_y;
            bool stick_moving = (fabsf(stick_norm_x) > 0.001f) || (fabsf(stick_norm_y) > 0.001f);

            if (position_hold_active) {
                if (stick_moving) {
                    // смещаем точку удержания к текущей позиции, чтобы не тянуло назад
                    target_pos_x = pos_x;
                    target_pos_y = pos_y;
                    PID_Reset(&position_pid_x);
                    PID_Reset(&position_pid_y);
                    target_speed_x = stick_speed_x;
                    target_speed_y = stick_speed_y;
                } else {
                    target_speed_x = PID_Compute(&position_pid_x, target_pos_x - pos_x, 0.020f);
                    target_speed_y = PID_Compute(&position_pid_y, target_pos_y - pos_y, 0.020f);
                }
                target_speed_x = constrain_float(target_speed_x, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);
                target_speed_y = constrain_float(target_speed_y, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);
            } else {
                target_speed_x = stick_speed_x;
                target_speed_y = stick_speed_y;
            }

            error_pitch_mtf = target_speed_y - meas_speed_y;
            error_roll_mtf  = target_speed_x - meas_speed_x;

	            // ПИД скорости вычисляет ТРЕБУЕМЫЙ УГОЛ НАКЛОНА (в градусах)
	            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_y, 0.020f);
	            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_x, 0.020f);

            optical_flow_results.new_optical_data_available = false;
        }
#endif

        // Ограничиваем максимальный наклон для безопасности 
        target_angle_pitch_mtf = constrain_float(target_angle_pitch_mtf, -25.0f, 25.0f);
        target_angle_roll_mtf = constrain_float(target_angle_roll_mtf, -25.0f, 25.0f);

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
        // EKF3: позиция/скорость предсказываются на 1 кГц, коррекция по потоку ~50 Гц.
        pos_x = ekf3.pos_earth_m[0] * 100.0f;
        pos_y = ekf3.pos_earth_m[1] * 100.0f;
        float meas_speed_x = ekf3.vel_body_cms[0];
        float meas_speed_y = ekf3.vel_body_cms[1];

        float target_speed_x = PID_Compute(&position_pid_x, target_pos_x - pos_x, CONTROL_LOOP_DT);
        float target_speed_y = PID_Compute(&position_pid_y, target_pos_y - pos_y, CONTROL_LOOP_DT);
        target_speed_x = constrain_float(target_speed_x, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);
        target_speed_y = constrain_float(target_speed_y, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);

        error_pitch_mtf = target_speed_y - meas_speed_y;
        error_roll_mtf  = target_speed_x - meas_speed_x;

        target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_y, CONTROL_LOOP_DT);
        target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_x, CONTROL_LOOP_DT);

        // D-член от акселерометра (горизонтальное ускорение, без интеграции) — демпфирование
        target_angle_roll_mtf  -= MTF_D_KD * (filtered_Ax * 9.81f - MTF_D_ACC_SIGN_X * 9.81f * sinf(roll  * DEG_TO_RAD)) * 100.0f;
        target_angle_pitch_mtf -= MTF_D_KD * (filtered_Ay * 9.81f - MTF_D_ACC_SIGN_Y * 9.81f * sinf(pitch * DEG_TO_RAD)) * 100.0f;
#else
        // Старая схема: горизонталь привязана к кадру потока (~50 Гц)
        if (optical_flow_results.new_optical_data_available) {
            pos_x += optical_flow_results.speed_cm_s_x * 0.020f;
            pos_y += optical_flow_results.speed_cm_s_y * 0.020f;
            float meas_speed_x = optical_flow_results.speed_cm_s_x;
            float meas_speed_y = optical_flow_results.speed_cm_s_y;

            float target_speed_x = PID_Compute(&position_pid_x, target_pos_x - pos_x, 0.020f);
            float target_speed_y = PID_Compute(&position_pid_y, target_pos_y - pos_y, 0.020f);
            target_speed_x = constrain_float(target_speed_x, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);
            target_speed_y = constrain_float(target_speed_y, -POSITION_MAX_SPEED, POSITION_MAX_SPEED);

            error_pitch_mtf = target_speed_y - meas_speed_y;
            error_roll_mtf  = target_speed_x - meas_speed_x;

            target_angle_pitch_mtf = PID_DoM_Compute(&pitch_pid_mtf_DoM, error_pitch_mtf, meas_speed_y, 0.020f);
            target_angle_roll_mtf = PID_DoM_Compute(&roll_pid_mtf_DoM, error_roll_mtf, meas_speed_x, 0.020f);

            optical_flow_results.new_optical_data_available = false;
        }
#endif

        // Ограничиваем максимальный наклон для безопасности
        target_angle_pitch_mtf = constrain_float(target_angle_pitch_mtf, -25.0f, 25.0f);
        target_angle_roll_mtf = constrain_float(target_angle_roll_mtf, -25.0f, 25.0f);

        // Высота: сглаживание LiDAR (50 Гц по кадру — LiDAR в EKF горизонтали не входит)
        if (optical_flow_results.new_optical_data_available) {
            float raw_alt = (float)distance * cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD);
            smooth_altitude_mm = smooth_altitude_mm * 0.7f + raw_alt * 0.3f;
            optical_flow_results.new_optical_data_available = false;
        }

        // Внутренний контур ANGLE (1000 Гц)
        error_pitch_angle = target_angle_pitch_mtf - pitch;
        error_roll_angle = target_angle_roll_mtf - roll;

        target_velocity_pitch = PID_DoM_Compute(&pitch_pid_angle_DoM, error_pitch_angle, pitch, 0.001f);
        target_velocity_roll = PID_DoM_Compute(&roll_pid_angle_DoM, error_roll_angle, roll, 0.001f);
        target_velocity_yaw = 0.0f;   // yaw тоже фиксируем (стики игнорируются)

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
        // Газ игнорируется — тягу задаёт P-регулятор высоты вокруг
        // захваченной базы тяги (мощность в момент включения удержания).
        float alt_err = target_altitude_mm - smooth_altitude_mm;
        final_throttle = hover_throttle_base + ALTITUDE_HOLD_GAIN_TICKS_PER_MM * alt_err;
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
	   total_power_1 = final_throttle + pid_correction_1;
	   total_power_1 = constrain(trim_percent(total_power_1, TRIM_PERCENT_FRONT_LEFT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_2 = -forse_roll_rate + forse_pitch_rate - forse_yaw_rate;
	   pid_correction_2 = constrain_float(pid_correction_2, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_2 = final_throttle + pid_correction_2;
	   total_power_2 = constrain(trim_percent(total_power_2, TRIM_PERCENT_FRONT_RIGHT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_3 = forse_roll_rate - forse_pitch_rate - forse_yaw_rate;
	   pid_correction_3 = constrain_float(pid_correction_3, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_3 = final_throttle + pid_correction_3;
	   total_power_3 = constrain(trim_percent(total_power_3, TRIM_PERCENT_REAR_LEFT),MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

	   pid_correction_4 = -forse_roll_rate - forse_pitch_rate + forse_yaw_rate;
	   pid_correction_4 = constrain_float(pid_correction_4, -max_pid_correction_mshot, max_pid_correction_mshot);
	   total_power_4 = final_throttle + pid_correction_4;
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

  // Позиционные ПИД-регуляторы (удержание точки по оптическому потоку)
  PID_Init(&position_pid_x, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD, 1.0f, 1.0f, POSITION_INTEGRAL_LIMIT, 1.0f);
  PID_Init(&position_pid_y, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD, 1.0f, 1.0f, POSITION_INTEGRAL_LIMIT, 1.0f);
  
  // Термопрогрев IMU: читаем данные впустую ~2 секунды,
  // пока гироскоп и акселерометр не стабилизируются после включения
  for (int i = 0; i < 2000; i++) {
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
/*
    snprintf(buf, sizeof(buf),"fvY %d,fvX %d | OLD y %7.2f x %7.2f | EKF y %7.2f x %7.2f | innov y %7.2f x %7.2f | q %u\n",
    flow_velocity_y, flow_velocity_x,
    optical_flow_results.speed_cm_s_y, optical_flow_results.speed_cm_s_x,
    ekf3.vel_body_cms[1], ekf3.vel_body_cms[0],
    ekf3.innovation[1] * 100.0f, ekf3.innovation[0] * 100.0f,
    (unsigned)flow_quality);
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
