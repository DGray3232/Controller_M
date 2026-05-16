/* globals.h */
#ifndef GLOBALS_H
#define GLOBALS_H

// ВАЖНО: main.h должен быть первым, чтобы подтянуть stm32f4xx.h и определения FPU
#include "main.h"

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"
#include "config_param.h"

// Подключаем заголовки с определениями типов (структур)
#include "bmx055.h"
#include "AHRSAlgorithms.h"
#include "pid.h"
#include "median_moving_average_filter.h"
#include "calculate_notch_coeffs.h"
#include "vibration_analysis.h"
#include "mtf02.h"
#include "optical_flow_compensation.h"

/* --- Структуры и датчики --- */
extern GyroIntegration_t gyro_integration;
extern OpticalFlowResults_t optical_flow_results;
extern BMX055_t BMX055;

/* --- PID Контроллеры --- */
extern PID_2_Controller pitch_pid_rate;
extern PID_2_Controller roll_pid_rate;
extern PID_2_Controller yaw_pid_rate;

extern PID_Controller altitude_pid;
extern float target_altitude_mm;
extern float altitude_error_mm;
extern float throttle_altitude_correction;
extern bool altitude_hold_active;
extern bool last_button_2_state;
extern float final_throttle;

extern PID_DoM_Controller pitch_pid_rate_DoM;
extern PID_DoM_Controller roll_pid_rate_DoM;
extern PID_DoM_Controller yaw_pid_rate_DoM;

extern PID_DoM_Controller pitch_pid_angle_DoM;
extern PID_DoM_Controller roll_pid_angle_DoM;

extern PID_DoM_Controller pitch_pid_mtf_DoM;
extern PID_DoM_Controller roll_pid_mtf_DoM;

/* --- Фильтры (Медианные и LPF) --- */
extern MedianFilter Gyro_x;
extern MedianFilter Gyro_y;
extern MedianFilter Gyro_z;
extern MedianFilter Accel_x;
extern MedianFilter Accel_y;
extern MedianFilter Accel_z;
extern MedianFilter Gyro_x_bias;
extern MedianFilter Gyro_y_bias;
extern MedianFilter Gyro_z_bias;
extern MedianFilter Accel_x_ofset;
extern MedianFilter Accel_y_ofset;
extern MedianFilter Accel_z_ofset;

extern Filter_lpf lpf_mtf_y;
extern Filter_lpf lpf_mtf_x;

/* --- DSP Фильтры (CMSIS-DSP Instances) --- */
extern arm_biquad_cascade_df2T_instance_f32 imu_Gx_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gx_notch;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gy_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gy_notch;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gz_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gz_notch;

extern arm_biquad_cascade_df2T_instance_f32 imu_Ax_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Ax_notch;
extern arm_biquad_cascade_df2T_instance_f32 imu_Ay_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Ay_notch;
extern arm_biquad_cascade_df2T_instance_f32 imu_Az_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Az_notch;

extern arm_biquad_cascade_df2T_instance_f32 imu_Gx_D_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gy_D_lpf;
extern arm_biquad_cascade_df2T_instance_f32 imu_Gz_D_lpf;

extern arm_biquad_cascade_df2T_instance_f32 lpf_Ax;
extern arm_biquad_cascade_df2T_instance_f32 lpf_Ay;
extern arm_biquad_cascade_df2T_instance_f32 lpf_Az;

extern arm_biquad_cascade_df2T_instance_f32 hpf_Ax;
extern arm_biquad_cascade_df2T_instance_f32 hpf_Ay;
extern arm_biquad_cascade_df2T_instance_f32 hpf_Az;

extern arm_rfft_fast_instance_f32 fft_params;

/* --- Переменные состояния и управления --- */
extern int16_t joystick_x;
extern int16_t joystick_y;
extern int16_t right_left;
extern uint16_t potentiometer_value;
extern uint16_t button;
extern uint16_t button_2;

extern bool mavlink_connection_active;
extern uint32_t mav_last_packet_time;

/* --- Данные IMU и вычисления --- */
extern float32_t Ax[1];
extern float32_t Ay[1];
extern float32_t Az[1];
extern float32_t offset_Ax;
extern float32_t offset_Ay;
extern float32_t offset_Az;
extern float32_t filter_Ax_lpf[1];
extern float32_t filter_Ay_lpf[1];
extern float32_t filter_Az_lpf[1];
extern float32_t filter_Ax_notch[1];
extern float32_t filter_Ay_notch[1];
extern float32_t filter_Az_notch[1];

extern float32_t Gx[1];
extern float32_t Gy[1];
extern float32_t Gz[1];
extern float32_t bias_Gx;
extern float32_t bias_Gy;
extern float32_t bias_Gz;
extern float32_t filter_Gx_D_lpf[1];
extern float32_t filter_Gy_D_lpf[1];
extern float32_t filter_Gz_D_lpf[1];

extern float32_t filter_Gx_lpf[1];
extern float32_t filter_Gx_notch[1];
extern float32_t filter_Gy_lpf[1];
extern float32_t filter_Gy_notch[1];
extern float32_t filter_Gz_lpf[1];
extern float32_t filter_Gz_notch[1];

/* --- Переменные PID логики --- */
extern float32_t actual_velocity_pitch;
extern float32_t actual_velocity_roll;
extern float32_t actual_velocity_yaw;
extern float32_t actual_velocity_pitch_D;
extern float32_t actual_velocity_roll_D;
extern float32_t actual_velocity_yaw_D;
extern float32_t target_velocity_pitch;
extern float32_t target_velocity_roll;
extern float32_t target_velocity_yaw;

extern float32_t filtered_Gx;
extern float32_t filtered_Gy;
extern float32_t filtered_Gz;
extern float32_t filtered_Gx_D;
extern float32_t filtered_Gy_D;
extern float32_t filtered_Gz_D;
extern float32_t filtered_Ax;
extern float32_t filtered_Ay;
extern float32_t filtered_Az;

extern float32_t prev_filtered_Gx;
extern float32_t prev_filtered_Gy;
extern float32_t prev_filtered_Gz;
extern float32_t prev_filtered_Ax;
extern float32_t prev_filtered_Ay;
extern float32_t prev_filtered_Az;

extern float error_pitch_rate;
extern float error_roll_rate;
extern float error_yaw_rate;

extern float error_pitch_rate_D;
extern float error_roll_rate_D;
extern float error_yaw_rate_D;

extern float error_pitch_angle;
extern float error_roll_angle;

extern float error_pitch_mtf;
extern float error_roll_mtf;

extern int forse_pitch_rate;
extern int forse_roll_rate;
extern int forse_yaw_rate;

extern uint16_t throttle_mshot;
extern float max_pid_correction_mshot;

extern float pitch, yaw, roll;
extern float Quat_actual[4];

/* --- Моторы --- */
extern float pid_correction_1;
extern float pid_correction_2;
extern float pid_correction_3;
extern float pid_correction_4;
extern int total_power_1;
extern int total_power_2;
extern int total_power_3;
extern int total_power_4;

extern uint32_t m1_pulse[1];
extern uint32_t m2_pulse[1];
extern uint32_t m3_pulse[1];
extern uint32_t m4_pulse[1];

extern int filtered_power_1;
extern int filtered_power_2;
extern int filtered_power_3;
extern int filtered_power_4;

/* --- FFT и Вибрации --- */
extern float32_t Ax_fft[FFT_LEN];
extern float32_t Ay_fft[FFT_LEN];
extern float32_t Az_fft[FFT_LEN];
extern uint16_t x_index;
extern uint16_t y_index;
extern uint16_t z_index;

extern float32_t Ax_for_fft[1];
extern float32_t Ay_for_fft[1];
extern float32_t Az_for_fft[1];

extern float32_t filter_hpf_Ax_for_fft[1];
extern float32_t filter_hpf_Ay_for_fft[1];
extern float32_t filter_hpf_Az_for_fft[1];

extern float32_t filter_lpf_Ax_for_fft[1];
extern float32_t filter_lpf_Ay_for_fft[1];
extern float32_t filter_lpf_Az_for_fft[1];

extern float32_t filter_Ax_for_fft;
extern float32_t filter_Ay_for_fft;
extern float32_t filter_Az_for_fft;

extern float vibration_frequency_x;
extern float vibration_frequency_y;
extern float vibration_frequency_z;

extern float previous_freq_x;
extern float previous_freq_y;
extern float previous_freq_z;

/* --- Оптический поток и данные --- */
extern uint8_t buffer_message_mtf02[MIKOLINL];
extern uint32_t distance;
extern uint8_t distance_strength;
extern uint8_t distance_precision;
extern uint8_t distance_status;
extern int16_t flow_velocity_x;
extern int16_t flow_velocity_y;
extern uint8_t flow_quality;
extern uint8_t flow_status;

/* --- Режимы и флаги --- */
extern uint8_t flight_mode;
extern int button_mode;
extern uint8_t active_mode;

extern int count_calculate_frequency;
extern uint32_t count_calculate_frequency_flag;

/* --- Буферы I2C DMA --- */
extern uint8_t dma_accel_buffer[7];
extern uint8_t dma_gyro_buffer[6];

/* --- Флаги состояния --- */
extern volatile uint8_t current_device;
extern volatile uint8_t data_ready_gyro;
extern volatile uint8_t data_ready_accel;
extern volatile uint8_t i2c_busy_flag;
extern uint32_t i2c_timeout_counter;

/* --- Внутренние буферы фильтров --- */
extern float fft_output_buffer[FFT_LEN * 2];
extern float fft_magnitude_buffer[FFT_LEN / 2];

extern float32_t filterState_Gx_lpf[NUM_STAGES_GYRO_LPF * 4];
extern float32_t filterState_Gx_notch[NUM_STAGES_GYRO_NOTCH * 4];
extern float32_t filterState_Gy_lpf[NUM_STAGES_GYRO_LPF * 4];
extern float32_t filterState_Gy_notch[NUM_STAGES_GYRO_NOTCH * 4];
extern float32_t filterState_Gz_lpf[NUM_STAGES_GYRO_LPF * 4];
extern float32_t filterState_Gz_notch[NUM_STAGES_GYRO_NOTCH * 4];

extern float32_t filterState_Ax_lpf[NUM_STAGES_ACCEL_LPF * 4];
extern float32_t filterState_Ax_notch[NUM_STAGES_ACCEL_NOTCH * 4];
extern float32_t filterState_Ay_lpf[NUM_STAGES_ACCEL_LPF * 4];
extern float32_t filterState_Ay_notch[NUM_STAGES_ACCEL_NOTCH * 4];
extern float32_t filterState_Az_lpf[NUM_STAGES_ACCEL_LPF * 4];
extern float32_t filterState_Az_notch[NUM_STAGES_ACCEL_NOTCH * 4];

extern float32_t filterState_Gx_D_lpf[NUM_STAGES_D_GYRO_LPF * 4];
extern float32_t filterState_Gy_D_lpf[NUM_STAGES_D_GYRO_LPF * 4];
extern float32_t filterState_Gz_D_lpf[NUM_STAGES_D_GYRO_LPF * 4];

extern float32_t filterState_Ax_lpf_fft[NUM_STAGES_ACCEL_FFT * 4];
extern float32_t filterState_Ay_lpf_fft[NUM_STAGES_ACCEL_FFT * 4];
extern float32_t filterState_Az_lpf_fft[NUM_STAGES_ACCEL_FFT * 4];

extern float32_t filterState_Ax_hpf_fft[NUM_STAGES_ACCEL_FFT * 4];
extern float32_t filterState_Ay_hpf_fft[NUM_STAGES_ACCEL_FFT * 4];
extern float32_t filterState_Az_hpf_fft[NUM_STAGES_ACCEL_FFT * 4];

/* --- Коэффициенты фильтров --- */
extern float32_t Coeffs_D_Gyro_lpf[NUM_STAGES_D_GYRO_LPF * 5];
extern float32_t Coeffs_accel_lpf_fft[NUM_STAGES_ACCEL_FFT * 5];
extern float32_t Coeffs_accel_hpf_fft[NUM_STAGES_ACCEL_FFT * 5];
extern float32_t Coeffs_gyro_lpf[NUM_STAGES_GYRO_LPF * 5];
extern float32_t Coeffs_accel_lpf[NUM_STAGES_GYRO_LPF * 5];
extern float32_t Coeffs_notch_x[NUM_STAGES_GYRO_NOTCH * 5];
extern float32_t Coeffs_notch_y[NUM_STAGES_GYRO_NOTCH * 5];
extern float32_t Coeffs_notch_z[NUM_STAGES_GYRO_NOTCH * 5];

#endif /* GLOBALS_H */
