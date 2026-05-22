/* config_param.h */
#ifndef CONFIG_PARAM_H
#define CONFIG_PARAM_H

#include <math.h> // Для M_PI

/* --- Математические константы --- */
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG (180.0f / M_PI)

/* --- Настройки фильтров --- */
#define NUM_STAGES_D_GYRO_LPF 2
#define NUM_STAGES_ACCEL_FFT 2
#define BLOCK_SIZE 1 // Размер блока данных для обработки

#define NUM_STAGES_GYRO_NOTCH 2
#define NUM_STAGES_GYRO_LPF 4
#define NUM_STAGES_ACCEL_NOTCH 2
#define NUM_STAGES_ACCEL_LPF 4

/* --- Параметры FFT --- */
#define FFT_LEN 512     // Длина FFT (должна быть степенью 2)
#define F_SAMPLE 1000   // Частота дискретизации, Гц
#define FREQ_HYSTERESIS 2.0f // гистерезис для изменении частоты

/* --- Параметры системы --- */
#define CONTROL_LOOP_DT 0.001f // 1000 Гц
#define BIAS_OFSET 2048
#define MIKOLINL 27

/* --- Multishot и моторы --- */
#define MULTISHOT_MIN 500
#define MULTISHOT_MAX 2500
#define MIN_PULSE_WIDTH       500
#define MAX_PULSE_WIDTH       2500
#define MOTOR_OUTPUT_FILTER_ALPHA 0.5f

/* --- Ограничения (Safety Limits) --- */
#define MAX_CORRECTION        500
#define MAX_PID_RATE          1500
#define MAX_P                 150

/* --- Триммирование (процентов) --- */
#define TRIM_PERCENT_FRONT_LEFT   0
#define TRIM_PERCENT_FRONT_RIGHT  -1
#define TRIM_PERCENT_REAR_LEFT    5
#define TRIM_PERCENT_REAR_RIGHT   0

/* --- Режимы полета --- */
#define FLIGHT_MODE_ANGLE 0
#define FLIGHT_MODE_ACRO  1
#define FLIGHT_MODE_MTF   2

/* --- PID регуляторы (DoM) --- */
#define PITCH_PID_KP_DoM          1.2  
#define PITCH_PID_KI_DoM          0.02
#define PITCH_PID_KD_DoM          0.05 
#define ROLL_PID_KP_DoM           0.72  
#define ROLL_PID_KI_DoM           0.02
#define ROLL_PID_KD_DoM           0.05 
#define YAW_PID_KP_DoM            5.0  
#define YAW_PID_KI_DoM            0.1
#define YAW_PID_KD_DoM            0.05 
#define ALPHA_DoM                 0.1
#define ALPHA_DERIVATIVE_DoM      0.1
#define INTEGRAL_LIMIT_DoM        5.0
#define SCALE_FACTOR_DoM          1.0

#define PITCH_PID_KP_RATE_DoM     13.0
#define PITCH_PID_KI_RATE_DoM     0.1
#define PITCH_PID_KD_RATE_DoM     0.15
#define ROLL_PID_KP_RATE_DoM      13.0
#define ROLL_PID_KI_RATE_DoM      0.1
#define ROLL_PID_KD_RATE_DoM      0.15
#define YAW_PID_KP_RATE_DoM       21.5
#define YAW_PID_KI_RATE_DoM       0.2
#define YAW_PID_KD_RATE_DoM       0.1
#define ALPHA_RATE_DoM            1.0
#define ALPHA_DERIVATIVE_RATE_DoM 0.1
#define INTEGRAL_LIMIT_RATE_DoM   5.0
#define SCALE_FACTOR_RATE_DoM     1.0

#define PITCH_PID_KP_MTF_DoM      0.36
#define PITCH_PID_KI_MTF_DoM      0.1 
#define PITCH_PID_KD_MTF_DoM      0.018
#define ROLL_PID_KP_MTF_DoM       0.9
#define ROLL_PID_KI_MTF_DoM       0.1 
#define ROLL_PID_KD_MTF_DoM       0.45 
#define ALPHA_MTF_DoM             1.0
#define ALPHA_DERIVATIVE_MTF_DoM  0.1
#define INTEGRAL_LIMIT_MTF_DoM    5.0
#define SCALE_FACTOR_MTF_DoM      1.0

/* --- ПИД высоты --- */
#define ALTITUDE_PID_KP           25.0 
#define ALTITUDE_PID_KI           0.1 
#define ALTITUDE_PID_KD           0.1 
#define ALTITUDE_INTEGRAL_LIMIT   5.0 
#define ALTITUDE_ALPHA            1.0
#define ALTITUDE_ALPHA_DERIVATIVE 0.1
#define ALTITUDE_SCALE_FACTOR     1.0
#define ALTITUDE_MAX_CORRECTION   100.0

#endif /* CONFIG_PARAM_H */
