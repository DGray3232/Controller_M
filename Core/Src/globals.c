/* globals.c */
#include "globals.h"

GyroIntegration_t gyro_integration = {0};
float target_angle_pitch_mtf = 0.0f;
float target_angle_roll_mtf = 0.0f;
float smooth_altitude_mm = 0.0f;
OpticalFlowResults_t optical_flow_results = {0};
EKF3_t ekf3 = {0};

float imu_ax = 0.0f, imu_ay = 0.0f, imu_az = 0.0f;
float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = 0.0f;
float imu_temp_degc = 25.0f;             // °C
float imu_temp = 23.0f;

PID_2_Controller pitch_pid_rate;
PID_2_Controller roll_pid_rate;
PID_2_Controller yaw_pid_rate;

float target_altitude_mm = 0.0f;
float hover_throttle_base = 0.0f;
float altitude_control = 0.0f;
float alt_hold_capture_pot = 0.0f;
float alt_hold_capture_alt_mm = 0.0f;
float final_throttle = 0;
float yaw_capture = 0.0f;

MedianFilter Gyro_x = {0};
MedianFilter Gyro_y = {0};
MedianFilter Gyro_z = {0};
MedianFilter Accel_x = {0};
MedianFilter Accel_y = {0};
MedianFilter Accel_z = {0};
MedianFilter Gyro_x_bias = {0};
MedianFilter Gyro_y_bias = {0};
MedianFilter Gyro_z_bias = {0};
MedianFilter Accel_x_ofset = {0};
MedianFilter Accel_y_ofset = {0};
MedianFilter Accel_z_ofset = {0};

Filter_lpf lpf_mtf_y;
Filter_lpf lpf_mtf_x;

// PID-регуляторы DoM (основные)
PID_DoM_Controller pitch_pid_rate_DoM;
PID_DoM_Controller roll_pid_rate_DoM;
PID_DoM_Controller yaw_pid_rate_DoM;

PID_DoM_Controller pitch_pid_angle_DoM;
PID_DoM_Controller roll_pid_angle_DoM;

PID_DoM_Controller pitch_pid_mtf_DoM;
PID_DoM_Controller roll_pid_mtf_DoM;

PID_DoM_Controller altitude_pid;

// CMSIS-DSP фильтры
arm_biquad_cascade_df2T_instance_f32 imu_Gx_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Gx_notch;
arm_biquad_cascade_df2T_instance_f32 imu_Gy_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Gy_notch;
arm_biquad_cascade_df2T_instance_f32 imu_Gz_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Gz_notch;

arm_biquad_cascade_df2T_instance_f32 imu_Ax_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Ax_notch;
arm_biquad_cascade_df2T_instance_f32 imu_Ay_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Ay_notch;
arm_biquad_cascade_df2T_instance_f32 imu_Az_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Az_notch;

arm_biquad_cascade_df2T_instance_f32 imu_Gx_D_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Gy_D_lpf;
arm_biquad_cascade_df2T_instance_f32 imu_Gz_D_lpf;

arm_biquad_cascade_df2T_instance_f32 lpf_Ax;
arm_biquad_cascade_df2T_instance_f32 lpf_Ay;
arm_biquad_cascade_df2T_instance_f32 lpf_Az;

arm_biquad_cascade_df2T_instance_f32 hpf_Ax;
arm_biquad_cascade_df2T_instance_f32 hpf_Ay;
arm_biquad_cascade_df2T_instance_f32 hpf_Az;

arm_rfft_fast_instance_f32 fft_params;

// Данные с пульта (MAVLink)
int16_t joystick_x = 0;
int16_t joystick_y = 0;
uint16_t potentiometer_value = 0;
int16_t right_left = 0;
uint16_t button = 0;
uint16_t button_2 = 0;
uint16_t button_alt_hold = 0;

bool mavlink_connection_active = false;
uint32_t mav_last_packet_time = 0;

// Данные IMU
float32_t Ax[1] = {0};
float32_t Ay[1] = {0};
float32_t Az[1] = {0};
float32_t offset_Ax = {0};
float32_t offset_Ay = {0};
float32_t offset_Az = {0};
float32_t filter_Ax_lpf[1] = {0};
float32_t filter_Ay_lpf[1] = {0};
float32_t filter_Az_lpf[1] = {0};
float32_t filter_Ax_notch[1] = {0};
float32_t filter_Ay_notch[1] = {0};
float32_t filter_Az_notch[1] = {0};

float32_t Gx[1] = {0};
float32_t Gy[1] = {0};
float32_t Gz[1] = {0};
float32_t bias_Gx = {0};
float32_t bias_Gy = {0};
float32_t bias_Gz = {0};
float32_t filter_Gx_D_lpf[1] = {0};
float32_t filter_Gy_D_lpf[1] = {0};
float32_t filter_Gz_D_lpf[1] = {0};

float32_t filter_Gx_lpf[1] = {0};
float32_t filter_Gx_notch[1] = {0};
float32_t filter_Gy_lpf[1] = {0};
float32_t filter_Gy_notch[1] = {0};
float32_t filter_Gz_lpf[1] = {0};
float32_t filter_Gz_notch[1] = {0};

float32_t actual_velocity_pitch = 0;
float32_t actual_velocity_roll = 0;
float32_t actual_velocity_yaw = 0;
float32_t actual_velocity_pitch_D = 0;
float32_t actual_velocity_roll_D = 0;
float32_t actual_velocity_yaw_D = 0;
float32_t target_velocity_pitch = 0;
float32_t target_velocity_roll = 0;
float32_t target_velocity_yaw = 0;

float32_t filtered_Gx = 0;
float32_t filtered_Gy = 0;
float32_t filtered_Gz = 0;
float32_t filtered_Gx_D = 0;
float32_t filtered_Gy_D = 0;
float32_t filtered_Gz_D = 0;
float32_t filtered_Ax = 0;
float32_t filtered_Ay = 0;
float32_t filtered_Az = 0;

float32_t prev_filtered_Gx = 0;
float32_t prev_filtered_Gy = 0;
float32_t prev_filtered_Gz = 0;
float32_t prev_filtered_Ax = 0;
float32_t prev_filtered_Ay = 0;
float32_t prev_filtered_Az = 0;

float error_pitch_rate = 0;
float error_roll_rate = 0;
float error_yaw_rate = 0;

float error_pitch_rate_D = 0;
float error_roll_rate_D = 0;
float error_yaw_rate_D = 0;

float error_pitch_angle = 0;
float error_roll_angle = 0;

float error_pitch_mtf = 0;
float error_roll_mtf = 0;

int forse_pitch_rate = 0;
int forse_roll_rate = 0;
int forse_yaw_rate = 0;

uint16_t throttle_mshot = 0;
float max_pid_correction_mshot = 0;

float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
float Quat_actual[4] = {1.0f, 0.0f, 0.0f, 0.0f};

float pid_correction_1 = 0;
float pid_correction_2 = 0;
float pid_correction_3 = 0;
float pid_correction_4 = 0;
int total_power_1 = 0;
int total_power_2 = 0;
int total_power_3 = 0;
int total_power_4 = 0;

uint32_t m1_pulse[1] = {MIN_PULSE_WIDTH};
uint32_t m2_pulse[1] = {MIN_PULSE_WIDTH};
uint32_t m3_pulse[1] = {MIN_PULSE_WIDTH};
uint32_t m4_pulse[1] = {MIN_PULSE_WIDTH};

int filtered_power_1 = MIN_PULSE_WIDTH;
int filtered_power_2 = MIN_PULSE_WIDTH;
int filtered_power_3 = MIN_PULSE_WIDTH;
int filtered_power_4 = MIN_PULSE_WIDTH;

// FFT и анализ вибраций
float32_t Ax_fft[FFT_LEN] = {0};
float32_t Ay_fft[FFT_LEN] = {0};
float32_t Az_fft[FFT_LEN] = {0};

uint16_t x_index = 0;
uint16_t y_index = 0;
uint16_t z_index = 0;

float32_t Ax_for_fft[1] = {0};
float32_t Ay_for_fft[1] = {0};
float32_t Az_for_fft[1] = {0};

float32_t filter_hpf_Ax_for_fft[1] = {0};
float32_t filter_hpf_Ay_for_fft[1] = {0};
float32_t filter_hpf_Az_for_fft[1] = {0};

float32_t filter_lpf_Ax_for_fft[1] = {0};
float32_t filter_lpf_Ay_for_fft[1] = {0};
float32_t filter_lpf_Az_for_fft[1] = {0};

float32_t filter_Ax_for_fft = 0;
float32_t filter_Ay_for_fft = 0;
float32_t filter_Az_for_fft = 0;

float vibration_frequency_x = 0;
float vibration_frequency_y = 0;
float vibration_frequency_z = 0;

float previous_freq_x = 0;
float previous_freq_y = 0;
float previous_freq_z = 0;

// Данные MTF-02 (оптический поток + дальномер)
uint8_t buffer_message_mtf02[MTF_DMA_BUFFER_SIZE];

uint32_t distance = 0;
uint8_t distance_strength = 0;
uint8_t distance_precision = 0;
uint8_t distance_status = 0;
int16_t flow_velocity_x = 0;
int16_t flow_velocity_y = 0;
uint8_t flow_quality = 0;
uint8_t flow_status = 0;
volatile uint8_t flow_frame_received = 0;

// Режимы и флаги
uint8_t flight_mode = FLIGHT_MODE_ACRO;
int button_mode = 0;
uint8_t active_mode = 0;

int count_calculate_frequency = 0;
uint32_t count_calculate_frequency_flag = 0;

float fft_output_buffer[FFT_LEN * 2];
float fft_magnitude_buffer[FFT_LEN / 2];

float32_t filterState_Gx_lpf[NUM_STAGES_GYRO_LPF * 4] = {0};
float32_t filterState_Gx_notch[NUM_STAGES_GYRO_NOTCH * 4] = {0};
float32_t filterState_Gy_lpf[NUM_STAGES_GYRO_LPF * 4] = {0};
float32_t filterState_Gy_notch[NUM_STAGES_GYRO_NOTCH * 4] = {0};
float32_t filterState_Gz_lpf[NUM_STAGES_GYRO_LPF * 4] = {0};
float32_t filterState_Gz_notch[NUM_STAGES_GYRO_NOTCH * 4] = {0};

float32_t filterState_Ax_lpf[NUM_STAGES_ACCEL_LPF * 4] = {0};
float32_t filterState_Ax_notch[NUM_STAGES_ACCEL_NOTCH * 4] = {0};
float32_t filterState_Ay_lpf[NUM_STAGES_ACCEL_LPF * 4] = {0};
float32_t filterState_Ay_notch[NUM_STAGES_ACCEL_NOTCH * 4] = {0};
float32_t filterState_Az_lpf[NUM_STAGES_ACCEL_LPF * 4] = {0};
float32_t filterState_Az_notch[NUM_STAGES_ACCEL_NOTCH * 4] = {0};

float32_t filterState_Gx_D_lpf[NUM_STAGES_D_GYRO_LPF * 4] = {0};
float32_t filterState_Gy_D_lpf[NUM_STAGES_D_GYRO_LPF * 4] = {0};
float32_t filterState_Gz_D_lpf[NUM_STAGES_D_GYRO_LPF * 4] = {0};

float32_t filterState_Ax_lpf_fft[NUM_STAGES_ACCEL_FFT * 4] = {0};
float32_t filterState_Ay_lpf_fft[NUM_STAGES_ACCEL_FFT * 4] = {0};
float32_t filterState_Az_lpf_fft[NUM_STAGES_ACCEL_FFT * 4] = {0};

float32_t filterState_Ax_hpf_fft[NUM_STAGES_ACCEL_FFT * 4] = {0};
float32_t filterState_Ay_hpf_fft[NUM_STAGES_ACCEL_FFT * 4] = {0};
float32_t filterState_Az_hpf_fft[NUM_STAGES_ACCEL_FFT * 4] = {0};

float32_t Coeffs_D_Gyro_lpf[NUM_STAGES_D_GYRO_LPF * 5] = {
      0.04613200f,   0.09226400f,   0.04613200f,   1.30728500f,  -0.49181200f,
      0.04613200f,   0.09226400f,   0.04613200f,   1.30728500f,  -0.49181200f
};

float32_t Coeffs_accel_lpf_fft[NUM_STAGES_ACCEL_FFT * 5] = {
      0.43284664f,   0.86569330f,   0.43284664f,  -1.04859958f,  -0.29614036f,
      1.00000000f,   1.99999997f,   1.00000001f,  -1.32091343f,  -0.63273879f
};

float32_t Coeffs_accel_hpf_fft[NUM_STAGES_ACCEL_FFT * 5] = {
	  0.95978223f,  -1.91956449f,   0.95978222f,   1.94263823f,  -0.94359728f,
      1.00000000f,  -1.99999997f,   1.00000001f,   1.97526963f,  -0.97624479f
};

float32_t Coeffs_gyro_lpf[NUM_STAGES_GYRO_LPF * 5] = {
      0.06745500f,   0.13491100f,   0.06745500f,   1.14298100f,  -0.41280200f,
      0.06745500f,   0.13491100f,   0.06745500f,   1.14298100f,  -0.41280200f
};
float32_t Coeffs_accel_lpf[NUM_STAGES_ACCEL_LPF * 5] = {
      0.02008300f,   0.04016700f,   0.02008300f,   1.56101800f,  -0.64135200f,
      0.02008300f,   0.04016700f,   0.02008300f,   1.56101800f,  -0.64135200f
};
float32_t Coeffs_notch_x[NUM_STAGES_GYRO_NOTCH * 5] = {
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
float32_t Coeffs_notch_y[NUM_STAGES_GYRO_NOTCH * 5] = {
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
float32_t Coeffs_notch_z[NUM_STAGES_GYRO_NOTCH * 5] = {
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

char buf[512];