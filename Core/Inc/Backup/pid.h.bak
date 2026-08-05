#ifndef PID_H_
#define PID_H_

// Структура для PID-контроллера (используется для altitude_pid)
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float alpha;
	float alpha_derivative;
	float integral_limit;
	float integral;
	float error_previous;
	float previous_output;
	float previous_derivative;
	float scale_factor; // Коэффициент масштабирования
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float alpha, float alpha_derivative, float integral_limit, float scale_factor);
float PID_Compute(PID_Controller *pid, float error, float dt);
void PID_Reset(PID_Controller *pid);

// Структура для PID-контроллера (используется только PID_2_Reset при disarm)
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float alpha;
	float alpha_derivative;
	float integral_limit;
	float integral;
	float error_previous;
	float previous_output;
	float previous_derivative;
	float scale_factor; // Коэффициент масштабирования
} PID_2_Controller;

void PID_2_Reset(PID_2_Controller *pid);

// Структура для PID-контроллера с производной от измерения (DoM) — основной регулятор
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float alpha;
    float alpha_derivative;
    float integral_limit;
    float integral;
    float previous_measurement; // Храним измерение, а не ошибку
    float previous_output;
    float previous_derivative;
    float scale_factor;
} PID_DoM_Controller;

void PID_DoM_Init(PID_DoM_Controller *pid, float Kp, float Ki, float Kd, float alpha, float alpha_derivative, float integral_limit, float scale_factor);
float PID_DoM_Compute(PID_DoM_Controller *pid, float setpoint, float measurement, float dt);
void PID_DoM_Reset(PID_DoM_Controller *pid);

#endif /* PID_H_ */
