#ifndef PID_H_
#define PID_H_

// Структура для PID-контроллера
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

// Функция инициализации PID-контроллера
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float alpha, float alpha_derivative, float integral_limit, float scale_factor);

// Функция вычисления управляющего сигнала
float PID_Compute(PID_Controller *pid, float error, float dt);


// Структура для P-контроллера
typedef struct {
	float Kp_p;
	float alpha_p;
	float previous_output_p;
	float scale_factor_p; // Коэффициент масштабирования
} P_Controller;

// Функция инициализации P-контроллера
void P_Init(P_Controller *p, float Kp_p, float alpha_p, float scale_factor_p);

// Функция вычисления управляющего сигнала
float P_Compute(P_Controller *p, float error_p);

// Структура для PID-контроллера
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

// Функция инициализации PID-контроллера
void PID_2_Init(PID_2_Controller *pid, float Kp, float Ki, float Kd, float alpha, float alpha_derivative, float integral_limit, float scale_factor);

// Функция вычисления управляющего сигнала
float PID_2_Compute(PID_2_Controller *pid, float error, float error_d, float dt);

void PID_2_Reset(PID_2_Controller *pid);
void PID_Reset(PID_Controller *pid);

void PID_2_Update_PID(PID_2_Controller *pid, float Kp,float Ki,float Kd);

// Структура для PID-контроллера с производной от измерения (DoM)
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
