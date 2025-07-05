#ifndef PID_H_
#define PID_H_

// Структура для PID-контроллера
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double alpha;
    double alpha_derivative;
    double integral_limit;
    double integral;
    double error_previous;
    double previous_output;
    double previous_derivative;
    double scale_factor; // Коэффициент масштабирования
} PID_Controller;

// Функция инициализации PID-контроллера
void PID_Init(PID_Controller *pid, double Kp, double Ki, double Kd, double alpha, double alpha_derivative, double integral_limit, double scale_factor);

// Функция вычисления управляющего сигнала
double PID_Compute(PID_Controller *pid, double error, double trim);

#endif /* PID_H_ */
