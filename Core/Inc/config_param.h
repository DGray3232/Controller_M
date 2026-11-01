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
#define NUM_STAGES_GYRO_LPF 2
#define NUM_STAGES_ACCEL_NOTCH 2
#define NUM_STAGES_ACCEL_LPF 2

/* --- Параметры FFT --- */
#define FFT_LEN 512     // Длина FFT (должна быть степенью 2)
#define F_SAMPLE 1000   // Частота дискретизации, Гц
#define FREQ_HYSTERESIS 2.0f // гистерезис для изменении частоты

/* --- Параметры системы --- */
#define CONTROL_LOOP_DT 0.001f // 1000 Гц
#define BIAS_OFSET 2048
#define MIKOLINL 27
#define MTF_DMA_BUFFER_SIZE 128 // размер кольцевого DMA-буфера MTF-02 (кадр 27 байт + запас)
#define STICK_DEADZONE 2.0f     // мёртвая зона стика (диапазон стика -25..25)

/* --- Оптический поток (компенсация вращения) --- */
#define OF_QUALITY_MIN        25      // [0..255] минимальное качество потока; ниже — кадр игнорируется
#define OPTICAL_FLOW_UPDATE_PERIOD_MS 20  // [мс] период кадров потока (частота MTF-02 ~50 Гц)
#define OF_SENSOR_LATENCY_MS  20      // [мс] задержка датчика (≈1 кадр) — окно гироскопа сдвигается назад
#define OF_ROT_COMP_GAIN_X    1.0f    // множитель компенсации вращения по X (калибровка: corr(скорость,гиро)→0)
#define OF_ROT_COMP_GAIN_Y    1.0f    // множитель компенсации вращения по Y

/* --- EKF3 (loosely-coupled, optical flow + accel, MTF) --- */
#define USE_EKF3              1       // 1 = EKF3 идёт в PID (земная скорость+позиция), 0 = текущая компенсация.
                                       // EKF3 считается и логируется в blackbox ВСЕГДА (для A/B сравнения).
#define EKF3_OF_QUALITY_MIN   25      // [0..255] ниже — кадр потока не обновляет EKF
#define EKF3_P0_VEL           1.0f    // начальная дисперсия скорости [(м/с)²]
#define EKF3_P0_POS           1.0f    // начальная дисперсия позиции [м²]
#define EKF3_Q_VEL            0.005f   // немоделируемое гориз. ускорение [(м/с)²/с] — меньше = больше доверия акселу
#define EKF3_Q_POS            0.01f   // шум процесса позиции [м²/с³] (спектральная плотность)
#define EKF3_P0_BIAS          0.25f   // начальная дисперсия смещения аксела [(м/с²)²]
#define EKF3_Q_BIAS           1e-4f   // random walk смещения аксела [(м/с²)²/с] — медленный дрейф g·sin-протечки
#define EKF3_R_FLOW           0.001f  // шум измерения потока [(м/с)²] при идеальном качестве. Меньше = больше доверия потоку.
#define EKF3_R_FLOW_MAX       2.0f    // потолок R при наихудшем качестве [(м/с)²]
#define EKF3_FLOW_SIGN_X      1.0f    // ±1 — знак оси X потока (по монтажу датчика)
#define EKF3_FLOW_SIGN_Y      1.0f    // ±1 — знак оси Y потока
#define EKF3_FLOW_SWAP        0       // 1 — поменять оси X/Y потока местами
#define EKF3_ROT_SIGN_X      -1.0f    // знак ложной скорости по X от вращения (ω×h)
#define EKF3_ROT_SIGN_Y       1.0f    // знак ложной скорости по Y от вращения (ω×h)
#define EKF3_ROT_COMP_GAIN_X  1.0f    // множитель компенсации вращения по X (1.0 = точное ω×h; >1 = перекомпенсация → круг)
#define EKF3_ROT_COMP_GAIN_Y  1.0f    // множитель компенсации вращения по Y (1.0 = точное ω×h; >1 = перекомпенсация → круг)
#define EKF3_POS_CLAMP        0.5f    // [м] предел позиции (страховка от неограниченного дрейфа)

/* --- Multishot и моторы --- */
#define MULTISHOT_MIN 500
#define MULTISHOT_MAX 2500
#define MIN_PULSE_WIDTH       500
#define MAX_PULSE_WIDTH       2500
#define MOTOR_OUTPUT_FILTER_ALPHA 0.3f

/* --- Ограничения (Safety Limits) --- */
#define MAX_CORRECTION        500
#define MAX_PID_RATE          1500
#define MAX_P                 150

/* --- Триммирование (процентов) --- */
#define TRIM_PERCENT_FRONT_LEFT   3  // подъём спереди (компенсация статического перекоса вперёд)
#define TRIM_PERCENT_FRONT_RIGHT  3
#define TRIM_PERCENT_REAR_LEFT    0
#define TRIM_PERCENT_REAR_RIGHT   0

/* --- Режимы полета --- */
#define FLIGHT_MODE_ANGLE 0
#define FLIGHT_MODE_ACRO  1
#define FLIGHT_MODE_MTF   2
#define FLIGHT_MODE_ALT_HOLD 3  // захват текущей высоты и удержание (газ и стики игнорируются)
#define MTF_MAX_ALTITUDE_MM 3000  // выше этой высоты оптический поток не работает → принудительный ACRO
#define MTF_MIN_VALID_ALTITUDE_MM 80  // ниже 8 см данные MTF (поток+LiDAR) мусорные — не учитываем в фильтрах
#define MTF_GROUND_DEBOUNCE_CYCLES 100  // [циклов @1кГц = мс] устойчивость «на земле» перед заморозкой EKF и сбросом на взлёте (защита от выбросов LiDAR до 0)
#define MAX_TILT_MTF           15.0f  // максимальный наклон в MTF/ALT_HOLD [°] — меньше = медленнее разгон (для тесного помещения)

/* --- PID регуляторы (DoM) --- */
#define PITCH_PID_KP_DoM          3.0 
#define PITCH_PID_KI_DoM          0.1
#define PITCH_PID_KD_DoM          0.05 
#define ROLL_PID_KP_DoM           3.0 
#define ROLL_PID_KI_DoM           0.1
#define ROLL_PID_KD_DoM           0.05
#define YAW_PID_KP_DoM            5.0  
#define YAW_PID_KI_DoM            0.1
#define YAW_PID_KD_DoM            0.05 
#define ALPHA_DoM                 1.0
#define ALPHA_DERIVATIVE_DoM      0.1
#define INTEGRAL_LIMIT_DoM        5.0
#define SCALE_FACTOR_DoM          1.0

#define PITCH_PID_KP_RATE_DoM     3.0
#define PITCH_PID_KI_RATE_DoM     0.3
#define PITCH_PID_KD_RATE_DoM     0.05 
#define ROLL_PID_KP_RATE_DoM      3.0
#define ROLL_PID_KI_RATE_DoM      0.3
#define ROLL_PID_KD_RATE_DoM      0.05 
#define YAW_PID_KP_RATE_DoM       7.0
#define YAW_PID_KI_RATE_DoM       0.4
#define YAW_PID_KD_RATE_DoM       0.05
#define ALPHA_RATE_DoM            1.0
#define ALPHA_DERIVATIVE_RATE_DoM 0.1
#define INTEGRAL_LIMIT_RATE_DoM   5.0
#define SCALE_FACTOR_RATE_DoM     1.0

#define PITCH_PID_KP_MTF_DoM      0.08
#define PITCH_PID_KI_MTF_DoM      0.1 
#define ROLL_PID_KP_MTF_DoM       0.08
#define ROLL_PID_KI_MTF_DoM       0.1 
#if USE_EKF3
// EKF3: контур на 1 кГц — D-компонента даёт дребезг от скачков потока, выключена.
#define PITCH_PID_KD_MTF_DoM      0.0
#define ROLL_PID_KD_MTF_DoM       0.0
#else
// Старая схема: контур на 50 Гц — D полезен как демпфер.
#define PITCH_PID_KD_MTF_DoM      0.05
#define ROLL_PID_KD_MTF_DoM       0.05
#endif
#define ALPHA_MTF_DoM             1.0
#define ALPHA_DERIVATIVE_MTF_DoM  0.1
#define INTEGRAL_LIMIT_MTF_DoM    15.0
#define SCALE_FACTOR_MTF_DoM      1.0

/* D-член от акселерометра (горизонтальное ускорение) — демпфирование скоростного контура.
 * Скорость от потока ступенчатая (random-walk), поэтому обычный D по производной скорости не работает.
 * Вместо этого берём ускорение напрямую из акселерометра (без интеграции → без накопления bias/g-протечки). */
#define MTF_D_KD              0.07f   // KD D-члена [°/ (см/с²)] — демпфирование скоростного контура (гашение раскачки сноса)
#define MTF_D_ACC_SIGN_X      1.0f    // знак горизонтального ускорения по X (калибровка: если раскачивает — инвертировать)
#define MTF_D_ACC_SIGN_Y      1.0f    // знак горизонтального ускорения по Y

/* --- Высотная модель ALT_HOLD (захват высоты, газ/стики игнорируются) --- */
#define GROUND_DISTANCE_MM   100     // [мм] Расстояние LiDAR, ниже которого считаем «на земле» (захват высоты при входе в ALT_HOLD)
#define ALTITUDE_HOLD_KP   0.4f    // [тик/мм] P: тиков газа на 1 мм ошибки высоты.
#define ALTITUDE_HOLD_KI   0.25f   // [тик/(мм·с)] I: убирает статическую ошибку (база тяги vs реальное висение).
#define ALTITUDE_HOLD_KD   0.20f   // [тик/(мм/с)] D (от измерения): демпфирование вертикальной скорости — меньше рывков в GUI.
#define ALTITUDE_HOLD_ALPHA           1.0f    // фильтр выхода (1.0 = без фильтра)
#define ALTITUDE_HOLD_ALPHA_D         0.4f   // фильтр D-члена (0..1) — сильнее сглаживаем производную
#define ALTITUDE_HOLD_INTEGRAL_LIMIT  4000.0f // [мм·с] предел интеграла (анти-виндап); Ki·limit ≈ 200 тик макс вклада I.
#define ALTITUDE_HOLD_SCALE           1.0f
#define ALTITUDE_HOLD_MAX_OUTPUT      400.0f // [тик] предел выхода PID высоты (анти-виндап): база 900 + 400 = макс 1300 тик
#define ALT_HOLD_POT_TO_MM        10.0f   // [мм/тик] смещение целевой высоты на 1 тик крестовины (pot, 0..1000)
#define ALT_HOLD_CLIMB_RATE_MAX   200.0f  // [мм/с] предел вертикальной скорости изменения целевой высоты (плавный взлёт/посадка)
#define ALT_HOLD_TAKEOFF_ALTITUDE 700.0f  // [мм] высота, на которую дрон взлетает с земли при входе в ALT_HOLD
#define ALT_HOLD_TAKEOFF_THROTTLE 850.0f  // [тик] база тяги при взлёте с земли (подобрать ≈ чуть ниже тяги висения)
#define ALT_HOLD_POS_HOLD_MIN_ALTITUDE 150.0f  // [мм] ниже этой высоты горизонтальный контур выключен (вертикальный взлёт) — выше отрыва, чтобы не ловить взлётный рывок
#define YAW_HOLD_KP                5.0f    // [°/с на °] P-коэффициент удержания курса (yaw) в ALT_HOLD

#endif /* CONFIG_PARAM_H */
