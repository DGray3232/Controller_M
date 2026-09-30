#include "optical_flow_compensation.h"
#include "math.h"
#include "config_param.h"

// Флаг «пришёл свежий кадр потока» выставляется в get_mtf_data() (main.c),
// когда декодер MicoLink собрал полное сообщение RANGE_SENSOR.
extern volatile uint8_t flow_frame_received;

// Константы
#define OPTICAL_FLOW_UPDATE_PERIOD_MS 20   // период окна компенсации (частота потока ~50 Гц)

/////////////////////////////////////////////////////
// Функция для накопления данных гироскопа (1000 Гц)
// ЦЕЛЬ:
// Пишем в кольцевой буфер по одной записи на каждый вызов (1 кГц = 1 мс).
// Каждая запись хранит накопленный угол за этот 1 мс и таймстамп.
// Это позволяет при приходе кадра потока взять накопление гироскопа ровно
// за окно [t-20мс, t], привязанное к моменту кадра, а не к локальному таймеру.
/////////////////////////////////////////////////////
void gyro_integration_update(GyroIntegration_t* gyro_int, float filtered_Gx, float filtered_Gy)
{
    if (gyro_int == NULL){
    	return;
    }

    // Цикл жёстко привязан к TIM11 (1 кГц), поэтому dt фиксированный 1 мс.
    // Отказ от HAL_GetTick() убирает джиттер 0/1/2 мс из-за фазового сдвига SysTick/TIM11.
    const float dt = 0.001f;

    uint8_t h = gyro_int->head;

    // Знаки как раньше: -Gx и Gy (точное соответствие осей уточняется на стенде).
    gyro_int->dangle_x[h] = (-filtered_Gx) * dt;
    gyro_int->dangle_y[h] = filtered_Gy * dt;
    gyro_int->timestamp_us[h] = HAL_GetTick() * 1000UL;

    gyro_int->head = (h + 1) % GYRO_HISTORY_SIZE;
    if (gyro_int->count < GYRO_HISTORY_SIZE) {
        gyro_int->count++;
    }
}

/////////////////////////////////////////////////////
// Суммирует накопление гироскопа за окно window_ms, сдвинутое на latency_ms
// назад от самого свежего слота. Сдвиг учитывает задержку датчика: кадр потока,
// пришедший «сейчас», был измерен на latency_ms раньше.
/////////////////////////////////////////////////////
void gyro_get_window_sum(GyroIntegration_t* gyro_int, uint32_t window_ms, uint32_t latency_ms, float* out_x, float* out_y)
{
    *out_x = 0.0f;
    *out_y = 0.0f;
    if (gyro_int == NULL || out_x == NULL || out_y == NULL){
        return;
    }

    // Суммируем слоты в диапазоне возрастов [latency, latency+window)
    uint32_t n = latency_ms + window_ms;
    if (n > GYRO_HISTORY_SIZE) n = GYRO_HISTORY_SIZE;
    if (n > gyro_int->count)   n = gyro_int->count;

    for (uint32_t age = latency_ms; age < n; age++) {
        // head указывает на следующую (ещё не записанную) ячейку, поэтому
        // самая свежая запись — head-1, и идём назад по возрасту.
        uint8_t idx = (gyro_int->head + GYRO_HISTORY_SIZE - 1 - age) % GYRO_HISTORY_SIZE;
        *out_x += gyro_int->dangle_x[idx];
        *out_y += gyro_int->dangle_y[idx];
    }
}

/////////////////////////////////////////////////////
// Вычисление "ложной" линейной скорости (из-за вращения), используя
// накопленный за окно угол и время окна.
/////////////////////////////////////////////////////
void calculate_linear_velocity_from_saved_data(OpticalFlowResults_t* results,
                                               uint32_t distance,
                                               float pitch_deg, float roll_deg,
                                               float saved_acc_x, float saved_acc_y,
                                               uint32_t saved_time_us)
{
    if (results == NULL) return;

    // 1. Истинная высота с учётом наклона (в метрах)
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad  = roll_deg * DEG_TO_RAD;
    results->distance_m = (distance / 1000.0f) * cosf(pitch_rad) * cosf(roll_rad);

    // 2. Если данные валидны, вычисляем ложную линейную скорость
    if (results->distance_m > 0.001f && saved_time_us > 0) {
        // Накопленный угол из градусов в радианы
        float accumulated_rotation_x = saved_acc_x * DEG_TO_RAD;
        float accumulated_rotation_y = saved_acc_y * DEG_TO_RAD;

        // Время окна в секундах
        float time_s = saved_time_us / 1000000.0f;

        // Средняя угловая скорость (рад/с)
        float angular_velocity_x = accumulated_rotation_x / time_s;
        float angular_velocity_y = accumulated_rotation_y / time_s;

        // Линейная скорость от вращения: V = ω * h (м/с), с калибровочным множителем
        float V_linear_x_gyro = angular_velocity_x * results->distance_m * OF_ROT_COMP_GAIN_X;
        float V_linear_y_gyro = angular_velocity_y * results->distance_m * OF_ROT_COMP_GAIN_Y;

        // Результат в см/с
        results->speed_cm_s_x = V_linear_x_gyro * 100.0f;
        results->speed_cm_s_y = V_linear_y_gyro * 100.0f;
    } else {
        // Нет достоверных данных – обнуляем
        results->speed_cm_s_x = 0.0f;
        results->speed_cm_s_y = 0.0f;
    }
}

/////////////////////////////////////////////////////
// Компенсация вращения в данных оптического потока
/////////////////////////////////////////////////////
void compensate_rotation_for_optical_flow(OpticalFlowResults_t* results,
                                         Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                                         int16_t flow_velocity_x, int16_t flow_velocity_y)
{
    if (results == NULL || lpf_x == NULL || lpf_y == NULL){
    	return;
    }

    if (results->distance_m > 0.001f) {
        // Общая линейная скорость (см/с) по данным сенсора
        float mtf_x = (float)flow_velocity_x * results->distance_m;
        float mtf_y = (float)flow_velocity_y * results->distance_m;
        // Вычитание ложной скорости от вращения
        float compensated_x = mtf_x - results->speed_cm_s_x;
        float compensated_y = mtf_y - results->speed_cm_s_y;
        // Фильтрация
        results->speed_cm_s_x = low_pass_filter(lpf_x, compensated_x);
        results->speed_cm_s_y = low_pass_filter(lpf_y, compensated_y);
    }
}

/////////////////////////////////////////////////////
// Основная функция обработки оптического потока.
// Вызывается на 1 кГц, но реально срабатывает только когда пришёл
// свежий кадр потока (flow_frame_received). Накопление гироскопа берётся
// ровно за окно 20 мс, привязанное к моменту кадра.
/////////////////////////////////////////////////////
void process_optical_flow_data(OpticalFlowResults_t* results,
                              GyroIntegration_t* gyro_int,
                              Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                              uint32_t distance,
                              int16_t flow_velocity_x, int16_t flow_velocity_y,
                              uint8_t flow_quality,
                              float pitch_deg, float roll_deg)
{
    if (results == NULL || gyro_int == NULL) return;

    // Ждём свежий кадр потока (ставится в get_mtf_data)
    if (!flow_frame_received) return;
    flow_frame_received = 0;

    // Гейт по качеству потока: при плохой текстуре/свете данные мусорные.
    // Пропускаем кадр, чтобы не гнать мусор в позицию и PID.
    if (flow_quality < OF_QUALITY_MIN) {
        // Обнуляем результат, чтобы позиция не дрейфовала от мусора
        results->speed_cm_s_x = 0.0f;
        results->speed_cm_s_y = 0.0f;
        results->new_optical_data_available = true;   // сигнал есть, но скорость 0
        return;
    }

    // Гейт по высоте: ниже 8 см данные MTF (поток+LiDAR) мусорные — пропускаем.
    if (distance < MTF_MIN_VALID_ALTITUDE_MM) {
        results->speed_cm_s_x = 0.0f;
        results->speed_cm_s_y = 0.0f;
        results->new_optical_data_available = true;
        return;
    }

    // Накопление гироскопа за окно 20 мс, сдвинутое на задержку датчика назад
    float window_acc_x = 0.0f, window_acc_y = 0.0f;
    gyro_get_window_sum(gyro_int, OPTICAL_FLOW_UPDATE_PERIOD_MS, OF_SENSOR_LATENCY_MS,
                        &window_acc_x, &window_acc_y);
    uint32_t window_us = OPTICAL_FLOW_UPDATE_PERIOD_MS * 1000UL;

    // Ложная скорость от вращения (сохраняется в results->speed_cm_s_x/y)
    calculate_linear_velocity_from_saved_data(results, distance, pitch_deg, roll_deg,
                                              window_acc_x, window_acc_y, window_us);

    // Вычитаем ложную скорость из потока → чистая скорость
    compensate_rotation_for_optical_flow(results, lpf_x, lpf_y, flow_velocity_x, flow_velocity_y);

    // Сигнал для main.c, что данные готовы
    results->new_optical_data_available = true;
}

/////////////////////////////////////////////////////
// Вспомогательные функции
/////////////////////////////////////////////////////

// Сброс кольцевого буфера гироскопа
void gyro_integration_reset(GyroIntegration_t* gyro_int)
{
    if (gyro_int == NULL){
    	return;
    }
    gyro_int->head = 0;
    gyro_int->count = 0;
    for (uint8_t i = 0; i < GYRO_HISTORY_SIZE; i++) {
        gyro_int->dangle_x[i] = 0.0f;
        gyro_int->dangle_y[i] = 0.0f;
        gyro_int->timestamp_us[i] = 0;
    }
}

// Инициализация структуры результатов
void optical_flow_results_init(OpticalFlowResults_t* results)
{
    if (results == NULL){
    	return;
    }
    results->speed_cm_s_x = 0;
    results->speed_cm_s_y = 0;
    results->distance_m = 0;
    results->last_optical_flow_update = 0;
    results->new_optical_data_available = false;
}
