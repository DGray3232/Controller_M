#include "optical_flow_compensation.h"
#include "math.h"
#include "config_param.h"

// Константы
#define OPTICAL_FLOW_UPDATE_RATE_HZ 50     // 50 Гц - частота обновления оптического потока
#define OPTICAL_FLOW_UPDATE_PERIOD_MS (1000 / OPTICAL_FLOW_UPDATE_RATE_HZ) // 20 мс

/////////////////////////////////////////////////////
// Функция для накопления данных гироскопа (1000 Гц)
// ЦЕЛЬ:
// функция вызывается очень часто (1000 Гц), в то время как
// оптический поток обновляется редко (50 Гц).
// нужно собрать (проинтегрировать) все показания гироскопа,
// которые произошли между обновлениями оптического потока.
/////////////////////////////////////////////////////
void gyro_integration_update(GyroIntegration_t* gyro_int, float filtered_Gx, float filtered_Gy)
{
    if (gyro_int == NULL){
    	return;
    }
    // Текущее время в миллисекундах (сырой HAL_GetTick без умножения)
    uint32_t current_time = HAL_GetTick();
    // Инициализация при первом запуске
    if (gyro_int->last_update == 0) {
        gyro_int->last_update = current_time;
        return;
    }
    // Дельта времени в миллисекундах (без переполнения — unsigned wraparound корректен)
    uint32_t dt_ms = current_time - gyro_int->last_update;
    if (dt_ms > 0) {
        // Переводим в секунды для интеграции угловой скорости
        float dt = dt_ms / 1000.0f;
        // Накопление: Угловая Скорость (град/с) × Время (с) = Накопленный Угол (в градусах)
        // используем -Gx и Gy, чтобы соответствовать осям дрона и
        // направлению, которое сенсор оптического потока считает "положительным".
        gyro_int->acc_x += (-filtered_Gx) * dt;
        gyro_int->acc_y += filtered_Gy * dt;
        // Также накапливаем общее время в микросекундах.
        // Интегратор сбрасывается каждые 20 мс, так что переполнения не будет.
        gyro_int->time_us += dt_ms * 1000;
        gyro_int->update_count++;
    }
    // Обновляем время последнего вызова
    gyro_int->last_update = current_time;
}
/////////////////////////////////////////////////////
// Вычисление "ложной" линейной скорости (из-за вращения)
// ЦЕЛЬ:
// Рассчитать, какую линейную скорость увидел бы сенсор,
// если бы дрон только вращался (данные из gyro_int), но не двигался
// поступательно.
// ФОРМУЛА: Линейная Скорость = Угловая Скорость * Высота
/////////////////////////////////////////////////////
void calculate_linear_velocity(GyroIntegration_t* gyro_int, OpticalFlowResults_t* results, uint32_t distance, float pitch_deg, float roll_deg)
{
    if (gyro_int == NULL || results == NULL){
    	return;
    }
    // 1. Получаем высоту в метрах
    //results->distance_m = distance / 1000.0f;
    // Истинная высота по нормали к земле
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad = roll_deg * DEG_TO_RAD;
    results->distance_m = (distance / 1000.0f) * cosf(pitch_rad) * cosf(roll_rad);
    // Убедимся, что у нас есть корректные данные (высота > 0 и время накопления > 0)
    if (results->distance_m > 0.001f && gyro_int->time_us > 0) {
        // 2. Получаем накопленный угол в РАДИАНАХ
        float accumulated_rotation_x = gyro_int->acc_x * DEG_TO_RAD;
        float accumulated_rotation_y = gyro_int->acc_y * DEG_TO_RAD;
        // 3. Получаем общее время накопления в СЕКУНДАХ
        float time_s = gyro_int->time_us / 1000000.0f; // (например, ~0.02 сек)
        // 4. Находим среднюю угловую скорость (в рад/с) за этот период
        // Средняя Угл. Скорость = Накопленный Угол / Время
        float angular_velocity_x = accumulated_rotation_x / time_s;
        float angular_velocity_y = accumulated_rotation_y / time_s;

        // 5. Расчитываем ложную линейную скорость (в м/с)
        // V (м/с) = w (рад/с) * h (м)
        float V_linear_x_gyro = angular_velocity_x * results->distance_m;
        float V_linear_y_gyro = angular_velocity_y * results->distance_m;
        // 6. Переводим в см/с и временно сохраняем в results
        // Это только та часть скорости, которая вызвана вращением гироскопа.
        results->speed_cm_s_x = V_linear_x_gyro * 100.0f;
        results->speed_cm_s_y = V_linear_y_gyro * 100.0f;
    } else {
        // Если данных нет, считаем ложную скорость нулевой
        results->speed_cm_s_x = 0;
        results->speed_cm_s_y = 0;
    }
}
/////////////////////////////////////////////////////
/**
 * @brief  Вычисляет "ложную" линейную скорость от вращения, используя сохранённые данные интегратора.
 *         Заменяет calculate_linear_velocity при работе с копией накоплений.
 * 
 * @param results         Указатель на структуру с результатами (скорость будет записана в см/с)
 * @param distance        Текущая высота по дальномеру (в мм)
 * @param pitch_deg       Текущий тангаж (градусы)
 * @param roll_deg        Текущий крен (градусы)
 * @param saved_acc_x     Накопленный угол по оси X из gyro_integration (в градусах)
 * @param saved_acc_y     Накопленный угол по оси Y из gyro_integration (в градусах)
 * @param saved_time_us   Накопленное время из gyro_integration (в микросекундах)
 */
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
        // Переводим накопленный угол из градусов в радианы
        float accumulated_rotation_x = saved_acc_x * DEG_TO_RAD;
        float accumulated_rotation_y = saved_acc_y * DEG_TO_RAD;

        // Общее время в секундах
        float time_s = saved_time_us / 1000000.0f;

        // Средняя угловая скорость (рад/с)
        float angular_velocity_x = accumulated_rotation_x / time_s;
        float angular_velocity_y = accumulated_rotation_y / time_s;

        // Линейная скорость от вращения: V = ω * h (м/с)
        float V_linear_x_gyro = angular_velocity_x * results->distance_m;
        float V_linear_y_gyro = angular_velocity_y * results->distance_m;

        // Результат в см/с (как того ожидают остальные части кода)
        results->speed_cm_s_x = V_linear_x_gyro * 100.0f;
        results->speed_cm_s_y = V_linear_y_gyro * 100.0f;
    } else {
        // Нет достоверных данных – обнуляем
        results->speed_cm_s_x = 0.0f;
        results->speed_cm_s_y = 0.0f;
    }
}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
// Компенсация вращения в данных оптического потока
// ЦЕЛЬ:
// Выполнить вычитание:
// [Чистая Скорость] = [Общая Скорость с сенсора] - [Ложная Скорость от гироскопа]
/////////////////////////////////////////////////////
void compensate_rotation_for_optical_flow(OpticalFlowResults_t* results,
                                         Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                                         int16_t flow_velocity_x, int16_t flow_velocity_y)
{
    if (results == NULL || lpf_x == NULL || lpf_y == NULL){
    	return;
    }

    if (results->distance_m > 0.001f) {
        // 1. Рассчитываем общую линейную скорость (в см/с) по данным сенсора
        float mtf_x = (float)flow_velocity_x * results->distance_m;
        float mtf_y = (float)flow_velocity_y * results->distance_m;
        // 2. выполняем компенсацию
        //    results->speed_cm_s_x сейчас содержит ложную скорость (из `calculate_linear_velocity`)
        float compensated_x = mtf_x - results->speed_cm_s_x; // [Общая] - [Ложная]
        float compensated_y = mtf_y - results->speed_cm_s_y; // [Общая] - [Ложная]
        // 3. фильтруем и перезаписываем
        //    Теперь 'compensated_x' - это "Чистая" (реальная) скорость.
        //    Фильтруем ее для сглаживания и записываем обратно
        //    в `results->speed_cm_s_x`.
        //    Теперь эта переменная содержит финальный, чистый результат.
        results->speed_cm_s_x = low_pass_filter(lpf_x, compensated_x);
        results->speed_cm_s_y = low_pass_filter(lpf_y, compensated_y);
    }
}

/////////////////////////////////////////////////////
// Основная функция обработки оптического потока (50 Гц)
// ЦЕЛЬ:
// функция которая запускает все шаги в правильном порядке
// и сбрасывает накопитель.
/////////////////////////////////////////////////////
void process_optical_flow_data(OpticalFlowResults_t* results,
                              GyroIntegration_t* gyro_int,
                              Filter_lpf* lpf_x, Filter_lpf* lpf_y,
                              uint32_t distance,
                              int16_t flow_velocity_x, int16_t flow_velocity_y,
                              float pitch_deg, float roll_deg)
{
    if (results == NULL || gyro_int == NULL) return;
    uint32_t current_time = HAL_GetTick(); // Текущее время в мс
    // Проверяем, прошло ли 20 мс с последней обработки (т.е. работаем на 50 Гц)
    if (!should_process_optical_flow(current_time, results->last_optical_flow_update)) {
        return; // Еще не время, выходим
    }
    // 1. Сохраняем накопления и СРАЗУ сбрасываем интегратор
    float saved_acc_x = gyro_int->acc_x;
    float saved_acc_y = gyro_int->acc_y;
    uint32_t saved_time_us = gyro_int->time_us;
    gyro_integration_reset(gyro_int);   // <-- сброс ДО вычислений
/*
    // 1: Рассчитать ложную скорость по гироскопу
    // (Результат временно сохранится в results->speed_cm_s_x)
    calculate_linear_velocity(gyro_int, results, distance, pitch_deg, roll_deg);
*/
    calculate_linear_velocity_from_saved_data(results, distance,pitch_deg, roll_deg,saved_acc_x, saved_acc_y,saved_time_us);
    // 2: Рассчитать общую скорость, вычесть из нее ложную и
    // отфильтрованный результат чистую скорость сохранить
    // обратно в results->speed_cm_s_x
    compensate_rotation_for_optical_flow(results, lpf_x, lpf_y, flow_velocity_x, flow_velocity_y);
/*    
    // 3: Сбросить накопитель гироскопа.
    // потратили накопленные данные, теперь начинаем копить
    // для следующего 20мс-интервала.
    gyro_integration_reset(gyro_int);
*/    
    //4: Обновить время и выставить флаг
    results->last_optical_flow_update = current_time;
    results->new_optical_data_available = true; // Сигнал для main.c, что данные готовы
}

/////////////////////////////////////////////////////
// Проверка необходимости обработки оптического потока
// ЦЕЛЬ:
// Ограничить частоту выполнения `process_optical_flow_data`
// до 50 Гц (раз в 20 мс).
/////////////////////////////////////////////////////
bool should_process_optical_flow(uint32_t current_time, uint32_t last_processing_time)
{
    // При первом запуске
    if (last_processing_time == 0){
    	return true;
    }
    // Вычисляем, сколько мс прошло с последней обработки
    uint32_t time_since_last_update = current_time - last_processing_time;
    // Если прошло >= 20 мс, возвращаем true
    return (time_since_last_update >= OPTICAL_FLOW_UPDATE_PERIOD_MS);
}
/////////////////////////////////////////////////////
// Вспомогательные функции
/////////////////////////////////////////////////////
// Сброс накопителя гироскопа
void gyro_integration_reset(GyroIntegration_t* gyro_int)
{
    if (gyro_int == NULL){
    	return;
    }
    gyro_int->acc_x = 0;
    gyro_int->acc_y = 0;
    gyro_int->time_us = 0;
    gyro_int->update_count = 0;
    // 'last_update' НЕ сбрасываем, чтобы 'gyro_integration_update'
    // корректно рассчитала dt в следующей итерации.
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
