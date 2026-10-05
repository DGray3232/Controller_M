#ifndef EKF3_H
#define EKF3_H

#include <stdint.h>
#include <stdbool.h>

/*
 * EKF3 — loosely-coupled Kalman filter для MTF (оптический поток).
 *
 * Что оценивает: горизонтальную скорость и позицию в EARTH (NED) фрейме +
 * смещение акселерометра в земной СК (поглощает g·sin(θ)-протечку Mahony).
 * Ориентацию НЕ оценивает — берёт кватернион Mahony извне (Quat_actual).
 *
 * Состояние x = [vn, ve, pn, pe, abn, abe]  (6 состояний):
 *   vn,ve   — горизонтальная скорость North/East [м/с]
 *   pn,pe   — горизонтальная позиция North/East [м]
 *   abn,abe — смещение горизонтального ускорения North/East [м/с²].
 *             Ошибка ориентации Mahony (1–2°) проецирует g в горизонталь
 *             (g·sin(θ)-протечка) и выглядит как постоянное смещение аксела
 *             в земной СК — фильтр оценивает и вычитает его.
 *
 * Прогноз (1 кГц): скорость интегрируется из акселерометра (повёрнутого в earth)
 *                  за вычетом оценённого смещения; позиция — из скорости.
 * Коррекция (50 Гц): оптический поток (raw минус ω×h) как измерение скорости.
 * Невязка потока обновляет и смещение аксела (через кросс-ковариацию).
 *
 * Единицы внутри: м, м/с, м/с². Наружу: см/с для PID.
 */

#define EKF3_N 6

typedef struct {
    float x[EKF3_N];             // [vn, ve, pn, pe, abn, abe]
    float P[EKF3_N * EKF3_N];    // ковариация, row-major

    // Выходы (для control + логирования)
    float vel_earth_mps[2];   // vn, ve [м/с]
    float pos_earth_m[2];     // pn, pe [м]
    float vel_body_cms[2];    // скорость в body [см/с] (для speed-PID)
    float bias_mss[2];        // оценённое смещение аксела abn, abe [м/с²] (в blackbox)
    float innovation[2];      // невязка измерения потока [м/с] — диагностика

    bool  initialized;
} EKF3_t;

/* Инициализация: сброс состояния и ковариации */
void ekf3_init(EKF3_t *e);

/* Сброс позиции/скорости (при входе в MTF/ALT_HOLD и на взлёте).
 * Смещение аксела (abn, abe) СОХРАНЯЕТСЯ — оно накоплено и не зависит от точки взлёта. */
void ekf3_reset(EKF3_t *e);

/*
 * Прогноз на частоте контура (1 кГц).
 *  ax,ay,az — акселерометр (удельная сила) в body [м/с²]
 *  q[4]     — кватернион body→earth [w,x,y,z]
 *  dt       — шаг [с]
 */
void ekf3_predict(EKF3_t *e, float ax, float ay, float az, const float q[4], float dt);

/*
 * Коррекция по оптическому потоку (по приходу кадра, ~50 Гц).
 *  flow_x, flow_y — raw поток из MTF-02 [(см/с)/м]
 *  height_m       — высота (LiDAR с поправкой на наклон) [м]
 *  q[4]           — кватернион body→earth
 *  gx, gy, gz     — угловая скорость body [рад/с] (для ω×h)
 *  quality        — flow_quality 0..255 (ниже порога — пропуск)
 */
void ekf3_update_flow(EKF3_t *e, int16_t flow_x, int16_t flow_y, float height_m,
                      const float q[4], float gx, float gy, float gz, uint8_t quality);

#endif /* EKF3_H */
