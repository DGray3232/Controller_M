#include "ekf3.h"
#include "config_param.h"
#include <math.h>
#include <string.h>

/*
 * Реализация loosely-coupled EKF (6 состояний: скорость + позиция + смещение аксела).
 *
 * Состояние x = [vn, ve, pn, pe, abn, abe]:
 *   vn,ve   — горизонтальная скорость North/East [м/с]
 *   pn,pe   — горизонтальная позиция North/East [м]
 *   abn,abe — смещение горизонтального ускорения North/East [м/с²]
 *
 * Скорость интегрируется из акселерометра (повёрнутого в earth), а не random-walk:
 *   v += (R·f_body)_horiz · dt  −  bias · dt
 * Смещение аксела поглощает g·sin(θ)-протечку Mahony (ошибка ориентации 1–2°
 * проецирует гравитацию в горизонталь как «постоянное» смещение в земной СК).
 * Поток на 50 Гц корректирует скорость/позицию и, через кросс-ковариацию, смещение.
 *
 * Матрица поворота body→earth из кватерниона Mahony [q0,q1,q2,q3]=[w,x,y,z]:
 *
 *   R = [ 1-2(q2²+q3²)   2(q1q2-q0q3)   2(q1q3+q0q2) ]
 *       [ 2(q1q2+q0q3)   1-2(q1²+q3²)   2(q2q3-q0q1) ]
 *       [ 2(q1q3-q0q2)   2(q2q3+q0q1)   1-2(q1²+q2²) ]
 */

typedef struct { float r[3][3]; } Rot_t;

static Rot_t quat_to_dcm(const float q[4]) {
    Rot_t R;
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    R.r[0][0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R.r[0][1] = 2.0f*(q1*q2 - q0*q3);
    R.r[0][2] = 2.0f*(q1*q3 + q0*q2);
    R.r[1][0] = 2.0f*(q1*q2 + q0*q3);
    R.r[1][1] = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R.r[1][2] = 2.0f*(q2*q3 - q0*q1);
    R.r[2][0] = 2.0f*(q1*q3 - q0*q2);
    R.r[2][1] = 2.0f*(q2*q3 + q0*q1);
    R.r[2][2] = 1.0f - 2.0f*(q1*q1 + q2*q2);
    return R;
}

enum { VN=0, VE=1, PN=2, PE=3, ABN=4, ABE=5 };

void ekf3_init(EKF3_t *e) {
    memset(e, 0, sizeof(*e));
    e->P[VN*EKF3_N+VN]   = EKF3_P0_VEL;
    e->P[VE*EKF3_N+VE]   = EKF3_P0_VEL;
    e->P[PN*EKF3_N+PN]   = EKF3_P0_POS;
    e->P[PE*EKF3_N+PE]   = EKF3_P0_POS;
    e->P[ABN*EKF3_N+ABN] = EKF3_P0_BIAS;
    e->P[ABE*EKF3_N+ABE] = EKF3_P0_BIAS;
    e->initialized = true;
}

void ekf3_reset(EKF3_t *e) {
    e->x[VN] = 0.0f; e->x[VE] = 0.0f;
    e->x[PN] = 0.0f; e->x[PE] = 0.0f;
    // Смещение аксела (ABN, ABE) СОХРАНЯЕМ — оценка накоплена и не зависит от точки взлёта.

    memset(e->P, 0, sizeof(e->P));
    e->P[VN*EKF3_N+VN]   = EKF3_P0_VEL;
    e->P[VE*EKF3_N+VE]   = EKF3_P0_VEL;
    e->P[PN*EKF3_N+PN]   = EKF3_P0_POS;
    e->P[PE*EKF3_N+PE]   = EKF3_P0_POS;
    e->P[ABN*EKF3_N+ABN] = EKF3_P0_BIAS;
    e->P[ABE*EKF3_N+ABE] = EKF3_P0_BIAS;

    e->vel_earth_mps[0] = e->vel_earth_mps[1] = 0.0f;
    e->pos_earth_m[0]   = e->pos_earth_m[1]   = 0.0f;
    e->vel_body_cms[0]  = e->vel_body_cms[1]  = 0.0f;
    e->innovation[0]    = e->innovation[1]    = 0.0f;
    e->bias_mss[0]      = e->x[ABN];
    e->bias_mss[1]      = e->x[ABE];
}

void ekf3_predict(EKF3_t *e, float ax, float ay, float az, const float q[4], float dt) {
    if (!e->initialized) return;

    Rot_t R = quat_to_dcm(q);

    // Горизонтальное ускорение в earth = (R·f_body)_horiz. Гравитация лежит на
    // вертикальной оси earth-фрейма, поэтому на горизонталь не влияет — берём
    // просто первые две строки R (без вычитания g).
    float a_n = R.r[0][0]*ax + R.r[0][1]*ay + R.r[0][2]*az;
    float a_e = R.r[1][0]*ax + R.r[1][1]*ay + R.r[1][2]*az;

    // Позиция интегрируется из СТАРОЙ скорости (до обновления акселом),
    // чтобы F-матрица (∂pn'/∂vn = dt) была согласована с шагом.
    e->x[PN] += e->x[VN] * dt;
    e->x[PE] += e->x[VE] * dt;

    // Скорость интегрируется из ускорения за вычетом оценённого смещения.
    e->x[VN] += (a_n - e->x[ABN]) * dt;
    e->x[VE] += (a_e - e->x[ABE]) * dt;
    // Смещение — random walk (среднее не меняется, шум добавляется в Q).

    // --- Ковариация: P' = F·P·F^T + Q ---
    // F (row-major), ненулевые элементы:
    //   vn: [1, 0, 0, 0, -dt, 0]
    //   ve: [0, 1, 0, 0, 0, -dt]
    //   pn: [dt, 0, 1, 0, 0, 0]
    //   pe: [0, dt, 0, 1, 0, 0]
    //   abn, abe: единичные строки
    float Pn[EKF3_N*EKF3_N];
    for (int c = 0; c < EKF3_N; c++) {
        Pn[VN*EKF3_N+c]  = e->P[VN*EKF3_N+c]  - dt*e->P[ABN*EKF3_N+c];
        Pn[VE*EKF3_N+c]  = e->P[VE*EKF3_N+c]  - dt*e->P[ABE*EKF3_N+c];
        Pn[PN*EKF3_N+c]  = e->P[PN*EKF3_N+c]  + dt*e->P[VN*EKF3_N+c];
        Pn[PE*EKF3_N+c]  = e->P[PE*EKF3_N+c]  + dt*e->P[VE*EKF3_N+c];
        Pn[ABN*EKF3_N+c] = e->P[ABN*EKF3_N+c];
        Pn[ABE*EKF3_N+c] = e->P[ABE*EKF3_N+c];
    }
    float Pp[EKF3_N*EKF3_N];
    for (int r = 0; r < EKF3_N; r++) {
        Pp[r*EKF3_N+VN]  = Pn[r*EKF3_N+VN]  - dt*Pn[r*EKF3_N+ABN];
        Pp[r*EKF3_N+VE]  = Pn[r*EKF3_N+VE]  - dt*Pn[r*EKF3_N+ABE];
        Pp[r*EKF3_N+PN]  = Pn[r*EKF3_N+PN]  + dt*Pn[r*EKF3_N+VN];
        Pp[r*EKF3_N+PE]  = Pn[r*EKF3_N+PE]  + dt*Pn[r*EKF3_N+VE];
        Pp[r*EKF3_N+ABN] = Pn[r*EKF3_N+ABN];
        Pp[r*EKF3_N+ABE] = Pn[r*EKF3_N+ABE];
    }

    // Q — диагональный шум процесса
    float qv = EKF3_Q_VEL * dt;        // немоделируемое ускорение -> скорость
    float qp = EKF3_Q_POS * dt * dt;   // позиция
    float qb = EKF3_Q_BIAS * dt;       // дрейф смещения аксела
    Pp[VN*EKF3_N+VN]   += qv;
    Pp[VE*EKF3_N+VE]   += qv;
    Pp[PN*EKF3_N+PN]   += qp;
    Pp[PE*EKF3_N+PE]   += qp;
    Pp[ABN*EKF3_N+ABN] += qb;
    Pp[ABE*EKF3_N+ABE] += qb;

    memcpy(e->P, Pp, sizeof(Pp));

    // Выходы
    e->vel_earth_mps[0] = e->x[VN];
    e->vel_earth_mps[1] = e->x[VE];
    e->pos_earth_m[0]   = e->x[PN];
    e->pos_earth_m[1]   = e->x[PE];
    e->bias_mss[0]      = e->x[ABN];
    e->bias_mss[1]      = e->x[ABE];

    // Тело-скорость для speed-PID: v_body = R^T·[vn, ve, 0] → см/с
    float vbx = R.r[0][0]*e->x[VN] + R.r[1][0]*e->x[VE];
    float vby = R.r[0][1]*e->x[VN] + R.r[1][1]*e->x[VE];
    e->vel_body_cms[0] = vbx * 100.0f;
    e->vel_body_cms[1] = vby * 100.0f;
}

void ekf3_update_flow(EKF3_t *e, int16_t flow_x, int16_t flow_y, float height_m,
                      const float q[4], float gx, float gy, float gz, uint8_t quality) {
    if (!e->initialized) return;
    if (height_m < (MTF_MIN_VALID_ALTITUDE_MM / 1000.0f)) return; // ниже 8 см данные MTF мусорные
    if (quality < EKF3_OF_QUALITY_MIN) return; // плохое качество — пропуск кадра
    (void)gz;

    Rot_t R = quat_to_dcm(q);

    // Измерение скорости потока в body [м/с]: flow [(см/с)/м] * height [м] /100
    float fvx = (float)flow_x;
    float fvy = (float)flow_y;
#if EKF3_FLOW_SWAP
    float t = fvx; fvx = fvy; fvy = t;
#endif
    float v_flow_x = fvx * EKF3_FLOW_SIGN_X * height_m * 0.01f;
    float v_flow_y = fvy * EKF3_FLOW_SIGN_Y * height_m * 0.01f;

    // Ложная скорость от вращения ω×h (gain откалиброван)
    float v_rot_x = EKF3_ROT_SIGN_X * gx * height_m * EKF3_ROT_COMP_GAIN_X;
    float v_rot_y = EKF3_ROT_SIGN_Y * gy * height_m * EKF3_ROT_COMP_GAIN_Y;

    float v_body_x = v_flow_x - v_rot_x;
    float v_body_y = v_flow_y - v_rot_y;

    // Переводим измерение в earth-фрейм (только горизонталь)
    float z_n = R.r[0][0]*v_body_x + R.r[0][1]*v_body_y;
    float z_e = R.r[1][0]*v_body_x + R.r[1][1]*v_body_y;

    // Коррекция Калмана (H = [I 0 0], измеряем только скорость)
    float innov_n = z_n - e->x[VN];
    float innov_e = z_e - e->x[VE];
    e->innovation[0] = innov_n;
    e->innovation[1] = innov_e;

    // R-шум измерения зависит от качества
    float rn = EKF3_R_FLOW * (255.0f / (float)(quality > 0 ? quality : 1));
    if (rn > EKF3_R_FLOW_MAX) rn = EKF3_R_FLOW_MAX;

    // S = H·P·H^T + R (2x2)
    float S00 = e->P[VN*EKF3_N+VN] + rn;
    float S01 = e->P[VN*EKF3_N+VE];
    float S10 = e->P[VE*EKF3_N+VN];
    float S11 = e->P[VE*EKF3_N+VE] + rn;
    float det = S00*S11 - S01*S10;
    if (fabsf(det) < 1e-12f) return;
    float inv00 =  S11 / det;
    float inv01 = -S01 / det;
    float inv10 = -S10 / det;
    float inv11 =  S00 / det;

    // K = P·H^T·S^-1 (6x2)
    float K[EKF3_N][2];
    for (int r = 0; r < EKF3_N; r++) {
        K[r][0] = e->P[r*EKF3_N+VN]*inv00 + e->P[r*EKF3_N+VE]*inv10;
        K[r][1] = e->P[r*EKF3_N+VN]*inv01 + e->P[r*EKF3_N+VE]*inv11;
    }

    // x += K·innovation (все 6 состояний, включая смещение)
    for (int r = 0; r < EKF3_N; r++) {
        e->x[r] += K[r][0]*innov_n + K[r][1]*innov_e;
    }

    // P = (I - K·H)·P
    float Ppost[EKF3_N*EKF3_N];
    for (int r = 0; r < EKF3_N; r++) {
        for (int c = 0; c < EKF3_N; c++) {
            Ppost[r*EKF3_N+c] = e->P[r*EKF3_N+c]
                              - K[r][0]*e->P[VN*EKF3_N+c]
                              - K[r][1]*e->P[VE*EKF3_N+c];
        }
    }
    memcpy(e->P, Ppost, sizeof(Ppost));

    // Выходы
    e->vel_earth_mps[0] = e->x[VN];
    e->vel_earth_mps[1] = e->x[VE];
    e->pos_earth_m[0]   = e->x[PN];
    e->pos_earth_m[1]   = e->x[PE];
    e->bias_mss[0]      = e->x[ABN];
    e->bias_mss[1]      = e->x[ABE];

    float vbx = R.r[0][0]*e->x[VN] + R.r[1][0]*e->x[VE];
    float vby = R.r[0][1]*e->x[VN] + R.r[1][1]*e->x[VE];
    e->vel_body_cms[0] = vbx * 100.0f;
    e->vel_body_cms[1] = vby * 100.0f;
}
