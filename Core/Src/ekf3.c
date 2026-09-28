#include "ekf3.h"
#include "config_param.h"
#include <math.h>
#include <string.h>

/*
 * Реализация loosely-coupled EKF (4 состояния, random-walk скорость) для
 * оптического потока (MTF).
 *
 * Состояние x = [vn, ve, pn, pe]:
 *   vn,ve — горизонтальная скорость North/East [м/с]
 *   pn,pe — горизонтальная позиция North/East [м]
 *
 * Акселерометр в горизонтальную скорость НЕ интегрируем: g·sin(θ)-протечка от
 * неточной ориентации Mahony на малых скоростях даёт больше вреда, чем пользы.
 * Скорость — random walk, уточняется только измерением потока. Акселерометр
 * подаётся напрямую в D-член скоростного PID (в main.c), без интеграции.
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

enum { VN=0, VE=1, PN=2, PE=3 };

void ekf3_init(EKF3_t *e) {
    memset(e, 0, sizeof(*e));
    e->P[VN*4+VN] = EKF3_P0_VEL;
    e->P[VE*4+VE] = EKF3_P0_VEL;
    e->P[PN*4+PN] = EKF3_P0_POS;
    e->P[PE*4+PE] = EKF3_P0_POS;
    e->initialized = true;
}

void ekf3_reset(EKF3_t *e) {
    e->x[VN] = 0.0f; e->x[VE] = 0.0f;
    e->x[PN] = 0.0f; e->x[PE] = 0.0f;

    memset(e->P, 0, sizeof(e->P));
    e->P[VN*4+VN] = EKF3_P0_VEL;
    e->P[VE*4+VE] = EKF3_P0_VEL;
    e->P[PN*4+PN] = EKF3_P0_POS;
    e->P[PE*4+PE] = EKF3_P0_POS;

    e->vel_earth_mps[0] = e->vel_earth_mps[1] = 0.0f;
    e->pos_earth_m[0]   = e->pos_earth_m[1]   = 0.0f;
    e->vel_body_cms[0]  = e->vel_body_cms[1]  = 0.0f;
    e->innovation[0]    = e->innovation[1]    = 0.0f;
    e->bias_mss[0]      = e->bias_mss[1]      = 0.0f;
}

void ekf3_predict(EKF3_t *e, float ax, float ay, float az, const float q[4], float dt) {
    (void)ax; (void)ay; (void)az;  // random-walk: ускорение в скорость не интегрируем
    if (!e->initialized) return;

    Rot_t R = quat_to_dcm(q);

    // Скорость в прогнозе не меняется (random walk); позиция интегрируется из скорости.
    e->x[PN] += e->x[VN] * dt;
    e->x[PE] += e->x[VE] * dt;

    // F (4x4), constant velocity: vn'=vn, ve'=ve, pn'=pn+vn*dt, pe'=pe+ve*dt
    float Pn[16];
    for (int c = 0; c < 4; c++) Pn[VN*4+c] = e->P[VN*4+c];
    for (int c = 0; c < 4; c++) Pn[VE*4+c] = e->P[VE*4+c];
    for (int c = 0; c < 4; c++) Pn[PN*4+c] = e->P[PN*4+c] + dt*e->P[VN*4+c];
    for (int c = 0; c < 4; c++) Pn[PE*4+c] = e->P[PE*4+c] + dt*e->P[VE*4+c];

    float Pp[16];
    for (int r = 0; r < 4; r++) {
        Pp[r*4+VN] = Pn[r*4+VN];
        Pp[r*4+VE] = Pn[r*4+VE];
        Pp[r*4+PN] = Pn[r*4+PN] + dt*Pn[r*4+VN];
        Pp[r*4+PE] = Pn[r*4+PE] + dt*Pn[r*4+VE];
    }

    // Q: шум процесса
    float qv = EKF3_Q_VEL * dt;       // (м/с)² за шаг — random walk скорости
    float qp = EKF3_Q_POS * dt * dt;  // м² за шаг
    Pp[VN*4+VN] += qv;
    Pp[VE*4+VE] += qv;
    Pp[PN*4+PN] += qp;
    Pp[PE*4+PE] += qp;

    memcpy(e->P, Pp, sizeof(Pp));

    e->vel_earth_mps[0] = e->x[VN];
    e->vel_earth_mps[1] = e->x[VE];
    e->pos_earth_m[0]   = e->x[PN];
    e->pos_earth_m[1]   = e->x[PE];
    e->bias_mss[0]      = 0.0f;
    e->bias_mss[1]      = 0.0f;

    // Тело-скорость для speed-PID: v_body = R^T * [vn, ve, 0] → см/с
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

    // Ложная скорость от вращения ω×h (неперекрёстная связка, gain откалиброван)
    float v_rot_x = EKF3_ROT_SIGN_X * gx * height_m * EKF3_ROT_COMP_GAIN;
    float v_rot_y = EKF3_ROT_SIGN_Y * gy * height_m * EKF3_ROT_COMP_GAIN;

    float v_body_x = v_flow_x - v_rot_x;
    float v_body_y = v_flow_y - v_rot_y;

    // Переводим измерение в earth-фрейм (только горизонталь)
    float z_n = R.r[0][0]*v_body_x + R.r[0][1]*v_body_y;
    float z_e = R.r[1][0]*v_body_x + R.r[1][1]*v_body_y;

    // Коррекция Калмана (H = [I 0], измеряем только скорость)
    float innov_n = z_n - e->x[VN];
    float innov_e = z_e - e->x[VE];
    e->innovation[0] = innov_n;
    e->innovation[1] = innov_e;

    // R-шум измерения зависит от качества
    float rn = EKF3_R_FLOW * (255.0f / (float)(quality > 0 ? quality : 1));
    if (rn > EKF3_R_FLOW_MAX) rn = EKF3_R_FLOW_MAX;

    float S00 = e->P[VN*4+VN] + rn;
    float S01 = e->P[VN*4+VE];
    float S10 = e->P[VE*4+VN];
    float S11 = e->P[VE*4+VE] + rn;
    float det = S00*S11 - S01*S10;
    if (fabsf(det) < 1e-12f) return;
    float inv00 =  S11 / det;
    float inv01 = -S01 / det;
    float inv10 = -S10 / det;
    float inv11 =  S00 / det;

    // K = P H^T S^-1
    float K[4][2];
    for (int r = 0; r < 4; r++) {
        K[r][0] = e->P[r*4+VN]*inv00 + e->P[r*4+VE]*inv10;
        K[r][1] = e->P[r*4+VN]*inv01 + e->P[r*4+VE]*inv11;
    }

    for (int r = 0; r < 4; r++) {
        e->x[r] += K[r][0]*innov_n + K[r][1]*innov_e;
    }

    float Ppost[16];
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            Ppost[r*4+c] = e->P[r*4+c]
                         - K[r][0]*e->P[VN*4+c]
                         - K[r][1]*e->P[VE*4+c];
        }
    }
    memcpy(e->P, Ppost, sizeof(Ppost));

    e->vel_earth_mps[0] = e->x[VN];
    e->vel_earth_mps[1] = e->x[VE];
    e->pos_earth_m[0]   = e->x[PN];
    e->pos_earth_m[1]   = e->x[PE];
    e->bias_mss[0]      = 0.0f;
    e->bias_mss[1]      = 0.0f;

    float vbx = R.r[0][0]*e->x[VN] + R.r[1][0]*e->x[VE];
    float vby = R.r[0][1]*e->x[VN] + R.r[1][1]*e->x[VE];
    e->vel_body_cms[0] = vbx * 100.0f;
    e->vel_body_cms[1] = vby * 100.0f;
}
