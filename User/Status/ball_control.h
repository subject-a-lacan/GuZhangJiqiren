#ifndef __BALL_CONTROL_H
#define __BALL_CONTROL_H

#include <stdint.h>

struct STATUS;

/* ── 机械参数 ── */
#define BALL_STEPPER_ZERO_PULSE (3876)       /* 水平零点脉冲数 (54.5°)                   */
#define BALL_CONTROL_REL_PULSE_MIN (-2500)   /* 相对脉冲下限 (查表范围)                  */
#define BALL_CONTROL_REL_PULSE_MAX (6908)    /* 相对脉冲上限 (查表范围)                  */
#define BALL_CONTROL_CAR_ACCEL_LIMIT_MM_S2 (9810.0f) /* 小车加速度限幅 (mm/s²)           */
#define BALL_STEPPER_VELOCITY (50u)          /* 步进电机转速 (RPM)                       */
#define BALL_STEPPER_ACCEL_PARAM (0u)        /* 步进加速参数 (0=无曲线)                  */

/* ── PD 参数 ── */
#define BALL_CONTROL_KP (6.0f)               /* 位置增益 (s⁻²)                           */
#define BALL_CONTROL_KD (1.5f)               /* 速度阻尼 (s⁻¹)                           */
#define BALL_CONTROL_KI (20.0f)              /* 积分增益 (s⁻³)                           */
#define BALL_I_MAX_MM_S2  (200.0f)           /* 积分限幅 (mm/s²)                         */

/* ── α-β 估计器 ── */
#define BALL_ESTIMATOR_ALPHA (0.6f)          /* 位置滤波系数                             */
#define BALL_ESTIMATOR_BETA (0.1f)           /* 速度滤波系数                             */
#define BALL_ESTIMATOR_MAX_DT_MS (200u)      /* 帧间隔超时 (ms)                          */

/* ── 变速积分：误差权重 ── */
#define BALL_I_FULL_ERROR_MM   (20.0f)       /* |e|≤20mm 全速积分                        */
#define BALL_I_OFF_ERROR_MM    (45.0f)       /* |e|≥45mm 关闭积分                        */
/* ── 变速积分：速度权重 ── */
#define BALL_I_FULL_SPEED_MM_S   (3.0f)      /* |v|≤3mm/s 全速积分                       */
#define BALL_I_OFF_SPEED_MM_S   (12.0f)      /* |v|≥12mm/s 关闭积分                      */
#define BALL_I_RELEASE_SPEED_MM_S (5.0f)     /* |v|≥5mm/s 且向目标运动 → 快速泄放积分     */
#define BALL_I_CONFIRM_S         (0.10f)     /* 积分确认时间 (s)                          */
#define BALL_I_DECAY_PER_S       (8.0f)      /* 积分衰减速率 (/s)，球运动时快速释放       */

/* ── 停车死区 (带回差，防边界抖动) ── */
#define BALL_DEAD_ENTER_ERROR_MM   (7.0f)    /* 进入死区：误差 ≤8mm                       */
#define BALL_DEAD_ENTER_SPEED_MM_S (12.0f)   /* 进入死区：速度 ≤12mm/s                    */
#define BALL_DEAD_EXIT_ERROR_MM    (9.0f)   /* 退出死区：误差 ≥12mm                      */
#define BALL_DEAD_EXIT_SPEED_MM_S  (25.0f)   /* 退出死区：速度 ≥25mm/s                    */
#define BALL_DEAD_CONFIRM_S        (0.10f)   /* 死区确认时间 (s)                          */

void ball_control_init(struct STATUS *status);
void ball_control_request(struct STATUS *status, float target_mm,
                          float car_accel_mm_s2);
void ball_control_disable(struct STATUS *status);
void ball_control_service(struct STATUS *status);

#endif
