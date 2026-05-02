// @551

#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct PID {
  float kp;
  float ki;
  float kd;
  float T;  // 采样周期
  float error;
  float last_error;
  float integral;
  float derivative;
  float integral_max;
  float InteralCoef;  // 变速积分系数: k=1/(1+InteralCoef*|error|), 0=关闭
  uint8_t is_first;
  float out;
} PID;

// PID 初始化函数 参数 T: pid周期 单位ms integral_max: 积分限幅
PID init_pid(float kp, float ki, float kd, float T, float integral_max, float InteralCoef);
// PID 计算函数 参数 error: 误差
float compute_pid(PID *pid, float error);

/* 编码器脉冲 → 距离(cm) 转换参数 */
#define ENCODER_PPR 13                    // 编码器线数
#define GEAR_RATIO 28.0f                  // 减速比 1:28（电机:车轮）
#define WHEEL_DIAMETER_CM 6.723f          // 轮子直径(cm) 67.23mm
#define WHEEL_CIRCUMFERENCE_CM (3.1415926f * WHEEL_DIAMETER_CM)
#define PULSES_PER_WHEEL_REV (ENCODER_PPR * 4.0f * GEAR_RATIO)  // TI12 四倍频
#define CM_PER_PULSE (WHEEL_CIRCUMFERENCE_CM / PULSES_PER_WHEEL_REV)

float encoder_pulse_to_cm(int32_t pulse);

/*
使用示例：
PID pid = init_pid(1, 0.1, 0.1, 0.1, 100, 0.0f);

while(1) {
  float error = get_aerror();
  float output = compute_pid(&pid, error);
  set_output(output);
  HAL_Delay(100);
  }
*/

#endif
