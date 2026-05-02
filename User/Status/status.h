// @63 @551

#ifndef __STATUS_H
#define __STATUS_H

#include "button.h"
#include "buzzer.h"
#include "Defect.h"
#include "gw_anagloge.h"
#include "gy901.h"
#include "led.h"
#include "main.h"
#include "pid.h"
#include "servo.h"
#include "wheel.h"

#define MOTION_BASE_SPEED 2000

typedef struct STATUS_PID {
  PID follow_line_pid;
  PID keep_angle_pid;  // PID结构体
} STATUS_PID;

typedef enum MOTION_STATION {
  STOP,
  KEEP_ANGLE,
  FIND_LINE,
  MOTOR_TEST,
} MOTION_STATION;
/*
 * @brief 传感器结构体
 * @param gy901 GY901陀螺仪
 * @param gw_8bit GW_8BIT 8位传感器
 * @param gw_analogue GW_ANALOGUE 模拟传感器
 * typedef struct GYR {
  uint8_t data_buf[24];      // 读取数据暂存
  uint8_t device_addr;       // 设备iic地址 默认0xa1
  uint8_t data_start_addr;   // gy901数据寄存器起始地址 默认0x34
  PID gy901_keep_angle_pid;  // 陀螺仪保持角度PID
} GYR;

typedef struct GW_8BIT {
  uint8_t data_buf;          // 8路巡线数字量原始位图
  int16_t gw_bit_weight[8];  // 8路传感器权重表
  uint8_t integral;          // 路口判定积分缓存
  uint8_t maybe;             // 路口候选计数器
  uint8_t cross_cnt;         // 已识别路口计数
  Road cross;                // 当前道路类型
  PID gw_find_line_pid;      // 巡线PID参数与状态
  int32_t gw_diff;           // 计算出的线偏差
} GW_8BIT;

  typedef struct GW_ANALOGUE {
  uint8_t channel[8];                 // 0-7
  uint8_t sta;                        // 0工作模式 1校准模式
  uint8_t correction_data_w[8];       // 白色校准数据
  uint8_t correction_data_b[8];       // 黑色校准数据
  uint8_t digital_8bit;               // 8bit数字量
  uint8_t digital_high_threshold[8];  // 8bit高阈值
  uint8_t digital_low_threshold[8];   // 8bit低阈值
  float diff;

} GW_ANALOGUE;
 */
typedef struct SENSOR {
  GYR gy901;
  GW_ANALOGUE gw_analogue;
} SENSOR;
/*
LED:which LED编号 High_level_is_on 用于设置该led是高电平亮还是低电平亮 on 设置LED亮灭
BUZZER:which LED编号 High_level_is_on 用于设置该led是高电

*/
typedef struct DEVICE {
  LED led_on_board;
  LED led1;
  LED led2;
  BUTTON button_D2;
  BUTTON button_B11;
  BUZZER buzzer;
} DEVICE;

/*
wheel:which 轮子编号 trust 推力值 dir 方向 cur_speed 当前速度 tar_speed 目标速度 wheel_pid 轮子PID参数与状态
servo:which 舵机编号 angle 舵机目标角度 max_angle 舵机最大角度 如180 270
*/
typedef struct MOTOR {
  WHEEL wheel[4];
  SERVO servo[2];
} MOTOR;

typedef struct STATE {
  int8_t T;  // 系统周期单位ms
  uint64_t time;

  MOTION_STATION motion;  //小车运动模式
  float initial_angle;    
  float cur_angle;
  float tar_angle;

  int16_t base_speed;  // 基础速度

  uint8_t gw_8bit;

  STATUS_PID status_pid;  // PID结构体
} STATE;

typedef struct STATUS {
  STATE state;
  SENSOR sensor;  // 传感器数据
  MOTOR motor;    // 电机数据
  DEVICE device;
  TASK task;
} STATUS;

extern STATUS status;

void after_init_state();
void init_status(STATUS *status, uint8_t T);
void update_status(STATUS *status);
void driver_status(STATUS *status);

#endif
