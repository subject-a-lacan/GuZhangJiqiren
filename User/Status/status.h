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

// ── 可调参数（来自 xiao 项目）──
#define LINE_DIFF_LIMIT  35    // 巡线差速限幅
#define TURN_DIFF_LIMIT  35    // 转弯差速限幅
#define SEARCH_DIFF      15    // 角度到位没线→继续转的差速
#define TURN_BASE_SPEED  35    // 转弯基础速度
#define INIT_TURN_DIFF   25    // 初始转弯差速限幅(原地转)
#define ERR_THRESHOLD    5.0f  // 角度到位容差(度)
#define STARTUP_DELAY    2000  // 上电冷却(ms)
#define COOLDOWN_MS      200   // 转弯后冷却(ms) - 只防同一个路口重复触发，里程屏蔽兜底
#define LOST_LINE_TRIG   3     // 丢线N周期→启动转弯(20ms*N)
#define LINE_BACK_CNT    3     // 连续看到线N周期→切回巡线

typedef struct STATUS_PID {
  PID follow_line_pid;
  PID keep_angle_pid;
} STATUS_PID;

typedef enum MOTION_STATION {
  STOP,
  KEEP_ANGLE,
  FIND_LINE,
  MOTOR_TEST,
} MOTION_STATION;

typedef struct TURN_CTX {
  float target_yaw;      // 目标yaw角度(度)
  float entry_yaw;       // 转弯起始yaw角度(度)
  uint8_t search_active; // 角度到位但没找到线→继续搜线
  float search_dir;      // 搜线方向 (-1左 1右)
} TURN_CTX;

typedef struct SENSOR {
  GYR gy901;
  GW_ANALOGUE gw_analogue;
} SENSOR;

typedef struct DEVICE {
  LED led_on_board;
  LED led1;
  LED led2;
  BUTTON button_D2;
  BUTTON button_B11;
  BUZZER buzzer;
} DEVICE;

typedef struct MOTOR {
  WHEEL wheel[4];
  SERVO servo[2];
} MOTOR;

typedef struct STATE {
  int8_t T;
  uint64_t time;

  MOTION_STATION motion;
  float initial_angle;
  float cur_angle;
  float tar_angle;

  int16_t base_speed;

  TURN_CTX turn;           // 转弯上下文（来自xiao）

  STATUS_PID status_pid;

  uint8_t motion_bypass;   // 1=Defect.c 直接控制轮速，跳过 motion_execute
} STATE;

struct STATUS {
  STATE state;
  SENSOR sensor;
  MOTOR motor;
  DEVICE device;
  TASK task;
};

extern STATUS status;

void after_init_state();
void init_status(STATUS *status, uint8_t T);
void update_status(STATUS *status);
void driver_status(STATUS *status);

// 公共运动执行层
void motion_execute(STATUS *status);

#endif
