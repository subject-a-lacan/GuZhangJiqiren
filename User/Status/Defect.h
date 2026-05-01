#ifndef __DEFECT_H
#define __DEFECT_H

#include "stdint.h"

typedef struct STATUS STATUS;

typedef enum TASK_ID {
  TASK_BASIC_1 = 1,
  TASK_BASIC_2 = 2,
  TASK_ADV_1 = 3,
  TASK_ADV_2 = 4,
} TASK_ID;

typedef enum START_POSE {
  START_AB = 0,
  START_AD = 1,
} START_POSE;

/* Q1 race phases */
typedef enum Q1_RACE_PHASE {
  Q1_START_A_TURN,
  Q1_SIDE_AD,
  Q1_TURN_D,
  Q1_SIDE_DC,
  Q1_TURN_C,
  Q1_SIDE_CB,
  Q1_TURN_B,
  Q1_BA_FINAL,
} Q1_RACE_PHASE;

/* Q2/Q3/Q4 race phases — to be defined per-task */

typedef struct TASK {
  uint8_t task_id;     //任务编号
  uint8_t start_pose;  //起始位姿 第二问 第三问 要用
  uint8_t race_phase;  //每一阶段的控制

  uint8_t cross_cnt;   //已通过的路口计数

  uint8_t armed;       //出发允许
  uint8_t task_running;  //表示任务进行的标志位

  uint8_t start_request;  //出发请求 按钮和蓝牙只改动start_request 中断里判断后再给armed置1
  uint8_t stop_request;   //停止请求  不过感觉有点多余 Cz命令之后根本不会进task running标志位都置1了

  uint32_t phase_start_time;
  float phase_mileage;
} TASK;

void init_task(TASK *task);
// void update_task(STATUS *status);
void task_start(STATUS *status);  //每次进任务都初始化 避免上一次任务污染这一次
// void task_finish(STATUS *status);

#endif
