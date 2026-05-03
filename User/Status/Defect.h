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
  Q1_START_TO_A,  /* 从发车点走到 A 点附近 */
  Q1_TURN_A,      /* A 点左转 90° */
  Q1_FIND_AD,     /* A 转完后找 AD 线 */
  Q1_SIDE_AD,     /* AD 边巡线等 D 点 */
  Q1_TURN_D,      /* D 点左转 90° */
  Q1_FIND_DC,     /* D 转完后找 DC 线 */
  Q1_SIDE_DC,     /* DC 边巡线等 C 点 */
  Q1_TURN_C,      /* C 点左转 90° */
  Q1_FIND_CB,     /* C 转完后找 CB 线 */
  Q1_SIDE_CB,     /* CB 边巡线等 B 点 */
  Q1_TURN_B,      /* B 点左转 90° */
  Q1_FIND_BA,     /* B 转完后找 BA 线 */
  Q1_BA_FINAL,    /* BA 最后一段回到停车点 */
} Q1_RACE_PHASE;

/* Q2/Q3/Q4 race phases — to be defined per-task */

typedef struct TASK {
  uint8_t task_id;     //任务编号
  uint8_t start_pose;  //起始位姿 第二问 第三问 要用
  uint8_t race_phase;  //每一阶段的控制

  uint8_t cross_cnt;   //已通过的路口计数
  uint8_t cnt_seen;    //防止同一路口重复消费: 0=未消费 1=已消费 Straight时清零

  uint8_t armed;       //出发允许
  uint8_t task_running;  //表示任务进行的标志位

  uint8_t task_select_request;  // 请求切换任务标志位
  uint8_t requested_task_id;    // 任务切换的目标任务
  uint8_t pose_switch_request;  // 请求切换 AB/AD 发车模式标志位

  uint8_t start_request;  //出发请求 按钮和蓝牙只改动start_request 中断里判断后再给armed置1
  uint8_t stop_request;   //停止请求  不过感觉有点多余 Cz命令之后根本不会进task running标志位都置1了
  uint8_t stop_cmd;       //硬停止命令 1=禁止PWM输出 0=允许PWM输出

  uint32_t phase_start_time;
  float phase_mileage;
} TASK;

void init_task(TASK *task);
void task_start(STATUS *status);
void task_finish(STATUS *status);
void task_stop(STATUS *status);
void task_select(STATUS *status, uint8_t id);
void update_task(STATUS *status);
void update_task_led(STATUS *status);

#endif
