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
  Q1_WAIT_KEY2,
  Q1_START_A_TURN,
  Q1_SIDE_AD,
  Q1_TURN_D,
  Q1_SIDE_DC,
  Q1_TURN_C,
  Q1_SIDE_CB,
  Q1_TURN_B,
  Q1_BA_FINAL,
} Q1_RACE_PHASE;

/* Q2 race phases (AB 发车去程 A→D→C→B，掉头后 B→C→D→A) */
typedef enum Q2_RACE_PHASE {
  Q2_AB_START_A_TURN = 100,    // 起点 A 左转 -90，进入 AD 边
  Q2_AB_SIDE_AD,               // AD 边巡线
  Q2_AB_TURN_D,                // D 点左转 -90
  Q2_AB_SIDE_DC,               // DC 边巡线
  Q2_AB_TURN_C,                // C 点左转 -90，进入 CB 边
  Q2_AB_SIDE_CB_GUARD,         // CB 边巡线（去程过干扰纸）
  Q2_AB_B_REACHED,             // 到 B 点，准备掉头
  Q2_AB_TURN_180,              // B 点 180° 掉头
  Q2_AB_SIDE_BC_GUARD_RET,     // BC 边巡线（返程再过干扰纸）
  Q2_AB_TURN_C_RET,            // C 点右转 +90（返程方向反转）
  Q2_AB_SIDE_CD,               // CD 边巡线
  Q2_AB_TURN_D_RET,            // D 点右转 +90
  Q2_AB_SIDE_DA_FINAL,         // DA 边巡线 + 末段减速
  Q2_AB_FINAL_STOP,            // A 点停车

  /* Q2 AD 发车（车头朝 D，无需起点 -90），其余路径与 AB 完全一致 */
  Q2_AD_WAIT_START = 200,      // 等待 key2 长按释放，然后直接进入 AD 边巡线
  Q2_AD_SIDE_AD,               // AD 边巡线
  Q2_AD_TURN_D,                // D 点左转 -90
  Q2_AD_SIDE_DC,               // DC 边巡线
  Q2_AD_TURN_C,                // C 点左转 -90，进入 CB 边
  Q2_AD_SIDE_CB_GUARD,         // CB 边巡线（去程过干扰纸）
  Q2_AD_TURN_180,              // B 点 180° 掉头
  Q2_AD_SIDE_BC_GUARD_RET,     // BC 边巡线（返程再过干扰纸）
  Q2_AD_TURN_C_RET,            // C 点右转 +90
  Q2_AD_SIDE_CD,               // CD 边巡线
  Q2_AD_TURN_D_RET,            // D 点右转 +90
  Q2_AD_SIDE_DA_FINAL,         // DA 边巡线 + 末段减速
  Q2_AD_FINAL_STOP,            // A 点停车
} Q2_RACE_PHASE;

/* Q2 BC 干扰段里程屏蔽窗口（mm，从进入边算起）*/
#define Q2_BC_GUARD_START_MM   -10.0f   // 进入 CB/BC 前就开始屏蔽，防漏判
#define Q2_BC_GUARD_END_MM     250.0f   // 25cm 后恢复路口检测，按实际干扰纸长度调
#define Q2_SIDE_LEN_MM         1000.0f  // 一条边的标称长度
#define Q2_FINAL_SLOW_MM       700.0f   // DA 边 70cm 处开始减速
#define Q2_FINAL_LATCH_MM      900.0f   // DA 边 90cm 后允许灰度触发停车
#define Q2_FINAL_FORCE_MM      1100.0f  // DA 边 110cm 强制停车兜底

/* Q3/Q4 race phases — to be defined per-task */

typedef struct TASK {
  uint8_t task_id;     //任务编号
  uint8_t start_pose;  //起始位姿 第二问 第三问 要用
  uint8_t race_phase;  //每一阶段的控制

  uint8_t cross_cnt;   //已通过的路口计数

  uint8_t armed;       //出发允许
  uint8_t task_running;  //表示任务进行的标志位

  uint8_t task_select_request;  // 请求切换任务标志位
  uint8_t requested_task_id;    // 任务切换的目标任务
  uint8_t pose_switch_request;  // 请求切换 AB/AD 发车模式标志位

  uint8_t start_request;  //出发请求 按钮和蓝牙只改动start_request 中断里判断后再给armed置1
  uint8_t stop_request;   //停止请求  不过感觉有点多余 Cz命令之后根本不会进task running标志位都置1了
  uint8_t startup_release;  // key2长按释放启动门控

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
