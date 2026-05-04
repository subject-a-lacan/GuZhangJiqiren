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

/* Q2 AB race phases (AB发车: A→D→C→B→掉头→C→D→A) */
typedef enum Q2_AB_RACE_PHASE {
  Q2_AB_TURN_A_TO_AD,     /* A 点起步左转进入 AD */
  Q2_AB_FIND_AD_OUT,      /* 转弯后找 AD 线 */
  Q2_AB_SIDE_AD_OUT,      /* AD 边直行等 D 点 */
  Q2_AB_TURN_D_TO_DC,     /* D 点左转进入 DC */
  Q2_AB_FIND_DC_OUT,      /* 转弯后找 DC 线 */
  Q2_AB_SIDE_DC_OUT,      /* DC 边直行等 C 点 */
  Q2_AB_TURN_C_TO_CB,     /* C 点左转进入 CB */
  Q2_AB_FIND_CB_OUT,      /* 转弯后找 CB 线 */
  Q2_AB_SIDE_CB_OUT,      /* C→B 直行, 79cm 前过滤路口, 等 B 点 */
  Q2_AB_STOP_BEFORE_UTURN_B, /* B 点停车, 等轮速降为 0 */
  Q2_AB_UTURN_B,          /* B 点 170° 掉头 */
  Q2_AB_FIND_BC_RETURN,   /* 掉头后找 BC 线 */
  Q2_AB_SIDE_BC_RETURN,   /* B→C 直行, 79cm 前过滤路口, 等 C 点 */
  Q2_AB_TURN_C_RETURN,    /* C 点右转进入 CD */
  Q2_AB_FIND_CD_RETURN,   /* 转弯后找 CD 线 */
  Q2_AB_SIDE_CD_RETURN,   /* C→D 直行等 D 点 */
  Q2_AB_TURN_D_RETURN,    /* D 点右转进入 DA */
  Q2_AB_FIND_DA_RETURN,   /* 转弯后找 DA 线 */
  Q2_AB_SIDE_DA_RETURN,   /* D→A 最后一段, 80cm 后降速, 等 A 点停车 */
  Q2_AB_FINISH,           /* 停车收尾 */
} Q2_AB_RACE_PHASE;

/* Q2 AD race phases (AD发车: A→B→C→D→掉头→C→B→A) */
typedef enum Q2_AD_RACE_PHASE {
  Q2_AD_TURN_A_TO_AB,       /* A 点起步右转进入 AB */
  Q2_AD_FIND_AB_OUT,        /* 转弯后找 AB 线 */
  Q2_AD_SIDE_AB_OUT,        /* AB 边直行等 B 点 */
  Q2_AD_TURN_B_TO_BC,       /* B 点右转进入 BC */
  Q2_AD_FIND_BC_OUT,        /* 转弯后找 BC 线 */
  Q2_AD_SIDE_BC_OUT,        /* B→C 直行, 79cm 前过滤路口, 等 C 点 */
  Q2_AD_TURN_C_TO_CD,       /* C 点右转进入 CD */
  Q2_AD_FIND_CD_OUT,        /* 转弯后找 CD 线 */
  Q2_AD_SIDE_CD_OUT,        /* C→D 直行, 预减速后等 D 点 */
  Q2_AD_STOP_BEFORE_UTURN_D, /* D 点停车, 等轮速降为 0 */
  Q2_AD_UTURN_D,            /* D 点 170° 掉头 */
  Q2_AD_FIND_DC_RETURN,     /* 掉头后找 DC 线 */
  Q2_AD_SIDE_DC_RETURN,     /* D→C 直行等 C 点 */
  Q2_AD_TURN_C_RETURN,      /* C 点左转进入 CB */
  Q2_AD_FIND_CB_RETURN,     /* 转弯后找 CB 线 */
  Q2_AD_SIDE_CB_RETURN,     /* C→B 直行, 79cm 前过滤路口, 等 B 点 */
  Q2_AD_TURN_B_RETURN,      /* B 点左转进入 BA */
  Q2_AD_FIND_BA_RETURN,     /* 转弯后找 BA 线 */
  Q2_AD_SIDE_BA_RETURN,     /* B→A 最后一段, 80cm 后降速, 等 A 点停车 */
  Q2_AD_FINISH,             /* 停车收尾 */
} Q2_AD_RACE_PHASE;

/* Q3 AB race phases (AB发车: A→D→C→B→A, 一圈, CD边有待测A4) */
typedef enum Q3_AB_RACE_PHASE {
  Q3_AB_START_TO_A,
  Q3_AB_TURN_A_TO_AD,
  Q3_AB_FIND_AD,
  Q3_AB_SIDE_AD,
  Q3_AB_TURN_D_TO_DC,
  Q3_AB_FIND_DC,
  Q3_AB_SIDE_DC,       /* CD边, 里程控速 + 屏蔽A4干扰 */
  Q3_AB_TURN_C_TO_CB,
  Q3_AB_FIND_CB,
  Q3_AB_SIDE_CB,
  Q3_AB_TURN_B_TO_BA,
  Q3_AB_FIND_BA,
  Q3_AB_SIDE_BA_FINAL,
  Q3_AB_FINISH,
} Q3_AB_RACE_PHASE;

/* Q3 AD race phases (AD发车: A→B→C→D→A, 一圈, CD边有待测A4) */
typedef enum Q3_AD_RACE_PHASE {
  Q3_AD_START_TO_A,
  Q3_AD_TURN_A_TO_AB,
  Q3_AD_FIND_AB,
  Q3_AD_SIDE_AB,
  Q3_AD_TURN_B_TO_BC,
  Q3_AD_FIND_BC,
  Q3_AD_SIDE_BC,
  Q3_AD_TURN_C_TO_CD,
  Q3_AD_FIND_CD,
  Q3_AD_SIDE_CD,       /* CD边, 里程控速 + 屏蔽A4干扰 */
  Q3_AD_TURN_D_TO_DA,
  Q3_AD_FIND_DA,
  Q3_AD_SIDE_DA_FINAL,
  Q3_AD_FINISH,
} Q3_AD_RACE_PHASE;

/* Q4 race phases: A->D->DC near C, then scan third-points off the line. */
typedef enum Q4_RACE_PHASE {
  Q4_START_TO_A,
  Q4_TURN_A_TO_AD,
  Q4_FIND_AD,
  Q4_SIDE_AD,
  Q4_TURN_D_TO_DC,
  Q4_FIND_DC,
  Q4_SIDE_DC_TO_SCAN_START,
  Q4_SCAN_TURN_1,
  Q4_SCAN_DRIVE_1,
  Q4_SCAN_STOP_1,
  Q4_SCAN_DRIVE_2,
  Q4_SCAN_STOP_2,
  Q4_SCAN_TURN_2,
  Q4_SCAN_DRIVE_3,
  Q4_SCAN_STOP_3,
  Q4_SCAN_TURN_3,
  Q4_SCAN_DRIVE_4,
  Q4_SCAN_STOP_4,
  Q4_FINISH,
} Q4_RACE_PHASE;

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
