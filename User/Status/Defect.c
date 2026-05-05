#include "Defect.h"
#include "status.h"
#include "math_tool.h"
#include "log.h"
#include "pid.h"
#include "usart.h"
#include <stdio.h>

/* Q1 tunable parameters */
#define Q1_START_PULSE                 200   /* 起步脉冲阈值, 待标定 */
#define Q1_TURN_A_LEFT_ANGLE           80.0f /* A点左转角度 */
#define Q1_TURN_D_LEFT_ANGLE           85.0f /* D点左转角度 */
#define Q1_TURN_C_LEFT_ANGLE           85.0f /* C点左转角度 */
#define Q1_TURN_B_LEFT_ANGLE           85.0f /* B点左转角度 */
#define Q1_TURN_A_ANGLE_LIMIT         20.0f /* A点转弯角度环限幅 */
#define Q1_TURN_D_ANGLE_LIMIT         22.0f /* D点转弯角度环限幅 */
#define Q1_TURN_C_ANGLE_LIMIT         22.0f /* C点转弯角度环限幅 */
#define Q1_TURN_B_ANGLE_LIMIT         22.0f /* B点转弯角度环限幅 */
#define Q1_TURN_TOLERANCE_DEG          8.0f  /* 转弯完成的角度容差 */
#define Q1_TURN_TO_FIND_TOLERANCE_DEG  8.0f /* 转弯→找线接管角度阈值 */
#define Q1_TURN_LINE_MASK_6            0x7E  /* bit1~bit6, 中间6路 */
#define Q1_TURN_LINE_MASK_4            0x3C  /* bit2~bit5, 中间4路 */
#define Q1_LINE_STABLE_CNT             3     /* 中间4路稳定帧数 */
#define Q1_FLASH_SPEED                 55    /* 直线冲刺速度 */
#define Q1_CRUISE_SPEED                38    /* 巡线直走速度 (入弯前降速) */
#define Q1_TURN_SPEED                  23    /* 转弯时的基础速度 */
#define Q1_FIRST_TURN_SPEED            30    /* 第一个路口(A点)转弯速度, 距离短需降速 */
#define Q1_FINAL_SLOW_SPEED            20    /* 终点前降速 / 找线低速 */
#define Q1_STRAIGHT_FLASH_CM           58.0f /* 直线冲刺距离(cm) */
#define Q1_PRE_TURN_SLOW_CM            70.0f /* 普通直道入弯前减速距离(cm) */
#define Q1_PRE_TURN_SLOW_SPEED         25    /* 普通直道入弯前减速速度 */
#define Q1_FINAL_SLOW_CM               63.0f /* 终点降速距离(cm) */
#define Q1_BA_PULSE                    3500  /* BA 边停车脉冲阈值, 待标定 */
#define Q1_BA_STOP_CM                  80.0f /* BA 边停车里程阈值(cm) */

/* Q2 tunable parameters (AB发车) */
#define Q2_TURN_A_LEFT_ANGLE          76.0f /* A点左转角度 */
#define Q2_TURN_D_LEFT_ANGLE          83.0f /* D点左转角度 */
#define Q2_TURN_C_LEFT_ANGLE          81.0f /* C点左转角度 */
#define Q2_UTURN_B_ANGLE              175.0f /* B点掉头角度 */
#define Q2_TURN_C_RIGHT_ANGLE         -86.0f /* 回程C点右转角度 */
#define Q2_TURN_D_RIGHT_ANGLE         -86.0f /* 回程D点右转角度 */
#define Q2_TURN_A_ANGLE_LIMIT         27.0f /* A点转弯角度环限幅 */
#define Q2_TURN_D_ANGLE_LIMIT         27.0f /* D点转弯角度环限幅 */
#define Q2_TURN_C_ANGLE_LIMIT         27.0f /* C点转弯角度环限幅 */
#define Q2_TURN_C_RETURN_ANGLE_LIMIT  27.0f /* 回程C点转弯角度环限幅 */
#define Q2_TURN_D_RETURN_ANGLE_LIMIT  27.0f /* 回程D点转弯角度环限幅 */
#define Q2_TURN_FIND_TOLERANCE_DEG    10.0f /* 90°转弯→找线接管角度阈值 */
#define Q2_UTURN_TOLERANCE_DEG        6.0f /* 掉头→找线接管角度阈值 */
#define Q2_TURN_LINE_MASK_6           0x7E  /* bit1~bit6, 中间6路 */
#define Q2_TURN_LINE_MASK_4           0x3C  /* bit2~bit5, 中间4路 */
#define Q2_LINE_STABLE_CNT            3     /* 中间4路稳定帧数 */
#define Q2_FLASH_SPEED                55    /* 直线冲刺速度 */
#define Q2_CRUISE_SPEED               40    /* 巡线直走速度 */
#define Q2_TURN_SPEED                 23    /* 转弯时的基础速度 */
#define Q2_AB_FIRST_TURN_SPEED        38   /* AB发车第一个路口(A点)转弯速度 */
#define Q2_AD_FIRST_TURN_SPEED        38    /* AD发车第一个路口(A点)转弯速度 */
#define Q2_UTURN_SPEED                20    /* 掉头速度 */
#define Q2_FINAL_SLOW_SPEED           20    /* 终点前降速 / 找线低速 */
#define Q2_STRAIGHT_FLASH_CM          65.0f /* 直线冲刺距离(cm) */
#define Q2_PRE_TURN_SLOW_CM           70.0f /* 普通直道入弯前减速距离(cm) */
#define Q2_PRE_TURN_SLOW_SPEED        25    /* 普通直道入弯前减速目标速度 */
#define Q2_CB_ROAD_ENABLE_CM          79.0f /* C→B 路口使能里程(cm) */
#define Q2_BC_ROAD_ENABLE_CM          79.0f /* B→C 路口使能里程(cm) */
#define Q2_FINAL_SLOW_CM              70.0f /* 终点降速距离(cm) */
#define Q2_UTURN_PRE_SLOW_CM          70.0f /* B点掉头前预减速距离(cm) */
#define Q2_UTURN_PRE_SLOW_SPEED       20    /* B点掉头前预减速目标速度 */
#define Q2_AB_UTURN_ANGLE_LIMIT       93.0f /* AB掉头时角度环输出限幅 */
#define Q2_AB_NORMAL_ANGLE_LIMIT      27.0f /* AB正常角度环输出限幅 */
#define Q2_AD_UTURN_ANGLE_LIMIT       85.0f /* AD掉头时角度环输出限幅 */
#define Q2_AD_NORMAL_ANGLE_LIMIT      27.0f /* AD正常角度环输出限幅 */
#define Q2_AD_FIRST_TURN_ANGLE_LIMIT  25.0f  /* AD first turn angle output limit */
#define Q2_UTURN_STOP_SPEED_THRESHOLD 4     /* 掉头前停车轮速阈值 */

/* Q2 AD-specific tunable angles (AD发车: 顺时针去程右转, 逆时针回程左转) */
#define Q2_AD_TURN_A_RIGHT_ANGLE       -76.0f /* A点右转进入AB */
#define Q2_AD_TURN_B_RIGHT_ANGLE       -83.0f /* B点右转进入BC */
#define Q2_AD_TURN_C_RIGHT_ANGLE       -82.0f /* C点右转进入CD */
#define Q2_AD_UTURN_D_ANGLE            -170.0f /* D点掉头角度 */
#define Q2_AD_TURN_C_LEFT_ANGLE         83.0f /* 回程C点左转进入CB */
#define Q2_AD_TURN_B_LEFT_ANGLE         86.0f /* 回程B点左转进入BA */
#define Q2_AD_TURN_B_ANGLE_LIMIT        27.0f /* AD B点转弯角度环限幅 */
#define Q2_AD_TURN_C_ANGLE_LIMIT        27.0f /* AD C点转弯角度环限幅 */
#define Q2_AD_TURN_C_RETURN_ANGLE_LIMIT 24.0f /* AD回程C点转弯角度环限幅 */
#define Q2_AD_TURN_B_RETURN_ANGLE_LIMIT 24.0f /* AD回程B点转弯角度环限幅 */

/* Q3/Q4 feed-forward defaults. wheel0 = wheel[0]/which 1, wheel1 = wheel[1]/which 2. */
#define Q3_WHEEL0_FF_OFFSET             124.64f
#define Q3_WHEEL0_FF_K                  19.23f
#define Q3_WHEEL0_FF_MIN                260.0f
#define Q3_WHEEL1_FF_OFFSET             164.53f
#define Q3_WHEEL1_FF_K                  17.64f
#define Q3_WHEEL1_FF_MIN                260.0f

#define Q4_WHEEL0_FF_OFFSET             124.64f
#define Q4_WHEEL0_FF_K                  19.23f
#define Q4_WHEEL0_FF_MIN                260.0f
#define Q4_WHEEL1_FF_OFFSET             164.53f
#define Q4_WHEEL1_FF_K                  17.64f
#define Q4_WHEEL1_FF_MIN                260.0f

/* Q4 tunable parameters */
#define Q4_TRACK_SPEED                  30
#define Q4_SCAN_SPEED                   16
#define Q4_FIND_LINE_SPEED              16
#define Q4_TURN_BASE_SPEED              0
#define Q4_START_TO_A_MIN_CM            5.0f
#define Q4_ROAD_ENABLE_CM               60.0f
#define Q4_DC_SCAN_START_CM             66.7f
#define Q4_SCAN_SEGMENT_CM              33.3f
#define Q4_SCAN_STOP_MS                 100
#define Q4_TURN_A_LEFT_ANGLE            80.0f
#define Q4_TURN_D_LEFT_ANGLE            85.0f
#define Q4_SCAN_TURN_1_LEFT_ANGLE       83.0f
#define Q4_SCAN_TURN_2_LEFT_ANGLE       83.0f
#define Q4_SCAN_TURN_3_LEFT_ANGLE       83.0f
#define Q4_TURN_A_ANGLE_LIMIT           22.0f
#define Q4_TURN_D_ANGLE_LIMIT           22.0f
#define Q4_SCAN_TURN_1_ANGLE_LIMIT      22.0f
#define Q4_SCAN_TURN_2_ANGLE_LIMIT      22.0f
#define Q4_SCAN_TURN_3_ANGLE_LIMIT      22.0f
#define Q4_STRAIGHT_ANGLE_LIMIT         12.0f
#define Q4_TURN_TOLERANCE_DEG           8.0f
#define Q4_TURN_LINE_MASK_6             0x7E
#define Q4_LINE_STABLE_CNT              3

/* Q3 tunable parameters */

/* AB per-turn left angles (independently calibrated, like TASK2) */
#define Q3_AB_TURN_A_LEFT_ANGLE    80.0f
#define Q3_AB_TURN_D_LEFT_ANGLE    85.0f
#define Q3_AB_TURN_C_LEFT_ANGLE    85.0f
#define Q3_AB_TURN_B_LEFT_ANGLE    85.0f
#define Q3_AB_TURN_A_ANGLE_LIMIT   27.0f /* AB A点转弯角度环限幅 */
#define Q3_AB_TURN_D_ANGLE_LIMIT   20.0f /* AB D点转弯角度环限幅 */
#define Q3_AB_TURN_C_ANGLE_LIMIT   20.0f /* AB C点转弯角度环限幅 */
#define Q3_AB_TURN_B_ANGLE_LIMIT   23.0f /* AB B点转弯角度环限幅 */

/* AD per-turn right angles */
#define Q3_AD_TURN_A_RIGHT_ANGLE   -83.0f
#define Q3_AD_TURN_B_RIGHT_ANGLE   -83.0f
#define Q3_AD_TURN_C_RIGHT_ANGLE   -83.0f
#define Q3_AD_TURN_D_RIGHT_ANGLE   -83.0f
#define Q3_AD_TURN_A_ANGLE_LIMIT   27.0f /* AD A点转弯角度环限幅 */
#define Q3_AD_TURN_B_ANGLE_LIMIT   23.0f /* AD B点转弯角度环限幅 */
#define Q3_AD_TURN_C_ANGLE_LIMIT   23.0f /* AD C点转弯角度环限幅 */
#define Q3_AD_TURN_D_ANGLE_LIMIT   23.0f /* AD D点转弯角度环限幅 */

/* Turn control */
#define Q3_TURN_TOLERANCE_DEG       10.0f /* 转弯→找线接管角度阈值 */
#define Q3_TURN_LINE_MASK_6         0x7E  /* bit1~bit6, 中间6路 */
#define Q3_TURN_LINE_MASK_4         0x3C  /* bit2~bit5, 中间4路 */
#define Q3_LINE_STABLE_CNT          3     /* 中间4路稳定帧数, 防止假Straight误清零cnt_seen */

/* Speeds */
#define Q3_FLASH_SPEED              58
#define Q3_CRUISE_SPEED             43
#define Q3_TURN_SPEED               26
#define Q3_AB_FIRST_TURN_SPEED      30    /* AB发车第一个路口(A点)转弯速度 */
#define Q3_AD_FIRST_TURN_SPEED      30    /* AD发车第一个路口(A点)转弯速度 */
#define Q3_FINAL_SLOW_SPEED         20
#define Q3_CD_START_SPEED           30    /* CD边刚进线后的速度 */
#define Q3_CD_SLOW_SPEED            20    /* CD边给视觉更多帧的低速 */
#define Q3_CD_FAST_SPEED            30    /* CD边视觉识别完成后的提速 */

/* Distance thresholds (cm, 通过 encoder_pulse_to_cm 换算后比较) */
#define Q3_STRAIGHT_FLASH_CM        62.0f
#define Q3_PRE_TURN_SLOW_CM         56.0f /* 普通直道入弯前减速距离(cm) */
#define Q3_PRE_TURN_SLOW_SPEED      22   /* 普通直道入弯前减速目标速度 */
#define Q3_FINAL_SLOW_CM            70.0f
#define Q3_CD_SLOW_AFTER_CM         14.0f
#define Q3_CD_FAST_AFTER_CM         68.0f /* CD边视觉识别完成后的提速距离 */
#define Q3_CD_IGNORE_END_CM         75.0f /* A4横线干扰结束, 之后允许接受路口 */
#define Q3_ROAD_ENABLE_CM            70.0f /* 普通路口里程门槛, 防止误触发 */

#define Q3_CD_DISTANCE_MASK          0x66  /* bit1/bit2/bit5/bit6 */
#define Q3_CD_BLACK_COUNT_THRESHOLD  2
#define Q3_CD_WHITE_COUNT_THRESHOLD  1
#define Q3_CD_EDGE_MIN_GAP_CM        0.8f
#define Q3_CD_EDGE_COUNT             6

extern uint8_t cross_cnt;
extern uint8_t left_cnt;
extern uint8_t cross_delay;
extern Road road_buf;
extern volatile float l1;
extern volatile float l2;
extern volatile uint8_t task3_finished;

void init_task(TASK *task) {
  task->task_id = TASK_BASIC_1;
  task->start_pose = START_AB;
  task->race_phase = 0;

  task->cross_cnt = 0;
  task->cnt_seen = 0;

  task->armed = 0;
  task->task_running = 0;

  task->task_select_request = 0;
  task->requested_task_id = 0;
  task->pose_switch_request = 0;

  task->start_request = 0;
  task->stop_request = 0;
  task->stop_cmd = 1;

  task->phase_start_time = 0;
  task->phase_mileage = 0;
}

void task_start(STATUS *status) {
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.stop_cmd = 0;

  status->task.cross_cnt = 0;
  status->task.cnt_seen = 0;
  cross_cnt = 0;
  left_cnt = 0;
  cross_delay = 0;

  road_buf = Straight;
  status->sensor.gw_analogue.cross.integral = 0;
  status->sensor.gw_analogue.cross.data_buf = 0;
  status->sensor.gw_analogue.cross.maybe = 0;
  status->sensor.gw_analogue.cross.cross = Straight;
  status->sensor.gw_analogue.cross.cross_cnt = 0;

  status->task.phase_mileage = 0;

  if (status->task.task_id == TASK_BASIC_1 || status->task.task_id == TASK_BASIC_2) {
    apply_basic_control_param(status);
  } else if (status->task.task_id == TASK_ADV_1) {
    apply_adv_control_param(status);
    set_wheel_ff_param_by_which(1, Q3_WHEEL0_FF_OFFSET, Q3_WHEEL0_FF_K, Q3_WHEEL0_FF_MIN);
    set_wheel_ff_param_by_which(2, Q3_WHEEL1_FF_OFFSET, Q3_WHEEL1_FF_K, Q3_WHEEL1_FF_MIN);
  } else if (status->task.task_id == TASK_ADV_2) {
    apply_adv_control_param(status);
    set_wheel_ff_param_by_which(1, Q4_WHEEL0_FF_OFFSET, Q4_WHEEL0_FF_K, Q4_WHEEL0_FF_MIN);
    set_wheel_ff_param_by_which(2, Q4_WHEEL1_FF_OFFSET, Q4_WHEEL1_FF_K, Q4_WHEEL1_FF_MIN);
  }

  status->motor.wheel[0].trust = 0;
  status->motor.wheel[1].trust = 0;

  status->state.initial_angle = status->state.cur_angle;

  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      {
        uint8_t h3c[] = {0x48, 0x33, 0x43};
        HAL_UART_Transmit(&huart3, h3c, 3, 100);
      }
      status->task.race_phase = Q1_START_TO_A;
      break;
    case TASK_BASIC_2:
      if (status->task.start_pose == START_AB) {
        status->task.race_phase = Q2_AB_TURN_A_TO_AD;
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q2_TURN_A_LEFT_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_AB_FIRST_TURN_SPEED;
        status->state.status_pid.angle_output_limit = Q2_TURN_A_ANGLE_LIMIT;
        status->task.task_running = 1;
      } else {
        status->task.race_phase = Q2_AD_TURN_A_TO_AB;
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q2_AD_TURN_A_RIGHT_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_AD_FIRST_TURN_SPEED;
        status->state.status_pid.angle_output_limit = Q2_AD_FIRST_TURN_ANGLE_LIMIT;
        status->task.task_running = 1;
      }
      break;
    case TASK_ADV_1:
      l1 = 0;
      l2 = 0;
      task3_finished = 0;
      if (status->task.start_pose == START_AB) {
        status->task.race_phase = Q3_AB_START_TO_A;
      } else {
        status->task.race_phase = Q3_AD_START_TO_A;
      }
      break;
    case TASK_ADV_2:
      status->task.race_phase = Q4_START_TO_A;
      break;
  }

  status->task.phase_start_time = status->state.time;

  if (status->task.task_id != TASK_BASIC_2) {
    status->state.motion = STOP;
    status->state.base_speed = 0;
  }
}

void task_finish(STATUS *status) {
  status->task.task_running = 0;
  status->task.armed = 0;
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.stop_cmd = 1;
  status->task.task_select_request = 0;
  status->task.pose_switch_request = 0;
  status->state.motion = STOP;
  status->state.base_speed = 0;
  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;
  status->device.buzzer.on = 1;
  status->device.buzzer.off_time = status->state.time + 200;
}

void task_stop(STATUS *status) {
  status->task.task_running = 0;
  status->task.armed = 0;
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.stop_cmd = 1;
  status->state.motion = STOP;
  status->state.base_speed = 0;
  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;
}

void task_select(STATUS *status, uint8_t id) {
  if (id < TASK_BASIC_1 || id > TASK_ADV_2) {
    return;
  }
  status->task.task_id = id;
  if (id == TASK_BASIC_1 || id == TASK_ADV_2) {
    status->task.start_pose = START_AB;
  }
  update_task_led(status);
}

/* ---- TASK1 小工具函数 ---- */

static uint8_t q1_mid4_stable_cnt = 0;

static Road task1_map_road(Road raw) {
  if (raw == TLRoad) return LeftRoad;
  if (raw == TRRoad) return RightRoad;
  return raw;
}

static void task1_enter_phase(STATUS *status, uint8_t next_phase) {
  status->task.race_phase = next_phase;
  status->task.phase_mileage = 0;
  status->task.phase_start_time = status->state.time;
  q1_mid4_stable_cnt = 0;
}

static uint8_t task1_accept_left_road(STATUS *status, Road road) {
  if (road == LeftRoad && status->task.cnt_seen == 0) {
    status->task.cnt_seen = 1;
    status->task.cross_cnt++;
    return 1;
  }
  return 0;
}

static uint8_t task1_final_stop_condition(STATUS *status, Road road) {
  if (road == LeftRoad
      && status->task.cnt_seen == 0
      && encoder_pulse_to_cm((int32_t)status->task.phase_mileage) > Q1_BA_STOP_CM) {
        status->state.motion = STOP;
        status->task.stop_cmd = 1;
        status->task.task_running = 0;
    return 1;
  }
  return 0;
}

static float task1_angle_error(STATUS *status) {
  float target = status->state.tar_angle + status->state.initial_angle;
  float diff = target - status->state.cur_angle;
  if (diff > 180.0f)        diff -= 360.0f;
  else if (diff < -180.0f)  diff += 360.0f;
  return diff;
}

static uint8_t task1_turn_angle_ready(STATUS *status) {
  return (ABS(task1_angle_error(status)) < Q1_TURN_TO_FIND_TOLERANCE_DEG);
}

static uint8_t task1_middle6_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q1_TURN_LINE_MASK_6) != 0;
}

static uint8_t task1_middle4_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q1_TURN_LINE_MASK_4) != 0;
}

static void task1_apply_side_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (q1_mid4_stable_cnt < Q1_LINE_STABLE_CNT) {
    status->state.base_speed = Q1_FINAL_SLOW_SPEED;
  } else if (cm <= Q1_STRAIGHT_FLASH_CM) {
    status->state.base_speed = Q1_FLASH_SPEED;
  } else if (cm >= Q1_PRE_TURN_SLOW_CM) {
    status->state.base_speed = Q1_PRE_TURN_SLOW_SPEED;
  } else {
    status->state.base_speed = Q1_CRUISE_SPEED;
  }
}

static void task1_apply_final_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm <= Q1_FINAL_SLOW_CM) {
    status->state.base_speed = Q1_CRUISE_SPEED;
  } else {
    status->state.base_speed = Q1_FINAL_SLOW_SPEED;
  }
}

static void driver_task1(STATUS *status) {
  Road road = task1_map_road(status->sensor.gw_analogue.cross.cross);

  if (road == Straight && status->state.motion == FIND_LINE
      && status->task.race_phase != Q1_FIND_AD
      && status->task.race_phase != Q1_FIND_DC
      && status->task.race_phase != Q1_FIND_CB
      && status->task.race_phase != Q1_FIND_BA) {
    status->task.cnt_seen = 0;
  }

  switch (status->task.race_phase) {

    case Q1_START_TO_A:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q1_FIRST_TURN_SPEED;

      if (task1_accept_left_road(status, road)) {
        task1_enter_phase(status, Q1_TURN_A);
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q1_TURN_A_LEFT_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q1_FIRST_TURN_SPEED;
        status->state.status_pid.angle_output_limit = Q1_TURN_A_ANGLE_LIMIT;
      }
      break;

    /* ---- 转弯: 角度环 KEEP_ANGLE ---- */
    case Q1_TURN_A:
    case Q1_TURN_D:
    case Q1_TURN_C:
    case Q1_TURN_B:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;

      if (task1_turn_angle_ready(status)) {
        uint8_t next;
        if (status->task.race_phase == Q1_TURN_A)      next = Q1_FIND_AD;
        else if (status->task.race_phase == Q1_TURN_D) next = Q1_FIND_DC;
        else if (status->task.race_phase == Q1_TURN_C) next = Q1_FIND_CB;
        else                                           next = Q1_FIND_BA;

        task1_enter_phase(status, next);
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q1_FINAL_SLOW_SPEED;
      }
      break;

    /* ---- 转弯后低速找线 ---- */
    case Q1_FIND_AD:
    case Q1_FIND_DC:
    case Q1_FIND_CB:
    case Q1_FIND_BA:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q1_FINAL_SLOW_SPEED;

      if (task1_middle6_seen(status)) {
        uint8_t next;
        if (status->task.race_phase == Q1_FIND_AD)      next = Q1_SIDE_AD;
        else if (status->task.race_phase == Q1_FIND_DC) next = Q1_SIDE_DC;
        else if (status->task.race_phase == Q1_FIND_CB) next = Q1_SIDE_CB;
        else                                            next = Q1_BA_FINAL;

        task1_enter_phase(status, next);
      }
      break;

    /* ---- 直线巡线 ---- */
    case Q1_SIDE_AD:
    case Q1_SIDE_DC:
    case Q1_SIDE_CB:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;

      if (task1_middle4_seen(status)) {
        q1_mid4_stable_cnt++;
      } else {
        q1_mid4_stable_cnt = 0;
      }
      task1_apply_side_speed(status);

      if (task1_accept_left_road(status, road)) {
        uint8_t next;
        if (status->task.race_phase == Q1_SIDE_AD)      next = Q1_TURN_D;
        else if (status->task.race_phase == Q1_SIDE_DC) next = Q1_TURN_C;
        else                                             next = Q1_TURN_B;

        task1_enter_phase(status, next);
        status->state.initial_angle = status->state.cur_angle;
        if (next == Q1_TURN_D)      status->state.tar_angle = Q1_TURN_D_LEFT_ANGLE;
        else if (next == Q1_TURN_C) status->state.tar_angle = Q1_TURN_C_LEFT_ANGLE;
        else                        status->state.tar_angle = Q1_TURN_B_LEFT_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q1_TURN_SPEED;
        if (next == Q1_TURN_D)      status->state.status_pid.angle_output_limit = Q1_TURN_D_ANGLE_LIMIT;
        else if (next == Q1_TURN_C) status->state.status_pid.angle_output_limit = Q1_TURN_C_ANGLE_LIMIT;
        else                        status->state.status_pid.angle_output_limit = Q1_TURN_B_ANGLE_LIMIT;
      }
      break;

    /* ---- BA 最后一段, 巡线回到发车点停车 ---- */
    case Q1_BA_FINAL:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      task1_apply_final_speed(status);

      if (task1_final_stop_condition(status, road)) {
        task_finish(status);
      }
      break;
  }
}

/* ---- TASK2 AB 小工具函数 ---- */

static uint8_t q2_mid4_stable_cnt = 0;

static Road task2_map_road(Road raw) {
  if (raw == TLRoad) return LeftRoad;
  if (raw == TRRoad) return RightRoad;
  return raw;
}

static void task2_enter_phase(STATUS *status, uint8_t next_phase) {
  status->task.race_phase = next_phase;
  status->task.phase_mileage = 0;
  status->task.phase_start_time = status->state.time;
  q2_mid4_stable_cnt = 0;
}

static float task2_angle_error(STATUS *status) {
  float target = status->state.tar_angle + status->state.initial_angle;
  float diff = target - status->state.cur_angle;
  if (diff > 180.0f)        diff -= 360.0f;
  else if (diff < -180.0f)  diff += 360.0f;
  return diff;
}

static uint8_t task2_turn_angle_ready(STATUS *status) {
  return (ABS(task2_angle_error(status)) < Q2_TURN_FIND_TOLERANCE_DEG);
}

static uint8_t task2_uturn_angle_ready(STATUS *status) {
  return (ABS(task2_angle_error(status)) < Q2_UTURN_TOLERANCE_DEG);
}

static uint8_t task2_middle6_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q2_TURN_LINE_MASK_6) != 0;
}

static uint8_t task2_middle4_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q2_TURN_LINE_MASK_4) != 0;
}

static uint8_t task2_accept_road(STATUS *status, Road expected, float min_cm) {
  Road road = task2_map_road(status->sensor.gw_analogue.cross.cross);
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm < min_cm) return 0;
  if (road != expected) return 0;
  if (status->task.cnt_seen == 1) return 0;
  status->task.cnt_seen = 1;
  status->task.cross_cnt++;
  return 1;
}

static void task2_apply_side_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (q2_mid4_stable_cnt < Q2_LINE_STABLE_CNT) {
    status->state.base_speed = Q2_FINAL_SLOW_SPEED;       /* 20, 不稳时低速找线 */
  } else if (cm <= Q2_STRAIGHT_FLASH_CM) {
    status->state.base_speed = Q2_FLASH_SPEED;             /* 55, 0~65cm 高速 */
  } else if (cm <= Q2_PRE_TURN_SLOW_CM) {
    status->state.base_speed = Q2_CRUISE_SPEED;            /* 44, 65~72cm 中速 */
  } else {
    status->state.base_speed = Q2_PRE_TURN_SLOW_SPEED;     /* 25, >72cm 入弯前减速 */
  }
}

static void task2_apply_final_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm <= Q2_FINAL_SLOW_CM) {
    status->state.base_speed = Q2_CRUISE_SPEED;
  } else {
    status->state.base_speed = Q2_FINAL_SLOW_SPEED;
  }
}

static void driver_task2(STATUS *status) {
  if (!status->task.task_running) return;

  if (status->task.start_pose == START_AB) {
    /* ====== AB 发车: A→D→C→B→掉头→C→D→A ====== */
    Road road = task2_map_road(status->sensor.gw_analogue.cross.cross);

    if (road == Straight && status->state.motion == FIND_LINE
        && status->task.race_phase != Q2_AB_FIND_AD_OUT
        && status->task.race_phase != Q2_AB_FIND_DC_OUT
        && status->task.race_phase != Q2_AB_FIND_CB_OUT
        && status->task.race_phase != Q2_AB_FIND_BC_RETURN
        && status->task.race_phase != Q2_AB_FIND_CD_RETURN
        && status->task.race_phase != Q2_AB_FIND_DA_RETURN) {
      if (status->task.race_phase == Q2_AB_SIDE_DA_RETURN || q2_mid4_stable_cnt >= Q2_LINE_STABLE_CNT) {
        status->task.cnt_seen = 0;
      }
    }

    switch (status->task.race_phase) {

      /* ====== 去程: A→D→C→B ====== */

      case Q2_AB_TURN_A_TO_AD:
        status->task.task_running = 1;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_AB_FIRST_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AB_FIND_AD_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AB_FIND_AD_OUT:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AB_SIDE_AD_OUT);
        }
        break;

      case Q2_AB_SIDE_AD_OUT:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        if (task2_accept_road(status, LeftRoad, 0)) {
          task2_enter_phase(status, Q2_AB_TURN_D_TO_DC);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_TURN_D_LEFT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_TURN_D_ANGLE_LIMIT;
        }
        break;

      case Q2_AB_TURN_D_TO_DC:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AB_FIND_DC_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AB_FIND_DC_OUT:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AB_SIDE_DC_OUT);
        }
        break;

      case Q2_AB_SIDE_DC_OUT:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        if (task2_accept_road(status, LeftRoad, 0)) {
          task2_enter_phase(status, Q2_AB_TURN_C_TO_CB);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_TURN_C_LEFT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_TURN_C_ANGLE_LIMIT;
        }
        break;

      case Q2_AB_TURN_C_TO_CB:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AB_FIND_CB_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AB_FIND_CB_OUT:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AB_SIDE_CB_OUT);
        }
        break;

      case Q2_AB_SIDE_CB_OUT:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        /* B点掉头前预减速: 超过70cm后降速到20 */
        if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) > Q2_UTURN_PRE_SLOW_CM) {
          status->state.base_speed = Q2_UTURN_PRE_SLOW_SPEED;
        }
        if (task2_accept_road(status, LeftRoad, Q2_CB_ROAD_ENABLE_CM)) {
          task2_enter_phase(status, Q2_AB_STOP_BEFORE_UTURN_B);
          status->state.motion = STOP;
          status->state.base_speed = 0;
          status->motor.wheel[0].tar_speed = 0;
          status->motor.wheel[1].tar_speed = 0;
        }
        break;

      /* ====== B 点掉头前停车 ====== */

      case Q2_AB_STOP_BEFORE_UTURN_B:
        status->state.motion = STOP;
        status->state.base_speed = 0;
        status->motor.wheel[0].tar_speed = 0;
        status->motor.wheel[1].tar_speed = 0;
        if (ABS(status->motor.wheel[0].cur_speed) <= Q2_UTURN_STOP_SPEED_THRESHOLD
            && ABS(status->motor.wheel[1].cur_speed) <= Q2_UTURN_STOP_SPEED_THRESHOLD) {
          task2_enter_phase(status, Q2_AB_UTURN_B);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_UTURN_B_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = 0;
          status->state.status_pid.angle_output_limit = Q2_AB_UTURN_ANGLE_LIMIT;
        }
        break;

      /* ====== B 点掉头 ====== */

      case Q2_AB_UTURN_B:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = 0;
        status->state.status_pid.angle_output_limit = Q2_AB_UTURN_ANGLE_LIMIT;  /* AB掉头放大角度限幅 */
        if (task2_uturn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AB_FIND_BC_RETURN);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AB_FIND_BC_RETURN:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AB_SIDE_BC_RETURN);
        }
        break;

      /* ====== 回程: B→C→D→A ====== */

      case Q2_AB_SIDE_BC_RETURN:
        status->state.motion = FIND_LINE;
        status->state.status_pid.angle_output_limit = Q2_AB_NORMAL_ANGLE_LIMIT;  /* AB恢复正常角度限幅 */
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        if (task2_accept_road(status, RightRoad, Q2_BC_ROAD_ENABLE_CM)) {
          task2_enter_phase(status, Q2_AB_TURN_C_RETURN);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_TURN_C_RIGHT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_TURN_C_RETURN_ANGLE_LIMIT;
        }
        break;

      case Q2_AB_TURN_C_RETURN:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AB_FIND_CD_RETURN);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AB_FIND_CD_RETURN:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AB_SIDE_CD_RETURN);
        }
        break;

      case Q2_AB_SIDE_CD_RETURN:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        if (task2_accept_road(status, RightRoad, 0)) {
          task2_enter_phase(status, Q2_AB_TURN_D_RETURN);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_TURN_D_RIGHT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_TURN_D_RETURN_ANGLE_LIMIT;
        }
        break;

      case Q2_AB_TURN_D_RETURN:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AB_FIND_DA_RETURN);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AB_FIND_DA_RETURN:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AB_SIDE_DA_RETURN);
        }
        break;

      case Q2_AB_SIDE_DA_RETURN:
        status->state.motion = FIND_LINE;
        task2_apply_final_speed(status);
        if (task2_accept_road(status, RightRoad, Q2_FINAL_SLOW_CM)) {
          task2_enter_phase(status, Q2_AB_FINISH);
        }
        break;

      case Q2_AB_FINISH:
        task_finish(status);
        break;
    }

  } else {
    /* ====== AD 发车: A→B→C→D→掉头→C→B→A ====== */
    Road road = task2_map_road(status->sensor.gw_analogue.cross.cross);

    if (road == Straight && status->state.motion == FIND_LINE
        && status->task.race_phase != Q2_AD_FIND_AB_OUT
        && status->task.race_phase != Q2_AD_FIND_BC_OUT
        && status->task.race_phase != Q2_AD_FIND_CD_OUT
        && status->task.race_phase != Q2_AD_FIND_DC_RETURN
        && status->task.race_phase != Q2_AD_FIND_CB_RETURN
        && status->task.race_phase != Q2_AD_FIND_BA_RETURN) {
      if (status->task.race_phase == Q2_AD_SIDE_BA_RETURN || q2_mid4_stable_cnt >= Q2_LINE_STABLE_CNT) {
        status->task.cnt_seen = 0;
      }
    }

    switch (status->task.race_phase) {

      /* ====== 去程: A→B→C→D ====== */

      case Q2_AD_TURN_A_TO_AB:
        status->task.task_running = 1;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_AD_FIRST_TURN_SPEED;
        status->state.status_pid.angle_output_limit = Q2_AD_FIRST_TURN_ANGLE_LIMIT;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_AB_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
          status->state.status_pid.angle_output_limit = Q2_AD_NORMAL_ANGLE_LIMIT;
        }
        break;

      case Q2_AD_FIND_AB_OUT:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AD_SIDE_AB_OUT);
        }
        break;

      case Q2_AD_SIDE_AB_OUT:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        if (task2_accept_road(status, RightRoad, 0)) {
          task2_enter_phase(status, Q2_AD_TURN_B_TO_BC);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_AD_TURN_B_RIGHT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_AD_TURN_B_ANGLE_LIMIT;
        }
        break;

      case Q2_AD_TURN_B_TO_BC:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_BC_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AD_FIND_BC_OUT:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AD_SIDE_BC_OUT);
        }
        break;

      case Q2_AD_SIDE_BC_OUT:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        /* B→C 有A4纸干扰, 79cm前禁止消费路口 */
        if (task2_accept_road(status, RightRoad, Q2_BC_ROAD_ENABLE_CM)) {
          task2_enter_phase(status, Q2_AD_TURN_C_TO_CD);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_AD_TURN_C_RIGHT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_AD_TURN_C_ANGLE_LIMIT;
        }
        break;

      case Q2_AD_TURN_C_TO_CD:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_CD_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AD_FIND_CD_OUT:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AD_SIDE_CD_OUT);
        }
        break;

      case Q2_AD_SIDE_CD_OUT:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        /* D点掉头前预减速: 超过70cm后降速到20 */
        if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) > Q2_UTURN_PRE_SLOW_CM) {
          status->state.base_speed = Q2_UTURN_PRE_SLOW_SPEED;
        }
        if (task2_accept_road(status, RightRoad, 0)) {
          task2_enter_phase(status, Q2_AD_STOP_BEFORE_UTURN_D);
          status->state.motion = STOP;
          status->state.base_speed = 0;
          status->motor.wheel[0].tar_speed = 0;
          status->motor.wheel[1].tar_speed = 0;
        }
        break;

      /* ====== D 点掉头前停车 ====== */

      case Q2_AD_STOP_BEFORE_UTURN_D:
        status->state.motion = STOP;
        status->state.base_speed = 0;
        status->motor.wheel[0].tar_speed = 0;
        status->motor.wheel[1].tar_speed = 0;
        if (ABS(status->motor.wheel[0].cur_speed) <= Q2_UTURN_STOP_SPEED_THRESHOLD
            && ABS(status->motor.wheel[1].cur_speed) <= Q2_UTURN_STOP_SPEED_THRESHOLD) {
          task2_enter_phase(status, Q2_AD_UTURN_D);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_AD_UTURN_D_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = 0;
          status->state.status_pid.angle_output_limit = Q2_AD_UTURN_ANGLE_LIMIT;
        }
        break;

      /* ====== D 点掉头 ====== */

      case Q2_AD_UTURN_D:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = 0;
        status->state.status_pid.angle_output_limit = Q2_AD_UTURN_ANGLE_LIMIT;  /* AD掉头放大角度限幅 */
        if (task2_uturn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_DC_RETURN);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AD_FIND_DC_RETURN:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AD_SIDE_DC_RETURN);
        }
        break;

      /* ====== 回程: D→C→B→A ====== */

      case Q2_AD_SIDE_DC_RETURN:
        status->state.motion = FIND_LINE;
        status->state.status_pid.angle_output_limit = Q2_AD_NORMAL_ANGLE_LIMIT;  /* AD恢复正常角度限幅 */
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        if (task2_accept_road(status, LeftRoad, 0)) {
          task2_enter_phase(status, Q2_AD_TURN_C_RETURN);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_AD_TURN_C_LEFT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_AD_TURN_C_RETURN_ANGLE_LIMIT;
        }
        break;

      case Q2_AD_TURN_C_RETURN:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_CB_RETURN);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AD_FIND_CB_RETURN:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AD_SIDE_CB_RETURN);
        }
        break;

      case Q2_AD_SIDE_CB_RETURN:
        status->state.motion = FIND_LINE;
        if (task2_middle4_seen(status)) {
          q2_mid4_stable_cnt++;
        } else {
          q2_mid4_stable_cnt = 0;
        }
        task2_apply_side_speed(status);
        /* C→B 再次经过A4纸, 79cm前禁止消费路口 */
        if (task2_accept_road(status, LeftRoad, Q2_CB_ROAD_ENABLE_CM)) {
          task2_enter_phase(status, Q2_AD_TURN_B_RETURN);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q2_AD_TURN_B_LEFT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q2_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q2_AD_TURN_B_RETURN_ANGLE_LIMIT;
        }
        break;

      case Q2_AD_TURN_B_RETURN:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_BA_RETURN);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        }
        break;

      case Q2_AD_FIND_BA_RETURN:
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q2_FINAL_SLOW_SPEED;
        if (task2_middle6_seen(status)) {
          task2_enter_phase(status, Q2_AD_SIDE_BA_RETURN);
        }
        break;

      case Q2_AD_SIDE_BA_RETURN:
        status->state.motion = FIND_LINE;
        task2_apply_final_speed(status);
        if (task2_accept_road(status, LeftRoad, Q2_FINAL_SLOW_CM)) {
          task2_enter_phase(status, Q2_AD_FINISH);
        }
        break;

      case Q2_AD_FINISH:
        task_finish(status);
        break;
    }
  }
}

/* ---- TASK3 小工具函数 ---- */

static uint8_t q3_mid4_stable_cnt = 0;
static uint8_t q3_cd_edge_index = 0;
static float q3_cd_last_edge_pulse = 0;
static float q3_cd_edge_pulse[Q3_CD_EDGE_COUNT] = {0};

static uint8_t task3_count_bits(uint8_t value) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (value & (1 << i)) {
      count++;
    }
  }
  return count;
}

static void task3_cd_distance_reset(STATUS *status) {
  q3_cd_edge_index = 0;
  q3_cd_last_edge_pulse = status->task.phase_mileage;
  for (uint8_t i = 0; i < Q3_CD_EDGE_COUNT; i++) {
    q3_cd_edge_pulse[i] = 0;
  }
}

static void task3_cd_record_edge(STATUS *status) {
  q3_cd_edge_pulse[q3_cd_edge_index] = status->task.phase_mileage;
  q3_cd_last_edge_pulse = status->task.phase_mileage;
  q3_cd_edge_index++;
  if (q3_cd_edge_index == 3) {
    l1 = encoder_pulse_to_cm((int32_t)(q3_cd_edge_pulse[2] - q3_cd_edge_pulse[1]));
  } else if (q3_cd_edge_index == 5) {
    l2 = encoder_pulse_to_cm((int32_t)(q3_cd_edge_pulse[4] - q3_cd_edge_pulse[3]));
  }
}

static void task3_cd_distance_update(STATUS *status) {
  uint8_t cur = status->sensor.gw_analogue.digital_8bit & Q3_CD_DISTANCE_MASK;
  uint8_t black_count = task3_count_bits(cur);
  uint8_t is_black = (black_count >= Q3_CD_BLACK_COUNT_THRESHOLD);
  uint8_t is_white = (black_count <= Q3_CD_WHITE_COUNT_THRESHOLD);

  if (q3_cd_edge_index < Q3_CD_EDGE_COUNT) {
    float gap_cm = encoder_pulse_to_cm((int32_t)(status->task.phase_mileage - q3_cd_last_edge_pulse));
    if (q3_cd_edge_index != 0 && gap_cm <= Q3_CD_EDGE_MIN_GAP_CM) {
      return;
    }

    uint8_t expect_black = ((q3_cd_edge_index & 1) == 0);
    if ((expect_black && is_black) || (!expect_black && is_white)) {
      task3_cd_record_edge(status);
    }
  }
}

/* 路口映射: T 路口合并为单边路口, 不改底层 get_road_type() */
static Road task3_map_road(Road raw) {
  if (raw == TLRoad) return LeftRoad;
  if (raw == TRRoad) return RightRoad;
  return raw;
}

static void task3_enter_phase(STATUS *status, uint8_t next_phase) {
  status->task.race_phase = next_phase;
  status->task.phase_mileage = 0;
  status->task.phase_start_time = status->state.time;
  q3_mid4_stable_cnt = 0;
  if (next_phase == Q3_AB_SIDE_DC || next_phase == Q3_AD_SIDE_CD) {
    task3_cd_distance_reset(status);
  }
}

static float task3_angle_error(STATUS *status) {
  float target = status->state.tar_angle + status->state.initial_angle;
  float diff = target - status->state.cur_angle;
  if (diff > 180.0f)        diff -= 360.0f;
  else if (diff < -180.0f)  diff += 360.0f;
  return diff;
}

static uint8_t task3_turn_angle_ready(STATUS *status) {
  return (ABS(task3_angle_error(status)) < Q3_TURN_TOLERANCE_DEG);
}

static uint8_t task3_middle6_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q3_TURN_LINE_MASK_6) != 0;
}

static uint8_t task3_middle4_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q3_TURN_LINE_MASK_4) != 0;
}

/* 里程门控路口消费: 只有行驶超过 min_cm 且路口类型匹配且未消费时才接受
   用于 CD 边屏蔽 A4 横线干扰 (传入 Q3_CD_IGNORE_END_CM) */
static uint8_t task3_accept_road(STATUS *status, Road expected, float min_cm) {
  Road road = task3_map_road(status->sensor.gw_analogue.cross.cross);
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm < min_cm) return 0;
  if (road != expected) return 0;
  if (status->task.cnt_seen == 1) return 0;
  status->task.cnt_seen = 1;
  status->task.cross_cnt++;
  return 1;
}

/* 普通边速度曲线: 稳定前低速找线 → 闪冲 → 中速 → 入弯前减速 */
static void task3_apply_side_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (q3_mid4_stable_cnt < Q3_LINE_STABLE_CNT) {
    status->state.base_speed = Q3_FINAL_SLOW_SPEED;       /* 20, 不稳时低速找线 */
  } else if (cm <= Q3_STRAIGHT_FLASH_CM) {
    status->state.base_speed = Q3_FLASH_SPEED;             /* 55, 0~65cm 高速 */
  } else if (cm <= Q3_PRE_TURN_SLOW_CM) {
    status->state.base_speed = Q3_CRUISE_SPEED;            /* 40, 65~72cm 中速 */
  } else {
    status->state.base_speed = Q3_PRE_TURN_SLOW_SPEED;     /* 25, >72cm 入弯前减速 */
  }
}

/* CD 边专用速度: 先 30, 14cm 后降到 20 给视觉更多帧, 68cm 后提到 48 */
static void task3_apply_cd_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm < Q3_CD_SLOW_AFTER_CM) {
    status->state.base_speed = Q3_CD_START_SPEED;
  } else if (cm < Q3_CD_FAST_AFTER_CM) {
    status->state.base_speed = Q3_CD_SLOW_SPEED;
  } else {
    status->state.base_speed = Q3_CD_FAST_SPEED;
  }
}

/* 最后一段速度: 正常巡线 → 终点前降速 */
static void task3_apply_final_speed(STATUS *status) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm <= Q3_FINAL_SLOW_CM) {
    status->state.base_speed = Q3_CRUISE_SPEED;
  } else {
    status->state.base_speed = Q3_FINAL_SLOW_SPEED;
  }
}

static void driver_task3(STATUS *status) {
  if (status->task.start_pose == START_AB) {
    /* ====== AB 发车: A→D→C→B→A, 逆时针, 全部左转 ====== */
    Road road = task3_map_road(status->sensor.gw_analogue.cross.cross);

    /* cnt_seen 复位: 仿 TASK2, 中间4路稳定后才清零, 防横线干扰误清零 */
    if (road == Straight && status->state.motion == FIND_LINE
        && status->task.race_phase != Q3_AB_FIND_AD
        && status->task.race_phase != Q3_AB_FIND_DC
        && status->task.race_phase != Q3_AB_FIND_CB
        && status->task.race_phase != Q3_AB_FIND_BA) {
      if (status->task.race_phase == Q3_AB_SIDE_BA_FINAL || q3_mid4_stable_cnt >= Q3_LINE_STABLE_CNT) {
        status->task.cnt_seen = 0;
      }
    }

    switch (status->task.race_phase) {

      /* ---- 起步, 走到 A 点左转 ---- */
      case Q3_AB_START_TO_A:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q3_AB_FIRST_TURN_SPEED;
        if (task3_accept_road(status, LeftRoad, 0)) {
          task3_enter_phase(status, Q3_AB_TURN_A_TO_AD);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q3_AB_TURN_A_LEFT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q3_AB_FIRST_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q3_AB_TURN_A_ANGLE_LIMIT;
        }
        break;

      /* ---- 转弯: 角度环 KEEP_ANGLE ---- */
      case Q3_AB_TURN_A_TO_AD:
      case Q3_AB_TURN_D_TO_DC:
      case Q3_AB_TURN_C_TO_CB:
      case Q3_AB_TURN_B_TO_BA:
        status->task.task_running = 1;
        status->state.motion = KEEP_ANGLE;
        if (task3_turn_angle_ready(status)) {
          uint8_t next;
          if (status->task.race_phase == Q3_AB_TURN_A_TO_AD)      next = Q3_AB_FIND_AD;
          else if (status->task.race_phase == Q3_AB_TURN_D_TO_DC) next = Q3_AB_FIND_DC;
          else if (status->task.race_phase == Q3_AB_TURN_C_TO_CB) next = Q3_AB_FIND_CB;
          else                                                     next = Q3_AB_FIND_BA;
          task3_enter_phase(status, next);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q3_FINAL_SLOW_SPEED;
        }
        break;

      /* ---- 转弯后低速找线, 中间6路看到线即切换 ---- */
      case Q3_AB_FIND_AD:
      case Q3_AB_FIND_DC:
      case Q3_AB_FIND_CB:
      case Q3_AB_FIND_BA:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q3_FINAL_SLOW_SPEED;
        if (task3_middle6_seen(status)) {
          uint8_t next;
          if (status->task.race_phase == Q3_AB_FIND_AD)      next = Q3_AB_SIDE_AD;
          else if (status->task.race_phase == Q3_AB_FIND_DC) next = Q3_AB_SIDE_DC;
          else if (status->task.race_phase == Q3_AB_FIND_CB) next = Q3_AB_SIDE_CB;
          else                                                next = Q3_AB_SIDE_BA_FINAL;
          task3_enter_phase(status, next);
        }
        break;

      /* ---- 普通边巡线: AD / DC(前半段) / CB ---- */
      case Q3_AB_SIDE_AD:
      case Q3_AB_SIDE_CB:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        if (task3_middle4_seen(status)) {
          q3_mid4_stable_cnt++;
        } else {
          q3_mid4_stable_cnt = 0;
        }
        task3_apply_side_speed(status);
        if (task3_accept_road(status, LeftRoad, Q3_ROAD_ENABLE_CM)) {
          uint8_t next;
          float angle;
          if (status->task.race_phase == Q3_AB_SIDE_AD) {
            next = Q3_AB_TURN_D_TO_DC;
            angle = Q3_AB_TURN_D_LEFT_ANGLE;
          } else {
            next = Q3_AB_TURN_B_TO_BA;
            angle = Q3_AB_TURN_B_LEFT_ANGLE;
          }
          task3_enter_phase(status, next);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = angle;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q3_TURN_SPEED;
          if (next == Q3_AB_TURN_D_TO_DC) status->state.status_pid.angle_output_limit = Q3_AB_TURN_D_ANGLE_LIMIT;
          else                            status->state.status_pid.angle_output_limit = Q3_AB_TURN_B_ANGLE_LIMIT;
        }
        break;

      /* ---- CD 边: 0~14cm 速度30 → 14~68cm 速度20 → 68cm后速度48 → 等 C 点 ---- */
      case Q3_AB_SIDE_DC:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        if (task3_middle4_seen(status)) {
          q3_mid4_stable_cnt++;
        } else {
          q3_mid4_stable_cnt = 0;
        }
        task3_apply_cd_speed(status);
        task3_cd_distance_update(status);
        if (task3_accept_road(status, LeftRoad, Q3_CD_IGNORE_END_CM)) {
          task3_enter_phase(status, Q3_AB_TURN_C_TO_CB);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q3_AB_TURN_C_LEFT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q3_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q3_AB_TURN_C_ANGLE_LIMIT;
        }
        break;

      /* ---- BA 最后一段: 巡线回发车点 ---- */
      case Q3_AB_SIDE_BA_FINAL:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        task3_apply_final_speed(status);
        if (task3_accept_road(status, LeftRoad, Q3_FINAL_SLOW_CM)) {
          task3_enter_phase(status, Q3_AB_FINISH);
        }
        break;

      case Q3_AB_FINISH:
        task3_finished = 1;
        task_finish(status);
        break;
    }

  } else {
    /* ====== AD 发车: A→B→C→D→A, 顺时针, 全部右转 ====== */
    Road road = task3_map_road(status->sensor.gw_analogue.cross.cross);

    if (road == Straight && status->state.motion == FIND_LINE
        && status->task.race_phase != Q3_AD_FIND_AB
        && status->task.race_phase != Q3_AD_FIND_BC
        && status->task.race_phase != Q3_AD_FIND_CD
        && status->task.race_phase != Q3_AD_FIND_DA) {
      if (status->task.race_phase == Q3_AD_SIDE_DA_FINAL || q3_mid4_stable_cnt >= Q3_LINE_STABLE_CNT) {
        status->task.cnt_seen = 0;
      }
    }

    switch (status->task.race_phase) {

      case Q3_AD_START_TO_A:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q3_AD_FIRST_TURN_SPEED;
        if (task3_accept_road(status, RightRoad, 0)) {
          task3_enter_phase(status, Q3_AD_TURN_A_TO_AB);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q3_AD_TURN_A_RIGHT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q3_AD_FIRST_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q3_AD_TURN_A_ANGLE_LIMIT;
        }
        break;

      /* ---- 转弯 ---- */
      case Q3_AD_TURN_A_TO_AB:
      case Q3_AD_TURN_B_TO_BC:
      case Q3_AD_TURN_C_TO_CD:
      case Q3_AD_TURN_D_TO_DA:
        status->task.task_running = 1;
        status->state.motion = KEEP_ANGLE;
        if (task3_turn_angle_ready(status)) {
          uint8_t next;
          if (status->task.race_phase == Q3_AD_TURN_A_TO_AB)      next = Q3_AD_FIND_AB;
          else if (status->task.race_phase == Q3_AD_TURN_B_TO_BC) next = Q3_AD_FIND_BC;
          else if (status->task.race_phase == Q3_AD_TURN_C_TO_CD) next = Q3_AD_FIND_CD;
          else                                                     next = Q3_AD_FIND_DA;
          task3_enter_phase(status, next);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q3_FINAL_SLOW_SPEED;
        }
        break;

      /* ---- 找线 ---- */
      case Q3_AD_FIND_AB:
      case Q3_AD_FIND_BC:
      case Q3_AD_FIND_CD:
      case Q3_AD_FIND_DA:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q3_FINAL_SLOW_SPEED;
        if (task3_middle6_seen(status)) {
          uint8_t next;
          if (status->task.race_phase == Q3_AD_FIND_AB)      next = Q3_AD_SIDE_AB;
          else if (status->task.race_phase == Q3_AD_FIND_BC) next = Q3_AD_SIDE_BC;
          else if (status->task.race_phase == Q3_AD_FIND_CD) next = Q3_AD_SIDE_CD;
          else                                                next = Q3_AD_SIDE_DA_FINAL;
          task3_enter_phase(status, next);
        }
        break;

      /* ---- 普通边巡线: AB / BC ---- */
      case Q3_AD_SIDE_AB:
      case Q3_AD_SIDE_BC:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        if (task3_middle4_seen(status)) {
          q3_mid4_stable_cnt++;
        } else {
          q3_mid4_stable_cnt = 0;
        }
        task3_apply_side_speed(status);
        if (task3_accept_road(status, RightRoad, Q3_ROAD_ENABLE_CM)) {
          uint8_t next;
          float angle;
          if (status->task.race_phase == Q3_AD_SIDE_AB) {
            next = Q3_AD_TURN_B_TO_BC;
            angle = Q3_AD_TURN_B_RIGHT_ANGLE;
          } else {
            next = Q3_AD_TURN_C_TO_CD;
            angle = Q3_AD_TURN_C_RIGHT_ANGLE;
          }
          task3_enter_phase(status, next);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = angle;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q3_TURN_SPEED;
          if (next == Q3_AD_TURN_B_TO_BC) status->state.status_pid.angle_output_limit = Q3_AD_TURN_B_ANGLE_LIMIT;
          else                            status->state.status_pid.angle_output_limit = Q3_AD_TURN_C_ANGLE_LIMIT;
        }
        break;

      /* ---- CD 边: 0~14cm 速度30 → 14~68cm 速度20 → 68cm后速度48 → 等 D 点 ---- */
      case Q3_AD_SIDE_CD:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        if (task3_middle4_seen(status)) {
          q3_mid4_stable_cnt++;
        } else {
          q3_mid4_stable_cnt = 0;
        }
        task3_apply_cd_speed(status);
        task3_cd_distance_update(status);
        if (task3_accept_road(status, RightRoad, Q3_CD_IGNORE_END_CM)) {
          task3_enter_phase(status, Q3_AD_TURN_D_TO_DA);
          status->state.initial_angle = status->state.cur_angle;
          status->state.tar_angle = Q3_AD_TURN_D_RIGHT_ANGLE;
          status->state.motion = KEEP_ANGLE;
          status->state.base_speed = Q3_TURN_SPEED;
          status->state.status_pid.angle_output_limit = Q3_AD_TURN_D_ANGLE_LIMIT;
        }
        break;

      /* ---- DA 最后一段 ---- */
      case Q3_AD_SIDE_DA_FINAL:
        status->task.task_running = 1;
        status->state.motion = FIND_LINE;
        task3_apply_final_speed(status);
        if (task3_accept_road(status, RightRoad, Q3_FINAL_SLOW_CM)) {
          task3_enter_phase(status, Q3_AD_FINISH);
        }
        break;

      case Q3_AD_FINISH:
        task3_finished = 1;
        task_finish(status);
        break;
    }
  }
}

/* ---- TASK4 helpers ---- */

static uint8_t q4_line_stable_cnt = 0;

static Road task4_map_road(Road raw) {
  if (raw == TLRoad) return LeftRoad;
  if (raw == TRRoad) return RightRoad;
  return raw;
}

static void task4_enter_phase(STATUS *status, uint8_t next_phase) {
  status->task.race_phase = next_phase;
  status->task.phase_mileage = 0;
  status->task.phase_start_time = status->state.time;
  q4_line_stable_cnt = 0;
}

static float task4_angle_error(STATUS *status) {
  float target = status->state.tar_angle + status->state.initial_angle;
  float diff = target - status->state.cur_angle;
  if (diff > 180.0f)        diff -= 360.0f;
  else if (diff < -180.0f)  diff += 360.0f;
  return diff;
}

static uint8_t task4_turn_angle_ready(STATUS *status) {
  return (ABS(task4_angle_error(status)) < Q4_TURN_TOLERANCE_DEG);
}

static uint8_t task4_scan_turn_angle_ready(STATUS *status) {
  return (ABS(task4_angle_error(status)) <= 0.0f);
}

static uint8_t task4_middle6_seen(STATUS *status) {
  return (status->sensor.gw_analogue.digital_8bit & Q4_TURN_LINE_MASK_6) != 0;
}

static void task4_start_left_turn(STATUS *status, uint8_t next_phase, float angle, float limit) {
  task4_enter_phase(status, next_phase);
  status->state.initial_angle = status->state.cur_angle;
  status->state.tar_angle = angle;
  status->state.motion = KEEP_ANGLE;
  status->state.base_speed = Q4_TURN_BASE_SPEED;
  status->state.status_pid.angle_output_limit = limit;
}

static void task4_start_heading_drive(STATUS *status, uint8_t next_phase) {
  task4_enter_phase(status, next_phase);
  status->state.initial_angle = status->state.cur_angle;
  status->state.tar_angle = 0;
  status->state.motion = KEEP_ANGLE;
  status->state.base_speed = Q4_SCAN_SPEED;
  status->state.status_pid.angle_output_limit = Q4_STRAIGHT_ANGLE_LIMIT;
}

static uint8_t task4_accept_road(STATUS *status, Road road, Road expected, float min_cm) {
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm < min_cm) return 0;
  if (road != expected) return 0;
  if (status->task.cnt_seen == 1) return 0;
  status->task.cnt_seen = 1;
  status->task.cross_cnt++;
  return 1;
}

static void task4_wait_stop(STATUS *status, uint8_t next_phase) {
  status->state.motion = STOP;
  status->state.base_speed = 0;
  if (status->state.time - status->task.phase_start_time >= Q4_SCAN_STOP_MS) {
    task4_start_heading_drive(status, next_phase);
  }
}

static void driver_task4(STATUS *status) {
  Road road = task4_map_road(status->sensor.gw_analogue.cross.cross);

  if (road == Straight && status->state.motion == FIND_LINE
      && status->task.race_phase != Q4_FIND_AD
      && status->task.race_phase != Q4_FIND_DC) {
    status->task.cnt_seen = 0;
  }

  switch (status->task.race_phase) {
    case Q4_START_TO_A:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q4_TRACK_SPEED;
      if (task4_accept_road(status, road, LeftRoad, Q4_START_TO_A_MIN_CM)) {
        task4_start_left_turn(status, Q4_TURN_A_TO_AD, Q4_TURN_A_LEFT_ANGLE, Q4_TURN_A_ANGLE_LIMIT);
      }
      break;

    case Q4_TURN_A_TO_AD:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_TURN_BASE_SPEED;
      if (task4_turn_angle_ready(status)) {
        task4_enter_phase(status, Q4_FIND_AD);
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q4_FIND_LINE_SPEED;
      }
      break;

    case Q4_FIND_AD:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q4_FIND_LINE_SPEED;
      if (task4_middle6_seen(status)) {
        q4_line_stable_cnt++;
      } else {
        q4_line_stable_cnt = 0;
      }
      if (q4_line_stable_cnt >= Q4_LINE_STABLE_CNT) {
        task4_enter_phase(status, Q4_SIDE_AD);
      }
      break;

    case Q4_SIDE_AD:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q4_TRACK_SPEED;
      if (task4_accept_road(status, road, LeftRoad, Q4_ROAD_ENABLE_CM)) {
        task4_start_left_turn(status, Q4_TURN_D_TO_DC, Q4_TURN_D_LEFT_ANGLE, Q4_TURN_D_ANGLE_LIMIT);
      }
      break;

    case Q4_TURN_D_TO_DC:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_TURN_BASE_SPEED;
      if (task4_turn_angle_ready(status)) {
        task4_enter_phase(status, Q4_FIND_DC);
        status->state.motion = FIND_LINE;
        status->state.base_speed = Q4_FIND_LINE_SPEED;
      }
      break;

    case Q4_FIND_DC:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q4_FIND_LINE_SPEED;
      if (task4_middle6_seen(status)) {
        q4_line_stable_cnt++;
      } else {
        q4_line_stable_cnt = 0;
      }
      if (q4_line_stable_cnt >= Q4_LINE_STABLE_CNT) {
        task4_enter_phase(status, Q4_SIDE_DC_TO_SCAN_START);
      }
      break;

    case Q4_SIDE_DC_TO_SCAN_START:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q4_SCAN_SPEED;
      if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) >= Q4_DC_SCAN_START_CM) {
        task4_start_left_turn(status, Q4_SCAN_TURN_1, Q4_SCAN_TURN_1_LEFT_ANGLE, Q4_SCAN_TURN_1_ANGLE_LIMIT);
      }
      break;

    case Q4_SCAN_TURN_1:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_TURN_BASE_SPEED;
      if (task4_scan_turn_angle_ready(status)) {
        task4_start_heading_drive(status, Q4_SCAN_DRIVE_1);
      }
      break;

    case Q4_SCAN_DRIVE_1:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_SCAN_SPEED;
      if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) >= Q4_SCAN_SEGMENT_CM) {
        task4_enter_phase(status, Q4_SCAN_STOP_1);
      }
      break;

    case Q4_SCAN_STOP_1:
      status->task.task_running = 1;
      task4_wait_stop(status, Q4_SCAN_DRIVE_2);
      break;

    case Q4_SCAN_DRIVE_2:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_SCAN_SPEED;
      if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) >= Q4_SCAN_SEGMENT_CM) {
        task4_enter_phase(status, Q4_SCAN_STOP_2);
      }
      break;

    case Q4_SCAN_STOP_2:
      status->task.task_running = 1;
      status->state.motion = STOP;
      status->state.base_speed = 0;
      if (status->state.time - status->task.phase_start_time >= Q4_SCAN_STOP_MS) {
        task4_start_left_turn(status, Q4_SCAN_TURN_2, Q4_SCAN_TURN_2_LEFT_ANGLE, Q4_SCAN_TURN_2_ANGLE_LIMIT);
      }
      break;

    case Q4_SCAN_TURN_2:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_TURN_BASE_SPEED;
      if (task4_scan_turn_angle_ready(status)) {
        task4_start_heading_drive(status, Q4_SCAN_DRIVE_3);
      }
      break;

    case Q4_SCAN_DRIVE_3:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_SCAN_SPEED;
      if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) >= Q4_SCAN_SEGMENT_CM) {
        task4_enter_phase(status, Q4_SCAN_STOP_3);
      }
      break;

    case Q4_SCAN_STOP_3:
      status->task.task_running = 1;
      status->state.motion = STOP;
      status->state.base_speed = 0;
      if (status->state.time - status->task.phase_start_time >= Q4_SCAN_STOP_MS) {
        task4_start_left_turn(status, Q4_SCAN_TURN_3, Q4_SCAN_TURN_3_LEFT_ANGLE, Q4_SCAN_TURN_3_ANGLE_LIMIT);
      }
      break;

    case Q4_SCAN_TURN_3:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_TURN_BASE_SPEED;
      if (task4_scan_turn_angle_ready(status)) {
        task4_start_heading_drive(status, Q4_SCAN_DRIVE_4);
      }
      break;

    case Q4_SCAN_DRIVE_4:
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;
      status->state.base_speed = Q4_SCAN_SPEED;
      if (encoder_pulse_to_cm((int32_t)status->task.phase_mileage) >= Q4_SCAN_SEGMENT_CM) {
        task4_enter_phase(status, Q4_SCAN_STOP_4);
      }
      break;

    case Q4_SCAN_STOP_4:
      status->task.task_running = 1;
      status->state.motion = STOP;
      status->state.base_speed = 0;
      if (status->state.time - status->task.phase_start_time >= Q4_SCAN_STOP_MS) {
        task4_enter_phase(status, Q4_FINISH);
      }
      break;

    case Q4_FINISH:
      task_finish(status);
      break;
  }
}

void update_task_led(STATUS *status) {
  switch (status->task.task_id) {
    case TASK_BASIC_1:
      status->device.led_on_board.on = 1;
      status->device.led1.on = 0;
      status->device.led2.on = 1;
      break;
    case TASK_BASIC_2:
      if (status->task.start_pose == START_AB) {
        status->device.led_on_board.on = 1;
        status->device.led1.on = 1;
        status->device.led2.on = 1;
      } else {
        status->device.led_on_board.on = 0;
        status->device.led1.on = 1;
        status->device.led2.on = 1;
      }
      break;
    case TASK_ADV_1:
      if (status->task.start_pose == START_AB) {
        status->device.led_on_board.on = 1;
        status->device.led1.on = 0;
        status->device.led2.on = 0;
      } else {
        status->device.led_on_board.on = 0;
        status->device.led1.on = 0;
        status->device.led2.on = 0;
      }
      break;
    case TASK_ADV_2:
      status->device.led_on_board.on = 1;
      status->device.led1.on = 1;
      status->device.led2.on = 0;
      break;
  }
}

void update_task(STATUS *status) {
  if (status->task.stop_request) {
    task_stop(status);
    return;
  }

  if (!status->task.task_running && !status->task.armed) {
    if (status->task.task_select_request) {
      task_select(status, status->task.requested_task_id);
      status->task.task_select_request = 0;
    }

    if (status->task.pose_switch_request) {
      if (status->task.task_id == TASK_BASIC_2 || status->task.task_id == TASK_ADV_1) {
        status->task.start_pose = (status->task.start_pose == START_AB) ? START_AD : START_AB;
        update_task_led(status);
      }
      status->task.pose_switch_request = 0;
    }
  }

  if (status->task.start_request) {
    if (!status->task.task_running && !status->task.armed) {
      status->task.armed = 1;
      task_start(status);
    }
    status->task.start_request = 0;
  }

  if (!status->task.armed) {
    return;
  }

  int32_t wheel0_pulse = status->motor.wheel[0].cur_speed;
  int32_t wheel1_pulse = status->motor.wheel[1].cur_speed;
  if (wheel0_pulse < 0) wheel0_pulse = -wheel0_pulse;
  if (wheel1_pulse < 0) wheel1_pulse = -wheel1_pulse;
  status->task.phase_mileage += ((float)wheel0_pulse + (float)wheel1_pulse) / 2.0f;

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      driver_task1(status);
      break;
    case TASK_BASIC_2:
      driver_task2(status);
      break;
    case TASK_ADV_1:
      driver_task3(status);
      break;
    case TASK_ADV_2:
      driver_task4(status);
      break;
  }
}
