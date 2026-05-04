#include "Defect.h"
#include "status.h"
#include "math_tool.h"
#include "log.h"
#include "pid.h"
#include <stdio.h>

/* Q1 tunable parameters */
#define Q1_START_PULSE                 200   /* 起步脉冲阈值, 待标定 */
#define Q1_TURN_TARGET_ANGLE           79.0f /* 左转目标角度 */
#define Q1_TURN_TARGET_ANGLE_C         77.0f /* C点左转目标角度, 多转4°补偿 */
#define Q1_TURN_TOLERANCE_DEG          6.0f  /* 转弯完成的角度容差 */
#define Q1_TURN_TO_FIND_TOLERANCE_DEG  10.0f /* 转弯→找线接管角度阈值 */
#define Q1_TURN_LINE_MASK_6            0x7E  /* bit1~bit6, 中间6路 */
#define Q1_TURN_LINE_MASK_4            0x3C  /* bit2~bit5, 中间4路 */
#define Q1_LINE_STABLE_CNT             3     /* 中间4路稳定帧数 */
#define Q1_FLASH_SPEED                 55    /* 直线冲刺速度 */
#define Q1_CRUISE_SPEED                44    /* 巡线直走速度 */
#define Q1_TURN_SPEED                  35    /* 转弯时的基础速度 */
#define Q1_FINAL_SLOW_SPEED            20    /* 终点前降速 / 找线低速 */
#define Q1_STRAIGHT_FLASH_CM           65.0f /* 直线冲刺距离(cm) */
#define Q1_FINAL_SLOW_CM               70.0f /* 终点降速距离(cm) */
#define Q1_BA_PULSE                    3500  /* BA 边停车脉冲阈值, 待标定 */
#define Q1_BA_STOP_CM                  80.0f /* BA 边停车里程阈值(cm) */

/* Q2 tunable parameters (AB发车) */
#define Q2_TURN_A_LEFT_ANGLE          76.0f /* A点左转角度 */
#define Q2_TURN_D_LEFT_ANGLE          80.0f /* D点左转角度 */
#define Q2_TURN_C_LEFT_ANGLE          80.0f /* C点左转角度 */
#define Q2_UTURN_B_ANGLE              170.0f /* B点掉头角度 */
#define Q2_TURN_C_RIGHT_ANGLE         -80.0f /* 回程C点右转角度 */
#define Q2_TURN_D_RIGHT_ANGLE         -80.0f /* 回程D点右转角度 */
#define Q2_TURN_FIND_TOLERANCE_DEG    10.0f /* 90°转弯→找线接管角度阈值 */
#define Q2_UTURN_TOLERANCE_DEG        8.0f /* 掉头→找线接管角度阈值 */
#define Q2_TURN_LINE_MASK_6           0x7E  /* bit1~bit6, 中间6路 */
#define Q2_TURN_LINE_MASK_4           0x3C  /* bit2~bit5, 中间4路 */
#define Q2_LINE_STABLE_CNT            3     /* 中间4路稳定帧数 */
#define Q2_FLASH_SPEED                55    /* 直线冲刺速度 */
#define Q2_CRUISE_SPEED               44    /* 巡线直走速度 */
#define Q2_TURN_SPEED                 35    /* 转弯时的基础速度 */
#define Q2_UTURN_SPEED                20    /* 掉头速度 */
#define Q2_FINAL_SLOW_SPEED           20    /* 终点前降速 / 找线低速 */
#define Q2_STRAIGHT_FLASH_CM          65.0f /* 直线冲刺距离(cm) */
#define Q2_CB_ROAD_ENABLE_CM          79.0f /* C→B 路口使能里程(cm) */
#define Q2_BC_ROAD_ENABLE_CM          79.0f /* B→C 路口使能里程(cm) */
#define Q2_FINAL_SLOW_CM              80.0f /* 终点降速距离(cm) */
#define Q2_UTURN_PRE_SLOW_CM          70.0f /* B点掉头前预减速距离(cm) */
#define Q2_UTURN_PRE_SLOW_SPEED       20    /* B点掉头前预减速目标速度 */
#define Q2_UTURN_ANGLE_LIMIT          80.0f /* 掉头时角度环输出限幅 */
#define Q2_NORMAL_ANGLE_LIMIT         25.0f /* 正常角度环输出限幅 */
#define Q2_UTURN_STOP_SPEED_THRESHOLD 5     /* 掉头前停车轮速阈值 */

/* Q2 AD-specific tunable angles (AD发车: 顺时针去程右转, 逆时针回程左转) */
#define Q2_AD_TURN_A_RIGHT_ANGLE       -80.0f /* A点右转进入AB */
#define Q2_AD_TURN_B_RIGHT_ANGLE       -80.0f /* B点右转进入BC */
#define Q2_AD_TURN_C_RIGHT_ANGLE       -80.0f /* C点右转进入CD */
#define Q2_AD_UTURN_D_ANGLE            -170.0f /* D点掉头角度 */
#define Q2_AD_TURN_C_LEFT_ANGLE         80.0f /* 回程C点左转进入CB */
#define Q2_AD_TURN_B_LEFT_ANGLE         80.0f /* 回程B点左转进入BA */

/* Q3/Q4 feed-forward defaults. wheel0 = wheel[0]/which 1, wheel1 = wheel[1]/which 2. */
#define Q3_WHEEL0_FF_OFFSET             157.0f
#define Q3_WHEEL0_FF_K                  18.3f
#define Q3_WHEEL0_FF_MIN                254.0f
#define Q3_WHEEL1_FF_OFFSET             157.0f
#define Q3_WHEEL1_FF_K                  18.3f
#define Q3_WHEEL1_FF_MIN                254.0f

#define Q4_WHEEL0_FF_OFFSET             157.0f
#define Q4_WHEEL0_FF_K                  18.3f
#define Q4_WHEEL0_FF_MIN                254.0f
#define Q4_WHEEL1_FF_OFFSET             157.0f
#define Q4_WHEEL1_FF_K                  18.3f
#define Q4_WHEEL1_FF_MIN                254.0f

extern uint8_t cross_cnt;
extern uint8_t left_cnt;
extern uint8_t cross_delay;
extern Road road_buf;

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
      status->task.race_phase = Q1_START_TO_A;
      break;
    case TASK_BASIC_2:
      if (status->task.start_pose == START_AB) {
        status->task.race_phase = Q2_AB_TURN_A_TO_AD;
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q2_TURN_A_LEFT_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        status->task.task_running = 1;
      } else {
        status->task.race_phase = Q2_AD_TURN_A_TO_AB;
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q2_AD_TURN_A_RIGHT_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q2_TURN_SPEED;
        status->task.task_running = 1;
      }
      break;
    case TASK_ADV_1:
      status->task.race_phase = 0;  // TODO: Q3_RACE_PHASE
      break;
    case TASK_ADV_2:
      status->task.race_phase = 0;  // TODO: Q4_RACE_PHASE
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
      status->state.base_speed = Q1_TURN_SPEED;

      if (task1_accept_left_road(status, road)) {
        task1_enter_phase(status, Q1_TURN_A);
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q1_TURN_TARGET_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q1_TURN_SPEED;
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
        status->state.tar_angle = (next == Q1_TURN_C) ? Q1_TURN_TARGET_ANGLE_C : Q1_TURN_TARGET_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q1_TURN_SPEED;
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
    status->state.base_speed = Q2_FINAL_SLOW_SPEED;
  } else if (cm <= Q2_STRAIGHT_FLASH_CM) {
    status->state.base_speed = Q2_FLASH_SPEED;
  } else {
    status->state.base_speed = Q2_CRUISE_SPEED;
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
        status->state.base_speed = Q2_TURN_SPEED;
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
          status->state.status_pid.angle_output_limit = Q2_UTURN_ANGLE_LIMIT;
        }
        break;

      /* ====== B 点掉头 ====== */

      case Q2_AB_UTURN_B:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = 0;
        status->state.status_pid.angle_output_limit = Q2_UTURN_ANGLE_LIMIT;  /* 掉头放大角度限幅 */
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
        status->state.status_pid.angle_output_limit = Q2_NORMAL_ANGLE_LIMIT;  /* 恢复正常角度限幅 */
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
        status->state.base_speed = Q2_TURN_SPEED;
        if (task2_turn_angle_ready(status)) {
          task2_enter_phase(status, Q2_AD_FIND_AB_OUT);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q2_FINAL_SLOW_SPEED;
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
          status->state.status_pid.angle_output_limit = Q2_UTURN_ANGLE_LIMIT;
        }
        break;

      /* ====== D 点掉头 ====== */

      case Q2_AD_UTURN_D:
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = 0;
        status->state.status_pid.angle_output_limit = Q2_UTURN_ANGLE_LIMIT;  /* 掉头放大角度限幅 */
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
        status->state.status_pid.angle_output_limit = Q2_NORMAL_ANGLE_LIMIT;  /* 恢复正常角度限幅 */
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

static void driver_task3(STATUS *status) {
  if (!status->task.task_running) return;
  status->task.task_running = 1;

  if (status->task.race_phase == 0) {
    status->state.initial_angle = status->state.cur_angle;
    status->state.tar_angle = -90.0f;
    status->state.motion = KEEP_ANGLE;
    status->state.base_speed = 35;
    status->task.race_phase = 1;
    return;
  }

  if (status->task.race_phase == 1) {
    float target = status->state.tar_angle + status->state.initial_angle;
    float diff_angle = target - status->state.cur_angle;
    if (diff_angle > 180.0f)  diff_angle -= 360.0f;
    else if (diff_angle < -180.0f) diff_angle += 360.0f;

    if (ABS(diff_angle) < 3.0f) {
      task_finish(status);
    }
  }
}

static void driver_task4(STATUS *status) {
  if (!status->task.task_running) return;
  Road road = status->sensor.gw_analogue.cross.cross;
  if (road == TLRoad) road = LeftRoad;
  if (road == TRRoad) road = RightRoad;

  if (status->task.race_phase == 0) {
    status->task.task_running = 1;
    status->state.base_speed = 30;
    status->state.motion = FIND_LINE;
    status->task.race_phase = 1;
    return;
  }

  if (status->task.race_phase == 1 && road == LeftRoad) {
    status->state.motion = STOP;
    status->task.race_phase = 2;
    return;
  }

  if (status->task.race_phase == 2) {
    float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
    printf("%.2f\r\n", cm);
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
