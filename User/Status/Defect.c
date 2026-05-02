#include "Defect.h"
#include "status.h"

#include "log.h"
#include "pid.h"

/* Q1 tunable parameters */
#define Q1_START_TO_A_TIME  500       /* ms, 起步到 A 点的时间阈值 */
#define Q1_TURN_TARGET_ANGLE 90.0f    /* 左转目标角度 */
#define Q1_TURN_TOLERANCE    3.0f     /* 转弯完成的角度容差 */
#define Q1_TURN_STABLE_MS    400      /* ms, 角度需稳定这段时间才算完成 */
#define Q1_CRUISE_SPEED      40       /* 巡线直走速度 */
#define Q1_TURN_SPEED        30       /* 转弯时的基础速度 */
#define Q1_BA_LENGTH_CM      50.0f    /* BA 边停车里程阈值(cm), 待标定 */

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

  status->state.status_pid.follow_line_pid.error = 0;
  status->state.status_pid.follow_line_pid.last_error = 0;
  status->state.status_pid.follow_line_pid.integral = 0;
  status->state.status_pid.follow_line_pid.derivative = 0;
  status->state.status_pid.follow_line_pid.out = 0;
  status->state.status_pid.follow_line_pid.is_first = 1;

  status->state.status_pid.keep_angle_pid.error = 0;
  status->state.status_pid.keep_angle_pid.last_error = 0;
  status->state.status_pid.keep_angle_pid.integral = 0;
  status->state.status_pid.keep_angle_pid.derivative = 0;
  status->state.status_pid.keep_angle_pid.out = 0;
  status->state.status_pid.keep_angle_pid.is_first = 1;

  status->state.initial_angle = status->state.cur_angle;

  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      status->task.race_phase = Q1_START_TO_A;
      break;
    case TASK_BASIC_2:
      status->task.race_phase = 0;  // TODO: Q2_RACE_PHASE
      break;
    case TASK_ADV_1:
      status->task.race_phase = 0;  // TODO: Q3_RACE_PHASE
      break;
    case TASK_ADV_2:
      status->task.race_phase = 0;  // TODO: Q4_RACE_PHASE
      break;
  }

  status->task.phase_start_time = status->state.time;

  status->state.motion = STOP;
  status->state.base_speed = 0;
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

static void task_set_phase(STATUS *status, uint8_t next_phase) {
  status->task.race_phase = next_phase;
  status->task.phase_start_time = status->state.time;
  status->task.phase_mileage = 0;
  status->task.cnt_seen = 0;
}

static uint8_t q1_final_stop_condition(STATUS *status, Road road) {
  /* 方案 A: 里程停车 (待标定) */
  float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
  if (cm > Q1_BA_LENGTH_CM) {
    return 1;
  }
  /* 方案 B: 最终路口辅助停车 (预留)
  if (road == LeftRoad && status->task.cnt_seen == 0) {
    return 1;
  }
  */
  return 0;
}

//待定

static void driver_task1(STATUS *status) {
  /* 1. T 型路口合并: 不改传感器层, 只在 TASK1 内部映射 */
  Road road = status->sensor.gw_analogue.cross.cross;
  if (road == TLRoad) road = LeftRoad;
  if (road == TRRoad) road = RightRoad;

  /* 2. cnt_seen 维护: Straight 时清零, 允许下一个路口被消费 */
  if (road == Straight) {
    status->task.cnt_seen = 0;
  }

  switch (status->task.race_phase) {

    /* ---- 起步到 A 点: 先走一段, 避免起步就误判路口 ---- */
    case Q1_START_TO_A:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q1_TURN_SPEED;

      if (status->state.time - status->task.phase_start_time >= Q1_START_TO_A_TIME) {
        task_set_phase(status, Q1_TURN_A);
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q1_TURN_TARGET_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q1_TURN_SPEED;
      }
      break;

    /* ---- 转弯阶段: A / D / C / B ---- */
    case Q1_TURN_A:
    case Q1_TURN_D:
    case Q1_TURN_C:
    case Q1_TURN_B: {
      status->task.task_running = 1;
      status->state.motion = KEEP_ANGLE;

      float target = status->state.tar_angle + status->state.initial_angle;
      float diff_angle = target - status->state.cur_angle;
      if (diff_angle > 180.0f) diff_angle -= 360.0f;
      else if (diff_angle < -180.0f) diff_angle += 360.0f;

      if (ABS(diff_angle) < Q1_TURN_TOLERANCE) {
        if (status->state.time - status->task.phase_start_time >= Q1_TURN_STABLE_MS) {
          uint8_t next;
          if (status->task.race_phase == Q1_TURN_A) next = Q1_SIDE_AD;
          else if (status->task.race_phase == Q1_TURN_D) next = Q1_SIDE_DC;
          else if (status->task.race_phase == Q1_TURN_C) next = Q1_SIDE_CB;
          else next = Q1_BA_FINAL;
          task_set_phase(status, next);
          status->state.motion = FIND_LINE;
          status->state.base_speed = Q1_CRUISE_SPEED;
        }
      } else {
        status->task.phase_start_time = status->state.time;
      }
      break;
    }

    /* ---- 巡线直边: AD / DC / CB ---- */
    case Q1_SIDE_AD:
    case Q1_SIDE_DC:
    case Q1_SIDE_CB:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q1_CRUISE_SPEED;

      if (road == LeftRoad && status->task.cnt_seen == 0) {
        status->task.cnt_seen = 1;
        status->task.cross_cnt++;

        uint8_t next;
        if (status->task.race_phase == Q1_SIDE_AD) next = Q1_TURN_D;
        else if (status->task.race_phase == Q1_SIDE_DC) next = Q1_TURN_C;
        else next = Q1_TURN_B;

        task_set_phase(status, next);
        status->state.initial_angle = status->state.cur_angle;
        status->state.tar_angle = Q1_TURN_TARGET_ANGLE;
        status->state.motion = KEEP_ANGLE;
        status->state.base_speed = Q1_TURN_SPEED;
      }
      break;

    /* ---- BA 最后一段: 巡线回到停车点 ---- */
    case Q1_BA_FINAL:
      status->task.task_running = 1;
      status->state.motion = FIND_LINE;
      status->state.base_speed = Q1_CRUISE_SPEED;

      if (q1_final_stop_condition(status, road)) {
        task_finish(status);
      }
      break;
  }
}

static void driver_task2(STATUS *status) {
  status->task.task_running = 1;
  status->state.base_speed = 40;
  status->state.motion = MOTOR_TEST;
}

static void driver_task3(STATUS *status) {
  status->task.task_running = 1;
  status->state.base_speed = 40;
  status->state.motion = FIND_LINE;
}

static void driver_task4(STATUS *status) {
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
    float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
    log_uprintf(&huart1, "pulse=%.0f  cm=%.2f\r\n", status->task.phase_mileage, cm);
    status->task.race_phase = 1;
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
  // if (wheel0_pulse < 0) wheel0_pulse = -wheel0_pulse;
  // if (wheel1_pulse < 0) wheel1_pulse = -wheel1_pulse;
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
