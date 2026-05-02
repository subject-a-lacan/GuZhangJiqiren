#include "Defect.h"
#include "status.h"

#include "pid.h"

extern uint8_t cross_cnt;
extern uint8_t left_cnt;
extern uint8_t cross_delay;
extern Road road_buf;

void init_task(TASK *task) {
  task->task_id = TASK_BASIC_1;
  task->start_pose = START_AB;
  task->race_phase = 0;

  task->cross_cnt = 0;

  task->armed = 0;
  task->task_running = 0;

  task->task_select_request = 0;
  task->requested_task_id = 0;
  task->pose_switch_request = 0;

  task->start_request = 0;
  task->stop_request = 0;

  task->phase_start_time = 0;
  task->phase_mileage = 0;
}

void task_start(STATUS *status) {
  status->task.start_request = 0;
  status->task.stop_request = 0;

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
      status->task.race_phase = Q1_START_A_TURN;
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

//待定

static void task_basic_1_update(STATUS *status) {
  status->task.task_running = 1;
  status->state.motion = FIND_LINE;
  status->state.base_speed = 40;
}

static void task_basic_2_update(STATUS *status) {
  status->task.task_running = 1;
  task_finish(status);
}

static void task_adv_1_update(STATUS *status) {
  status->task.task_running = 1;
  task_finish(status);
}

static void task_adv_2_update(STATUS *status) {
  status->task.task_running = 1;
  task_finish(status);
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

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      task_basic_1_update(status);
      break;
    case TASK_BASIC_2:
      task_basic_2_update(status);
      break;
    case TASK_ADV_1:
      task_adv_1_update(status);
      break;
    case TASK_ADV_2:
      task_adv_2_update(status);
      break;
  }
}
