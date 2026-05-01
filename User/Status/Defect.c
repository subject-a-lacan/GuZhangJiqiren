#include "Defect.h"
#include "status.h"

#include "pid.h"
#include "road.h"

extern uint8_t cross_cnt;
extern Road road_buf;

void init_task(TASK *task) {
  task->task_id = TASK_BASIC_1;
  task->start_pose = START_AB;
  task->race_phase = 0;

  task->cross_cnt = 0;

  task->armed = 0;
  task->task_running = 0;

  task->start_request = 0;
  task->stop_request = 0;

  task->phase_start_time = 0;
  task->phase_mileage = 0;
}

void task_start(STATUS *status) {
  status->task.task_running = 1;
  status->task.start_request = 0;
  status->task.stop_request = 0;

  status->task.cross_cnt = 0;
  cross_cnt = 0;

  road_buf = Straight;
  init_road_determine(&status->state.road_determine);

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

  status->device.buzzer.on = 1;
}
