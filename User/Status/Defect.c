#include "Defect.h"
#include "status.h"

#include "pid.h"
#include "road.h"

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
  status->task.task_running = 1;
  status->task.start_request = 0;
  status->task.stop_request = 0;

  status->task.cross_cnt = 0;
  cross_cnt = 0;
  left_cnt = 0;
  cross_delay = 0;

  road_buf = Straight;
  status->state.road_determine.integral = 0;
  status->state.road_determine.data_buf = 0;
  status->state.road_determine.maybe = 0;
  status->state.road_determine.cross = Straight;
  status->state.road_determine.cross_cnt = 0;

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
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.task_select_request = 0;
  status->task.pose_switch_request = 0;
  status->state.motion = STOP;
  status->state.base_speed = 0;
  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;
  status->device.buzzer.on = 1;
}

void task_stop(STATUS *status) {
  status->task.task_running = 0;
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
}

//待定

static void task_basic_1_update(STATUS *status) {
  (void)status;
  status->task.task_running=0;
}

static void task_basic_2_update(STATUS *status) {
  (void)status;
    status->task.task_running=0;
}

static void task_adv_1_update(STATUS *status) {
  (void)status;
    status->task.task_running=0;
}

static void task_adv_2_update(STATUS *status) {
  (void)status;
    status->task.task_running=0;
}

void update_task(STATUS *status) {
  if (status->task.stop_request) {
    task_stop(status);  //停止的优先级最高
    return;
  }

  if (!status->task.task_running) {  //当且仅当任务不在运行时才能选任务
    if (status->task.task_select_request) {//如果请求标志位置1了
      task_select(status, status->task.requested_task_id);
      status->task.task_select_request = 0; //清零请求标志位
    }

    if (status->task.pose_switch_request) {//如果切换请求挂起
      if (status->task.task_id == TASK_BASIC_2 || status->task.task_id == TASK_ADV_1) {//只有这两种任务需要切换起始位姿
        status->task.start_pose = (status->task.start_pose == START_AB) ? START_AD : START_AB;
      }
      status->task.pose_switch_request = 0;//清零请求标志位
    }
  }

  if (status->task.start_request) {
    if (!status->task.task_running && status->task.armed) {//只有任务不在运行时才能出发
      task_start(status);  //初始化init任务
    }
    status->task.start_request = 0;//清零请求标志位
  }

  if (!status->task.task_running) {
    return;
  }//任务只有运行中才能进入下面的小状态机

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
