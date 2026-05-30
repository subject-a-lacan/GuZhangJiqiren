#include "Defect.h"
#include "status.h"
#include "math_tool.h"
#include "tim.h"

extern uint8_t cross_cnt;
extern uint8_t left_cnt;
extern uint8_t cross_delay;
extern Road road_buf;

#define TASK2_BACK_RECOVER_ANGLE  -20.0f

int16_t  task2_front_kick_pwm  = 2500;
uint32_t task2_front_kick_time = 100;
int16_t  task2_recover_pwm     = 2000;
uint32_t task2_recover_time    = 100;
uint8_t  task2_direct_pwm      = 0;

typedef enum {
  TASK2_FRONT_KICK = 0,
  TASK2_BALANCE_LQR,
  TASK2_BACK_RECOVER,
} TASK2_STATE;

static TASK2_STATE task2_state;
static uint32_t task2_last_switch_time;
static int8_t  task2_square_dir;
static float task2_car_position;

static void task2_set_pwm(int16_t pwm) {
  pwm = CONFINE(pwm, -TRUST_CONFINE, TRUST_CONFINE);

  status.motor.wheel[0].trust = pwm;
  status.motor.wheel[1].trust = pwm;

  set_wheel_dir(&status.motor.wheel[0], pwm);
  set_wheel_dir(&status.motor.wheel[1], pwm);

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ABS(pwm));
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ABS(pwm));
}

float get_task2_state_debug(void) {
  return (float)task2_state;
}

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

static void task2_enter_front_kick(STATUS *status) {
  task2_state = TASK2_FRONT_KICK;
  task2_last_switch_time = status->state.time;
  task2_direct_pwm = 1;
  (void)status;
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

  status->motor.wheel[0].trust = 0;
  status->motor.wheel[1].trust = 0;

  status->state.initial_angle = status->state.cur_angle;

  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      apply_basic_control_param(status);
      break;
    case TASK_BASIC_2:
      apply_basic_control_param(status);
      task2_car_position = 0;
      task2_enter_front_kick(status);
      break;
    case TASK_ADV_1:   break;
    case TASK_ADV_2:   break;
  }

  status->task.phase_start_time = status->state.time;
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
  task2_direct_pwm = 0;
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
  task2_direct_pwm = 0;
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


static uint8_t t1_black_frames = 0;

static void driver_task1(STATUS *status) {
  status->task.task_running = 1;
  status->state.motion = FIND_LINE;
  status->state.base_speed = 10;

  uint8_t mid4 = status->sensor.gw_analogue.digital_8bit & 0x3C;
  uint8_t black = ((mid4 & 0x04) != 0)
                + ((mid4 & 0x08) != 0)
                + ((mid4 & 0x10) != 0)
                + ((mid4 & 0x20) != 0);

  if (black >= 3) {
    t1_black_frames++;
  } else {
    t1_black_frames = 0;
  }

  if (t1_black_frames >= 3
      && encoder_pulse_to_cm((int32_t)status->task.phase_mileage) > 99.0f) {
    task_finish(status);
  }
}

static void driver_task2(STATUS *status) {
  float roll = get_gyr_value(&status->sensor.gy901, gyr_x_roll);
  float angle_error = 0.0f - roll;
  float balance_out;
  float car_speed;
  float position_out;
  int16_t kick_pwm;
  PID *balance_param = &status->state.status_pid.balance_pid;
  PID *mileage_param = &status->state.status_pid.mileage_pid;

  status->task.task_running = 1;
  status->task.stop_cmd = 0;
  task2_direct_pwm = 1;

  switch (task2_state) {
    case TASK2_FRONT_KICK:
      if (status->state.time - task2_last_switch_time >= task2_front_kick_time) {
        task2_state = TASK2_BALANCE_LQR;
        balance_param->is_first = 1;
        /* fall through to LQR */
      } else {
        kick_pwm = task2_front_kick_pwm;
        status->state.motion = STRAIGHT;
        status->state.base_speed = 0;
        task2_set_pwm(kick_pwm);
        break;
      }

    case TASK2_BACK_RECOVER:
      if (status->state.time - task2_last_switch_time >= task2_recover_time) {
        task2_last_switch_time = status->state.time;
        task2_square_dir = -task2_square_dir;
        if (task2_square_dir == 1) {
          task2_state = TASK2_BALANCE_LQR;
          balance_param->is_first = 1;
        }
      }
      kick_pwm = (task2_square_dir == 1) ? task2_recover_pwm : -task2_recover_pwm;
      status->state.motion = STRAIGHT;
      status->state.base_speed = 0;
      task2_set_pwm(kick_pwm);
      break;

    case TASK2_BALANCE_LQR:
    default:
      if (roll < TASK2_BACK_RECOVER_ANGLE) {
        task2_state = TASK2_BACK_RECOVER;
        task2_last_switch_time = status->state.time;
        task2_square_dir = 1;
        break;
      }

      balance_out = compute_pid(balance_param, angle_error);
      car_speed = ((float)status->motor.wheel[0].cur_speed
                 + (float)status->motor.wheel[1].cur_speed) / 2.0f;
      position_out = -(mileage_param->kp * task2_car_position
                     + mileage_param->kd * car_speed);
      status->state.motion = STRAIGHT;
      status->state.base_speed = 0;
      task2_set_pwm((int16_t)(balance_out + position_out));
      break;
  }
}


static void driver_task3(STATUS *status) {
}

static void driver_task4(STATUS *status) {
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
  if (status->task.task_id == TASK_BASIC_2) {
    task2_car_position += ((float)wheel0_pulse + (float)wheel1_pulse) / 2.0f;
  }
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
