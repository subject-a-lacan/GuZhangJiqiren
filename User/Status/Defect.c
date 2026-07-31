#include "Defect.h"
#include "status.h"
#include "log.h"
#include "uart_gyro.h"
#include "maixcam.h"
#include "Emm_v5.h"
#include "uart_it.h"
#include "ball_control.h"
#include <math.h>
#include <stdio.h>

#define TASK2_CURVE_RATIO_THRESHOLD 1.0f
#define TASK2_STRAIGHT_RATIO_THRESHOLD 1.0f
#define TASK2_CONFIRM_FRAMES 1u
#define TASK2_MIN_SPEED 1.0f
#define TASK2_V_CURVE_MIN 1.0f
#define TASK2_CURVE_SPEED_K 1.0f
#define TASK2_STOP_ROAD_TYPE 1u
#define TASK7_BALL_TARGET_MM 0.0f
#define TASK7_CAR_ACCEL_MM_S2 0.0f

enum {
  TASK2_STRAIGHT = 0,
  TASK2_CURVE,
  TASK2_FINAL_CURVE,
};

enum { T7_MSG_NONE, T7_MSG_RX, T7_MSG_CT1, T7_MSG_CM1, T7_MSG_CD1, T7_MSG_FOUND };
enum { T7_CMD_NONE, T7_CMD_T, T7_CMD_M, T7_CMD_D, T7_CMD_CDA };

static volatile uint8_t task7_rx_msg_type;
static volatile uint8_t task7_dbg_msg_type;
static volatile uint8_t task7_cmd_type;

/* captured from ISR, consumed in flush */
static struct {
  uint8_t found; int32_t x10, y10, d10;
  char rx_raw[64];
} task7_cap;

static uint8_t task7_tx(const char *s, uint16_t len) {
  return maixcam_uart_tx_enqueue((const uint8_t *)s, len);
}

static uint8_t task7_flush_one(uint8_t type) {
  char buf[128];
  uint16_t len = 0;
  switch (type) {
    case T7_MSG_RX:
      len = (uint16_t)snprintf(buf, sizeof(buf), "RX:%s\r\n", task7_cap.rx_raw);
      break;
    case T7_MSG_CT1:
      { const char *s = "CT1#\r\n"; while (*s && len < sizeof(buf)-1) buf[len++] = *s++; buf[len] = '\0'; }
      break;
    case T7_MSG_CM1:
      { const char *s = "CM1#\r\n"; while (*s && len < sizeof(buf)-1) buf[len++] = *s++; buf[len] = '\0'; }
      break;
    case T7_MSG_CD1:
      { const char *s = "CD1#\r\n"; while (*s && len < sizeof(buf)-1) buf[len++] = *s++; buf[len] = '\0'; }
      break;
    case T7_MSG_FOUND:
      len = (uint16_t)snprintf(buf, sizeof(buf), "FOUND:%d X:%.1f Y:%.1f DIST:%.1f\r\n",
              task7_cap.found,
              (double)(task7_cap.x10 / 10.0), (double)(task7_cap.y10 / 10.0),
              (double)(task7_cap.d10 / 10.0));
      break;
    default: return 1;
  }
  if (len == 0) return 1;
  return task7_tx(buf, len);
}

void task7_flush(void) {
  if (task7_cmd_type != T7_CMD_NONE) {
    uint8_t c = task7_cmd_type;
    task7_cmd_type = T7_CMD_NONE;
    switch (c) {
      case T7_CMD_T: maixcam_cmd_T(1); break;
      case T7_CMD_M: maixcam_cmd_M(1); break;
      case T7_CMD_D: maixcam_cmd_D(1); break;
      case T7_CMD_CDA: maixcam_cmd_CDA(); break;
    }
  }
  {
    uint8_t m = task7_rx_msg_type;
    if (m != T7_MSG_NONE && task7_flush_one(m)) task7_rx_msg_type = T7_MSG_NONE;
  }
  {
    uint8_t m = task7_dbg_msg_type;
    if (m != T7_MSG_NONE && task7_flush_one(m)) task7_dbg_msg_type = T7_MSG_NONE;
  }
}

static uint32_t task_last_report;

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
  task->phase_mileage = 0.0f;
  task->curve_cnt = 0;
  task->phase_confirm_cnt = 0;
  task->last_curve_ratio = 0.0f;
  task->curve_entry_yaw = 0.0f;
}

void update_task_led(STATUS *status) {
  status->device.led_on_board.on = (status->task.task_id & 4u) != 0u;
  status->device.led1.on = (status->task.task_id & 2u) != 0u;
  status->device.led2.on = (status->task.task_id & 1u) != 0u;
}

void task_select(STATUS *status, uint8_t id) {
  if (id < TASK_BASIC_1 || id > TASK_ADV_4) return;
  status->task.task_id = id;
  update_task_led(status);
}

void task_start(STATUS *status) {
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.armed = 1;
  status->task.task_running = 1;
  status->task.stop_cmd = 0;
  status->task.phase_start_time = status->state.time;
  status->task.phase_mileage = 0.0f;
  if (status->task.task_id == TASK_BASIC_2) {
    status->task.race_phase = TASK2_STRAIGHT;
    status->task.curve_cnt = 0;
    status->task.phase_confirm_cnt = 0;
    status->task.last_curve_ratio = 0.0f;
    status->task.curve_entry_yaw = 0.0f;
  }
  task_last_report = status->state.time;
}

void task_finish(STATUS *status) { task_stop(status); }

void task_stop(STATUS *status) {
  task7_cmd_type = T7_CMD_NONE;
  ball_control_disable(status);
  status->task.task_running = 0;
  status->task.armed = 0;
  status->task.stop_cmd = 1;
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->state.motion = STOP;
  status->state.base_speed = 0;
}

static uint8_t every_100ms(STATUS *status) {
  if (status->state.time - task_last_report < 100) return 0;
  task_last_report = status->state.time;
  return 1;
}
static void driver_task1(STATUS *status) {
  int16_t base = status->state.base_speed;
  status->state.motion = STRAIGHT;
  status->task.stop_cmd = 0;
  status->motor.wheel[0].tar_speed = (float)base;
  status->motor.wheel[1].tar_speed = (float)base;
  status->motor.wheel[2].tar_speed = (float)base;
  status->motor.wheel[3].tar_speed = (float)base;
  static uint32_t last_print;
  if (status->state.time - last_print >= 80) {
    last_print = status->state.time;
    UART_send_justfloat(&huart1, 12,
      (float)status->motor.wheel[0].cur_speed, status->motor.wheel[0].tar_speed, (float)status->motor.wheel[0].trust,
      (float)status->motor.wheel[1].cur_speed, status->motor.wheel[1].tar_speed, (float)status->motor.wheel[1].trust,
      (float)status->motor.wheel[2].cur_speed, status->motor.wheel[2].tar_speed, (float)status->motor.wheel[2].trust,
      (float)status->motor.wheel[3].cur_speed, status->motor.wheel[3].tar_speed, (float)status->motor.wheel[3].trust);
  }
}
static void driver_task2(STATUS *status) {
  float average_speed;
  float ratio;

  status->state.motion = FIND_LINE;
  status->task.stop_cmd = 0;

  if (status->task.race_phase == TASK2_FINAL_CURVE) {
    float theta;
    float curve_speed;

    if ((uint8_t)status->sensor.gw_analogue.cross.cross == TASK2_STOP_ROAD_TYPE) {
      task_finish(status);
      return;
    }

    theta = fabsf(status->sensor.uart_gyr.yaw - status->task.curve_entry_yaw);
    if (theta > 180.0f) theta = 360.0f - theta;

    curve_speed = TASK2_V_CURVE_MIN + TASK2_CURVE_SPEED_K * (170.0f - theta);
    if (curve_speed < TASK2_V_CURVE_MIN) curve_speed = TASK2_V_CURVE_MIN;
    if (curve_speed > 15.0f) curve_speed = 15.0f;
    status->state.base_speed = (int16_t)curve_speed;
    return;
  }

  status->state.base_speed = 15;
  average_speed =
      ((float)status->motor.wheel[0].cur_speed +
       (float)status->motor.wheel[1].cur_speed +
       (float)status->motor.wheel[2].cur_speed +
       (float)status->motor.wheel[3].cur_speed) / 4.0f;
  average_speed = fabsf(average_speed);

  if (!status->sensor.uart_gyr.valid || average_speed < TASK2_MIN_SPEED) {
    status->task.phase_confirm_cnt = 0;
    status->task.last_curve_ratio = 0.0f;
    return;
  }

  ratio = fabsf(status->sensor.uart_gyr.gyro_z) / average_speed;

  if (status->task.race_phase == TASK2_STRAIGHT) {
    if (ratio > TASK2_CURVE_RATIO_THRESHOLD &&
        ratio > status->task.last_curve_ratio) {
      if (status->task.phase_confirm_cnt < 0xFFu) {
        status->task.phase_confirm_cnt++;
      }
    } else {
      status->task.phase_confirm_cnt = 0;
    }

    if (status->task.phase_confirm_cnt > TASK2_CONFIRM_FRAMES) {
      status->task.curve_cnt++;
      status->task.phase_confirm_cnt = 0;
      if (status->task.curve_cnt >= 2u) {
        status->task.race_phase = TASK2_FINAL_CURVE;
        status->task.curve_entry_yaw = status->sensor.uart_gyr.yaw;
      } else {
        status->task.race_phase = TASK2_CURVE;
      }
    }
  } else if (status->task.race_phase == TASK2_CURVE) {
    if (ratio < TASK2_STRAIGHT_RATIO_THRESHOLD) {
      if (status->task.phase_confirm_cnt < 0xFFu) {
        status->task.phase_confirm_cnt++;
      }
    } else {
      status->task.phase_confirm_cnt = 0;
    }

    if (status->task.phase_confirm_cnt > TASK2_CONFIRM_FRAMES) {
      status->task.race_phase = TASK2_STRAIGHT;
      status->task.phase_confirm_cnt = 0;
    }
  }

  status->task.last_curve_ratio = ratio;
}
static void driver_task3(STATUS *status) {
  static uint32_t last;
  if (status->state.time - last >= 80) {
    last = status->state.time;
    uint8_t d = status->sensor.gw_analogue.digital_8bit;
    UART_send_justfloat(&huart1, 8,
      (float)((d >> 7) & 1), (float)((d >> 6) & 1),
      (float)((d >> 5) & 1), (float)((d >> 4) & 1),
      (float)((d >> 3) & 1), (float)((d >> 2) & 1),
      (float)((d >> 1) & 1), (float)((d >> 0) & 1));
  }
}
static void driver_task4(STATUS *status) {
  static uint32_t last;
  status->state.motion = FIND_LINE;
  status->state.base_speed = 10;
  status->task.stop_cmd = 0;
  if (status->state.time - last >= 50) {
    last = status->state.time;
    UART_send_justfloat(&huart1, 5,
      status->sensor.uart_gyr.gyro_z,
      (float)status->motor.wheel[0].cur_speed,
      (float)status->motor.wheel[1].cur_speed,
      (float)status->motor.wheel[2].cur_speed,
      (float)status->motor.wheel[3].cur_speed);
  }
}
static void driver_task5(STATUS *status) {
  typedef enum { Q5_GO_5, Q5_OVERRIDE, Q5_WAIT_55, Q5_COOLDOWN } Q5_STATE;
  static Q5_STATE q5_state = Q5_GO_5;
  static uint32_t q5_t0;

  status->state.motion = STOP;
  status->state.base_speed = 0;

  switch (q5_state) {
  case Q5_GO_5:
    /* 5 deg absolute, slow so it takes time */
    status->stepper.reached = 0;
    status->stepper.busy = 1;
    Emm_V5_Pos_Control(1, 0, 5, 5, 356, true, false);
    q5_t0 = status->state.time;
    q5_state = Q5_OVERRIDE;
    break;

  case Q5_OVERRIDE:
    /* after 500ms, override to 55 deg regardless of reached */
    if (status->state.time - q5_t0 >= 500) {
      status->stepper.reached = 0;
      status->stepper.busy = 1;
      Emm_V5_Pos_Control(1, 0, 50, 0, 3911, true, false);
      q5_t0 = status->state.time;
      q5_state = Q5_WAIT_55;
    }
    break;

  case Q5_WAIT_55:
    if (status->stepper.reached || status->state.time - q5_t0 >= 3000) {
      q5_t0 = status->state.time;
      q5_state = Q5_COOLDOWN;
    }
    break;

  case Q5_COOLDOWN:
    if (status->state.time - q5_t0 >= 5000) {
      q5_state = Q5_GO_5;
    }
    break;
  }
}
static void driver_task6(STATUS *status) {
  status->state.motion = STOP;
  status->state.base_speed = 0;

  if (status->task.phase_mileage == 0) {
    status->task.phase_mileage = 1;
    maixcam_cmd_D(1);
    ball_control_request(status, 0.0f, 0.0f);
  }
}
static void driver_task7(STATUS *status) {
  status->task.task_running = 1;
  status->state.motion = STOP;
  status->state.base_speed = 0;

  if (status->task.phase_mileage == 0) {
    task7_cmd_type = T7_CMD_CDA;
    status->task.phase_mileage = 1;
  }

  ball_control_request(status, TASK7_BALL_TARGET_MM, TASK7_CAR_ACCEL_MM_S2);
}

void update_task(STATUS *status) {
  if (status->task.stop_request) {
    task_stop(status);
    return;
  }
  if (status->task.task_select_request && !status->task.armed) {
    task_select(status, status->task.requested_task_id);
    status->task.task_select_request = 0;
  }
  if (status->task.start_request && !status->task.armed) task_start(status);
  if (!status->task.armed) return;
  switch (status->task.task_id) {
    case TASK_BASIC_1: driver_task1(status); break;
    case TASK_BASIC_2: driver_task2(status); break;
    case TASK_ADV_1: driver_task3(status); break;
    case TASK_ADV_2: driver_task4(status); break;
    case TASK_BASIC_3: driver_task5(status); break;
    case TASK_ADV_3: driver_task6(status); break;
    case TASK_ADV_4: driver_task7(status); break;
    default: task_stop(status); break;
  }
}
