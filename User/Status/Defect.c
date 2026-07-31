#include "Defect.h"
#include "status.h"
#include "log.h"
#include "uart_gyro.h"
#include "maixcam.h"
#include "Emm_v5.h"
#include <stdio.h>

enum { T7_MSG_NONE, T7_MSG_RX, T7_MSG_CT1, T7_MSG_CM1, T7_MSG_CD1, T7_MSG_FOUND };
enum { T7_CMD_NONE, T7_CMD_T, T7_CMD_M, T7_CMD_D };

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
  task_last_report = status->state.time;
}

void task_finish(STATUS *status) { task_stop(status); }

void task_stop(STATUS *status) {
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
  static uint32_t last;
  if (status->state.time - last >= 80) {
    last = status->state.time;
    UART_send_justfloat(&huart1, 1, status->sensor.gw_analogue.diff);
  }
}
static void driver_task3(STATUS *status) {
  static uint8_t inited;
  static uint32_t last;
  if (!inited) { status->state.base_speed = 8; inited = 1; }
  status->state.motion = FIND_LINE;
  status->task.stop_cmd = 0;
  if (status->state.time - last >= 100) {
    last = status->state.time;
    UART_send_justfloat(&huart1, 3,
      status->sensor.gw_analogue.diff,
      status->state.status_pid.follow_line_pid.error,
      status->state.status_pid.follow_line_pid.out);
  }
}
static void driver_task4(STATUS *status) { status->state.motion = FIND_LINE; status->state.base_speed = 3; }
static void driver_task5(STATUS *status) {
  static uint32_t last;
  if (status->state.time - last >= 200) {
    last = status->state.time;
    UART_send_justfloat(&huart1, 2,
      status->sensor.uart_gyr.gyro_z,
      status->sensor.uart_gyr.yaw);
  }
}
static void driver_task6(STATUS *status) {
  if (every_100ms(status))
    UART_send_justfloat(&huart1, 1,
      iic_gyr_get_value(&status->sensor.gy901, gyr_a_y));
}
static void driver_task7(STATUS *status) {
  typedef enum { Q7_SEND = 0, Q7_WAIT, Q7_DONE } Q7_STATE;
  static Q7_STATE q7_state = Q7_SEND;
  static uint8_t q7_dir;
  static uint32_t q7_timeout;
  const uint32_t q7_clk = 6400u;

  status->task.task_running = 1;
  status->state.motion = STOP;
  status->state.base_speed = 0;

  if (status->task.phase_mileage == 0) {
    q7_state = Q7_SEND;
    q7_dir = 0;
    status->task.phase_mileage = 1;
  }

  switch (q7_state) {

  case Q7_SEND:
    stepper_request_move(q7_clk, 20, 0);
    q7_timeout = status->state.time + 3000;
    q7_state = Q7_WAIT;
    break;

  case Q7_WAIT:
    if (status->stepper.reached) {
      status->stepper.reached = 0;
      q7_dir = (uint8_t)(q7_dir ^ 1u);
      q7_state = Q7_DONE;
    } else if (status->state.time >= q7_timeout) {
      q7_state = Q7_DONE;
    }
    break;

  case Q7_DONE:
    /* single shot — stop to re-arm */
    break;
  }
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
