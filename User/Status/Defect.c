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
  status->state.motion = STRAIGHT;
  status->state.base_speed = 5;
  status->task.stop_cmd = 0;
  status->motor.wheel[0].tar_speed = 5.0f;
  status->motor.wheel[1].tar_speed = 5.0f;
  status->motor.wheel[2].tar_speed = 5.0f;
  status->motor.wheel[3].tar_speed = 5.0f;
  if (every_100ms(status))
    UART_send_justfloat(&huart1, 12,
      (float)status->motor.wheel[0].cur_speed, status->motor.wheel[0].tar_speed, (float)status->motor.wheel[0].trust,
      (float)status->motor.wheel[1].cur_speed, status->motor.wheel[1].tar_speed, (float)status->motor.wheel[1].trust,
      (float)status->motor.wheel[2].cur_speed, status->motor.wheel[2].tar_speed, (float)status->motor.wheel[2].trust,
      (float)status->motor.wheel[3].cur_speed, status->motor.wheel[3].tar_speed, (float)status->motor.wheel[3].trust);
  /* raw TIM3 CNT debug */
  static uint32_t last_debug;
  if (status->state.time - last_debug >= 100) {
    last_debug = status->state.time;
    UART_send_justfloat(&huart1, 2,
      (float)(TIM3->CNT - 30000), (float)TIM3->CNT);
  }
}
static void driver_task2(STATUS *status) {
  if (every_100ms(status)) UART_send_justfloat(&huart1, 8,
    (float)status->sensor.gw_analogue.channel[0], (float)status->sensor.gw_analogue.channel[1],
    (float)status->sensor.gw_analogue.channel[2], (float)status->sensor.gw_analogue.channel[3],
    (float)status->sensor.gw_analogue.channel[4], (float)status->sensor.gw_analogue.channel[5],
    (float)status->sensor.gw_analogue.channel[6], (float)status->sensor.gw_analogue.channel[7]);
}
static void driver_task3(STATUS *status) {
  if (every_100ms(status)) UART_send_justfloat(&huart1, 8,
    (float)((status->sensor.gw_analogue.digital_8bit >> 0) & 1), (float)((status->sensor.gw_analogue.digital_8bit >> 1) & 1),
    (float)((status->sensor.gw_analogue.digital_8bit >> 2) & 1), (float)((status->sensor.gw_analogue.digital_8bit >> 3) & 1),
    (float)((status->sensor.gw_analogue.digital_8bit >> 4) & 1), (float)((status->sensor.gw_analogue.digital_8bit >> 5) & 1),
    (float)((status->sensor.gw_analogue.digital_8bit >> 6) & 1), (float)((status->sensor.gw_analogue.digital_8bit >> 7) & 1));
}
static void driver_task4(STATUS *status) { status->state.motion = FIND_LINE; status->state.base_speed = 3; }
static void driver_task5(STATUS *status) {
  static uint32_t last;
  if (status->state.time - last >= 200) {
    last = status->state.time;
    UART_send_justfloat(&huart2, 2, 1.0f, 2.0f);
  }
}
static void driver_task6(STATUS *status) {
  if (every_100ms(status))
    UART_send_justfloat(&huart1, 1,
      iic_gyr_get_value(&status->sensor.gy901, gyr_a_y));
}
static void driver_task7(STATUS *status) {
  static uint8_t q7_dir;
  static uint16_t q7_timer;
  const uint32_t q7_angle_clk = 6400u; /* 90 degrees at 25600 pulses/rev (128-step) */

  status->task.task_running = 1;
  status->state.motion = STOP;
  status->state.base_speed = 0;

  if (status->task.phase_mileage == 0) {
    q7_dir = 0;
    q7_timer = 0;
    status->task.phase_mileage = 1;
    Emm_V5_En_Control(1, true, false);
    Emm_V5_Reset_CurPos_To_Zero(1);
    Emm_V5_Pos_Control(1, q7_dir, 20, 0, q7_angle_clk, false, false);
  }

  q7_timer += (uint16_t)status->state.T;

  if (q7_timer >= 5000) {
    q7_timer = 0;
    q7_dir = (uint8_t)(q7_dir ^ 1u);
    Emm_V5_Pos_Control(1, q7_dir, 20, 0, q7_angle_clk, false, false);
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
