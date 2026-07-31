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
/* ================================================================
 *  Task1: UART4 与 MaixCam2 通信测试 Demo
 *
 *  协议:
 *    MCU → MaixCam2:  C + 命令 + 数值 + #    例: CD1#
 *    MaixCam2 → MCU:  V + 类型 + 数据 + #    例: VDA1#  VL,3.3,#
 *
 *  流程:
 *    1. 发送 CD1# (启动检测)
 *    2. 等待 VDA1# 应答帧，超时重发直到收到
 *    3. 收到应答后，打印接收到的 VL 位置数据帧
 *
 *  串口助手测试方法:
 *    - UART4 (PC10/PC11): 接 USB-TTL 模块
 *    - USART1 (PC4/PC5):  接 ST-Link VCP，查看调试输出
 *    - 用串口助手发送 VDA1# 模拟应答
 *    - 用串口助手发送 VL,3.3,#  模拟位置数据
 * ================================================================ */

enum {
  COMM_IDLE = 0,
  COMM_SEND_CD1,
  COMM_WAIT_ACK,
  COMM_RUNNING,
};

static uint8_t  comm_state       = COMM_IDLE;
static uint32_t comm_last_tx     = 0;
static uint8_t  comm_retry_cnt   = 0;
static uint32_t comm_last_report = 0;

static void driver_task1(STATUS *status) {
  status->state.motion = STOP;
  status->task.stop_cmd = 0;

  switch (comm_state) {

  /* ── 状态0: 初始化，进入发送 ── */
  case COMM_IDLE:
    comm_state     = COMM_SEND_CD1;
    comm_retry_cnt = 0;
    break;

  /* ── 状态1: 通过 DMA 发送 CD1# ── */
  case COMM_SEND_CD1:
    maixcam_cmd_D(1);                    /* 内部使用 HAL_UART_Transmit_DMA */
    comm_last_tx = (uint32_t)status->state.time;
    comm_retry_cnt++;
    comm_state = COMM_WAIT_ACK;
    PRINTF("[TEST] TX: CD1# (attempt %u)\r\n", comm_retry_cnt);
    break;

  /* ── 状态2: 等待 VDA1# 应答，超时重发 ── */
  case COMM_WAIT_ACK:
    if (maixcam_cmd.state == MAIXCAM_CMD_OK) {
      /* 收到 VDA1# 应答 */
      PRINTF("[TEST] RX: VDA1# ACK OK! Ready for position data.\r\n");
      maixcam_cmd.state = MAIXCAM_CMD_IDLE;
      comm_state       = COMM_RUNNING;
      comm_last_report = 0;
    } else if (status->state.time - comm_last_tx > 300) {
      /* 300ms 超时，重发 CD1# */
      comm_state = COMM_SEND_CD1;
    }
    break;

  /* ── 状态3: 已握手，持续解析并打印位置数据 ── */
  case COMM_RUNNING:
    if (maixcam_loc_new) {
      maixcam_loc_new = 0;
      float x_mm = (float)maixcam_loc.x10 / 10.0f;
      if (status->state.time - comm_last_report >= 100) {
        PRINTF("[TEST] POS: X=%.1f mm\r\n", (double)x_mm);
        comm_last_report = (uint32_t)status->state.time;
      }
    }
    break;
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
