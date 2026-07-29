#include "Defect.h"
#include "status.h"
#include "log.h"
#include "uart_gyro.h"

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
  status->state.base_speed = 3;
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
static void driver_task4(STATUS *status) { status->state.motion = FIND_LINE; status->state.base_speed = 3; follow_line(status); }
static void driver_task5(STATUS *status) { (void)status; if (every_100ms(status)) UART_send_justfloat(&huart1, 2, uart_gyr_get_z(), uart_gyr_get_yaw()); }
static void driver_task6(STATUS *status) {
  /* UART4 is reserved for the MaixCAM2 ASCII protocol; responses are handled by its RX path. */
  if (every_100ms(status)) {
    static const uint8_t command[] = "CT1#";
    HAL_UART_Transmit(&huart4, (uint8_t *)command, sizeof(command) - 1, 20);
  }
}
static void driver_task7(STATUS *status) { (void)status; }

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
