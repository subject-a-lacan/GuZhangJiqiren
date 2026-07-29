#include "uart_it.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>
#include "uart_gyro.h"
#include "status.h"
#include "maixcam.h"

static uint8_t maixcam_rx_byte;

void UART_PID_Tune(uint8_t cmd, float val, uint8_t has_val);

typedef struct {
  uint8_t byte;
  uint8_t cmd;
  uint8_t state;
  uint8_t index;
  char buf[20];
} UART_PID_RX;

static UART_PID_RX uart1_pid_rx = {0};
static UART_PID_RX uart2_pid_rx = {0};
static UART_PID_RX uart3_pid_rx = {0};
static uint8_t uart1_pid_byte;

static void reset_uart_pid_rx(UART_PID_RX *rx) {
  rx->cmd = 0;
  rx->state = 0;
  rx->index = 0;
}

static void parse_uart_pid_byte(UART_PID_RX *rx) {
  switch (rx->state) {
    case 0:
      if (rx->byte == 'C') {
        rx->state = 1;
      }
      break;

    case 1:
      rx->cmd = rx->byte;
      rx->index = 0;
      rx->state = 2;
      break;

    case 2:
      if (rx->byte == '\r' || rx->byte == '\n') {
        rx->buf[rx->index] = '\0';
        UART_PID_Tune(rx->cmd, atof(rx->buf), rx->index > 0);
        rx->state = 0;
      } else if (rx->index < sizeof(rx->buf) - 1) {
        rx->buf[rx->index++] = rx->byte;
      }
      break;

    default:
      rx->state = 0;
      break;
  }
}

void init_uart_pid_tune(void) {
  HAL_UART_Receive_IT(&huart1, &uart1_pid_byte, 1);
}

void init_uart_gyr(void) {
  uart_gyr_init(&status.sensor.uart_gyr);
  uart_gyr_start_receive(&status.sensor.uart_gyr);
}

void init_maixcam_uart(void) {
  maixcam_init();
  HAL_UART_Receive_IT(&huart4, &maixcam_rx_byte, 1);
}

void UART_PID_Tune(uint8_t cmd, float val, uint8_t has_val) {
  if (!has_val) {
    if (cmd == 's') status.task.start_request = 1;
    if (cmd == 'p') status.task.stop_request = 1;
    return;
  }
  status.device.buzzer.on = 1;
  status.device.buzzer.off_time = status.state.time + 140;
  switch (cmd) {
    case 'a': status.state.status_pid.follow_line_pid.kp = val; break;
    case 'c': status.state.status_pid.follow_line_pid.ki = val; break;
    case 'e': status.state.status_pid.follow_line_pid.kd = val; break;
    case 'd': status.state.status_pid.keep_angle_pid.kp = val; break;
    case 'f': status.state.status_pid.keep_angle_pid.ki = val; break;
    case 'g': status.state.status_pid.keep_angle_pid.kd = val; break;
    case 'm': status.motor.wheel[0].wheel_pid.kp = val; break;
    case 'o': status.motor.wheel[0].wheel_pid.ki = val; break;
    case 'q': status.motor.wheel[0].wheel_pid.kd = val; break;
    case 's': status.motor.wheel[3].wheel_pid.kp = val; break;
    case 'u': status.motor.wheel[3].wheel_pid.ki = val; break;
    case 'w': status.motor.wheel[3].wheel_pid.kd = val; break;
    case 'b': status.motor.wheel[0].wheel_pid.integral_max = val; break;
    case 'n': status.motor.wheel[3].wheel_pid.integral_max = val; break;
    case 'h': status.state.base_speed = (int16_t)val; break;
    case 't': task_select(&status, (uint8_t)val); break;
    default: break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    UART_GYRO *gyro = &status.sensor.uart_gyr;
    uart_gyr_rx_feed(gyro, gyro->rx_byte, (uint32_t)status.state.time);
    uart_gyr_start_receive(gyro);
  }
  if (huart == &huart1) {
    uart1_pid_rx.byte = uart1_pid_byte;
    parse_uart_pid_byte(&uart1_pid_rx);
    HAL_UART_Receive_IT(&huart1, &uart1_pid_byte, 1);
  }
  if (huart == &huart4) {
    maixcam_rx_feed(maixcam_rx_byte);
    HAL_UART_Receive_IT(&huart4, &maixcam_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) HAL_UART_Receive_IT(&huart1, &uart1_pid_byte, 1);
  if (huart == &huart2) {
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    status.sensor.uart_gyr.count = 0;
    uart_gyr_start_receive(&status.sensor.uart_gyr);
  }
  if (huart == &huart4) HAL_UART_Receive_IT(&huart4, &maixcam_rx_byte, 1);
}

