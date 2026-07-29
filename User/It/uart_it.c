#include "uart_it.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>
#include "uart_gyro.h"
#include "status.h"

static UART_GYRO uart_gyro;

void UART_PID_Tune(uint8_t cmd, float val);

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
        UART_PID_Tune(rx->cmd, atof(rx->buf));
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

void init_uart_pid_tune_it(void) {
  uart_gyr_init(&uart_gyro);
  uart_gyr_start_receive(&uart_gyro);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    uart_gyr_rx_feed(&uart_gyro, uart_gyro.rx_byte, (uint32_t)status.state.time);
    uart_gyr_start_receive(&uart_gyro);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) uart_gyr_start_receive(&uart_gyro);
}
