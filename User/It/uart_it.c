#include "uart_it.h"
#include "usart.h"
#include "status.h"
#include <stdint.h>

static uint8_t huart1_rx_byte;

void init_uart_pid_tune_it(void) {
  HAL_UART_Receive_IT(&huart1, &huart1_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    if (huart1_rx_byte == '9' && status.task.task_id == TASK_ADV_2) {
      status.task.task_running = 0;
      status.task.armed = 0;
      status.task.start_request = 0;
      status.task.stop_request = 0;
      status.task.stop_cmd = 1;
      status.state.motion = STOP;
      status.state.base_speed = 0;
      status.motor.wheel[0].tar_speed = 0;
      status.motor.wheel[1].tar_speed = 0;
    }
    HAL_UART_Receive_IT(&huart1, &huart1_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    HAL_UART_AbortReceive_IT(&huart1);
    HAL_UART_Receive_IT(&huart1, &huart1_rx_byte, 1);
  }
}
