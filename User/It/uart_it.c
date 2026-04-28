#include "dma.h"
#include "log.h"
#include "usart.h"
#include <stdlib.h>

extern uint8_t rx_byte;
extern uint8_t rx_cmd;
extern uint8_t rx_state;
extern char rx_buf[20];
extern uint8_t rx_index;
void UART_PID_Tune(uint8_t cmd, float val);

#define BUFFER_SIZE 255

uint8_t uart2_buf[BUFFER_SIZE] = {0};
uint8_t uart3_buf[BUFFER_SIZE] = {0};
uint8_t uart4_buf[BUFFER_SIZE] = {0};

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;

void start_uart_idle_it(UART_HandleTypeDef *huart, uint8_t *buf) {
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  HAL_UART_Receive_DMA(huart, buf, 255);
}

void init_uart_idle_it() {
  start_uart_idle_it(&huart2, uart2_buf);
  start_uart_idle_it(&huart3, uart3_buf);
  start_uart_idle_it(&huart4, uart4_buf);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    switch (rx_state) {
      case 0: // 【状态0：等帧头 'C'】
        if (rx_byte == 'C') rx_state = 1;
        break;
      case 1:// 【状态1：存指令字符】
        rx_cmd = rx_byte;
        rx_index = 0;
        rx_state = 2;
        break;
      case 2:// 【状态2：存数值，直到换行】
        if (rx_byte == '\r' || rx_byte == '\n') {
          rx_buf[rx_index] = '\0';
          UART_PID_Tune(rx_cmd, atof(rx_buf));
          rx_state = 0;
        } else {
          if (rx_index < sizeof(rx_buf) - 1) rx_buf[rx_index++] = rx_byte;
        }
        break;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    return;
  }

  if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    HAL_UART_DMAStop(huart);
  }
  if (huart == &huart2) {
    log_uprintf(&huart1, "uart2 idle\n");
    start_uart_idle_it(&huart2, uart2_buf);
    uint8_t data_length = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  } else if (huart == &huart3) {
    log_uprintf(&huart1, "uart3 idle\n");
    start_uart_idle_it(&huart3, uart3_buf);
    uint8_t data_length = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
  } else if (huart == &huart4) {
    log_uprintf(&huart1, "uart3 idle\n");
    start_uart_idle_it(&huart4, uart4_buf);
    uint8_t data_length = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
  }
}