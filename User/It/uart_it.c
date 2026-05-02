#include "usart.h"

#define BUFFER_SIZE 255

uint8_t uart1_buf[BUFFER_SIZE] = {0};
uint8_t uart2_buf[BUFFER_SIZE] = {0};
uint8_t uart3_buf[BUFFER_SIZE] = {0};
uint8_t uart4_buf[BUFFER_SIZE] = {0};

void start_uart_idle_it(UART_HandleTypeDef *huart, uint8_t *buf) {
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  HAL_UART_Receive_DMA(huart, buf, 255);
}

void init_uart_idle_it() {
  start_uart_idle_it(&huart1, uart1_buf);
  start_uart_idle_it(&huart2, uart2_buf);
  start_uart_idle_it(&huart3, uart3_buf);
  start_uart_idle_it(&huart4, uart4_buf);
}
