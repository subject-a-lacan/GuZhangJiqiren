// @63

#include "log.h"

#include "stdarg.h"
#include "stdio.h"
#include "usart.h"

#define LOG_FORMAT_BUF_LENGTH 256

#define STM32

#ifdef STM32

HAL_StatusTypeDef UART_send_bytes(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size) {
  uint16_t i;
  for (i = 0; i < size; ++i) {
    HAL_StatusTypeDef ret = HAL_UART_Transmit(huart, (uint8_t *)&data[i], 1, 20);
    if (ret != HAL_OK) {
      HAL_UART_AbortTransmit(huart);
      huart->gState = HAL_UART_STATE_READY;
      ret = HAL_UART_Transmit(huart, (uint8_t *)&data[i], 1, 20);
      if (ret != HAL_OK) return ret;
    }
  }
  return HAL_OK;
}

void log_uprintf(UART_HandleTypeDef *huart, const char *format, ...) {
  static unsigned char abbuf = 0;
  static char buf[2][LOG_FORMAT_BUF_LENGTH];

  abbuf = abbuf ? 0 : 1;

  va_list args;
  va_start(args, format);
  unsigned int len =
      vsnprintf(buf[abbuf], LOG_FORMAT_BUF_LENGTH - 1, format, args);
  va_end(args);

  UART_send_bytes(huart, (const uint8_t *)buf[abbuf], (uint16_t)len);
}

void UART_send_justfloat(UART_HandleTypeDef *huart, unsigned int count, ...) {
  float data[24];
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
  va_list args;
  if (count > 24) count = 24;
  va_start(args, count);
  for (unsigned int i = 0; i < count; ++i) data[i] = (float)va_arg(args, double);
  va_end(args);
  UART_send_bytes(huart, (const uint8_t *)data, (uint16_t)(count * sizeof(float)));
  UART_send_bytes(huart, tail, sizeof(tail));
}

#endif
