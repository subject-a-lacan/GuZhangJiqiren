// @63

#include "log.h"

#include "stdarg.h"
#include "stdio.h"
#include "usart.h"

#define LOG_FORMAT_BUF_LENGTH 256

#define STM32

#ifdef STM32

void log_uprintf(UART_HandleTypeDef *huart, const char *format, ...) {
  static unsigned char abbuf = 0;
  static char buf[2][LOG_FORMAT_BUF_LENGTH];

  abbuf = abbuf ? 0 : 1;

  va_list args;
  va_start(args, format);
  unsigned int len =
      vsnprintf(buf[abbuf], LOG_FORMAT_BUF_LENGTH - 1, format, args);
  va_end(args);

  HAL_UART_Transmit(huart, (uint8_t *)buf[abbuf], len, 100);
}

void UART_send_justfloat(UART_HandleTypeDef *huart, uint8_t count, ...) {
  float data[16];
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
  va_list args;
  if (count > 16) count = 16;
  va_start(args, count);
  for (uint8_t i = 0; i < count; ++i) data[i] = (float)va_arg(args, double);
  va_end(args);
  HAL_UART_Transmit(huart, (uint8_t *)data, (uint16_t)(count * sizeof(float)), 100);
  HAL_UART_Transmit(huart, tail, sizeof(tail), 100);
}

#endif
