#ifndef __UART_GYRO_H
#define __UART_GYRO_H

#include "main.h"

typedef struct UART_GYRO {
  float yaw;
  float gyro_z;
  uint32_t update_tick;
  uint8_t valid;
  uint8_t rx_byte;
  uint8_t frame[5];
  uint8_t count;
  uint32_t rx_cb_cnt;
} UART_GYRO;

void uart_gyr_init(UART_GYRO *gyro);
void uart_gyr_rx_feed(UART_GYRO *gyro, uint8_t byte, uint32_t now_ms);
void uart_gyr_start_receive(UART_GYRO *gyro);
void uart_gyr_ensure_receive(UART_GYRO *gyro);
void uart_gyr_send_yaw_zero(void);
float uart_gyr_get_yaw(void);
float uart_gyr_get_z(void);

#endif
