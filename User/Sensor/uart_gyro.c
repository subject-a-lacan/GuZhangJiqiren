#include "uart_gyro.h"
#include "usart.h"

static UART_GYRO *active_gyro;

static const uint8_t gyro_key[5] = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
static const uint8_t gyro_zero[5] = {0x55, 0xAA, 0x15, 0x00, 0x00};
static const uint8_t gyro_save[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};

void uart_gyr_init(UART_GYRO *gyro) {
  active_gyro = gyro;
  gyro->yaw = 0.0f;
  gyro->gyro_z = 0.0f;
  gyro->update_tick = 0;
  gyro->valid = 0;
  gyro->count = 0;
}

float uart_gyr_get_yaw(void) { return active_gyro ? active_gyro->yaw : 0.0f; }
float uart_gyr_get_z(void) { return active_gyro ? active_gyro->gyro_z : 0.0f; }

void uart_gyr_rx_feed(UART_GYRO *gyro, uint8_t byte, uint32_t now_ms) {
  uint8_t sum;
  int16_t raw;

  if (gyro->count == 0 && byte != 0x5A) return;
  if (gyro->count >= sizeof(gyro->frame)) gyro->count = 0;
  gyro->frame[gyro->count++] = byte;
  if (gyro->count < 5) return;

  sum = (uint8_t)(gyro->frame[0] + gyro->frame[1] + gyro->frame[2] + gyro->frame[3]);
  if (sum == gyro->frame[4] &&
      (gyro->frame[1] == 0xAA || gyro->frame[1] == 0xBB)) {
    raw = (int16_t)(((uint16_t)gyro->frame[3] << 8) | gyro->frame[2]);
    if (gyro->frame[1] == 0xAA) gyro->gyro_z = (float)raw * (2000.0f / 32768.0f);
    if (gyro->frame[1] == 0xBB) gyro->yaw = (float)raw * (180.0f / 32768.0f);
    if (gyro->frame[1] == 0xAA || gyro->frame[1] == 0xBB) {
      gyro->update_tick = now_ms;
      gyro->valid = 1;
    }
  }
  gyro->count = 0;
}

void uart_gyr_start_receive(UART_GYRO *gyro) {
  (void)gyro;
}

static void send_command(const uint8_t *cmd) {
  HAL_UART_Transmit(&huart2, (uint8_t *)cmd, 5, 20);
}

void uart_gyr_send_yaw_zero(void) {
  send_command(gyro_key);
  HAL_Delay(100);
  send_command(gyro_zero);
  HAL_Delay(100);
  send_command(gyro_save);
}
