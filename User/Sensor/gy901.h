#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "main.h"
#include "pid.h"

typedef struct GYR {
  uint8_t data_buf[24];
  uint8_t device_addr;
  uint8_t data_start_addr;
  PID gy901_keep_angle_pid;
} GYR;

enum gyroscope {
  gyr_a_x = 0x34,
  gyr_a_y = 0x35,
  gyr_a_z = 0x36,
  gyr_w_x = 0x37,
  gyr_w_y = 0x38,
  gyr_w_z = 0x39,
  gyr_x_roll = 0x3D,
  gyr_y_pitch = 0x3E,
  gyr_z_yaw = 0x3F,
};

extern volatile uint8_t gyro_dma_ready;
extern volatile uint8_t gyro_dma_busy;
extern volatile uint8_t iic_gyr_initial_ready;
extern volatile uint32_t iic_gyr_dma_complete_tick;
extern volatile uint32_t iic_gyr_dma_error_count;

HAL_StatusTypeDef iic_gyr_read_dma(I2C_HandleTypeDef *i2c, GYR *gyr);
float iic_gyr_get_value(GYR *gyr, enum gyroscope key);
void iic_gyr_init(GYR *gyr);

#endif
