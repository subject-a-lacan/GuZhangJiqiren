// @551

// 警告 使用该库时需要开启I2C中断

#include "gy901.h"

#include "i2c.h"

#define GYR_ADDR 0xa0

void init_gyr(GYR *gyr) {
  gyr->device_addr = GYR_ADDR;
  gyr->data_start_addr = 0x34;
  for (int i = 0; i < 24; i++) {
    gyr->data_buf[i] = 0;
  }
  gyr->cur_angle = 0;
  gyr->initial_angle = 0;
  gyr->tar_angle = 0;
}
  /**
      * @brief  从 GY901 传感器读取原始二进制数据
      * @param  i2c: 指向 I2C 句柄的指针（如 &hi2c1）
      * @param  gyr: 指向陀螺仪结构体的指针，用于存放读取到的原始数据
      * @retval 无
      * @note   该函数通过 I2C 连续读取 24 字节数据，涵盖了加速度、角速度和欧拉角等核心信息。
      *         数据存放在 gyr->data_buf 中，等待 get_gyr_value 函数解析。
      */
void get_gyr_raw_data(I2C_HandleTypeDef *i2c, GYR *gyr) {
  uint8_t buf_a[24];
  uint8_t buf_b[24];
  HAL_StatusTypeDef ret;

  ret = HAL_I2C_Mem_Read(i2c, GYR_ADDR, gyr->data_start_addr,
                          I2C_MEMADD_SIZE_8BIT, buf_a, 24, 5);
  if (ret != HAL_OK) return;

  ret = HAL_I2C_Mem_Read(i2c, GYR_ADDR, gyr->data_start_addr,
                          I2C_MEMADD_SIZE_8BIT, buf_b, 24, 5);
  if (ret != HAL_OK) {
    for (int i = 0; i < 24; i++) gyr->data_buf[i] = buf_a[i];
    return;
  }

  if (buf_a[22] == buf_b[22] && buf_a[23] == buf_b[23] &&
      buf_a[18] == buf_b[18] && buf_a[19] == buf_b[19]) {
    for (int i = 0; i < 24; i++) gyr->data_buf[i] = buf_b[i];
  }
}
 /**
     * @brief  解析原始数据并转换为实际物理量
     * @param  gyr: 指向陀螺仪结构体的指针
     * @param  key: 枚举类型，指定要获取哪种数据（加速度、角速度或角度）
     * @return 转换后的浮点型物理数值
     * @note   转换逻辑说明：
     *         1. 加速度 (a): 原始值 * 16 * 9.8 / 32768 (映射到 ±16G 范围)
     *         2. 角速度 (w): 原始值 / (32768 / 2000) (映射到 ±2000 deg/s 范围)
     *         3. 角度 (roll/pitch/yaw): 原始值 * 180 / 32768 (映射到 ±180 度范围)
     */
float get_gyr_value(GYR *gyr, enum gyroscope key) {
  uint8_t cnt = (key - gyr->data_start_addr) * 2;
  float value = (short)(((short)gyr->data_buf[cnt + 1] << 8) | gyr->data_buf[cnt]);

  switch (key) {
    case gyr_a_x:
    case gyr_a_y:
    case gyr_a_z:
      return value / 32768 * 16 * 9.8;
    case gyr_w_x:
    case gyr_w_y:
    case gyr_w_z:
      return value / 32768 * 2000;
    case gyr_x_roll:
    case gyr_y_pitch:
    case gyr_z_yaw:
      return value * 180 / 32768;
  }
}
