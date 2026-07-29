// @551

// 璀﹀憡 浣跨敤璇ュ簱鏃堕渶瑕佸紑鍚疘2C涓柇

#include "gy901.h"

#include "i2c.h"
#include "pid.h"

#define GYR_ADDR 0xa0

volatile uint8_t gyro_dma_ready = 0;
volatile uint8_t gyro_dma_busy = 0;

void iic_gyr_init(GYR *gyr) {
  gyr->device_addr = GYR_ADDR;
  gyr->data_start_addr = 0x34;
  for (int i = 0; i < 24; i++) {
    gyr->data_buf[i] = 0;
  }
  gyr->gy901_keep_angle_pid = init_pid(50, 0, 0, 50, 500, 0.0f);
  return;
}
  /**
      * @brief  浠?GY901 浼犳劅鍣ㄨ鍙栧師濮嬩簩杩涘埗鏁版嵁
      * @param  i2c: 鎸囧悜 I2C 鍙ユ焺鐨勬寚閽堬紙濡?&hi2c1锛?
      * @param  gyr: 鎸囧悜闄€铻轰华缁撴瀯浣撶殑鎸囬拡锛岀敤浜庡瓨鏀捐鍙栧埌鐨勫師濮嬫暟鎹?
      * @retval 鏃?
      * @note   璇ュ嚱鏁伴€氳繃 I2C 杩炵画璇诲彇 24 瀛楄妭鏁版嵁锛屾兜鐩栦簡鍔犻€熷害銆佽閫熷害鍜屾鎷夎绛夋牳蹇冧俊鎭€?
      *         鏁版嵁瀛樻斁鍦?gyr->data_buf 涓紝绛夊緟 iic_gyr_get_value 鍑芥暟瑙ｆ瀽銆?
      */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    gyro_dma_busy = 0;
    gyro_dma_ready = 1;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    gyro_dma_busy = 0;
  }
}

HAL_StatusTypeDef iic_gyr_read_dma(I2C_HandleTypeDef *i2c, GYR *gyr) {
  HAL_StatusTypeDef ret;

  if (gyro_dma_busy) {
    return HAL_BUSY;
  }

  ret = HAL_I2C_Mem_Read_DMA(i2c, GYR_ADDR, gyr->data_start_addr, I2C_MEMADD_SIZE_8BIT, gyr->data_buf, 24);
  if (ret == HAL_OK) {
    gyro_dma_ready = 0;
    gyro_dma_busy = 1;
  }

  return ret;
}
 /**
     * @brief  瑙ｆ瀽鍘熷鏁版嵁骞惰浆鎹负瀹為檯鐗╃悊閲?
     * @param  gyr: 鎸囧悜闄€铻轰华缁撴瀯浣撶殑鎸囬拡
     * @param  key: 鏋氫妇绫诲瀷锛屾寚瀹氳鑾峰彇鍝鏁版嵁锛堝姞閫熷害銆佽閫熷害鎴栬搴︼級
     * @return 杞崲鍚庣殑娴偣鍨嬬墿鐞嗘暟鍊?
     * @note   杞崲閫昏緫璇存槑锛?
     */
float iic_gyr_get_value(GYR *gyr, enum gyroscope key) {
  uint8_t cnt = (key - gyr->data_start_addr) * 2;
  float value = (short)(((short)gyr->data_buf[cnt + 1] << 8) | gyr->data_buf[cnt]);

  switch (key) {
    case gyr_a_x:
    case gyr_a_y:
    case gyr_a_z:
      return value * 16 * 9.8;
    case gyr_w_x:
    case gyr_w_y:
    case gyr_w_z:
      return value / 2000;
    case gyr_x_roll:
    case gyr_y_pitch:
    case gyr_z_yaw:
      return value * 180 / 32768;
  }
}

