#include "led.h"

#include "gpio.h"
#include "log.h"
#include "status.h"
#include "stdbool.h"

void driver_LED(LED *led) {
  if (led->which == 1) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1 ^ ((bool)(led->on) ^ (bool)(led->High_level_is_on)));
  }//异或：恰有一个为真时为真 反之为假 符号为 ^
  if (led->which == 2) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1 ^ ((bool)(led->on) ^ (bool)(led->High_level_is_on)));
  }
  if (led->which == 3) {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 ^ ((bool)(led->on) ^ (bool)(led->High_level_is_on)));
  }

  return;
}
/*
 * @brief 初始化LED 写入结构体元素
 * @param led LED结构体指针
 * @param which LED编号
 * @param High_level_is_on 高电平是否点亮LED
 * @return 无
 */
void init_LED(LED *led, uint8_t which, uint8_t High_level_is_on) {
  led->High_level_is_on = High_level_is_on;
  led->on = 0;
  led->which = which;
  return;
}