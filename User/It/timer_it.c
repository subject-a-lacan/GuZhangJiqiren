#include "buzzer.h"
#include "led.h"
#include "log.h"
#include "lq_step.h"
#include "servo.h"
#include "status.h"
#include "tim.h"
#include "usart.h"

uint8_t update_or_driver = 0;    // 0 : upadte  1 : driver
extern int32_t rw_time_cur;      // 临时使用的时间变量
extern int32_t rw_time_tar;      // 临时使用的时间变量
extern uint8_t cross_cnt;        // 路口计数器
uint8_t wait_finish_flag = 0;    // 等待完成标志位
extern int32_t keep_angle_time;  // 保持角度时间
extern uint8_t speed_show_flag;  // 显示速度标志位
uint8_t is_init = 0;

uint8_t find_voice[3] = {0xAA, 0x01, 0xBB};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim != &htim5) return;

  status.state.time += 1;

  if (status.state.time % 5u == 0u) {
    update_status(&status);
  }
}
