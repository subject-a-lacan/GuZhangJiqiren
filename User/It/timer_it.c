#include "buzzer.h"

#include "gw_anagloge.h"
#include "led.h"
#include "log.h"
#include "lq_step.h"
#include "servo.h"
#include "status.h"
#include "tim.h"
#include "usart.h"

extern int32_t rw_time_cur;
extern int32_t rw_time_tar;
extern int32_t keep_angle_time;
extern uint8_t speed_show_flag;
uint8_t wait_finish_flag = 0;
uint8_t is_init = 0;

uint8_t maixcam[3] = {0xAA, 0x02, 0xBB};
uint8_t find_voice[3] = {0xAA, 0x01, 0xBB};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim5) {
    status.state.time += status.state.T;

    // 上电闪烁（LED1指示系统启动）
    if (status.state.time == 50)
      status.device.led1.on = 1;
    else if (status.state.time == 100)
      status.device.led1.on = 0;
    else if (status.state.time == 150)
      status.device.led1.on = 1;
    else if (status.state.time == 200)
      status.device.led1.on = 0;
    else if (status.state.time == 250)
      status.device.led1.on = 1;
    else if (status.state.time == 300)
      status.device.led1.on = 0;

    // 每10ms：灰度传感器采集（raw→digital→diff→road）
    if (status.state.time % 10 == 0) {
      driver_gw_analogue(&status.sensor.gw_analogue);
    }

    // 每20ms：主控制循环
    if (status.state.time % 20 == 0) {
      update_status(&status);
    }
  }
}
