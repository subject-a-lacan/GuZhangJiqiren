#include "button.h"

#include "gpio.h"
#include "gw_anagloge.h"
#include "led.h"
#include "log.h"
#include "lq_step.h"
#include "math_tool.h"
#include "status.h"
#include "usart.h"

extern PID balance_pid;
extern int32_t rw_time_cur;
uint8_t is_p = 1;
extern uint8_t wait_finish_flag;
uint8_t button_2_cnt = 0;  // 按键2计数器

/*
 * @brief 按键事件业务处理函数（由 driver_button 识别事件后调用）
 * @param button 按键结构体指针，用于区分是哪个按键
 * @param station 按键事件类型（BUTTON_DOWN / BUTTON_UP / BUTTON_LONG）
 * @return 无
 *@note 按键触发逻辑如下：
 *       1) which==1（D2 按键）:
 *          - 仅在 BUTTON_DOWN（按下沿）时触发 `correct_gw_analogue(...)`，用于灰度校准。
 *       2) which==2（B11 按键）且仅当 `status.state.motion == STOP` 时生效：
 *          - 第 1 次触发（button_2_cnt == 0）且事件为 BUTTON_DOWN：
 *            记录当前时间 `rw_time_cur`，设置 `base_speed=40`，并把运动模式切到 `FIND_LINE`。
 *          - 第 2 次触发（button_2_cnt == 1）：
 *            置 `wait_finish_flag=1`，并将 `led1.on=0`。
 *       3) 每次进入该 which==2 分支后，`button_2_cnt` 自增一次。
 */
void server_button(BUTTON *button, BUTTON_STATION station) {
  if (button->which == 1) {
    if (station == BUTTON_DOWN) {
      correct_gw_analogue(&status.sensor.gw_analogue);
    }
  }
  if (status.state.motion == STOP) {
    if (button->which == 2) {
      if (button_2_cnt == 0) {
        if (station == BUTTON_DOWN) {
          rw_time_cur = status.state.time;
          status.state.base_speed = 40;
          status.state.motion = FIND_LINE;
          rw_time_cur = status.state.time;
        }
      } else if (button_2_cnt == 1) {
        wait_finish_flag = 1;
      }
      button_2_cnt++;
    }
  }
  return;
}

void driver_button(BUTTON *button) {
  // 1) 按按键编号读取当前引脚电平，写入 now
  if (button->which == 1) {
    button->now = HAL_GPIO_ReadPin(BUTTON_D2_GPIO_Port, BUTTON_D2_Pin);
  } else if (button->which == 2) {
    button->now = HAL_GPIO_ReadPin(BUTTON_B11_GPIO_Port, BUTTON_B11_Pin);
  }

  // 2) 长按检测：先判断“当前是否处于按下态”
  // Press_is_high_level=1 表示高电平按下；=0 表示低电平按下
  // 这里的表达式等价于：button->now == button->Press_is_high_level
  if (1 ^ (button->now ^ button->Press_is_high_level)) {
    // 按下持续时，对 long_press_cnt 递减，到 0 触发一次 BUTTON_LONG
    if (button->long_press_cnt > 0) {
      button->long_press_cnt--;
    } else if (button->long_press_cnt == 0) {
      server_button(button, BUTTON_LONG);
      // 置为 -1，避免在持续按下期间重复触发长按事件
      button->long_press_cnt = -1;
    }
  } else {
    // 未按下时恢复长按计数器
    button->long_press_cnt = LONG_PRESS_CNT;
  }

  // 3) 边沿检测：now 与 last 不同，说明按键状态发生变化
  if (button->now != button->last) {
    if (button->Press_is_high_level == 1) {
      // 高电平按下：now=1 为按下沿，now=0 为释放沿
      if (button->now == 1) {
        server_button(button, BUTTON_DOWN);
        // 按下沿额外处理：如果长按计数还没到阈值，继续递减
        // 否则补发一次长按事件（兼容低频调用场景）
        if (button->long_press_cnt - 1 >= 0) {
          button->long_press_cnt--;
        } else {
          server_button(button, BUTTON_LONG);
        }
      } else {
        server_button(button, BUTTON_UP);
        // 释放后恢复长按计数
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    } else {
      // 低电平按下：now=0 为按下沿，now=1 为释放沿
      if (button->now == 0) {
        server_button(button, BUTTON_DOWN);
      } else {
        server_button(button, BUTTON_UP);
        // 释放后恢复长按计数
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    }
    // 4) 本轮处理结束，刷新 last 供下一轮做边沿比较
    button->last = button->now;
  }
}

void init_button(BUTTON *button, uint8_t which, uint8_t Press_is_high_level) {
  button->which = which;
  button->Press_is_high_level = Press_is_high_level;
  button->last = Press_is_high_level ? 0 : 1;
  button->now = Press_is_high_level ? 0 : 1;
  button->long_press_cnt = LONG_PRESS_CNT;
  return;
}