#include "button.h"

#include "gpio.h"
#include "gw_anagloge.h"
#include "status.h"

extern PID balance_pid;
uint8_t is_p = 1;

/*
 * @brief 按键事件业务处理函数（由 driver_button 识别事件后调用）
 * @param button 按键结构体指针，用于区分是哪个按键
 * @param station 按键事件类型（BUTTON_DOWN / BUTTON_UP / BUTTON_LONG）
 * @return 无
 *@note 按键逻辑按 jiegou.md §7：
 *       PB11(B11) 短按 → 轮换 task_id 并置 task_select_request
 *       PB11(B11) 长按 → 仅 TASK2/TASK3 时置 pose_switch_request + 长响
 *       PD2(D2)   短按 → armed 且空闲时置 start_request + 短响
 *       PD2(D2)   长按 → 灰度校准
 */
void server_button(BUTTON *button, BUTTON_STATION station) {
  // PB11 (which == 2)
  if (button->which == 2) {
    if (station == BUTTON_UP) {
      if (status.task.task_running == 0) {
        uint8_t next = status.task.task_id + 1;
        if (next > 4) next = 1;
        status.task.requested_task_id = next;
        status.task.task_select_request = 1;
      }
    }
    if (station == BUTTON_LONG) {
      if (status.task.task_running == 0 && (status.task.task_id == TASK_BASIC_2 || status.task.task_id == TASK_ADV_1)) {
        status.task.pose_switch_request = 1;
        status.device.buzzer.on = 1;
        status.device.buzzer.off_time = status.state.time + 1000;
      }
    }
  }

  // PD2 (which == 1)
  if (button->which == 1) {
    if (station == BUTTON_UP) {
      if (status.task.armed == 1 && status.task.task_running == 0) {
        status.task.start_request = 1;
        status.device.buzzer.on = 1;
        status.device.buzzer.off_time = status.state.time + 200;
      }
    }
    if (station == BUTTON_LONG) {
      correct_gw_analogue(&status.sensor.gw_analogue);
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


    //注意注意 高电平按下有点问题 不过现在是低电平按下 所以无所谓
    
    if (button->long_press_cnt > 0) {
      button->long_press_cnt--;
    } else if (button->long_press_cnt == 0) {
      server_button(button, BUTTON_LONG);
      // 置为 -1，避免在持续按下期间重复触发长按事件
      button->long_press_cnt = -1;
      button->long_triggered = 1;
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
        button->long_triggered = 0;
        // 按下沿额外处理：如果长按计数还没到阈值，继续递减
        // 否则补发一次长按事件（兼容低频调用场景）
        if (button->long_press_cnt - 1 >= 0) {
          button->long_press_cnt--;
        } else {
          server_button(button, BUTTON_LONG);
          button->long_triggered = 1;
        }
      } else {
        if (button->long_triggered == 0) {
          server_button(button, BUTTON_UP);
        }
        // 释放后恢复长按计数
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    } else {
      // 低电平按下：now=0 为按下沿，now=1 为释放沿
      if (button->now == 0) {
        server_button(button, BUTTON_DOWN);
        button->long_triggered = 0;
      } else {
        if (button->long_triggered == 0) {
          server_button(button, BUTTON_UP);
        }
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
  button->long_triggered = 0;
  return;
}