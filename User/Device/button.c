#include "button.h"

#include "gpio.h"
#include "gw_anagloge.h"
#include "status.h"

extern PID balance_pid;
uint8_t is_p = 1;

/*
 * @brief 鎸夐敭浜嬩欢涓氬姟澶勭悊鍑芥暟锛堢敱 driver_button 璇嗗埆浜嬩欢鍚庤皟鐢級
 * @param button 鎸夐敭缁撴瀯浣撴寚閽堬紝鐢ㄤ簬鍖哄垎鏄摢涓寜閿?
 * @param station 鎸夐敭浜嬩欢绫诲瀷锛圔UTTON_DOWN / BUTTON_UP / BUTTON_LONG锛?
 * @return 鏃?
 *@note 鎸夐敭閫昏緫鎸?jiegou.md 搂7锛?
 *       PB11(B11) 鐭寜 鈫?杞崲 task_id 骞剁疆 task_select_request
 *       PB11(B11) 闀挎寜 鈫?浠?TASK2/TASK3 鏃剁疆 pose_switch_request + 闀垮搷
 *       PD2(D2)   鐭寜 鈫?armed 涓旂┖闂叉椂缃?start_request + 鐭搷
 *       PD2(D2)   闀挎寜 鈫?鐏板害鏍″噯
 */
void server_button(BUTTON *button, BUTTON_STATION station) {
  // PB11 (which == 2)
  if (button->which == 2) {
    if (station == BUTTON_UP) {
      if (status.task.task_running == 0) {
        uint8_t next = status.task.task_id + 1;
        if (next > 7) next = 1;
        status.task.requested_task_id = next;
        status.task.task_select_request = 1;
        status.device.buzzer.on = 1;
        status.device.buzzer.off_time = status.state.time + 280;
      }
    }
    /* PB11 long press has no task meaning; posture switching was removed. */
  }

  // PD2 (which == 1)
  if (button->which == 1) {
    if (station == BUTTON_UP) {
      if (status.task.armed == 0 && status.task.task_running == 0) {
        status.task.start_request = 1;
        status.device.buzzer.on = 1;
        status.device.buzzer.off_time = status.state.time + 280;
      }
      else if (status.task.task_running) {
        status.task.stop_request = 1;
        status.device.buzzer.on = 1;
        status.device.buzzer.off_time = status.state.time + 280;
      }
    }
    if (station == BUTTON_LONG) {
      if (status.task.task_running == 0) {
        correct_gw_analogue(&status.sensor.gw_analogue);
        status.device.buzzer.on = 1;
        status.device.buzzer.off_time = status.state.time + 1050;
      }
    }
  }

  return;
}

void driver_button(BUTTON *button) {
  // 1) 鎸夋寜閿紪鍙疯鍙栧綋鍓嶅紩鑴氱數骞筹紝鍐欏叆 now
  if (button->which == 1) {
    button->now = HAL_GPIO_ReadPin(BUTTON_D2_GPIO_Port, BUTTON_D2_Pin);
  } else if (button->which == 2) {
    button->now = HAL_GPIO_ReadPin(BUTTON_B11_GPIO_Port, BUTTON_B11_Pin);
  }

  // 2) 闀挎寜妫€娴嬶細鍏堝垽鏂€滃綋鍓嶆槸鍚﹀浜庢寜涓嬫€佲€?
  // Press_is_high_level=1 琛ㄧず楂樼數骞虫寜涓嬶紱=0 琛ㄧず浣庣數骞虫寜涓?
  // 杩欓噷鐨勮〃杈惧紡绛変环浜庯細button->now == button->Press_is_high_level
  if (1 ^ (button->now ^ button->Press_is_high_level)) {
    // 鎸変笅鎸佺画鏃讹紝瀵?long_press_cnt 閫掑噺锛屽埌 0 瑙﹀彂涓€娆?BUTTON_LONG


    //娉ㄦ剰娉ㄦ剰 楂樼數骞虫寜涓嬫湁鐐归棶棰?涓嶈繃鐜板湪鏄綆鐢靛钩鎸変笅 鎵€浠ユ棤鎵€璋?
    
    if (button->long_press_cnt > 0) {
      button->long_press_cnt--;
    } else if (button->long_press_cnt == 0) {
      server_button(button, BUTTON_LONG);
      // 缃负 -1锛岄伩鍏嶅湪鎸佺画鎸変笅鏈熼棿閲嶅瑙﹀彂闀挎寜浜嬩欢
      button->long_press_cnt = -1;
      button->long_triggered = 1;
    }
  } else {
    // 鏈寜涓嬫椂鎭㈠闀挎寜璁℃暟鍣?
    button->long_press_cnt = LONG_PRESS_CNT;
  }

  // 3) 杈规部妫€娴嬶細now 涓?last 涓嶅悓锛岃鏄庢寜閿姸鎬佸彂鐢熷彉鍖?
  if (button->now != button->last) {
    if (button->Press_is_high_level == 1) {
      // 楂樼數骞虫寜涓嬶細now=1 涓烘寜涓嬫部锛宯ow=0 涓洪噴鏀炬部
      if (button->now == 1) {
        server_button(button, BUTTON_DOWN);
        button->long_triggered = 0;
        // 鎸変笅娌块澶栧鐞嗭細濡傛灉闀挎寜璁℃暟杩樻病鍒伴槇鍊硷紝缁х画閫掑噺
        // 鍚﹀垯琛ュ彂涓€娆￠暱鎸変簨浠讹紙鍏煎浣庨璋冪敤鍦烘櫙锛?
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
        // 閲婃斁鍚庢仮澶嶉暱鎸夎鏁?
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    } else {
      // 浣庣數骞虫寜涓嬶細now=0 涓烘寜涓嬫部锛宯ow=1 涓洪噴鏀炬部
      if (button->now == 0) {
        server_button(button, BUTTON_DOWN);
        button->long_triggered = 0;
      } else {
        if (button->long_triggered == 0) {
          server_button(button, BUTTON_UP);
        }
        // 閲婃斁鍚庢仮澶嶉暱鎸夎鏁?
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    }
    // 4) 鏈疆澶勭悊缁撴潫锛屽埛鏂?last 渚涗笅涓€杞仛杈规部姣旇緝
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


