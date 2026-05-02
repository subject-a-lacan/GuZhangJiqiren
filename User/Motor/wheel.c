#include "wheel.h"

#include "gpio.h"
#include "log.h"
#include "math_tool.h"
#include "status.h"
#include "tim.h"

int16_t get_wheel_speed(WHEEL *wheel) {
  int16_t speed = 0;
  if (wheel->which == 1) {
    speed = TIM1->CNT - 30000;
    TIM1->CNT = 30000;
  } else if (wheel->which == 2) {
    speed = TIM2->CNT - 30000;
    TIM2->CNT = 30000;
  } else if (wheel->which == 3) {
    speed = TIM3->CNT - 30000;
    TIM3->CNT = 30000;
  } else if (wheel->which == 4) {
    speed = TIM4->CNT - 30000;
    TIM4->CNT = 30000;
  }

  return speed * wheel->dir;
}
//隔20ms进一次中断 读取cnt 并清零 减去30000的作用是为了避免计数器溢出和处理负数情况 

/**
 * @brief 设置方向
 * @param wheel 轮子结构体指针
 * @param trust 推力值
 * @return 无
 * @note tb6612
 */
void set_wheel_dir(WHEEL *wheel, int16_t trust) {
  if (wheel->which == 1) {
    if (wheel->trust * wheel->dir > 0) {
      HAL_GPIO_WritePin(M1D1_GPIO_Port, M1D1_Pin, 1);
      HAL_GPIO_WritePin(M1D2_GPIO_Port, M1D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M1D1_GPIO_Port, M1D1_Pin, 0);
      HAL_GPIO_WritePin(M1D2_GPIO_Port, M1D2_Pin, 1);
    }
  } else if (wheel->which == 2) {
    if (wheel->trust * wheel->dir < 0) {
      HAL_GPIO_WritePin(M2D1_GPIO_Port, M2D1_Pin, 1);
      HAL_GPIO_WritePin(M2D2_GPIO_Port, M2D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M2D1_GPIO_Port, M2D1_Pin, 0);
      HAL_GPIO_WritePin(M2D2_GPIO_Port, M2D2_Pin, 1);
    }
  } else if (wheel->which == 3) {
    if (wheel->trust * wheel->dir > 0) {
      HAL_GPIO_WritePin(M3D1_GPIO_Port, M3D1_Pin, 1);
      HAL_GPIO_WritePin(M3D2_GPIO_Port, M3D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M3D1_GPIO_Port, M3D1_Pin, 0);
      HAL_GPIO_WritePin(M3D2_GPIO_Port, M3D2_Pin, 1);
    }
  } else if (wheel->which == 4) {
    if (wheel->trust * wheel->dir > 0) {
      HAL_GPIO_WritePin(M4D1_GPIO_Port, M4D1_Pin, 1);
      HAL_GPIO_WritePin(M4D2_GPIO_Port, M4D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M4D1_GPIO_Port, M4D1_Pin, 0);
      HAL_GPIO_WritePin(M4D2_GPIO_Port, M4D2_Pin, 1);
    }
  }
}

/**
 * @brief 轮子驱动函数
 * @param wheel 轮子结构体指针
 * @return 无
 * @note 通过PID计算得到推力，并设置电机方向和PWM占空比
 */
void driver_wheel(WHEEL *wheel) {
  if (status.task.stop_cmd) {
    wheel->trust = 0;
    if (wheel->which == 1) {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    } else if (wheel->which == 2) {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    } else if (wheel->which == 3) {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    } else if (wheel->which == 4) {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
    }
    return;
  }

  wheel->trust = compute_pid(&wheel->wheel_pid, wheel->tar_speed - wheel->cur_speed);
  wheel->trust = CONFINE(wheel->trust, -TRUST_CONFINE, TRUST_CONFINE); //trust的单位是PWM占空比的值 

  if (wheel->tar_speed == 0 && ABS(wheel->cur_speed) < 3) {
    wheel->trust = 0;
  }

  if (ABS(wheel->cur_speed) < 10) {
    wheel->trust = CONFINE(wheel->trust, -1500, 1500);//起步限幅 如果车速很小 限幅防止PID调控过大
  }

  set_wheel_dir(wheel, wheel->trust);
  //输出PWM
  if (wheel->which == 1) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ABS(wheel->trust));
  } else if (wheel->which == 2) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ABS(wheel->trust));
  } else if (wheel->which == 3) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, ABS(wheel->trust));
  } else if (wheel->which == 4) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, ABS(wheel->trust));
  }

  return;
}
/**
 * @brief 初始化轮子
 * @param wheel 轮子结构体指针
 * @param which 轮子编号
 * @param dir 方向
 * @return 无
 */
void init_wheel(WHEEL *wheel, uint8_t which, int8_t dir) {
  wheel->which = which;
  wheel->trust = 0;
  wheel->cur_speed = 0;
  wheel->tar_speed = 0;
  wheel->dir = dir;
  wheel->wheel_pid = init_pid(40, 25, 5, 20, 24);  // kd=0，导数项已禁用  P I D T integral_max
  if (wheel->which == 1) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  } else if (wheel->which == 2) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  } else if (wheel->which == 3) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  } else if (wheel->which == 4) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  }

  if (wheel->which == 1) {
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    TIM1->CNT = 30000;  // 设置计数器初始值为30000，避免0时出现负数 同时避免溢出处理 也就是人为设置零点
  } else if (wheel->which == 2) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    TIM2->CNT = 30000;
  } else if (wheel->which == 3) {
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    TIM3->CNT = 30000;
  } else if (wheel->which == 4) {
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    TIM4->CNT = 30000;
  }

  return;
}
