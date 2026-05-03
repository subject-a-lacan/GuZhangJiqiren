// @551

#include "servo.h"

#include "log.h"
#include "status.h"
#include "tim.h"

void driver_servo(SERVO *servo) {
  float real_angle = servo->angle + servo->angle_error;
  if (real_angle < 0) real_angle = 0;
  if (real_angle > servo->max_angle) real_angle = servo->max_angle;
  if (servo->which == 1) {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, (int)((((float)real_angle / (float)servo->max_angle) * 0.1 + 0.025) * 50000));
  } else if (servo->which == 2) {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (int)((((float)real_angle / (float)servo->max_angle) * 0.1 + 0.025) * 50000));
  }

  return;
}

void init_servo(SERVO *servo, uint8_t which, float max_angle, float angle_error) {
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  servo->which = which;
  servo->max_angle = max_angle;
  servo->angle_error = angle_error;
  servo->angle = max_angle/2 + servo->angle_error;
  if (which == 1) {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, (int)((((float)servo->angle / (float)servo->max_angle) * 0.1 + 0.025) * 50000));
  } else if (which == 2) {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (int)((((float)servo->angle / (float)servo->max_angle) * 0.1 + 0.025) * 50000));
  }
  return;
}
