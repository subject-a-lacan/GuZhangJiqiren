#include "status.h"

#include "button.h"
#include "buzzer.h"
#include "i2c.h"
#include "led.h"
#include "log.h"
#include "math_tool.h"
#include "servo.h"
#include "wheel.h"

STATUS status;

int32_t rw_time_cur = -1;
int32_t rw_time_tar = -1;
int32_t keep_angle_time = -1;
uint8_t speed_show_flag = 0;

// ── 任务层使用的全局状态变量 ──
uint8_t cross_delay = 0;

void init_motor() {
  init_servo(&status.motor.servo[0], 1, 270, 35);
  init_servo(&status.motor.servo[1], 2, 180, 0);
  init_wheel(&status.motor.wheel[0], 1, 1);
  init_wheel(&status.motor.wheel[1], 2, -1);
}

void init_device() {
  init_button(&status.device.button_D2, 1, 0);
  init_button(&status.device.button_B11, 2, 0);
  init_LED(&status.device.led_on_board, 1, 1);
  init_LED(&status.device.led1, 2, 1);
  init_LED(&status.device.led2, 3, 1);
  init_BUZZER(&status.device.buzzer, 1, 1);
}

void init_sensor(STATUS *status) {
  init_gyr(&status->sensor.gy901);
  init_gw_analogue(&status->sensor.gw_analogue);
}

void init_state(STATUS *status, uint8_t T) {
  status->state.T = T;
  status->state.time = 0;
  status->state.motion = STOP;
  status->state.cur_angle = 0;
  status->state.tar_angle = 90;
  status->state.base_speed = 0;
  status->state.motion_bypass = 0;

  // 初始化转弯上下文
  status->state.turn.target_yaw = 0;
  status->state.turn.entry_yaw = 0;
  status->state.turn.search_active = 0;
  status->state.turn.search_dir = 0;
}

void init_status_pid(STATUS *status) {
  status->state.status_pid.follow_line_pid = init_pid(0.8, 0.03, 2.5, 20, 30);
  status->state.status_pid.keep_angle_pid = init_pid(1.0, 0.0, 1.5, 20, 20);
}

void init_status(STATUS *status, uint8_t T) {
  init_state(status, T);
  init_status_pid(status);
  init_sensor(status);
  init_motor();
  init_device();
  init_task(&status->task);
}

// ── 公共差速输出 ──

static void diff_drive(STATUS *s, float base, float diff, float limit) {
  diff = CONFINE(diff, -limit, limit);
  int16_t ds = (int16_t)diff;
  s->motor.wheel[0].tar_speed = base + ds;
  s->motor.wheel[1].tar_speed = base - ds;
}

// ── 公共运动执行层（所有任务共用）──

void motion_execute(STATUS *s) {
  if (s->state.motion_bypass) return;  // Defect.c 直接控制轮速
  switch (s->state.motion) {
    case FIND_LINE: {
      float d = s->sensor.gw_analogue.diff;
      float out = compute_pid(&s->state.status_pid.follow_line_pid, d);
      diff_drive(s, s->state.base_speed, out, LINE_DIFF_LIMIT);
      break;
    }
    case KEEP_ANGLE: {
      // 角度到位但没找到线 → 继续朝搜线方向转（不刹车）
      if (s->state.turn.search_active) {
        diff_drive(s, TURN_BASE_SPEED,
                   s->state.turn.search_dir * SEARCH_DIFF, SEARCH_DIFF);
        break;
      }
      float err = s->state.turn.target_yaw - s->state.cur_angle;
      if (err > 180.0f) err -= 360.0f;
      else if (err < -180.0f) err += 360.0f;
      float out = compute_pid(&s->state.status_pid.keep_angle_pid, err);
      diff_drive(s, TURN_BASE_SPEED, out, TURN_DIFF_LIMIT);
      break;
    }
    case STOP:
      // 轮速由任务层控制（Defect.c 处理刹车逻辑）
      break;
    case MOTOR_TEST:
      s->motor.wheel[0].tar_speed = 40;
      s->motor.wheel[1].tar_speed = 40;
      break;
  }
}

// ── 更新层（感知 → 策略 → 执行 → 驱动）──

void update_status(STATUS *s) {
  // ── 感知层：读取所有传感器 ──
  s->motor.wheel[0].cur_speed = get_wheel_speed(&s->motor.wheel[0]);
  s->motor.wheel[1].cur_speed = get_wheel_speed(&s->motor.wheel[1]);
  get_gyr_raw_data(&hi2c1, &s->sensor.gy901);
  s->state.cur_angle = get_gyr_value(&s->sensor.gy901, gyr_z_yaw);

  driver_button(&s->device.button_D2);
  driver_button(&s->device.button_B11);

  // ── 策略层：任务调度 ──
  update_task(s);

  // ── 执行层：按 motion 执行控制 ──
  motion_execute(s);

  // ── 驱动层：输出到硬件 ──
  driver_LED(&s->device.led_on_board);
  driver_LED(&s->device.led1);
  driver_LED(&s->device.led2);
  driver_servo(&s->motor.servo[0]);
  driver_servo(&s->motor.servo[1]);

  if (s->device.buzzer.on && s->state.time >= s->device.buzzer.off_time) {
    s->device.buzzer.on = 0;
  }
  driver_BUZZER(&s->device.buzzer);

  driver_wheel(&s->motor.wheel[0]);
  driver_wheel(&s->motor.wheel[1]);
}

void driver_status(STATUS *status) {
}

void after_init_state() {
  get_gyr_raw_data(&hi2c1, &status.sensor.gy901);
  HAL_Delay(50);
  status.state.initial_angle = get_gyr_value(&status.sensor.gy901, gyr_z_yaw);
}
