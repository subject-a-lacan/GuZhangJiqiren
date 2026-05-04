#include "status.h"
#include "gw_anagloge.h"
#include "button.h"
#include "buzzer.h"
#include "Defect.h"
#include "i2c.h"
#include "led.h"
#include "log.h"
#include "math_tool.h"
#include "servo.h"
#include "wheel.h"

STATUS status;

extern int16_t cmd_speed;
int32_t rw_time_cur = -1;
int32_t rw_time_tar = -1;
extern uint8_t cross_cnt;      // 路口计数器
uint8_t cross_delay = 0;       // 路口延时计数器
uint8_t speed_show_flag = 0;   // 显示速度标志位

/**
 * @brief 初始化舵机和轮子
 * @param 无
 * @return 无
 *@note 调用 init_servo(&status.motor.servo[0], 1, 180)
 *       把 1 号舵机挂到状态树里，180 是这个舵机的最大角度，后续可按实物修改
 *@note 调用 init_servo(&status.motor.servo[1], 2, 270)
 *       把 2 号舵机挂到状态树里，270 是这个舵机的最大角度，后续可按实物修改
 *@note 调用 init_wheel(&status.motor.wheel[0], 1, -1)
 *       把左轮挂到状态树里，1 是硬件编号，-1 用来修正正反转方向，后续常调
 *@note 调用 init_wheel(&status.motor.wheel[1], 2, 1)
 *       把右轮挂到状态树里，2 是硬件编号，1 表示当前方向定义，后续常调
 */
void init_motor() {
  init_servo(&status.motor.servo[0], 1, 180);
  init_servo(&status.motor.servo[1], 2, 270);

  init_wheel(&status.motor.wheel[0], 1, -1);
  init_wheel(&status.motor.wheel[1], 2, 1);

  return;
}

/**
 * @brief 初始化按键、灯和蜂鸣器
 * @param 无
 * @return 无
 *@note 调用 init_button(&status.device.button_D2, 1, 0)
 *       初始化 D2 按键，1 是按键编号，0 表示低电平按下，后续可按接线改
 *@note 调用 init_button(&status.device.button_B11, 2, 0)
 *       初始化 B11 按键，2 是按键编号，0 表示低电平按下，后续可按接线改
 *@note 调用 init_LED(&status.device.led_on_board, 1, 0)
 *       初始化板载灯，1 是设备编号，0 表示低电平点亮，后续可按电路改
 *@note 调用 init_LED(&status.device.led1, 2, 0)
 *       初始化外接 LED1，2 是设备编号，点亮电平方式后续可调
 *@note 调用 init_LED(&status.device.led2, 3, 0)
 *       初始化外接 LED2，3 是设备编号，点亮电平方式后续可调
 *@note 调用 init_BUZZER(&status.device.buzzer, 1, 1)
 *       初始化蜂鸣器，1 是设备编号，1 表示高电平响，后续可按电路改
 */
void init_device() {
  init_button(&status.device.button_D2, 1, 0);
  init_button(&status.device.button_B11, 2, 0);
  init_LED(&status.device.led_on_board, 1, 1);
  init_LED(&status.device.led1, 2, 1);
  init_LED(&status.device.led2, 3, 1);
  init_BUZZER(&status.device.buzzer, 1, 1);

  return;
}

/**
 * @brief 初始化传感器
 * @param status 状态结构体指针，用来找到各个传感器在状态树中的位置
 * @return 无
 *@note 调用 init_gyr(&status->sensor.gy901)
 *       初始化陀螺仪的缓存区、设备地址和起始寄存器地址，其中地址参数后续可调
 *@note 调用 init_gw_8bit(&status->sensor.gw_8bit)
 *       初始化 8 路数字灰度的权重、路口缓存和巡线 PID，其中权重和 PID 参数后续可调
 *@note 调用 init_gw_analogue(&status->sensor.gw_analogue)
 *       初始化 8 路模拟灰度的通道值、阈值和校准数据，其中阈值参数后续可调
 */
void init_sensor(STATUS *status) {
  init_gyr(&status->sensor.gy901);
  init_gw_analogue(&status->sensor.gw_analogue);
}


/**
 * @brief 初始化小车运行时要用到的基础状态
 * @param status 状态结构体指针，用来写入整车的初始状态
 * @param T 系统控制周期，单位 ms，表示状态多久更新一次
 * @return 无
 *@note 这里会把时间清零，并把运动模式先设成 STOP，保证上电时车不会直接动
 *@note 这里会给当前角度、目标角度、基础速度和灰度状态这些运行变量设默认值
 *@note 这里会把 road_determine 里的路口缓存、计数和判路参数设成初值
 *       其中 integral_times = 6 是后续可调的一个灵敏度参数
 *@note 参数 T 会影响后续控制节奏，是后续常调的基础参数
 */
void init_state(STATUS *status, uint8_t T) {
  status->state.T = T;
  status->state.time = 0;
  status->state.motion = STOP;
  status->state.cur_angle = 0;
  status->state.tar_angle = 90;

  status->state.gw_8bit = 0x00;

  status->state.base_speed = 0;

  status->state.motion = STOP;

  return;
}

/**
 * @brief 初始化状态层控制用的 PID
 * @param status 状态结构体指针，用来保存巡线和保角 PID
 * @return 无
 *@note 调用 init_pid(1.5, 0, 0, 20, 20)
 *       初始化巡线 PID，这几个数字分别控制跟线反应快慢和积分限制，后续都可调
 *@note 调用 init_pid(1, 0, 1, 20, 20)
 *       初始化保角 PID，这几个数字决定转向纠偏力度和稳定性，后续都可调
 */
void init_status_pid(STATUS *status) {
  status->state.status_pid.follow_line_pid = init_pid(1, 0.03, 0, 20, 1, 0.0f);
  status->state.status_pid.keep_angle_pid = init_pid(1.2, 0.4, 0, 20, 1, 0.0f);
  status->state.status_pid.angle_output_limit = 25.0f;
}

static void apply_control_param(STATUS *status, CONTROL_PARAM p) {
  status->state.status_pid.follow_line_pid = p.follow_line_pid;
  status->state.status_pid.keep_angle_pid = p.keep_angle_pid;
  status->motor.wheel[0].wheel_pid = p.wheel_left_pid;
  status->motor.wheel[1].wheel_pid = p.wheel_right_pid;
  set_wheel_ff_param(p.ff_offset, p.ff_k, p.ff_min);
  status->state.status_pid.angle_output_limit = 25.0f;
}

void apply_basic_control_param(STATUS *status) {
  CONTROL_PARAM p;
  p.follow_line_pid = init_pid(1, 0.03, 0, 20, 1, 0.0f);
  p.keep_angle_pid  = init_pid(1, 0, 0, 20, 1, 0.0f);
  p.wheel_left_pid  = init_pid(8, 0, 0, 20, 100, 0.50f);
  p.wheel_right_pid = init_pid(8, 0, 0, 20, 100, 0.50f);
  p.ff_offset = 157.0f;
  p.ff_k = 18.3f;
  p.ff_min = 254.0f;
  apply_control_param(status, p);
}

void apply_adv_control_param(STATUS *status) {
  CONTROL_PARAM p;
  p.follow_line_pid = init_pid(1, 0.03, 0, 20, 1, 0.0f);   // TODO: 负重后实车标定
  p.keep_angle_pid  = init_pid(1, 0, 0, 20, 1, 0.0f);       // TODO: 负重后实车标定
  p.wheel_left_pid  = init_pid(8, 0, 0, 20, 100, 0.50f);    // TODO: 负重后实车标定
  p.wheel_right_pid = init_pid(8, 0, 0, 20, 100, 0.50f);    // TODO: 负重后实车标定
  p.ff_offset = 157.0f;   // TODO: 负重后实车标定
  p.ff_k = 18.3f;         // TODO: 负重后实车标定
  p.ff_min = 254.0f;      // TODO: 负重后实车标定
  apply_control_param(status, p);
}

/**
 * @brief 初始化整棵 status 状态树
 * @param status 状态结构体指针，整车所有状态都会挂在这里
 * @param T 系统控制周期，单位 ms
 * @return 无
 *@note 调用 init_state(status, T)
 *       先把时间、模式、角度、基础速度和路口判断缓存设成起始值，其中 T 和 integral_times 后续可调
 *@note 调用 init_status_pid(status)
 *       把巡线 PID 和保角 PID 先准备好，这两组参数后续调车时常改
 *@note 调用 init_sensor(status)
 *       把陀螺仪、数字灰度、模拟灰度的默认参数准备好，地址、阈值、权重等后续可调
 *@note 调用 init_motor()
 *       把舵机和轮子与实际硬件通道对应起来，舵机最大角度和轮子方向参数后续可调
 *@note 调用 init_device()
 *       把按键、LED、蜂鸣器的编号和电平逻辑设好，电平有效方式后续可调
 */
void init_status(STATUS *status, uint8_t T) {
  init_state(status, T);

  init_status_pid(status);

  init_sensor(status);

  init_motor();

  init_device();

  init_task(&status->task);

  return;
}

Road road_buf = Straight;  //存储上一次检测到的路口类型

/*
 * @brief 判断当前路况是直行还是转弯
 * @param 无
 * @return 路口类型
 */
Road Turn_or_Straight() {
  if (road_buf != status.sensor.gw_analogue.cross.cross) {
    status.motor.wheel[0].tar_speed = 0; //路况发生变化就先停车
    status.motor.wheel[1].tar_speed = 0;
    if ((ABS(status.motor.wheel[0].cur_speed) < 5) && (ABS(status.motor.wheel[1].cur_speed) < 5)) {
      road_buf = status.sensor.gw_analogue.cross.cross;
    }
  }
  // if (status.state.road_determine.cross == LeftRoad && left_cnt == 1) {
  //   status.state.base_speed = 60;
  //   status.state.road_determine.integral = 4;
  // }  还没看懂为什么特化左转
  return road_buf;
}
/*
 * @brief 巡线控制（纯 PID + 差速）。
 *        只根据当前传感器偏差计算左右轮目标速度，不判断路口，不执行转弯。
 *        路口观测结果由 driver_gw_analogue 写入 status.sensor.gw_analogue.cross.cross，
 *        转弯/停车/计数由 update_task 内的小状态机根据 race_phase 决定。
 * @param status 状态结构体指针
 * @return 无
 */
void follow_line(STATUS *status) {
  float diff = compute_pid(&status->state.status_pid.follow_line_pid, status->sensor.gw_analogue.diff);
  status->motor.wheel[0].tar_speed = status->state.base_speed - (int16_t)diff;
  status->motor.wheel[1].tar_speed = status->state.base_speed + (int16_t)diff;
}

void keep_angle(STATUS *status) {
  float target = status->state.tar_angle + status->state.initial_angle;  // 目标角度
  float diff_angle = target - status->state.cur_angle;
  if (diff_angle > 180.0) {
    diff_angle -= 360.0;
  } else if (diff_angle < -180.0) {
    diff_angle += 360.0;
  }
  float diff = compute_pid(&status->state.status_pid.keep_angle_pid, diff_angle);  // PID计算
  float angle_limit = status->state.status_pid.angle_output_limit;
  diff = CONFINE(diff, -angle_limit, angle_limit);                                 // 限制速度范围
  status->motor.wheel[0].tar_speed = status->state.base_speed + (int16_t)diff;
  status->motor.wheel[1].tar_speed = status->state.base_speed - (int16_t)diff;  // 设置电机速度
}
/*
 * @brief 更新按钮状态 调用srver_button函数执行具体按键逻辑
 * @param status 状态结构体指针
 * @return 无
 */
void update_status(STATUS *status) {
  driver_gw_analogue(&status->sensor.gw_analogue);

  status->motor.wheel[0].cur_speed = get_wheel_speed(&status->motor.wheel[0]);
  status->motor.wheel[1].cur_speed = get_wheel_speed(&status->motor.wheel[1]);
  status->motor.wheel[2].cur_speed = get_wheel_speed(&status->motor.wheel[2]);
  status->motor.wheel[3].cur_speed = get_wheel_speed(&status->motor.wheel[3]);
  //get_gyr_raw_data是获取原始数据 get_gyr_value则是解析映射原始数据
  get_gyr_raw_data(&hi2c1, &status->sensor.gy901);
  status->state.cur_angle = get_gyr_value(&status->sensor.gy901, gyr_z_yaw);

  

  // log_uprintf(&huart1, "%d %d %d %d\r\n", cross_cnt, cross_delay, Turn_or_Straight(), status->state.road_determine.cross);

  driver_button(&status->device.button_D2);
  driver_button(&status->device.button_B11);

  update_task(status);
 if (status->state.motion == FIND_LINE) {
    status->task.stop_cmd = 0;
    if (!(status->task.armed && status->task.task_running)) {
      status->state.base_speed = cmd_speed;
    }
    follow_line(status);
  }
  if (status->state.motion == KEEP_ANGLE) {
    status->task.stop_cmd = 0;
    keep_angle(status);
  }
  if (status->state.motion == STOP) {
    status->task.stop_cmd = 1;
    status->motor.wheel[0].tar_speed = 0;
    status->motor.wheel[1].tar_speed = 0;
  }
  if (status->state.motion == MOTOR_TEST) {
    status->task.stop_cmd = 0;
    status->state.base_speed = cmd_speed;
    status->motor.wheel[0].tar_speed = cmd_speed;
    status->motor.wheel[1].tar_speed = cmd_speed;
  }
  driver_LED(&status->device.led_on_board);
  driver_LED(&status->device.led1);
  driver_LED(&status->device.led2);

  driver_servo(&status->motor.servo[0]);
  driver_servo(&status->motor.servo[1]); //舵机转动定角度（结构体元素确定

  if (status->device.buzzer.on && status->state.time >= status->device.buzzer.off_time) {
    status->device.buzzer.on = 0;
  }
  driver_BUZZER(&status->device.buzzer);

  driver_wheel(&status->motor.wheel[0]);
  driver_wheel(&status->motor.wheel[1]);

  return;
}

void driver_status(STATUS *status) {  // 
}
/*
 * @brief 初始化状态后更新初始角度 避免读取旧缓存值
 * @param 无
 * @return 无
 */
void after_init_state() {
  get_gyr_raw_data(&hi2c1, &status.sensor.gy901);
  HAL_Delay(50);
  status.state.initial_angle = get_gyr_value(&status.sensor.gy901, gyr_z_yaw);
}
