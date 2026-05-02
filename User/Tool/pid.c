// @551

#include "pid.h"

#include "log.h"
#include "math_tool.h"
#include "status.h"

PID init_pid(float kp, float ki, float kd, float T, float integral_max) {
  PID pid;
  pid.kp = kp;
  pid.ki = ki;
  pid.kd = kd;
  pid.T = T;
  pid.integral_max = integral_max;
  pid.error = 0;
  pid.last_error = 0;
  pid.integral = 0;
  pid.derivative = 0;//微分
  pid.is_first = 1;
  pid.out = 0;

  return pid;
}
/**
 * @brief 计算PID输出
 * @param pid PID结构体指针
 * @param error 误差值
 * @return PID输出值
 */
float compute_pid(PID *pid, float error) {
  if (pid == NULL) {
    // WARN("pid is not initialized");
    return 0;
  }
  pid->error = error;
  pid->integral += pid->error * pid->T;
  pid->integral = CONFINE(pid->integral, -pid->integral_max, pid->integral_max);
  if (pid->is_first) {
    pid->derivative = 0; //微分项初始为0 避免第一次计算时出现较大输出
    pid->is_first = 0;
  } else {
    pid->derivative = (pid->error - pid->last_error) / pid->T;
  }
  pid->last_error = pid->error;
  pid->out = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;

  return pid->out;
}

/*
 * 将编码器累计脉冲数转换为实际行驶距离(cm)。
 * 调用方需自行维护脉冲累加器（如 phase_mileage），
 * 每 20ms 将 get_wheel_speed() 的绝对值累加后传入。
 */
float encoder_pulse_to_cm(int32_t pulse) {
  return (float)pulse * CM_PER_PULSE;
}
