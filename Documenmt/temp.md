我们之前讨论过的那个控制步进电机速度和加速度 使其更丝滑的接入我现有状态树代码的全套详细方案给我
思考了 11m 14s

结论：最适合你现有控制链的方案是：

保持 PID→查表→目标脉冲不变，在目标脉冲发布前加入“步进命令整形器”：小角度修正低速、柔和加速；大角度修正高速、较快加速；换向时强制降速；最短20 ms才发布一次。

这样不会给小球外环增加很重的位置滤波延迟，同时能解决现在 velocity=50、acc=0 的硬启停问题。

你当前对应文件是：

ball_control(4).c
ball_control(4).h
status(1).h
Defect(3).c
uart_it(2).c
一、你现在为什么不丝滑

当前代码固定：

#define BALL_STEPPER_VELOCITY    (50u)
#define BALL_STEPPER_ACCEL_PARAM (0u)

Emm42 V5协议中：

velocity：电机目标转速；
acc：曲线加减速档位；
acc=0：关闭曲线加减速，直接切换到设定速度；
acc越大，加速越快，并不是越柔和。

协议给出的关系是：

a
rpm/s
	​

=
256−acc
20000
	​


其中：

a
rpm/s
	​

：电机转速每秒变化多少RPM；
acc：发送给驱动器的加速度档位，范围1～255；
acc=0单独表示关闭曲线。

这些定义来自Emm42 V5.0协议手册。

因此你现在每次PID改变目标脉冲，电机都可能瞬间启停或换向。

二、整形器采用什么规律

定义：

Δp=p
raw
	​

−p
last
	​


其中：

p
raw
	​

：PID经过机构查表后得到的原始目标脉冲；
p
last
	​

：上一次真正发布给步进电机的目标脉冲；
∣Δp∣：本次杆角需要改变的大小。

归一化：

r=clip(
D
1
	​

−D
0
	​

∣Δp∣−D
0
	​

	​

,0,1)

其中：

D
0
	​

=3：3脉冲以内不发布，过滤细小抖动；
D
1
	​

=600：变化达到600脉冲时使用最高速度；
r：本次运动幅度，范围0～1。

采用平滑映射：

S(r)=3r
2
−2r
3

然后计算速度和物理加速度：

v=v
min
	​

+(v
max
	​

−v
min
	​

)S(r)
a=a
min
	​

+(a
max
	​

−a
min
	​

)S(r)

首轮参数：

最小速度：15 RPM
最大速度：80 RPM
最小加速度：180 RPM/s
最大加速度：600 RPM/s

对应驱动器加速度档位大约是：

180 RPM/s → acc=145
600 RPM/s → acc=223
三、修改 ball_control.h

删除：

#define BALL_STEPPER_VELOCITY (50u)
#define BALL_STEPPER_ACCEL_PARAM (0u)

替换为：

/* 步进电机动态速度范围，单位RPM */
#define BALL_STEPPER_VEL_MIN_RPM          (15.0f)
#define BALL_STEPPER_VEL_MAX_RPM          (80.0f)

/* 目标变化≤3脉冲时不重新发布 */
#define BALL_STEPPER_DEADBAND_PULSE       (3u)

/* 目标变化≥600脉冲时使用最高速度和加速度 */
#define BALL_STEPPER_FULL_SPEED_PULSE     (600u)

/* 物理加速度范围，单位RPM/s */
#define BALL_STEPPER_ACCEL_MIN_RPM_S      (180.0f)
#define BALL_STEPPER_ACCEL_MAX_RPM_S      (600.0f)

/* 最短命令发布间隔，防止Task7每5ms重规划一次 */
#define BALL_STEPPER_MIN_PUBLISH_MS       (20u)

/* 大幅突变允许立即发布 */
#define BALL_STEPPER_URGENT_PULSE         (300u)

/* 换向保护 */
#define BALL_STEPPER_REVERSE_MIN_PULSE    (30u)
#define BALL_STEPPER_REVERSE_GUARD_COUNT  (3u)
#define BALL_STEPPER_REVERSE_MAX_RPM      (30.0f)
#define BALL_STEPPER_REVERSE_ACCEL_RPM_S  (180.0f)
四、把整形器状态挂进状态树

在 status.h 的 BALL_CONTROL 前加入：

typedef struct {
  int32_t last_target_pulse;    /* 上次真正发布的绝对目标脉冲 */
  uint32_t last_publish_ms;     /* 上次发布时间 */

  uint16_t velocity_rpm;        /* 当前下发速度 */
  uint8_t accel_param;          /* 当前下发加速度档位 */

  int8_t last_direction;        /* -1减小脉冲，+1增大脉冲 */
  uint8_t reverse_guard;        /* 换向保护剩余更新次数 */

  float move_ratio;             /* 0～1，调试使用 */
} BALL_STEPPER_PROFILE;

然后在 BALL_CONTROL 中加入：

typedef struct {
  BALL_CONTROL_REQUEST request;
  BALL_ESTIMATOR estimator;

  uint32_t consumed_request_seq;
  uint32_t consumed_session_seq;

  float kp;
  float kd;
  float ki;

  float position_error_mm;
  float requested_accel_mm_s2;

  int32_t relative_target_pulse;
  int32_t absolute_target_pulse;

  float stuck_timer_s;
  float integral_accel_mm_s2;
  float hold_timer_s;
  uint8_t hold_active;

  /* 新增 */
  float consumed_target_mm;
  BALL_STEPPER_PROFILE stepper_profile;
} BALL_CONTROL;

consumed_target_mm用于检测Task3从 +50 mm切换到 -50 mm，切换时清除旧积分，但不清除速度估计。

五、在 ball_control.c 加入整形代码

放在 ball_find_relative_pulse() 后面：

typedef struct {
  uint8_t valid;
  int32_t target_pulse;
  uint16_t velocity_rpm;
  uint8_t accel_param;
  int8_t direction;
  uint8_t reverse_guard_after;
  float move_ratio;
} BALL_STEPPER_COMMAND;

static float ball_clampf(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

/*
 * 将物理加速度RPM/s换成Emm42的acc档位。
 * acc=0不能使用，因为0表示关闭曲线加减速。
 */
static uint8_t ball_stepper_accel_to_param(float accel_rpm_s) {
  float param;

  if (accel_rpm_s < 79.0f) {
    accel_rpm_s = 79.0f;
  }

  param = 256.0f - 20000.0f / accel_rpm_s;
  param = ball_clampf(param, 1.0f, 255.0f);

  return (uint8_t)(param + 0.5f);
}

static BALL_STEPPER_COMMAND ball_stepper_prepare_command(
    const BALL_CONTROL *ball,
    int32_t raw_target_pulse,
    uint32_t now_ms) {
  BALL_STEPPER_COMMAND cmd = {0};
  const BALL_STEPPER_PROFILE *profile = &ball->stepper_profile;

  int64_t delta64 =
      (int64_t)raw_target_pulse -
      (int64_t)profile->last_target_pulse;

  uint32_t distance =
      delta64 < 0 ? (uint32_t)(-delta64)
                  : (uint32_t)delta64;

  int8_t direction;
  uint8_t reversing;
  uint32_t elapsed_ms;
  float ratio;
  float smooth_ratio;
  float velocity_rpm;
  float accel_rpm_s;
  uint8_t guard;

  if (distance <= BALL_STEPPER_DEADBAND_PULSE) {
    return cmd;
  }

  direction = delta64 > 0 ? 1 : -1;

  reversing =
      profile->last_direction != 0 &&
      direction != profile->last_direction &&
      distance >= BALL_STEPPER_REVERSE_MIN_PULSE;

  elapsed_ms = now_ms - profile->last_publish_ms;

  /*
   * 普通小变化最多50Hz发布。
   * 大变化或换向立即响应。
   */
  if (elapsed_ms < BALL_STEPPER_MIN_PUBLISH_MS &&
      distance < BALL_STEPPER_URGENT_PULSE &&
      !reversing) {
    return cmd;
  }

  ratio =
      ((float)distance -
       (float)BALL_STEPPER_DEADBAND_PULSE) /
      ((float)BALL_STEPPER_FULL_SPEED_PULSE -
       (float)BALL_STEPPER_DEADBAND_PULSE);

  ratio = ball_clampf(ratio, 0.0f, 1.0f);

  /* 三次平滑阶跃：两端斜率都为0 */
  smooth_ratio =
      ratio * ratio * (3.0f - 2.0f * ratio);

  velocity_rpm =
      BALL_STEPPER_VEL_MIN_RPM +
      (BALL_STEPPER_VEL_MAX_RPM -
       BALL_STEPPER_VEL_MIN_RPM) *
          smooth_ratio;

  accel_rpm_s =
      BALL_STEPPER_ACCEL_MIN_RPM_S +
      (BALL_STEPPER_ACCEL_MAX_RPM_S -
       BALL_STEPPER_ACCEL_MIN_RPM_S) *
          smooth_ratio;

  guard = profile->reverse_guard;

  if (reversing) {
    guard = BALL_STEPPER_REVERSE_GUARD_COUNT;
  }

  /*
   * 换向后的连续几次命令都降低速度和加速度，
   * 防止刚开始反向就立刻恢复80RPM。
   */
  if (guard > 0u) {
    if (velocity_rpm > BALL_STEPPER_REVERSE_MAX_RPM) {
      velocity_rpm = BALL_STEPPER_REVERSE_MAX_RPM;
    }

    if (accel_rpm_s > BALL_STEPPER_REVERSE_ACCEL_RPM_S) {
      accel_rpm_s = BALL_STEPPER_REVERSE_ACCEL_RPM_S;
    }

    guard--;
  }

  cmd.valid = 1;
  cmd.target_pulse = raw_target_pulse;
  cmd.velocity_rpm =
      (uint16_t)(velocity_rpm + 0.5f);
  cmd.accel_param =
      ball_stepper_accel_to_param(accel_rpm_s);
  cmd.direction = direction;
  cmd.reverse_guard_after = guard;
  cmd.move_ratio = smooth_ratio;

  return cmd;
}

static void ball_stepper_commit_command(
    BALL_CONTROL *ball,
    const BALL_STEPPER_COMMAND *cmd,
    uint32_t now_ms) {
  BALL_STEPPER_PROFILE *profile =
      &ball->stepper_profile;

  profile->last_target_pulse =
      cmd->target_pulse;
  profile->last_publish_ms = now_ms;
  profile->velocity_rpm =
      cmd->velocity_rpm;
  profile->accel_param =
      cmd->accel_param;
  profile->last_direction =
      cmd->direction;
  profile->reverse_guard =
      cmd->reverse_guard_after;
  profile->move_ratio =
      cmd->move_ratio;
}
六、初始化整形器

在 ball_control_init() 最后加入：

status->control.ball.consumed_target_mm = 0.0f;

status->control.ball.stepper_profile.last_target_pulse =
    BALL_STEPPER_ZERO_PULSE;

status->control.ball.stepper_profile.last_publish_ms = 0u;
status->control.ball.stepper_profile.velocity_rpm = 0u;

status->control.ball.stepper_profile.accel_param =
    ball_stepper_accel_to_param(
        BALL_STEPPER_ACCEL_MIN_RPM_S);

status->control.ball.stepper_profile.last_direction = 0;
status->control.ball.stepper_profile.reverse_guard = 0u;
status->control.ball.stepper_profile.move_ratio = 0.0f;

在检测到新控制会话时：

if (request.session_seq != ball->consumed_session_seq) {
  ball->estimator.ready = 0;
  ball->estimator.control_ready = 0;

  ball->stuck_timer_s = 0.0f;
  ball->integral_accel_mm_s2 = 0.0f;
  ball->hold_timer_s = 0.0f;
  ball->hold_active = 0;

  ball->consumed_target_mm = request.target_mm;

  /*
   * 允许新任务第一条命令立即发布。
   * 保留last_target_pulse，避免重新从零点规划。
   */
  ball->stepper_profile.last_publish_ms =
      (uint32_t)status->state.time -
      BALL_STEPPER_MIN_PUBLISH_MS;

  ball->stepper_profile.last_direction = 0;
  ball->stepper_profile.reverse_guard = 0u;

  ball->consumed_session_seq =
      request.session_seq;
}

随后增加目标变化检测：

/*
 * Task3由+50切换到-50时：
 * 清积分和停车状态，但保留位置、速度估计。
 */
if (request.target_mm != ball->consumed_target_mm) {
  ball->stuck_timer_s = 0.0f;
  ball->integral_accel_mm_s2 = 0.0f;
  ball->hold_timer_s = 0.0f;
  ball->hold_active = 0;

  ball->consumed_target_mm =
      request.target_mm;
}
七、替换最终步进命令发布部分

你原来是：

if (!ball->hold_active && ...) {
  stepper_publish_absolute(
      ball->absolute_target_pulse,
      BALL_STEPPER_VELOCITY,
      BALL_STEPPER_ACCEL_PARAM);
}

整个替换为：

{
  uint32_t now_ms =
      (uint32_t)status->state.time;

  BALL_STEPPER_COMMAND cmd =
      ball_stepper_prepare_command(
          ball,
          ball->absolute_target_pulse,
          now_ms);

  if (cmd.valid) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    /*
     * 确保计算期间状态树没有切换任务或目标。
     */
    if (status->control.ball.request.enabled &&
        status->control.ball.request.publish_seq ==
            request.publish_seq &&
        status->control.ball.request.session_seq ==
            request.session_seq) {

      stepper_publish_absolute(
          cmd.target_pulse,
          cmd.velocity_rpm,
          cmd.accel_param);

      ball_stepper_commit_command(
          ball, &cmd, now_ms);
    }

    if (!primask) {
      __enable_irq();
    }
  }
}

注意这里没有：

!ball->hold_active

因为进入死区以后仍要发布一次“期望小球加速度为0”对应的杆角。

在查表前确保你的停车逻辑已经执行：

if (ball->hold_active) {
  ball->stuck_timer_s = 0.0f;
  ball->integral_accel_mm_s2 = 0.0f;
  ball->requested_accel_mm_s2 = 0.0f;
}
八、Task3状态树必须避免运动中突然反向

你现在只要：

err <= 5.0f

就立即把目标由 +50切成 -50。这可能发生在球高速经过 +50 时，步进电机必然突然换向。

推荐改成等待停车死区：

case Q3_WAIT_POS5:
  if (status->control.ball.estimator.control_ready &&
      status->control.ball.hold_active) {
    q3_state = Q3_GO_NEG5;
  }
  break;

负方向同理：

case Q3_WAIT_NEG5:
  if (status->control.ball.estimator.control_ready &&
      status->control.ball.hold_active) {
    q3_state = Q3_DONE;
  }
  break;

这样流程是：

到达+50附近
→ 速度降下来
→ 死区确认
→ 清空旧积分
→ 切换到-50
→ 换向保护连续生效3次

如果任务要求必须进入±5 mm，那么 BALL_DEAD_ENTER_ERROR_MM也应设为5 mm；否则状态树会按更宽的死区判断到达。

九、uart_it.c只建议补一处

在 stepper_publish_absolute() 检测到新命令时加入：

status.stepper.reached = 0;

即：

if (!target->enabled ||
    target->absolute_pulse != absolute_pulse ||
    target->velocity != vel ||
    target->accel_param != acc) {

  status.stepper.reached = 0;

  target->enabled = 1;
  target->absolute_pulse = absolute_pulse;
  target->velocity = vel;
  target->accel_param = acc;

  __DMB();
  target->publish_seq++;
}

stepper_service()和主循环不需要改。你现在：

ball_control_service(&status);
stepper_service();

这个顺序是正确的。

另外，Task5中直接调用：

Emm_V5_Pos_Control(...)

会绕过目标队列和状态记录。建议统一改用：

stepper_publish_absolute(...)

否则以后从Task5切换到球控制任务时，整形器不知道Task5把步进电机移到了哪里。




