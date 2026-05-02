# 速度内环前馈改造提示词

本文是给 AI agent 的代码修改提示词。目标是在现有 TASK 工程架构下，给直流电机速度内环加入可调前馈，让我可以通过主循环串口打印和 `Cy/Cz/Ch` 一类串口命令调出前馈参数。

只做速度内环前馈，不要重写 TASK 架构，不要改循迹环和角度环的控制结构。

# 当前问题

现在速度内环完全依赖 PID 输出：

```c
wheel->trust = compute_pid(&wheel->wheel_pid, wheel->tar_speed - wheel->cur_speed);
```

这会导致：

```text
1. PID 既要提供基础油门，又要修正误差。
2. 积分项经常打满，说明积分在冒充基础 PWM。
3. base_speed=40 时还能接受，base_speed=60 时误差明显变大。
4. 循迹外环会因为内环速度跟不上/抖动而变得很抖。
```

我要加的前馈不是复杂模型，只要能实车调参即可。

# 改造目标

把速度内环从：

```text
trust = PID(error)
```

改成：

```text
trust = feedforward(tar_speed) + PID(error)
```

其中：

```text
error = tar_speed - cur_speed
```

前馈公式先用最简单的线性模型：

```text
ff = sign(tar_speed) * (dead_pwm + kff * abs(tar_speed))
```

语义：

```text
dead_pwm:
  克服静摩擦/电机死区所需的基础 PWM。

kff:
  目标速度每增加 1 个编码器脉冲，对应需要增加多少 PWM。
```

当 `tar_speed == 0` 时，前馈必须为 0。

# 必须遵守的工程边界

1. 不要修改 `gw_analogue.c`、TASK 状态机、按钮逻辑。
2. 不要修改 `compute_pid()` 的参数形式。
3. 不要修改 `get_wheel_speed()` 的编码器读数逻辑。
4. 不要改变 `stop_cmd` 的硬停止语义。
5. `stop_cmd == 1` 时仍然必须直接 PWM=0，并且不要调用 `compute_pid()`。
6. 先实现最小可调前馈，不要引入复杂曲线、查表、自动辨识。

# 建议修改文件

主要改这些文件：

```text
User/Motor/wheel.h
User/Motor/wheel.c
Core/Src/main.c
```

如果需要声明外部全局变量，可以按现有代码风格处理，但不要大范围重构。

# WHEEL 结构体建议新增字段

在 `User/Motor/wheel.h` 的 `WHEEL` 结构体中增加：

```c
int16_t feedforward;
float dead_pwm;
float kff;
```

含义：

```text
feedforward:
  当前周期计算出的前馈 PWM，方便 VOFA 打印观察。

dead_pwm:
  可调死区/基础 PWM。

kff:
  可调速度线性系数。
```

如果担心每个轮子参数不同，可以让每个 `WHEEL` 自己保存 `dead_pwm/kff`。不要先做全局统一参数，因为左右轮可能不同。

# init_wheel 初始化要求

在 `init_wheel()` 中初始化：

```c
wheel->feedforward = 0;
wheel->dead_pwm = 初始值;
wheel->kff = 初始值;
```

建议初始值先保守：

```c
wheel->dead_pwm = 0;
wheel->kff = 0;
```

这样前馈默认关闭，不会破坏当前可跑版本。后续通过串口慢慢调。

# driver_wheel 修改要求

在 `driver_wheel()` 中，保持原来的 `stop_cmd` 分支不变：

```c
if (status.task.stop_cmd) {
  wheel->trust = 0;
  PWM = 0;
  return;
}
```

然后把原来的：

```c
wheel->trust = compute_pid(&wheel->wheel_pid, wheel->tar_speed - wheel->cur_speed);
```

改成类似：

```c
int16_t pid_out = compute_pid(&wheel->wheel_pid, wheel->tar_speed - wheel->cur_speed);
wheel->feedforward = get_wheel_feedforward(wheel);
wheel->trust = wheel->feedforward + pid_out;
```

注意：

```text
1. feedforward 要和 tar_speed 同号。
2. tar_speed == 0 时 feedforward 必须等于 0。
3. 最后仍然使用 TRUST_CONFINE 做总输出限幅。
4. 原来的起步限幅逻辑是否保留可以先保留，但要注意它会限制前馈效果。
```

建议写一个小工具函数，不要把公式直接塞进 driver_wheel：

```c
static int16_t get_wheel_feedforward(WHEEL *wheel)
```

伪代码：

```c
static int16_t get_wheel_feedforward(WHEEL *wheel) {
  int16_t target = wheel->tar_speed;

  if (target == 0) {
    return 0;
  }

  int16_t abs_target = target > 0 ? target : -target;
  float ff = wheel->dead_pwm + wheel->kff * abs_target;

  if (target < 0) {
    ff = -ff;
  }

  return (int16_t)ff;
}
```

# 串口调参命令要求

当前 `main.c` 的 `UART_PID_Tune(cmd, val)` 已经有：

```text
m/o/q -> wheel0 kp/ki/kd
s/u/w -> wheel1 kp/ki/kd
h     -> cmd_speed
y     -> 启动 MOTOR_TEST / TASK start_request
z     -> STOP
```

请新增 4 个命令，用于调左右轮前馈：

```text
b -> wheel0.dead_pwm
n -> wheel0.kff
j -> wheel1.dead_pwm
l -> wheel1.kff
```

命令格式继续沿用现有 `C + cmd + 数值 + 换行` 的解析方式。不要改串口协议框架。

示例：

```text
Cb300
Cn12
Cj300
Cl12
```

表示：

```text
wheel0.dead_pwm = 300
wheel0.kff = 12
wheel1.dead_pwm = 300
wheel1.kff = 12
```

# 主循环 VOFA 打印要求

我会用主循环串口打印来看具体数据，所以必须把前馈相关数据打印出来。

现有主循环已经打印：

```text
follow_line diff/out/kp/ki/kd
wheel0 cur_speed/tar_speed
wheel1 cur_speed/tar_speed
```

请扩展打印内容，至少包括：

```text
wheel0.cur_speed
wheel0.tar_speed
wheel0.trust
wheel0.feedforward
wheel0.wheel_pid.out
wheel0.wheel_pid.integral
wheel0.dead_pwm
wheel0.kff

wheel1.cur_speed
wheel1.tar_speed
wheel1.trust
wheel1.feedforward
wheel1.wheel_pid.out
wheel1.wheel_pid.integral
wheel1.dead_pwm
wheel1.kff

cmd_speed
```

格式必须是 VOFA 友好的 CSV：

```text
数据1,数据2,数据3,...\r\n
```

注意：

```text
printf 的 %.3f 占位符数量必须和参数数量完全一致。
不要再出现参数多于格式符，导致最后几个数据打印不出来的问题。
```

# 调参建议写进代码注释

请在 `wheel.c` 前馈函数附近写清楚注释：

```text
前馈只负责根据目标速度给基础 PWM。
PID 只负责修正 tar_speed - cur_speed 的误差。
如果 feedforward 已经接近目标速度，就降低 Ki，避免积分长期打满。
```

注释要说明“为什么这么做”，不要只复述代码。

# 实车调参流程

实现后，我会按这个流程调：

1. 先把速度 PID 调弱，避免 PID 掩盖前馈效果：

```text
Kp = 0
Ki = 0
Kd = 0
```

2. 设定 `cmd_speed = 40`，调 `dead_pwm/kff`，让实际速度接近 40。

3. 设定 `cmd_speed = 60`，继续调 `kff`，让 60 也接近目标。

4. 如果 40 准、60 偏低，说明 `kff` 小了。

5. 如果 40 准、60 偏高，说明 `kff` 大了。

6. 前馈能大体接近后，再加小 PI：

```text
Kp = 5~15
Ki = 2~8
Kd = 0
```

7. 如果 `wheel_pid.integral` 仍然长期打满，说明前馈还不够，需要继续调 `dead_pwm/kff`。

# 验收标准

完成后应满足：

1. `stop_cmd == 1` 时电机 PWM 仍然为 0。
2. `tar_speed == 0` 时 `feedforward == 0`。
3. `tar_speed > 0` 时前馈为正，`tar_speed < 0` 时前馈为负。
4. `driver_wheel()` 最终输出为 `feedforward + PID`，并经过 `TRUST_CONFINE` 限幅。
5. 可以通过串口命令在线调左右轮 `dead_pwm/kff`。
6. 主循环 VOFA 可以看到左右轮的 `cur_speed/tar_speed/trust/feedforward/pid.out/integral/dead_pwm/kff/cmd_speed`。
7. 不修改 TASK 状态机、循迹环、角度环、传感器层。
