# TASK2 起摆方案：方波起摆 + 小角度 PD 接管

当前判断：TASK2 不能一开始就靠平衡 PD 起摆。摆杆离直立位置太远时，PD 控制器只能“接住”，不能有效给摆注入足够能量。所以 TASK2 进入后先做开环方波前后运动，让小车往返抽动起摆；一旦角度差值进入可控范围，再切到 PD 平衡。

## 另一个评估里指出的问题

那些问题确实存在，尤其是前两个：

1. motion 冲突确实存在。
   - `update_status()` 的顺序是先 `update_task()`，再根据 `status->state.motion` 调对应控制函数。
   - 如果 TASK2 的 `SWING_UP` 阶段把 `motion` 设成 `BALANCE`，后面 `if (motion == BALANCE)` 会调用 `keep_balance()`，把方波起摆写入的 `tar_speed` 覆盖掉。

2. `compute_pid()` 不适合作为第一版 balance PD。
   - `compute_pid()` 的 D 项来自误差差分。
   - 现在更想直接用陀螺仪角速度 `roll_speed` 做 D 项。
   - `status->state.status_pid.balance_pid` 仍然保留，但只当作存放 `kp/kd` 参数的结构体空壳。
   - 所以第一版平衡接管阶段直接手写：

```c
balance_out = kp * angle_error + kd * roll_speed;
```

3. `task2_enter_swing_up()` 的调用时机必须明确。
   - 建议在 `task_start()` 进入 `TASK_BASIC_2` 时调用。
   - 或者在 `driver_task2()` 第一次执行时根据静态标志初始化。
   - 更清楚的做法是在 `task_start()` 里初始化 TASK2 小状态机。

## 加 STRAIGHT Motion

建议给 `MOTION_STATION` 加一个 `STRAIGHT`，专门表示“按当前 `base_speed` 直行，不做循迹、不做保角、不做平衡”。

```c
typedef enum MOTION_STATION {
  STOP,
  KEEP_ANGLE,
  FIND_LINE,
  MOTOR_TEST,
  BALANCE,
  STRAIGHT,
} MOTION_STATION;
```

然后在 `update_status()` 里增加：

```c
if (status->state.motion == STRAIGHT) {
    status->task.stop_cmd = 0;
    status->motor.wheel[0].tar_speed = status->state.base_speed;
    status->motor.wheel[1].tar_speed = status->state.base_speed;
}
```

这样 TASK2 的 `SWING_UP` 阶段只需要：

```c
status->state.motion = STRAIGHT;
status->state.base_speed = swing_speed;
```

后续不会被 `keep_balance()` 覆盖，也不会被 `MOTOR_TEST` 的 `cmd_speed` 覆盖。

## TASK2 状态划分

TASK2 只需要两个状态：

```c
typedef enum {
  TASK2_SWING_UP = 0,
  TASK2_BALANCE_PD,
} TASK2_STATE;
```

需要保存：

```c
static TASK2_STATE task2_state;
static uint32_t task2_last_switch_time;
static int8_t task2_square_dir;
```

不需要 `TASK2_FAIL_STOP`。调试阶段如果摆不起来，就继续方波起摆，不自动停止。

保留 `task2_square_dir`。方波方向用显式状态保存，每到半周期再翻转一次，方便调试和重置。

## 方波起摆参数第一版

```c
#define TASK2_CAPTURE_ANGLE_DEG   12.0f
#define TASK2_SWING_SPEED         50
#define TASK2_SWING_HALF_PERIOD   120   // ms
```

- `TASK2_SWING_SPEED` 是 8ms 速度单位下的目标速度，不是 PWM。
- `TASK2_SWING_HALF_PERIOD` 是方波半周期，每隔这段时间前后换向一次。
- 第一版不加最大次数停止，摆不起来就继续摆。

## 初始化时机

建议在 `task_start()` 中处理 `TASK_BASIC_2`：

```c
case TASK_BASIC_2:
  apply_basic_control_param(status);
  task2_enter_swing_up(status);
  break;
```

`task2_enter_swing_up()`：

```c
static void task2_enter_swing_up(STATUS *status)
{
    task2_state = TASK2_SWING_UP;
    task2_last_switch_time = status->state.time;
    task2_square_dir = -1;
}
```

## TASK2 大体框架

```c
static void driver_task2(STATUS *status)
{
    float roll = get_gyr_value(&status->sensor.gy901, gyr_x_roll);
    float roll_speed = get_gyr_value(&status->sensor.gy901, gyr_w_x);
    float target_roll = 0.0f;
    float angle_error = target_roll - roll;
    float balance_out;
    int16_t swing_speed;
    PID *balance_param = &status->state.status_pid.balance_pid;

    status->task.task_running = 1;
    status->task.stop_cmd = 0;

    switch (task2_state) {
      case TASK2_SWING_UP:
        if (ABS(angle_error) < TASK2_CAPTURE_ANGLE_DEG) {
            task2_state = TASK2_BALANCE_PD;
            break;
        }

        if (status->state.time - task2_last_switch_time >= TASK2_SWING_HALF_PERIOD) {
            task2_last_switch_time = status->state.time;
            task2_square_dir = -task2_square_dir;
        }

        swing_speed = task2_square_dir * TASK2_SWING_SPEED;
        status->state.motion = STRAIGHT;
        status->state.base_speed = swing_speed;
        break;

      case TASK2_BALANCE_PD:
      default:
        balance_out = balance_param->kp * angle_error + balance_param->kd * roll_speed;
        status->state.motion = STRAIGHT;
        status->state.base_speed = (int16_t)balance_out;
        break;
    }
}
```

这里 `TASK2_BALANCE_PD` 也可以把 `motion` 设成 `STRAIGHT`，因为 `driver_task2()` 已经算好了 `base_speed`，只需要运动分发层把 `base_speed` 下发给左右轮。

## 关键点

1. 起摆阶段不能用 `MOTOR_TEST`。
   - `MOTOR_TEST` 会使用 `cmd_speed` 覆盖 TASK2 自己的速度。

2. 起摆阶段也不能用 `BALANCE`。
   - `BALANCE` 会触发 `keep_balance()`，覆盖方波起摆速度。

3. 新增 `STRAIGHT` 可以解决 motion 冲突。
   - `STRAIGHT` 只负责把 `base_speed` 送到左右轮。
   - 方波起摆和 PD 接管都可以通过写 `base_speed` 工作。

4. 第一版 balance PD 不用 `compute_pid()`。
   - 直接使用陀螺仪角速度作为 D 项：
   - `balance_pid` 结构体仍然保留，只负责保存 `kp/kd` 参数。

```c
balance_out = kp * angle_error + kd * roll_speed;
```

5. `task2_square_dir` 保留。
   - 每隔 `TASK2_SWING_HALF_PERIOD` 翻转一次。
   - 重新进入 TASK2 时明确初始化为 `-1`。

6. 如果切入 PD 后经常接不住，再加角速度切换条件，例如：

```c
ABS(angle_error) < 12.0f && ABS(roll_speed) < 某个阈值
```

7. 如果 `TASK2_SWING_SPEED = 50` 还是起不来，优先加大速度或延长半周期，而不是继续提高 balance 的 `kp`。
