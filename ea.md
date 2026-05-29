# 8ms 控制周期修改备忘

本文记录如果把控制响应从 20ms 改到 8ms，需要一起检查和修改的地方。

当前建议：不要直接把 TIM5 硬件周期改成 8ms。保持 TIM5 每 1ms 进一次中断，让 `status.state.time` 继续表示毫秒，只把 `update_status()` 的触发周期从 20ms 改成 8ms。

## 必改点

### 1. update_status 调度周期

文件：`User/It/timer_it.c`

当前逻辑：

```c
if (status.state.time % 20 == 0) {
    update_status(&status);
}
```

改成：

```c
if (status.state.time % 8 == 0) {
    update_status(&status);
}
```

这是控制周期真正变快的核心。

### 2. PID 采样周期 T

所有原来 `init_pid(..., 20, ...)` 的 PID，如果它跟着 `update_status()` 每周期计算，都要把 `T` 从 20 改成 8。

重点位置：

- `User/Status/status.c`
  - `init_status_pid()`
  - `apply_basic_control_param()`
  - `apply_adv_control_param()`
- `User/Motor/wheel.c`
  - `init_wheel()` 里的默认轮速 PID

原因：`compute_pid()` 内部积分和微分都使用 `pid->T`。周期改了但 T 不改，会导致积分/微分量纲错误。

### 3. 长按计数

文件：`User/Device/button.h`

当前：

```c
#define LONG_PRESS_CNT 50
```

20ms 周期下：

```text
50 * 20ms = 1000ms
```

8ms 周期下如果仍然是 50：

```text
50 * 8ms = 400ms
```

如果还想保持约 1 秒长按，应改为：

```c
#define LONG_PRESS_CNT 125
```

## 需要重标或换算的点

### 4. 速度目标量纲

`cur_speed` 和 `tar_speed` 本质上是“单个控制周期内的编码器脉冲数”。

从 20ms 改成 8ms 后，同样物理速度下，每周期脉冲数约变为：

```text
8 / 20 = 0.4
```

所以如果想保持原来的实际车速，原来的速度目标大约要乘 0.4。

涉及：

- `cmd_speed`
- `status.state.base_speed`
- 任务里的固定速度值
- `keep_balance()` 输出到 `base_speed` 后的限幅/调参尺度

### 5. 轮子前馈参数

文件：`User/Motor/wheel.c`

当前前馈形式：

```c
ff_abs = offset + k * ABS(wheel->tar_speed);
```

周期改成 8ms 后，同样物理速度下 `tar_speed` 数值变小。如果不重标，前馈会不匹配。

临时理解：

```text
tar_speed_new ~= tar_speed_old * 0.4
k_new 可能需要约等于 k_old * 2.5
```

但前馈最好实车重新标定，不要只靠比例换算。

### 6. 轮速阈值

这些阈值也是“每周期脉冲数”，8ms 后同物理速度下数值会变小。

需要检查：

- `User/Status/status.c`
  - `ABS(cur_speed) < 5` 这类停车/路况更新判断
- `User/Motor/wheel.c`
  - `wheel->tar_speed == 0 && ABS(wheel->cur_speed) < 3`
  - `ABS(wheel->cur_speed) < 10 && status.state.motion != KEEP_ANGLE`

这些阈值如果想对应同样物理速度，也要大约乘 0.4，之后实车微调。

## 不一定要改的点

### 7. phase_mileage 距离累计

`phase_mileage` 现在是累加每次读取到的编码器脉冲。

改成 8ms 后，每次读到的脉冲少了，但读取次数多了。只要每次读取后仍然清编码器计数，总累计脉冲理论上仍然等于实际走过距离。

所以距离累计本身不用乘 0.4。

但是注释里如果写“每 20ms 累加”，应改成“每控制周期累加”。

### 8. PERIODIC_START 调试打印

`PERIODIC_START(NAME, T)` 用的是 `HAL_GetTick()`，单位是毫秒，不依赖 `update_status()` 周期。

所以主循环里：

```c
PERIODIC_START(Balance_Debug_Print, 100)
```

仍然是 100ms 打印一次，不需要因为控制周期改成 8ms 而改变。

## 需要风险验证

### 9. update_status 是否能在 8ms 内跑完

`update_status()` 里包含：

- 8 路模拟灰度 ADC 读取
- 4 路编码器读取
- GY901 I2C 读取
- 按键扫描
- `update_task()`
- motion 控制
- LED / 舵机 / 蜂鸣器 / 电机驱动

尤其要注意：

```c
get_gyr_raw_data(&hi2c1, &status->sensor.gy901);
```

它内部是阻塞 I2C 读取，timeout 当前是 10ms。正常成功时不会等满 10ms，但如果 I2C 异常，8ms 控制周期会被拖垮。

建议验证：

```text
GPIO 翻转测 update_status 耗时；
或者临时打印/示波器看 8ms 周期是否稳定。
```

## 建议修改顺序

1. 只把 `timer_it.c` 中 `update_status()` 触发从 `% 20` 改成 `% 8`。
2. 把跟随 `update_status()` 的 PID 初始化周期 `T` 从 20 改为 8。
3. 把 `LONG_PRESS_CNT` 从 50 改为 125。
4. 速度目标先按 0.4 缩放，低速验证。
5. 检查轮速阈值，先按 0.4 缩放再实车微调。
6. 前馈参数不要一次性相信比例换算，实车重新标定。
7. 验证 `update_status()` 在 8ms 内稳定跑完。

## 不推荐的做法

不要第一步就把 TIM5 的硬件周期改成 8ms。

原因：现在 `status.state.time += status.state.T`，而 `init_status(&status, 1)` 让 `status.state.time` 按 1ms 语义走。直接改 TIM5 硬件周期会让很多基于 `status.state.time` 的 ms 逻辑变错，比如蜂鸣器 `off_time`、调试时间判断等。
