# 任务2倒立摆控制方案记录

本文只记录方案和需要澄清的重点，暂时不直接改代码。

## 当前目标

任务2对应题目中的基本要求：

小车初始停在 A 点，一键启动小车起摆，摆锤在平衡位置前后摆动并维持平衡，持续大于 10s，期间倒立摆不能碰支架。

发挥要求还需要把小车相对 A 点的位置偏差压到 10cm 以内。

## 核心控制公式

目标控制量暂定为 `base_speed`：

```c
base_speed = balance_out + mileage_out;
```

展开后是：

```c
base_speed = K_angle * pend_angle
           + K_gyro  * pend_angle_speed
           + K_pos   * car_position
           + K_vel   * car_speed;
```

但符号不能死认这行文字，必须按实车坐标验证。控制效果应该满足：

```text
摆往哪边倒，车就往哪边追。
车离 A 点偏到哪边，位置项就把车往反方向拉。
车往哪边运动，速度项就给反向阻尼。
```

## 结构设计

计划在状态层增加两个控制结构：

1. `balance`
   - 负责摆角闭环。
   - `error` 使用 `0 - roll` 或 `roll - 0`，具体符号由实测决定。
   - 角速度项不使用 `compute_pid()` 内部差分微分，初始化时把 `kd` 设为 0。
   - `roll_speed` 作为外部测得的角速度项，按单独系数叠加到输出里。
   - 第一版只用它输出公式前两项：`K_angle * pend_angle + K_gyro * pend_angle_speed`。

2. `mileage`
   - 负责位置约束。
   - `error` 使用负的有符号编码器累计位移。
   - 速度项不使用 `compute_pid()` 内部差分微分，初始化时把 `kd` 设为 0。
   - 两轮平均速度作为外部测得的速度项，后续按单独系数叠加到输出里。
   - 它输出公式后两项：`K_pos * car_position + K_vel * car_speed`。
   - 当前阶段不接入，也不实现里程累加；在哪里累加、挂在哪个结构体里，后续再决定。

工程实现上可以先理解为两个 PID 输出相加：

```c
base_speed = balance_pid_out + mileage_pid_out;
```

但实际调试顺序必须先单独验证 `balance`，再小权重叠加 `mileage`。

## 第一版只做 balance

第一版新增 `BALANCE` 运动模式。

`driver_task2()` 只做：

```c
status->task.task_running = 1;
status->state.motion = BALANCE;
```

`update_status()` 中增加：

```c
if (status->state.motion == BALANCE) {
    status->task.stop_cmd = 0;
    keep_balance(status);
}
```

`keep_balance()` 第一版只计算前两项，不管位置：

```c
target = 0;
diff_balance = target - roll;   // 或 roll - target，待实测决定
balance_out = balance PID 输出;
status->state.base_speed = balance_out;
status->motor.wheel[0].tar_speed = status->state.base_speed;
status->motor.wheel[1].tar_speed = status->state.base_speed;
```

先不要加 `mileage_out`。原因是位置环可能和摆角环打架，导致还没调稳摆就被位置约束拉倒。

## 姿态数据打印

在主循环或周期调试输出中，先按下面格式打印：

```text
pitch,roll,pitch角速度,roll角速度\r\n
```

目的：

1. 确认 GY901 的 pitch / roll 哪个对应摆杆实际运动方向。
2. 确认角度正负号。
3. 确认角速度正负号。
4. 确认角速度量纲是否正常。

当前代码里 `get_gyr_value()` 的角速度换算需要重点核对。注释说角速度映射到 +/-2000 deg/s，但代码现在是：

```c
return value / 2000;
```

这很可能量纲不对。常见写法更接近：

```c
raw * 2000 / 32768
```

正式调 PID 前必须先看串口数据，不然 `K_gyro` 没法调。

## 有符号里程

当前决定：暂时不要加入有符号 `mileage` 的累加逻辑。这里仅保留后续设计注意点，不能作为本轮实现要求。

任务2位置控制不能用当前 `phase_mileage` 的绝对值累计方式。

当前 `phase_mileage` 类似：

```c
abs(left_speed) 和 abs(right_speed) 的平均值累加
```

它表示走过的总路程，不表示相对 A 点的前后偏移。

后续任务2需要新增或复用一个有符号位移，但具体在哪里累加暂不决定：

```c
car_position += (wheel0.cur_speed + wheel1.cur_speed) / 2;
```

清零位置也暂不定，可能在 `task_start()`，也可能在任务2进入平衡阶段时单独清零。

后续要把它换算成 cm：

```c
car_position_cm = encoder_pulse_to_cm(car_position);
```

只有这个量才能用来判断是否离 A 点 10cm 以内。

## 调参顺序

1. 只看姿态打印
   - 手动转动摆杆，确认 roll/pitch 和角速度方向。

2. 只开 `balance`
   - `mileage_out = 0`。
   - 先确认摆向前倒时车向前追，摆向后倒时车向后追。
   - 符号错了先改符号，不要靠负参数硬凑。

3. 调 `balance` 的 P 和 D
   - P 太小接不住摆。
   - P 太大容易冲来冲去。
   - D 是阻尼，太小会振，太大会迟钝或抖。
   - I 暂时不要开。

4. 再接入很小的 `mileage`
   - 这是后续阶段，不是当前实现内容。
   - 位置项输出必须限幅。
   - 先让它轻轻拉回 A 点，不要让它抢过摆角控制优先级。

## 需要澄清的重点

1. GY901 安装方向
   - 摆杆实际前后摆动对应 `roll` 还是 `pitch`？
   - `roll > 0` 时，摆是向车头方向倒还是向车尾方向倒？

2. 角速度轴
   - 与摆角对应的角速度是 `gyr_w_x` 还是 `gyr_w_y`？
   - 正方向是否和角度正方向一致？

3. 电机速度正方向
   - `base_speed > 0` 时，小车实际向 A->B 方向还是反方向？

4. 编码器方向
   - `(wheel0.cur_speed + wheel1.cur_speed) / 2 > 0` 是否和小车前进方向一致？
   - 如果左右轮编码器方向不一致，需要先统一符号。

5. PID 内部微分项处理方式
   - `compute_pid()` 的 D 项来自误差差分，但这次不需要用它。
   - 初始化 `balance` 和 `mileage` 两个 PID 结构体时，直接把 `kd = 0`。
   - 角速度项和车速项来自传感器/编码器实时测量，后续作为外部项按系数叠加即可，不需要重写 PID 函数。

6. 10cm 约束不是第一版目标
   - 第一版目标是能追摆、不碰支架、输出方向正确。
   - 10cm 内需要有符号里程和 `mileage` 位置环参与后再调；但里程在哪里累加暂时不定。

## 成功标准

第一阶段成功：

```text
姿态打印正常；
摆向某方向倒，车向同方向追；
BALANCE 模式下轮子输出连续、方向正确；
不接入位置环时，摆角控制输出不会明显反向。
```

第二阶段成功：

```text
加入小权重 mileage 后，小车不会越漂越远；
摆稳定时，位置能慢慢回到 A 点附近；
位置偏差逐步压到 20cm，再尝试压到 10cm。
```
