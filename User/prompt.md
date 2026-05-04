# Agent 提示词：纯化 keep_angle，并把 TASK2 的 B 点掉头改成停车后原地掉头

请先阅读并遵守 `C:\Users\19355\.codex\skills\karpathy-guidelines\SKILL.md`。

这次只做两个目标：

1. 把 `keep_angle()` 改成纯控制函数，不允许它偷偷修改 `base_speed`。
2. 优化 `TASK_BASIC_2` 的 AB 发车路线中 B 点掉头逻辑：检测到 B 点后先停车，车速降下来后再原地掉头。

不要改题目文档，不要重构无关代码，不要删除旧文件，不要改 `get_road_type()` / `road_new_from_bit()` 的左右路口枚举逻辑。

## 当前问题

### 问题 1：`keep_angle()` 不是纯控制函数

当前 `User/Status/status.c` 里的 `keep_angle()` 内部有类似这样的逻辑：

```c
if (ABS(diff_angle) < 1.0) {
  ...
  status->state.base_speed = 35;
}
```

这非常不适合现在的 TASK 状态机架构。

原因：

- `keep_angle()` 应该只根据 `base_speed` 和角度误差生成左右轮目标速度。
- `base_speed` 应该由 `driver_task1/2/3/4` 的小状态机显式控制。
- 如果 `keep_angle()` 自己修改 `base_speed`，会破坏 TASK 内部的速度策略。
- 对 B 点原地掉头尤其危险：状态机想设置 `base_speed = 0` 原地转，但 `keep_angle()` 可能在接近目标角度时把 `base_speed` 改成 35，导致小车往前窜。
- `cnt` / `flag` 这种全局静态变量会让第一次转弯和后续转弯行为不一致，也不适合当前架构。

### 问题 2：当前 B 点掉头是“带前进速度的弧线掉头”

当前 `Q2_AB_UTURN_B` 阶段大概率是：

```c
status->state.motion = KEEP_ANGLE;
status->state.base_speed = Q2_UTURN_SPEED;
```

而 `keep_angle()` 的输出形式是：

```c
wheel0.tar_speed = base_speed + diff;
wheel1.tar_speed = base_speed - diff;
```

只要 `base_speed != 0`，小车中心就一定有前进分量，所以它不是原地掉头，而是带前进速度的弧线掉头。实车现象是：虽然已经减速，也转了接近 180 度，但角度仍然差约 20 度，并且转完后会往前走一段。

## 修改目标 1：把 keep_angle() 改成纯控制函数

修改 `User/Status/status.c` 中的 `keep_angle()`。

要求：

1. 保留角度误差计算逻辑。
2. 保留 `compute_pid(&status->state.status_pid.keep_angle_pid, diff_angle)`。
3. 保留 `status->state.status_pid.angle_output_limit` 限幅逻辑。
4. 保留左右轮目标速度赋值：

```c
status->motor.wheel[0].tar_speed = status->state.base_speed + (int16_t)diff;
status->motor.wheel[1].tar_speed = status->state.base_speed - (int16_t)diff;
```

5. 删除 `keep_angle()` 内部所有修改 `status->state.base_speed` 的逻辑。
6. 如果 `cnt`、`flag`、`keep_angle_time` 只服务于这段旧逻辑，并且删除后没有其它有效用途，可以清理掉由本次改动造成的无用变量；不要删除无关旧代码。

改完后，`keep_angle()` 的职责必须变成：

```text
输入：tar_angle / initial_angle / cur_angle / base_speed
输出：wheel0.tar_speed / wheel1.tar_speed
副作用：不修改 base_speed，不切换 motion，不切换 race_phase
```

## 修改目标 2：TASK2 的 B 点改成停车后原地掉头

只修改 `TASK_BASIC_2` 的 AB 发车路线，不实现 AD 发车，不改 `driver_task1`。

### 建议新增阶段

在 `User/Status/Defect.h` 的 `Q2_AB_RACE_PHASE` 中，在下面两个阶段之间插入一个停车阶段：

```c
Q2_AB_SIDE_CB_OUT,
Q2_AB_STOP_BEFORE_UTURN_B,
Q2_AB_UTURN_B,
```

语义：

- `Q2_AB_SIDE_CB_OUT`：C -> B 巡线，79cm 后允许识别 B 点。
- `Q2_AB_STOP_BEFORE_UTURN_B`：检测到 B 点后先停车，等待两个轮子的实际速度降到很低。
- `Q2_AB_UTURN_B`：停车完成后，打开 PWM，`base_speed = 0`，原地掉头。

### 进入停车阶段

在 `Q2_AB_SIDE_CB_OUT` 中，如果 `task2_accept_road(status, LeftRoad, Q2_CB_ROAD_ENABLE_CM)` 成功，不要直接进入 `Q2_AB_UTURN_B`。

应该进入：

```c
task2_enter_phase(status, Q2_AB_STOP_BEFORE_UTURN_B);
status->state.motion = STOP;
status->state.base_speed = 0;
status->motor.wheel[0].tar_speed = 0;
status->motor.wheel[1].tar_speed = 0;
```

注意：

- 不要在 `motion = STOP` 的同时手动强行 `stop_cmd = 0`。
- 当前 `update_status()` 的 STOP 分支会把 `stop_cmd = 1`，这是正确的。
- 停车阶段的目的就是让 PWM 关闭，让车真正停住。

### 停车完成判定

在 `Q2_AB_STOP_BEFORE_UTURN_B` 阶段：

1. 保持：

```c
status->state.motion = STOP;
status->state.base_speed = 0;
status->motor.wheel[0].tar_speed = 0;
status->motor.wheel[1].tar_speed = 0;
```

2. 当两个轮子的实际速度都足够低时，再进入原地掉头阶段。

建议阈值先写成宏，方便实车调：

```c
#define Q2_UTURN_STOP_SPEED_THRESHOLD 5
```

判定逻辑类似：

```c
if (ABS(status->motor.wheel[0].cur_speed) <= Q2_UTURN_STOP_SPEED_THRESHOLD &&
    ABS(status->motor.wheel[1].cur_speed) <= Q2_UTURN_STOP_SPEED_THRESHOLD) {
    task2_enter_phase(status, Q2_AB_UTURN_B);
    status->state.initial_angle = status->state.cur_angle;
    status->state.tar_angle = Q2_UTURN_B_ANGLE;
    status->state.motion = KEEP_ANGLE;
    status->state.base_speed = 0;
    status->state.status_pid.angle_output_limit = Q2_UTURN_ANGLE_LIMIT;
}
```

不要用固定时间延时来判定停车完成。

### 原地掉头阶段

在 `Q2_AB_UTURN_B` 阶段：

```c
status->state.motion = KEEP_ANGLE;
status->state.base_speed = 0;
status->state.status_pid.angle_output_limit = Q2_UTURN_ANGLE_LIMIT;
```

这样 `keep_angle()` 会输出：

```text
wheel0.tar_speed = +diff
wheel1.tar_speed = -diff
```

小车才是真正原地转。

建议：

- `Q2_UTURN_ANGLE_LIMIT` 不要完全无限放开，先用 80 或 100。
- 如果之前是 60，可以先提高到 80 或 100。
- 不建议直接无限大，避免轮速目标过大、内环前馈过猛、掉头过冲、TB6612 发热。

### 掉头完成判定

当前如果 `Q2_UTURN_TOLERANCE_DEG` 是 15，可能太松。目标 180 度时，误差小于 15 就退出，理论上 165 度就能切阶段。

建议：

```c
#define Q2_UTURN_TOLERANCE_DEG 5.0f
```

或者先设为 8.0f 实车测试。

进入 `Q2_AB_FIND_BC_RETURN` 后：

```c
status->state.motion = FIND_LINE;
status->state.base_speed = Q2_FINAL_SLOW_SPEED;  // 例如 20
```

在 `Q2_AB_SIDE_BC_RETURN` 中恢复正常角度限幅：

```c
status->state.status_pid.angle_output_limit = Q2_NORMAL_ANGLE_LIMIT;
```

## 重要调用链约束

当前 `update_status()` 的运动分支大致是：

```c
if (motion == FIND_LINE) {
  stop_cmd = 0;
  follow_line(status);
}

if (motion == KEEP_ANGLE) {
  stop_cmd = 0;
  keep_angle(status);
}

if (motion == STOP) {
  stop_cmd = 1;
  wheel target = 0;
}
```

所以：

- 停车阶段必须用 `motion = STOP`。
- 原地掉头阶段必须切回 `motion = KEEP_ANGLE`，这样 PWM 才会重新打开。
- 不要在 STOP 阶段手动打开 PWM。
- 不要让按钮层、蓝牙层直接参与这个 B 点掉头流程。

## 验收标准

完成后请说明：

1. `keep_angle()` 是否已经不会修改 `base_speed`。
2. 删除或保留了哪些与旧 `keep_angle()` 加速逻辑相关的变量，理由是什么。
3. `Q2_AB_SIDE_CB_OUT` 检测到 B 点后是否先进入停车阶段，而不是直接掉头。
4. 停车阶段是否依靠 `cur_speed` 判定停车完成，而不是用时间延时。
5. `Q2_AB_UTURN_B` 阶段是否 `base_speed = 0`。
6. 原地掉头时 `angle_output_limit` 的值是多少，何时恢复正常。
7. 没有修改 `driver_task1`，没有实现 AD 发车，没有改路口左右枚举逻辑。
