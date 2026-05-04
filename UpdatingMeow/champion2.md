# TASK2 AD 发车状态机实现提示词

请先阅读并遵守 `C:\Users\19355\.codex\skills\karpathy-guidelines\SKILL.md`。

这次目标：在现有 `TASK_BASIC_2` 的 AB 发车状态机基础上，补齐 **AD 发车**逻辑。架构、难点处理、工具函数风格都要尽量仿照现有 AB 发车，不要另起一套复杂体系。

只实现第二问 AD 发车，不要改第一问、第三问、第四问，不要改 `get_road_type()` / `road_new_from_bit()` 的左右路口枚举逻辑。

## 一、任务难点和处理手段

### 1. AD 发车路线与 AB 发车不同

题目规定：

```text
AB 发车：逆时针行驶 3/4 圈至 B 点，掉头原路返回
AD 发车：顺时针行驶 3/4 圈至 D 点，掉头原路返回
```

因此 AD 发车路线应为：

```text
A -> B -> C -> D -> 掉头 -> C -> B -> A
```

去程主要是右转，回程主要是左转。不要误用 AB 的 `A -> D -> C -> B` 路线。

### 2. BC 边有干扰 A4 纸

基础第二问的干扰 A4 纸固定在 `BC` 边中部。AD 发车会两次经过 BC 边：

```text
去程：B -> C
回程：C -> B
```

处理方法必须与 AB 发车一致：

- 进入 `B -> C` 直行边后，阶段里程小于 `Q2_BC_ROAD_ENABLE_CM` 时，不允许消费任何路口。
- 进入 `C -> B` 直行边后，阶段里程小于 `Q2_CB_ROAD_ENABLE_CM` 时，不允许消费任何路口。
- 这两个阈值先沿用 AB 发车的 79cm 级别，后续实车调。
- 只能用编码器里程过滤，不要用时间延时。

### 3. 掉头必须单独处理

AD 发车是在 D 点掉头，不是 B 点。不要把 D 点当普通 90 度路口处理。

推荐结构：

```text
SIDE_CD_OUT -> STOP_BEFORE_UTURN_D -> UTURN_D -> FIND_DC_RETURN -> SIDE_DC_RETURN
```

处理方法：

1. 到 D 点前可以提前减速。
2. 检测到 D 点后先进入 `STOP`，等待两个轮子的实际速度降到阈值以下。
3. 停稳后切回 `KEEP_ANGLE`，重新打开 PWM。
4. 原地掉头时 `base_speed = 0`，只靠角度环差速转向。
5. 掉头阶段临时放大 `angle_output_limit`，例如 80 或 100。
6. 找到回程线并稳定进入回程直行边后，恢复正常角度限幅。

注意：不要在 `motion = STOP` 的同时强行 `stop_cmd = 0`。当前 `update_status()` 的 STOP 分支会把 `stop_cmd = 1`，这是正确的。原地掉头阶段必须切回 `KEEP_ANGLE`，这样 `update_status()` 才会重新打开 PWM。

### 4. `keep_angle()` 必须是纯控制函数

当前架构要求速度由 `driver_task2()` 的小状态机控制。

确认 `keep_angle()` 不再偷偷修改：

```c
status->state.base_speed
status->state.motion
status->task.race_phase
```

`keep_angle()` 只应该做：

```text
角度误差 -> PID -> angle_output_limit 限幅 -> 左右轮 tar_speed
```

这样 AD 发车的原地掉头才能可靠使用 `base_speed = 0`。

### 5. 每个转弯角度都封装成宏

不要把角度写死在状态机里。

AD 发车至少需要这些可调宏：

```c
#define Q2_AD_TURN_A_RIGHT_ANGLE
#define Q2_AD_TURN_B_RIGHT_ANGLE
#define Q2_AD_TURN_C_RIGHT_ANGLE

#define Q2_AD_UTURN_D_ANGLE

#define Q2_AD_TURN_C_LEFT_ANGLE
#define Q2_AD_TURN_B_LEFT_ANGLE
```

初值可以先参考 AB：

- 普通右转：`-80.0f`
- 普通左转：`80.0f`
- D 点掉头：先用 `-170.0f` 或 `170.0f`，按实车方向调

重要注意：如果当前角度误差函数会把误差限制到 `[-180, 180]`，不要天真地把掉头角写成 `190` 或 `-190`，因为它可能会被 wrap 成反方向的较短角度。若实车需要超过 180 度，优先通过实测调整方向和容差，必要时再设计“两段掉头”，不要直接硬塞超过 180 的目标角。

### 6. 路口消费要保持纯洁性

沿用 AB 发车的思路：

- `task2_accept_road(status, expected, min_cm)` 负责消费路口。
- `min_cm` 负责里程过滤。
- `cnt_seen` 防止同一个路口重复消费。
- 转弯后找线必须稳定后再允许释放 `cnt_seen`。
- 普通边也可以保留小的最小里程阈值，避免刚出路口就误消费旧路口。

### 7. 转弯后找线仍然三段式

每个普通转弯都按：

```text
TURN_xxx -> FIND_xxx -> SIDE_xxx
```

规则：

- `TURN_xxx`：`motion = KEEP_ANGLE`
- 角度误差进入 `Q2_TURN_FIND_TOLERANCE_DEG` 后切到 `FIND_xxx`
- `FIND_xxx`：低速找线
- 中间 6 路看到线后进入 `SIDE_xxx`
- 中间 4 路连续稳定 `Q2_LINE_STABLE_CNT` 次后，才允许正常巡线速度策略

### 8. 最后一段停车

AD 发车返回时最后一段是：

```text
B -> A
```

在最后一段 `B -> A` 中，超过 `Q2_FINAL_SLOW_CM` 后降速到 `Q2_FINAL_SLOW_SPEED`，检测到 A 点后 `task_finish(status)`。

结束必须进入：

```c
status->state.motion = STOP;
status->task.stop_cmd = 1;
status->task.task_running = 0;
status->task.armed = 0;
```

## 二、建议新增 AD 发车阶段枚举

在 `User/Status/Defect.h` 中，保留现有 AB 阶段，新增 AD 阶段。不要复用 AB 的阶段名，否则调试时会乱。

建议命名：

```c
Q2_AD_TURN_A_TO_AB,
Q2_AD_FIND_AB_OUT,
Q2_AD_SIDE_AB_OUT,

Q2_AD_TURN_B_TO_BC,
Q2_AD_FIND_BC_OUT,
Q2_AD_SIDE_BC_OUT,

Q2_AD_TURN_C_TO_CD,
Q2_AD_FIND_CD_OUT,
Q2_AD_SIDE_CD_OUT,

Q2_AD_STOP_BEFORE_UTURN_D,
Q2_AD_UTURN_D,
Q2_AD_FIND_DC_RETURN,
Q2_AD_SIDE_DC_RETURN,

Q2_AD_TURN_C_RETURN,
Q2_AD_FIND_CB_RETURN,
Q2_AD_SIDE_CB_RETURN,

Q2_AD_TURN_B_RETURN,
Q2_AD_FIND_BA_RETURN,
Q2_AD_SIDE_BA_RETURN,

Q2_AD_FINISH,
```

阶段含义：

- `OUT` 表示去程。
- `RETURN` 表示掉头后的回程。
- `STOP_BEFORE_UTURN_D` 是 D 点掉头前停车阶段。
- `UTURN_D` 是 D 点原地掉头阶段。

## 三、建议新增或复用的宏

优先复用已有通用宏：

```c
Q2_TURN_FIND_TOLERANCE_DEG
Q2_UTURN_TOLERANCE_DEG
Q2_TURN_LINE_MASK_6
Q2_TURN_LINE_MASK_4
Q2_LINE_STABLE_CNT
Q2_FLASH_SPEED
Q2_CRUISE_SPEED
Q2_TURN_SPEED
Q2_FINAL_SLOW_SPEED
Q2_STRAIGHT_FLASH_CM
Q2_CB_ROAD_ENABLE_CM
Q2_BC_ROAD_ENABLE_CM
Q2_FINAL_SLOW_CM
Q2_UTURN_PRE_SLOW_CM
Q2_UTURN_PRE_SLOW_SPEED
Q2_UTURN_ANGLE_LIMIT
Q2_NORMAL_ANGLE_LIMIT
Q2_UTURN_STOP_SPEED_THRESHOLD
```

AD 专用角度宏建议新增：

```c
#define Q2_AD_TURN_A_RIGHT_ANGLE       -80.0f
#define Q2_AD_TURN_B_RIGHT_ANGLE       -80.0f
#define Q2_AD_TURN_C_RIGHT_ANGLE       -80.0f

#define Q2_AD_UTURN_D_ANGLE            -170.0f

#define Q2_AD_TURN_C_LEFT_ANGLE         80.0f
#define Q2_AD_TURN_B_LEFT_ANGLE         80.0f
```

如果实车发现左右转角不同，只改宏，不要改状态机结构。

## 四、task_start 接入

`task_start()` 中 `TASK_BASIC_2` 分支应根据 `start_pose` 选择 AB 或 AD。

AB 保持现有逻辑不动。

AD 发车新增逻辑：

```c
if (status->task.start_pose == START_AD) {
    status->task.race_phase = Q2_AD_TURN_A_TO_AB;
    status->state.initial_angle = status->state.cur_angle;
    status->state.tar_angle = Q2_AD_TURN_A_RIGHT_ANGLE;
    status->state.motion = KEEP_ANGLE;
    status->state.base_speed = Q2_TURN_SPEED;
    status->task.task_running = 1;
}
```

启动时仍要沿用已有清理逻辑：

- 清 `task.cross_cnt`
- 清 `cnt_seen`
- 清全局 `cross_cnt / left_cnt / cross_delay`
- 清灰度路口缓存
- 清 `phase_mileage`
- 应用 BASIC 控制参数
- 清左右轮 `trust`

不要让 `START_AD` 保持 `armed = 1` 但 `task_running = 0` 的半启动状态。

## 五、AD 发车完整状态机架构

路线：

```text
A -> B -> C -> D -> 掉头 -> C -> B -> A
```

### 1. A 点起步转入 AB

```text
Q2_AD_TURN_A_TO_AB -> Q2_AD_FIND_AB_OUT -> Q2_AD_SIDE_AB_OUT
```

`Q2_AD_TURN_A_TO_AB`：

- `motion = KEEP_ANGLE`
- `base_speed = Q2_TURN_SPEED`
- `tar_angle = Q2_AD_TURN_A_RIGHT_ANGLE`
- 角度到位后进入 `Q2_AD_FIND_AB_OUT`

`Q2_AD_FIND_AB_OUT`：

- `motion = FIND_LINE`
- `base_speed = Q2_FINAL_SLOW_SPEED`
- 中间 6 路看到线后进入 `Q2_AD_SIDE_AB_OUT`

`Q2_AD_SIDE_AB_OUT`：

- `motion = FIND_LINE`
- 使用普通直行速度策略
- 检测到 B 点右路口后进入 `Q2_AD_TURN_B_TO_BC`
- 期望路口：`RightRoad`

### 2. B 点右转进入 BC

```text
Q2_AD_TURN_B_TO_BC -> Q2_AD_FIND_BC_OUT -> Q2_AD_SIDE_BC_OUT
```

`Q2_AD_TURN_B_TO_BC`：

- `motion = KEEP_ANGLE`
- `base_speed = Q2_TURN_SPEED`
- `tar_angle = Q2_AD_TURN_B_RIGHT_ANGLE`
- 角度到位后进入 `Q2_AD_FIND_BC_OUT`

`Q2_AD_FIND_BC_OUT`：

- 低速找线
- 中间 6 路看到线后进入 `Q2_AD_SIDE_BC_OUT`

`Q2_AD_SIDE_BC_OUT`：

- `B -> C`，这里在 BC 干扰 A4 上
- 阶段里程 `< Q2_BC_ROAD_ENABLE_CM` 时禁止消费任何路口
- 阶段里程 `>= Q2_BC_ROAD_ENABLE_CM` 后，检测到 C 点右路口才进入 `Q2_AD_TURN_C_TO_CD`
- 期望路口：`RightRoad`

### 3. C 点右转进入 CD

```text
Q2_AD_TURN_C_TO_CD -> Q2_AD_FIND_CD_OUT -> Q2_AD_SIDE_CD_OUT
```

`Q2_AD_TURN_C_TO_CD`：

- `motion = KEEP_ANGLE`
- `base_speed = Q2_TURN_SPEED`
- `tar_angle = Q2_AD_TURN_C_RIGHT_ANGLE`
- 角度到位后进入 `Q2_AD_FIND_CD_OUT`

`Q2_AD_FIND_CD_OUT`：

- 低速找线
- 中间 6 路看到线后进入 `Q2_AD_SIDE_CD_OUT`

`Q2_AD_SIDE_CD_OUT`：

- `C -> D`
- 到 D 前可在 `Q2_UTURN_PRE_SLOW_CM` 之后提前降速
- 检测到 D 点右路口后进入 `Q2_AD_STOP_BEFORE_UTURN_D`
- 期望路口：`RightRoad`

### 4. D 点停车后原地掉头

```text
Q2_AD_STOP_BEFORE_UTURN_D -> Q2_AD_UTURN_D -> Q2_AD_FIND_DC_RETURN
```

`Q2_AD_STOP_BEFORE_UTURN_D`：

- `motion = STOP`
- `base_speed = 0`
- 左右轮 `tar_speed = 0`
- 等两个轮子的 `cur_speed` 都小于 `Q2_UTURN_STOP_SPEED_THRESHOLD`
- 停稳后进入 `Q2_AD_UTURN_D`

`Q2_AD_UTURN_D`：

- `motion = KEEP_ANGLE`
- `base_speed = 0`
- `tar_angle = Q2_AD_UTURN_D_ANGLE`
- `angle_output_limit = Q2_UTURN_ANGLE_LIMIT`
- 角度误差小于 `Q2_UTURN_TOLERANCE_DEG` 后进入 `Q2_AD_FIND_DC_RETURN`

`Q2_AD_FIND_DC_RETURN`：

- `motion = FIND_LINE`
- `base_speed = Q2_FINAL_SLOW_SPEED`
- 中间 6 路看到线后进入 `Q2_AD_SIDE_DC_RETURN`

### 5. D -> C 回程

```text
Q2_AD_SIDE_DC_RETURN -> Q2_AD_TURN_C_RETURN
```

`Q2_AD_SIDE_DC_RETURN`：

- `motion = FIND_LINE`
- 进入该阶段后恢复 `angle_output_limit = Q2_NORMAL_ANGLE_LIMIT`
- 使用普通直行速度策略
- 检测到 C 点左路口后进入 `Q2_AD_TURN_C_RETURN`
- 期望路口：`LeftRoad`

### 6. C 点左转进入 CB

```text
Q2_AD_TURN_C_RETURN -> Q2_AD_FIND_CB_RETURN -> Q2_AD_SIDE_CB_RETURN
```

`Q2_AD_TURN_C_RETURN`：

- `motion = KEEP_ANGLE`
- `base_speed = Q2_TURN_SPEED`
- `tar_angle = Q2_AD_TURN_C_LEFT_ANGLE`
- 角度到位后进入 `Q2_AD_FIND_CB_RETURN`

`Q2_AD_FIND_CB_RETURN`：

- 低速找线
- 中间 6 路看到线后进入 `Q2_AD_SIDE_CB_RETURN`

`Q2_AD_SIDE_CB_RETURN`：

- `C -> B`，再次经过 BC 干扰 A4
- 阶段里程 `< Q2_CB_ROAD_ENABLE_CM` 时禁止消费任何路口
- 阶段里程 `>= Q2_CB_ROAD_ENABLE_CM` 后，检测到 B 点左路口才进入 `Q2_AD_TURN_B_RETURN`
- 期望路口：`LeftRoad`

### 7. B 点左转进入 BA

```text
Q2_AD_TURN_B_RETURN -> Q2_AD_FIND_BA_RETURN -> Q2_AD_SIDE_BA_RETURN
```

`Q2_AD_TURN_B_RETURN`：

- `motion = KEEP_ANGLE`
- `base_speed = Q2_TURN_SPEED`
- `tar_angle = Q2_AD_TURN_B_LEFT_ANGLE`
- 角度到位后进入 `Q2_AD_FIND_BA_RETURN`

`Q2_AD_FIND_BA_RETURN`：

- 低速找线
- 中间 6 路看到线后进入 `Q2_AD_SIDE_BA_RETURN`

`Q2_AD_SIDE_BA_RETURN`：

- `B -> A`，最后一段回发车点
- 阶段里程超过 `Q2_FINAL_SLOW_CM` 后降速到 `Q2_FINAL_SLOW_SPEED`
- 检测到 A 点左路口后进入 `Q2_AD_FINISH`
- 期望路口：`LeftRoad`

### 8. 结束

`Q2_AD_FINISH`：

```c
task_finish(status);
```

不要只设置 `stop_cmd = 1`，必须让 `task_finish()` 统一清理：

- `task_running = 0`
- `armed = 0`
- `motion = STOP`
- `base_speed = 0`
- 左右轮目标速度为 0
- 停车声光提示

## 六、AD 发车伪代码骨架

```c
case Q2_AD_TURN_A_TO_AB:
    KEEP_ANGLE, target = Q2_AD_TURN_A_RIGHT_ANGLE;
    angle_ready -> Q2_AD_FIND_AB_OUT;
    break;

case Q2_AD_FIND_AB_OUT:
    FIND_LINE low speed;
    middle6_seen -> Q2_AD_SIDE_AB_OUT;
    break;

case Q2_AD_SIDE_AB_OUT:
    FIND_LINE side speed;
    accept RightRoad -> Q2_AD_TURN_B_TO_BC;
    break;

case Q2_AD_TURN_B_TO_BC:
    KEEP_ANGLE, target = Q2_AD_TURN_B_RIGHT_ANGLE;
    angle_ready -> Q2_AD_FIND_BC_OUT;
    break;

case Q2_AD_FIND_BC_OUT:
    FIND_LINE low speed;
    middle6_seen -> Q2_AD_SIDE_BC_OUT;
    break;

case Q2_AD_SIDE_BC_OUT:
    FIND_LINE side speed;
    accept RightRoad after Q2_BC_ROAD_ENABLE_CM -> Q2_AD_TURN_C_TO_CD;
    break;

case Q2_AD_TURN_C_TO_CD:
    KEEP_ANGLE, target = Q2_AD_TURN_C_RIGHT_ANGLE;
    angle_ready -> Q2_AD_FIND_CD_OUT;
    break;

case Q2_AD_FIND_CD_OUT:
    FIND_LINE low speed;
    middle6_seen -> Q2_AD_SIDE_CD_OUT;
    break;

case Q2_AD_SIDE_CD_OUT:
    FIND_LINE side speed;
    after Q2_UTURN_PRE_SLOW_CM -> slow;
    accept RightRoad -> Q2_AD_STOP_BEFORE_UTURN_D;
    break;

case Q2_AD_STOP_BEFORE_UTURN_D:
    STOP;
    wheel speed low -> Q2_AD_UTURN_D;
    break;

case Q2_AD_UTURN_D:
    KEEP_ANGLE, base_speed = 0, angle_limit = Q2_UTURN_ANGLE_LIMIT;
    uturn_angle_ready -> Q2_AD_FIND_DC_RETURN;
    break;

case Q2_AD_FIND_DC_RETURN:
    FIND_LINE low speed;
    middle6_seen -> Q2_AD_SIDE_DC_RETURN;
    break;

case Q2_AD_SIDE_DC_RETURN:
    restore angle_limit;
    FIND_LINE side speed;
    accept LeftRoad -> Q2_AD_TURN_C_RETURN;
    break;

case Q2_AD_TURN_C_RETURN:
    KEEP_ANGLE, target = Q2_AD_TURN_C_LEFT_ANGLE;
    angle_ready -> Q2_AD_FIND_CB_RETURN;
    break;

case Q2_AD_FIND_CB_RETURN:
    FIND_LINE low speed;
    middle6_seen -> Q2_AD_SIDE_CB_RETURN;
    break;

case Q2_AD_SIDE_CB_RETURN:
    FIND_LINE side speed;
    accept LeftRoad after Q2_CB_ROAD_ENABLE_CM -> Q2_AD_TURN_B_RETURN;
    break;

case Q2_AD_TURN_B_RETURN:
    KEEP_ANGLE, target = Q2_AD_TURN_B_LEFT_ANGLE;
    angle_ready -> Q2_AD_FIND_BA_RETURN;
    break;

case Q2_AD_FIND_BA_RETURN:
    FIND_LINE low speed;
    middle6_seen -> Q2_AD_SIDE_BA_RETURN;
    break;

case Q2_AD_SIDE_BA_RETURN:
    final speed policy;
    accept LeftRoad after Q2_FINAL_SLOW_CM -> Q2_AD_FINISH;
    break;

case Q2_AD_FINISH:
    task_finish(status);
    break;
```

## 七、验收标准

1. `TASK_BASIC_2 + START_AB` 的现有行为不被破坏。
2. `TASK_BASIC_2 + START_AD` 能进入完整 AD 状态机，而不是卡在 `armed=1, task_running=0`。
3. AD 路线是 `A -> B -> C -> D -> 掉头 -> C -> B -> A`。
4. `B -> C` 和 `C -> B` 两次经过 BC 干扰边时，79cm 前都不消费路口。
5. D 点掉头前先停车，轮速降到阈值以下再原地掉头。
6. 原地掉头时 `base_speed = 0`，`angle_output_limit` 临时放大，回程直行后恢复正常。
7. 最后一段 `B -> A` 末端降速并在 A 点 `task_finish(status)`。
8. 所有角度、速度、里程阈值都通过宏配置，方便实车调参。
9. 不改 `get_road_type()` / `road_new_from_bit()` 的左右路口枚举逻辑。
