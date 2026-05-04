# TASK3 难点与处理方案

## 任务本质

TASK3 主运动框架和 TASK1 很接近：小车沿正方形赛道行驶 1 圈后回到发车点停车。

不同点在于 TASK3 载有 330ml 矿泉水，并且 CD 边中部有待测 A4 纸，需要获得 `l1` 和 `l2`，再通过蓝牙发送到手机显示。

## 1. 负重导致前馈不同

TASK3 使用负重参数，启动任务时走 `apply_adv_control_param()`，随后用 Q3 专用前馈宏覆盖左右轮：

```c
Q3_WHEEL0_FF_OFFSET
Q3_WHEEL0_FF_K
Q3_WHEEL0_FF_MIN

Q3_WHEEL1_FF_OFFSET
Q3_WHEEL1_FF_K
Q3_WHEEL1_FF_MIN
```

这样 TASK3 的两个轮子可以使用不同前馈系数，补偿负重和重心偏移。

## 2. CD 待测 A4 会干扰路口判断

CD 边的待测 A4 上有三条横线，这些横线不是普通路口，不能被当作真实路口消费。

处理方式：

- 在 CD 阶段使用里程数确定待测 A4 的干扰窗口。
- 进入该窗口后屏蔽普通路口消费。
- 离开窗口后恢复正常路口判断。

## 3. CD 阶段需要降速

CD 阶段前段先正常行驶，走过约 20cm 后降低速度。

目的：

- 给视觉模块更多帧数。
- 提高 `l1/l2` 数据稳定性。
- 降低横线对灰度循迹的冲击。

## 4. l1/l2 接收方式

继续沿用 `PID_Tune` 的命令格式：

```text
Clx\r\n
CLx\r\n
```

含义：

- `Clx`：视觉模块发送 `l1`
- `CLx`：视觉模块发送 `l2`

`UART_PID_Tune()` 中只负责解析并保存数值，不在串口中断里打印。

## 5. l1/l2 蓝牙发送方式

主函数定义全局变量保存 `l1` 和 `l2`，再定义全局标志位 `task3_finished`。

TASK3 完成后将 `task3_finished` 置 1。

主循环中检测到 `task3_finished == 1` 后，循环打印：

```c
printf("%.2f,%.2f\r\n", l1, l2);
```

当前 `printf` 已经重定向到 USART2，因此这条输出会通过蓝牙发送到手机。

## 6. 复用 TASK2 的两个优化

TASK3 不应该直接照搬 TASK1 的粗糙写法，而应该复用 TASK2 已经验证过的两个优化。

### 6.1 每个转弯角度单独用宏

TASK2 中每个关键转弯都有独立可调宏，例如：

```c
Q2_TURN_A_LEFT_ANGLE
Q2_TURN_D_LEFT_ANGLE
Q2_TURN_C_LEFT_ANGLE
Q2_TURN_C_RIGHT_ANGLE
Q2_TURN_D_RIGHT_ANGLE

Q2_AD_TURN_A_RIGHT_ANGLE
Q2_AD_TURN_B_RIGHT_ANGLE
Q2_AD_TURN_C_RIGHT_ANGLE
Q2_AD_TURN_C_LEFT_ANGLE
Q2_AD_TURN_B_LEFT_ANGLE
```

实现方式是进入某个转弯阶段时：

```c
status->state.initial_angle = status->state.cur_angle;
status->state.tar_angle = 当前路口对应的角度宏;
status->state.motion = KEEP_ANGLE;
```

TASK3 也应该这样做。不要所有左转共用一个角度，因为 A/D/C/B 每个角点的实际入弯姿态、地面摩擦、传感器压线位置都可能不一样。

### 6.2 `cnt_seen` 复位加入稳定条件

TASK1 中 `cnt_seen` 复位比较简单，看到 `Straight` 就可能清零：

```c
if (road == Straight && status->state.motion == FIND_LINE) {
  status->task.cnt_seen = 0;
}
```

TASK2 做得更稳：使用 `q2_mid4_stable_cnt` 和 `Q2_LINE_STABLE_CNT`，只有中间四路稳定看到线一段时间后，才允许把 `cnt_seen` 清零。

核心思路：

```c
if (task2_middle4_seen(status)) {
  q2_mid4_stable_cnt++;
} else {
  q2_mid4_stable_cnt = 0;
}

if (road == Straight
    && status->state.motion == FIND_LINE
    && q2_mid4_stable_cnt >= Q2_LINE_STABLE_CNT) {
  status->task.cnt_seen = 0;
}
```

这样可以防止刚离开路口、横线干扰、灰度瞬间抖动时，`cnt_seen` 被过早清零，导致同一个路口被重复消费。

TASK3 也应该沿用这个处理，尤其是 CD 待测 A4 上有三条横线，更容易制造短暂的假 `Straight` 或假路口。

## 7. 状态机处理原则

TASK3 可以大量复用 TASK1 的一圈行驶框架。

需要单独加入的逻辑只有：

- TASK3 使用负重前馈参数。
- CD 阶段用里程数屏蔽待测 A4 干扰。
- CD 阶段约 20cm 后降速。
- 每个转弯角度使用独立宏。
- `cnt_seen` 复位使用中间四路稳定计数条件。
- 接收视觉模块发送的 `l1/l2`。
- 任务结束后置 `task3_finished`，由主循环通过蓝牙反复发送 `l1/l2`。

## 8. 给 AI agent 的 TASK3 实现提示词

请你按照下面要求实现 `driver_task3()`，目标是让 TASK3 在现有 TASK 架构中跑通。整体思路是：直接仿照 TASK1 的一圈行驶状态机框架，但是必须吸收 TASK2 已经验证过的两个优化，并加入 TASK3 自己的 CD 待测 A4、视觉测距、蓝牙回传逻辑。

### 8.1 改动范围

优先只改这些文件：

- `User/Status/Defect.h`
- `User/Status/Defect.c`
- `Core/Src/main.c`

不要修改以下底层逻辑：

- 不要修改 `get_road_type()`。
- 不要修改 `road_new_from_bit()`。
- 不要修改灰度传感器底层采样和数字化逻辑。
- 不要修改 TASK1/TASK2 已经能跑的状态机行为。
- 不要修改 PID、wheel、feedforward 的底层计算方式。

### 8.2 TASK3 总体路线

TASK3 本质上和 TASK1 一样，是绕赛道跑一圈，但是 TASK3 多了 CD 边上的待测 A4 测距任务。

AB 发车时，路线按 TASK1 框架处理：

```text
AB 起步 -> A -> AD -> D -> DC/CD边 -> C -> CB -> B -> BA -> 回到发车点停车
```

AD 发车时，使用镜像方向：

```text
AD 起步 -> A -> AB -> B -> BC -> C -> CD边 -> D -> DA -> 回到发车点停车
```

如果当前代码时间不够，可以先完整实现 AB 发车；但结构上必须预留 AD 发车的 phase 和独立角度宏，不要把 AD 写死成 AB。

### 8.3 在 `Defect.h` 中增加 TASK3 race_phase

在现有 `Q1_RACE_PHASE`、`Q2_RACE_PHASE` 后面增加 TASK3 的 phase 枚举。命名要清晰，建议分 AB/AD 两套。

AB 发车建议：

```c
Q3_AB_START_TO_A,
Q3_AB_TURN_A_TO_AD,
Q3_AB_FIND_AD,
Q3_AB_SIDE_AD,
Q3_AB_TURN_D_TO_DC,
Q3_AB_FIND_DC,
Q3_AB_SIDE_DC,
Q3_AB_TURN_C_TO_CB,
Q3_AB_FIND_CB,
Q3_AB_SIDE_CB,
Q3_AB_TURN_B_TO_BA,
Q3_AB_FIND_BA,
Q3_AB_SIDE_BA_FINAL,
Q3_AB_FINISH,
```

AD 发车建议：

```c
Q3_AD_START_TO_A,
Q3_AD_TURN_A_TO_AB,
Q3_AD_FIND_AB,
Q3_AD_SIDE_AB,
Q3_AD_TURN_B_TO_BC,
Q3_AD_FIND_BC,
Q3_AD_SIDE_BC,
Q3_AD_TURN_C_TO_CD,
Q3_AD_FIND_CD,
Q3_AD_SIDE_CD,
Q3_AD_TURN_D_TO_DA,
Q3_AD_FIND_DA,
Q3_AD_SIDE_DA_FINAL,
Q3_AD_FINISH,
```

### 8.4 在 `Defect.c` 中增加 TASK3 可调宏

TASK3 的每个转弯角度必须独立成宏，不要所有左转/右转共用一个角度。

建议至少定义：

```c
#define Q3_AB_TURN_A_LEFT_ANGLE    ...
#define Q3_AB_TURN_D_LEFT_ANGLE    ...
#define Q3_AB_TURN_C_LEFT_ANGLE    ...
#define Q3_AB_TURN_B_LEFT_ANGLE    ...

#define Q3_AD_TURN_A_RIGHT_ANGLE   ...
#define Q3_AD_TURN_B_RIGHT_ANGLE   ...
#define Q3_AD_TURN_C_RIGHT_ANGLE   ...
#define Q3_AD_TURN_D_RIGHT_ANGLE   ...
```

再定义速度、稳定判定和里程阈值：

```c
#define Q3_FLASH_SPEED             ...
#define Q3_CRUISE_SPEED            ...
#define Q3_TURN_SPEED              ...
#define Q3_FINAL_SLOW_SPEED        ...
#define Q3_CD_SLOW_SPEED           ...

#define Q3_LINE_STABLE_CNT         ...
#define Q3_TURN_LINE_MASK_6        0x7E
#define Q3_TURN_LINE_MASK_4        0x3C

#define Q3_STRAIGHT_FLASH_CM       ...
#define Q3_FINAL_SLOW_CM           ...
#define Q3_CD_SLOW_AFTER_CM        20.0f
#define Q3_CD_IGNORE_START_CM      ...
#define Q3_CD_IGNORE_END_CM        ...
```

具体数值先给保守默认值，后续我会实车调。

### 8.5 `driver_task3()` 的实现原则

直接仿照 `driver_task1()` 的结构写，不要写成一个临时 demo。

必须拆出清晰的小函数，建议命名：

```c
static ROAD_TYPE task3_map_road(ROAD_TYPE road);
static void task3_enter_phase(STATUS *status, uint8_t phase);
static uint8_t task3_middle4_seen(STATUS *status);
static uint8_t task3_middle6_seen(STATUS *status);
static uint8_t task3_turn_angle_ready(STATUS *status);
static uint8_t task3_accept_road(STATUS *status, ROAD_TYPE expected, float min_cm);
static void task3_apply_side_speed(STATUS *status);
static void task3_apply_cd_speed(STATUS *status);
static uint8_t task3_final_stop_condition(STATUS *status);
```

这些函数要写简洁注释，说明它们解决的实际问题。

### 8.6 路口映射规则

TASK3 内部可以像 TASK1 一样合并 T 路口：

```c
TLRoad -> LeftRoad
TRRoad -> RightRoad
```

不要改 `get_road_type()` 本身，只在 TASK3 内部做映射。

### 8.7 `cnt_seen` 复位必须使用 TASK2 的稳定条件

不要照搬 TASK1 里简单看到 `Straight` 就清零的写法。

TASK3 必须仿照 TASK2 增加一个类似 `q3_mid4_stable_cnt` 的静态计数器：

```c
if (task3_middle4_seen(status)) {
  q3_mid4_stable_cnt++;
} else {
  q3_mid4_stable_cnt = 0;
}
```

只有满足：

```c
road == Straight
&& status->state.motion == FIND_LINE
&& q3_mid4_stable_cnt >= Q3_LINE_STABLE_CNT
```

才允许清 `status->task.cnt_seen`。

并且在 `Q3_*_FIND_*` 这种找线阶段不要清 `cnt_seen`，防止刚转弯回线时误清零。

### 8.8 CD 待测 A4 的专有处理

TASK3 最关键的是 CD 边。

AB 发车时，CD 待测 A4 位于：

```text
Q3_AB_SIDE_DC
```

AD 发车时，CD 待测 A4 位于：

```text
Q3_AD_SIDE_CD
```

这两个阶段必须用 `status->task.phase_mileage` 做特殊处理：

- 进入 CD 边阶段时，`phase_mileage` 清零。
- CD 阶段前一小段先正常巡线。
- 当 `phase_mileage` 超过 `Q3_CD_SLOW_AFTER_CM` 对应脉冲后，把 `base_speed` 降到 `Q3_CD_SLOW_SPEED`，提高视觉模块有效帧数。
- 在待测 A4 所在的里程窗口内，屏蔽路口消费逻辑，不要把 A4 上的三条横线当成真实路口。
- 只有超过 `Q3_CD_IGNORE_END_CM` 对应脉冲后，才允许重新接受下一个真实路口。

注意：`phase_mileage` 当前工程里是脉冲单位，只有比较 cm 宏时才换算，不要在累加时改成 cm。

### 8.9 `task_start()` 中 TASK3 的启动

`task_start()` 里 `TASK_ADV_1` 分支不能再只是：

```c
status->task.race_phase = 0;
```

必须根据 `status->task.start_pose` 设置 TASK3 初始 phase：

```c
if (status->task.start_pose == START_AB) {
  task3_enter_phase(status, Q3_AB_START_TO_A);
} else {
  task3_enter_phase(status, Q3_AD_START_TO_A);
}
```

TASK3 启动时要保留已有的负重前馈参数加载逻辑：

```c
apply_adv_control_param(status);
set_wheel_ff_param_by_which(... Q3 ...);
```

不要破坏 TASK1/TASK2/TASK4 的启动逻辑。

### 8.10 视觉数据接收和蓝牙回传

在 `main.c` 中增加全局变量：

```c
volatile float l1;
volatile float l2;
volatile uint8_t task3_finished;
```

在 `UART_PID_Tune()` 中增加命令解析：

```c
Clx   -> l1 = x
CLx   -> l2 = x
```

要求：

- `Clx\r\n` 表示视觉模块发送 l1。
- `CLx\r\n` 表示视觉模块发送 l2。
- `UART_PID_Tune()` 只负责接收和赋值，不要在串口中断回调里 `printf`。

在 TASK3 启动时清零：

```c
l1 = 0;
l2 = 0;
task3_finished = 0;
```

在 TASK3 正常结束时：

```c
task3_finished = 1;
task_finish(status);
```

主循环中如果检测到 `task3_finished == 1`，就通过已经重定向到 USART2 的 `printf` 循环发送：

```c
printf("%.2f,%.2f\r\n", l1, l2);
```

### 8.11 结束条件

TASK3 的停车逻辑先仿照 TASK1：

- 最后一条边进入 final phase 后，根据 `phase_mileage` 和灰度传感器状态判定停车。
- 停车时调用 `task_finish(status)`。
- 不要使用纯时间阈值作为主要阶段切换条件。

如果需要兜底，只能用里程数或路口判定，不要写类似：

```c
status->state.time - status->task.phase_start_time >= ...
```

这种时间阈值状态机。

### 8.12 验收标准

完成后请检查：

- TASK3 AB 发车能从 `TASK_ADV_1` 正确进入 TASK3 状态机。
- TASK3 AD 发车不会误走 AB 状态机。
- `cnt_seen` 不会在路口刚离开或 CD 横线干扰时被过早清零。
- CD 阶段能在约 20cm 后降速。
- CD 阶段能用里程窗口屏蔽待测 A4 横线。
- `Clx`、`CLx` 能正确更新 `l1/l2`。
- TASK3 结束后 `task3_finished` 置 1，主循环通过 USART2 反复打印 `l1,l2`。
- 不破坏 TASK1、TASK2、TASK4 的已有逻辑。
