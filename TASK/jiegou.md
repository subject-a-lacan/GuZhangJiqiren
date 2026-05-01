# 任务系统整体架构

本文档描述校赛四道题在现有 STM32 工程中的推荐嵌入方式。核心思想是：把“比赛任务状态”封装成独立的 `TASK` 结构体，挂到全局 `STATUS` 状态树中；按钮和蓝牙只负责改任务状态或置请求位；真正的任务启动、停止、题内状态机推进，统一放到 20ms 控制周期中的 `update_task()` 里完成。

简要来说：
TASK：任务层
MOTION：运动执行层



## 一、三层职责

工程中建议明确区分三层：

```text
status.task.task_id       当前选择第几问
status.task.race_phase    当前题目内部跑到哪一步
status.state.motion       底层运动方式，车现在怎么动
```

三者职责不同，不能混在一起：

| 层级 | 字段 | 职责 |
|---|---|---|
| 选题层 | `task_id` | 当前准备跑第几问：Q1/Q2/Q3/Q4 |
| 题内流程层 | `race_phase` | 当前题目内部阶段，例如起步、某条边、转弯、测距、停车 |
| 运动控制层 | `motion` | 底层运动模式，例如 `STOP`、`FIND_LINE`、`KEEP_ANGLE`、`MOTOR_TEST` |

例子：

```text
status.task.task_id = TASK_BASIC_1
status.task.race_phase = Q1_SIDE_AD
status.state.motion = FIND_LINE
```

含义是：当前跑第一问，题内阶段在 AD 边，底层正在循迹。

## 二、TASK 结构体

新增一个 `TASK` 结构体，专门管理比赛任务相关状态。

```c
typedef struct TASK {
    uint8_t task_id;        // 当前选择第几问
    uint8_t start_pose;     // 发车模式：START_AB / START_AD
    uint8_t race_phase;     // 当前题目内部阶段
    uint8_t cross_cnt;      // 当前任务内已确认的路口/角点计数

    uint8_t armed;          // 是否允许启动任务，原先讨论里的 cmd
    uint8_t task_running;   // 当前是否正在执行任务

    uint8_t start_request;  // 按键或蓝牙提出启动请求
    uint8_t stop_request;   // 蓝牙或紧急输入提出停止请求

    uint32_t phase_start_time; // 当前阶段开始时间，推荐预留
    float phase_mileage;       // 当前阶段累计里程，推荐预留
} TASK;
```

字段解释：

- `task_id`：当前选中的题目编号。
- `start_pose`：发车方向，第二问和第三问需要 AB/AD 二选一。
- `race_phase`：每道题自己的内部阶段。
- `cross_cnt`：当前任务内部使用的路口/角点计数。它属于任务流程，不应该长期依赖 `road.c` 里的全局计数。
- `armed`：武装标志。只有 `armed == 1` 时，启动请求才有效。
- `task_running`：任务是否正在运行。任务调度应该看这个字段，不应该看 `motion != STOP`。
- `start_request`：启动请求。按钮或蓝牙只置 1，由 `update_task()` 消费。
- `stop_request`：停止请求。急停优先级最高，由 `update_task()` 消费。
- `phase_start_time`：进入当前阶段时的系统时间，用于超时保护、蜂鸣器节奏等。
- `phase_mileage`：当前阶段里程，用于第一问停车、第三问测距等。

## 三、STATUS 挂载方式

把 `TASK` 挂到全局 `STATUS` 中：

```c
typedef struct STATUS {
    STATE state;
    SENSOR sensor;
    MOTOR motor;
    DEVICE device;
    TASK task;
} STATUS;
```

这样状态树含义更清楚：

```text
status.state.motion       车现在怎么动
status.state.base_speed   底层目标速度
status.state.cur_angle    当前航向角

status.task.task_id       当前选哪一问
status.task.start_pose    当前发车模式
status.task.race_phase    当前题内阶段
status.task.cross_cnt     当前任务确认过的路口/角点数量
status.task.task_running  当前任务是否运行
```

`STATE` 继续描述车本身的运动状态，`TASK` 描述比赛任务状态。

### cross_cnt 归属

`cross_cnt` 建议放在 `TASK` 结构体里。原因是它不是一个单纯的传感器原始量，而是“当前任务已经确认过几个有效路口/角点”的流程状态。

推荐分工：

```text
road.c / road_determine:
  负责从灰度数据里判断当前看起来像 Straight / LeftRoad / RightRoad / CrossRoad

status.task.cross_cnt:
  负责记录当前题目状态机已经消费并确认过几个有效路口/角点
```

也就是说，路口检测模块只提供“检测到了什么路况事件”，任务状态机决定这个事件是否有效、是否计数、是否触发阶段切换。比如第一问起点 A 的特殊路口、最后 BA 边靠近 A 的停车触发，都不能让底层 `road.c` 自己随便加全局计数。

如果短期为了少改代码保留 `road.c` 里的 legacy/global `cross_cnt`，也应把它当作兼容变量。长期建议统一迁移到 `status.task.cross_cnt`，避免 Q1/Q2/Q3/Q4 互相污染。

## 四、task_id 枚举

```c
typedef enum TASK_ID {
    TASK_BASIC_1 = 1,  // 第一问：跑 1 圈并停回发车点
    TASK_BASIC_2 = 2,  // 第二问：带干扰 A4，跑 3/4 圈后掉头返回
    TASK_ADV_1   = 3,  // 第三问：测 l1/l2 并无线发送
    TASK_ADV_2   = 4,  // 第四问：识别图形并导航停车
} TASK_ID;
```

建议从 1 开始，方便蓝牙命令 `'1'`、`'2'`、`'3'`、`'4'` 直接对应题号。

## 五、start_pose 枚举

```c
typedef enum START_POSE {
    START_AB = 0,  // AB 发车点
    START_AD = 1,  // AD 发车点
} START_POSE;
```

适用规则：

- 第一问固定 `START_AB`。
- 第二问由裁判选择 `START_AB` 或 `START_AD`，需要支持切换。
- 第三问由裁判选择 `START_AB` 或 `START_AD`，需要支持切换。
- 第四问固定 `START_AB`。

## 六、armed 武装标志

`armed` 对应之前讨论里的 `cmd`，但名字更清楚。

```text
armed = 0  未武装，不能启动任务
armed = 1  已武装，允许启动任务
```

推荐生命周期：

```text
上电:
  armed = 0
  task_running = 0
  motion = STOP

蓝牙 'S' 急停:
  stop_request = 1
  update_task() 消费后按远程停车语义停车

蓝牙 'M':
  armed = 1

任务正常完成:
  task_running = 0
  motion = STOP
  armed 保持 1，方便重跑
```

上电默认 `armed = 0`，避免误触发启动。必须通过蓝牙 `'M'`，或后续设计的硬件武装方式，将 `armed` 置 1 后，PD2 长按或蓝牙 `'G'` 才能真正启动任务。
事实上我就打算默认改成arm=0 傻逼gpt

## 七、按钮分配

按钮事件发生后，直接修改 `status.task` 或置 request，

| 按键 | 事件 | 条件 | 动作 |
|---|---|---|---|
| PB11 短按 | `BUTTON_UP` 且本次没有触发长按 | `task_running == 0` | `task_id` 在 1/2/3/4 之间轮换 |
| PB11 长按 | `BUTTON_LONG` | `task_running == 0` 且 `task_id` 为 2 或 3 | 切换 `start_pose`：AB/AD |
| PD2 短按 | `BUTTON_UP` 且本次没有触发长按 | 任意安全时刻 | 灰度校准 `correct_gw_analogue()` |
| PD2 长按 | `BUTTON_LONG` | `task_running == 0` 且 `armed == 1` | `start_request = 1` |

任务运行中，普通按键操作锁死。运行中只保留蓝牙急停或专门的硬件急停入口。

### 短按/长按互斥

`driver_button()` 的事件顺序是：

```text
按下瞬间       -> BUTTON_DOWN
按住足够久     -> BUTTON_LONG
释放           -> BUTTON_UP
```

所以短按动作不能放在 `BUTTON_DOWN`，否则长按也会先触发短按。推荐在 `BUTTON` 结构体中加入：

```c
uint8_t long_triggered;
```

处理逻辑：

```text
BUTTON_DOWN:
  button->long_triggered = 0

BUTTON_LONG:
  button->long_triggered = 1
  执行长按动作

BUTTON_UP:
  if (button->long_triggered == 0)
      执行短按动作
```

PB11 和 PD2 都应该使用这套互斥逻辑。

### LONG_PRESS_CNT 时序

当前 `driver_button()` 在 `update_status()` 中被调用，周期是 20ms。因此：

```text
LONG_PRESS_CNT = 20
真实长按时间 = 20 * 20ms = 400ms
```

注释里如果写“单位 ms”，需要注意实际含义是“调用次数”。

## 八、蓝牙命令

蓝牙命令和按键操作同一组 `status.task` 字段，避免两套逻辑冲突。

| 命令 | 效果 |
|---|---|
| `'1'` | `task_id = TASK_BASIC_1` |
| `'2'` | `task_id = TASK_BASIC_2` |
| `'3'` | `task_id = TASK_ADV_1` |
| `'4'` | `task_id = TASK_ADV_2` |
| `'A'` | `start_pose = START_AB` |
| `'D'` | `start_pose = START_AD` |
| `'M'` | `armed = 1` |
| `'G'` | 如果 `armed == 1`，置 `start_request = 1` |
| `'S'` | 置 `stop_request = 1` |

不用多字符命令 `ARM`，因为它会和单字符 `'A'` 冲突，并且需要额外字符串解析。

## 九、LED 与蜂鸣器反馈

推荐反馈方式：

| 状态 | 反馈 |
|---|---|
| `task_id = N` | 板载 LED 闪 N 次，停顿 1 秒，循环 |
| `start_pose = START_AB` | `led1` 亮，`led2` 灭 |
| `start_pose = START_AD` | `led1` 灭，`led2` 亮 |
| `armed == 1` | 任务选择闪烁结束后额外常亮 200ms |
| 切换 task_id/start_pose | 蜂鸣器短响 50ms |
| 启动任务 | 蜂鸣器长响 200ms |
| 急停 | 蜂鸣器快速响 3 次 |

反馈逻辑最好也由 20ms 周期状态机驱动，不要在中断里写长时间阻塞延时。

## 十、update_task() 任务调度入口

`update_task()` 每 20ms 在 `update_status()` 中调用。

推荐调用顺序：

```text
update_status():
  1. 读取灰度、轮速、陀螺仪等传感器
  2. driver_button() 处理按钮事件，按钮事件直接修改 status.task
  3. 解析蓝牙/串口命令，直接修改 status.task
  4. update_task(&status)
  5. 根据 status.state.motion 执行底层控制
       FIND_LINE  -> follow_line()
       KEEP_ANGLE -> keep_angle()
       STOP       -> 目标速度清零
  6. driver_LED / driver_BUZZER / driver_wheel
```

`update_task()` 不负责实时扫描按键。按钮和蓝牙事件发生时已经直接置好了 request。

## 十一、update_task() 内部逻辑

推荐顺序：

```text
update_task(status):

  // 1. 远程停车请求优先级最高
  if (status.task.stop_request) {
      task_stop(status);
      return;
  }

  // 2. 处理启动请求
  if (status.task.start_request) {
      if (!status.task.task_running && status.task.armed) {
          task_start(status);
      }
      status.task.start_request = 0;
  }

  // 3. 没有运行任务则不推进题目状态机
  if (!status.task.task_running) {
      return;
  }

  // 4. 根据 task_id 分发
  switch (status.task.task_id) {
      case TASK_BASIC_1:
          task_basic_1_update(status);
          break;
      case TASK_BASIC_2:
          task_basic_2_update(status);
          break;
      case TASK_ADV_1:
          task_adv_1_update(status);
          break;
      case TASK_ADV_2:
          task_adv_2_update(status);
          break;
  }
```

这里的 `task_basic_1_update()` 等函数只负责本题自己的 `race_phase` 推进，并设置底层运动参数。

## 十二、任务启动 task_start()

每次启动任务都必须统一初始化，避免上一轮状态污染下一轮。

```text
task_start(status):
  task_running = 1
  start_request = 0
  stop_request = 0

  清任务级路口计数 status.task.cross_cnt
  清 legacy/global cross_cnt，避免旧逻辑污染
  清 road_buf / road_determine
  清里程 mileage / phase_mileage
  清 PID integral / last_error / out
  刷新 initial_angle
  清轮子目标速度和实际控制输出

  根据 task_id 和 start_pose 设置 race_phase
  设置 phase_start_time = status.state.time
  设置 motion / base_speed / tar_angle 初值
  启动声光提示
```

不同题目的初始阶段例子：

```text
TASK_BASIC_1:
  race_phase = Q1_START_A_TURN
  start_pose 固定 START_AB

TASK_BASIC_2:
  race_phase 根据 start_pose 选择 Q2_START_AB 或 Q2_START_AD

TASK_ADV_1:
  race_phase 根据 start_pose 选择 Q3_START_AB 或 Q3_START_AD

TASK_ADV_2:
  race_phase = Q4_START_AB
  start_pose 固定 START_AB
```

## 十三、任务结束 task_finish()

任务正常完成时：

```text
task_finish(status):
  task_running = 0
  state.motion = STOP
  state.base_speed = 0
  左右轮 tar_speed = 0
  armed 保持 1
  完成声光提示
```

`armed` 保持 1 是为了方便重跑。如果需要更安全，可以完成后也置 0。

## 十四、远程停车 task_stop()

远程停车用于蓝牙 `'S'`，语义应尽量接近现有 `main.c` 里 `Cz` 指令的停车效果，但要额外阻止任务状态机在下一周期继续推进。

```text
task_stop(status):
  task_running = 0
  start_request = 0
  stop_request = 0
  state.motion = STOP
  state.base_speed = 0
  左右轮 tar_speed = 0
```

也就是：

```c
status.task.task_running = 0;
status.task.start_request = 0;
status.task.stop_request = 0;
status.state.motion = STOP;
status.state.base_speed = 0;
status.motor.wheel[0].tar_speed = 0;
status.motor.wheel[1].tar_speed = 0;
```

`task_stop()` 不清 `task_id`，不清 `start_pose`，不清 `race_phase`，不清 `armed`，不重置 PID，也不清路口计数。它只是让车停下，并让当前任务不再继续自动推进。

因此不再需要单独的 `task_abort()` 概念。如果后续确实需要“异常中止并完全清状态”的重逻辑，可以另起名字，例如 `task_reset_all()`，不要和蓝牙停车混在一起。

## 十五、题目状态机示例

第一问示意：

```text
Q1_START_A_TURN
  起点 A 特化左转，不走普通 LeftRoad 检测
  完成 -> Q1_SIDE_AD

Q1_SIDE_AD
  FIND_LINE
  检测到 D 点 -> Q1_TURN_D

Q1_TURN_D
  陀螺仪硬转 + 低速找线
  完成 -> Q1_SIDE_DC

Q1_SIDE_DC
  FIND_LINE
  检测到 C 点 -> Q1_TURN_C

Q1_TURN_C
  完成 -> Q1_SIDE_CB

Q1_SIDE_CB
  FIND_LINE
  检测到 B 点 -> Q1_TURN_B

Q1_TURN_B
  完成 -> Q1_BA_FINAL

Q1_BA_FINAL
  BA 边低速循迹
  编码器主停车，灰度 A 点兜底
  完成 -> task_finish()
```

这说明：`race_phase` 是题目内部流程核心，`motion` 只是每个阶段选择的底层运动方式。

## 十六、落地原则

1. 按钮和蓝牙事件发生时，直接修改 `status.task` 或置 request。
2. 不需要 `task_input()` 实时扫描按钮。
3. `update_task()` 每 20ms 消费 request，启动、停止、推进题目状态机。
4. `task_running` 决定任务是否推进，不能用 `motion != STOP` 代替。
5. `race_phase` 必须独立存在，不能只靠 `cross_cnt` 推导。
6. `motion` 只表示底层运动方式，不表示第几问。
7. 每次启动任务必须统一清理路口计数、PID、里程、阶段时间等状态。
8. 蓝牙和按钮共用同一套状态字段，避免双系统冲突。
