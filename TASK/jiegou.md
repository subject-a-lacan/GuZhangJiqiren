# 任务系统整体架构

本文档描述校赛四道题在现有 STM32 工程中的推荐嵌入方式。核心思想是：把“比赛任务状态”封装成独立的 `TASK` 结构体，挂到全局 `STATUS` 状态树中；按钮和蓝牙只负责改任务状态或置请求位；真正的任务启动、停止、题内状态机推进，统一放到 20ms 控制周期中的 `update_task()` 里完成。

简要来说：
TASK：任务层
MOTION：运动执行层

请从第162行开始阅读 前面是架构实例的调用链展示

## 零、从切换任务到发车运行的完整调用链

这里先把最重要的一条链路写清楚：外部输入只负责“提出请求”，真正改 `task_id`、启动任务、推进题内状态机，都在 `update_task()` 里完成。

### 1. 切换到 TASK_X

以 PB11 短按切换任务为例：

```text
PB11 按下并释放
  -> driver_button() 识别 BUTTON_UP
  -> server_button() 判断本次没有触发长按
  -> 计算 next_task_id
  -> status.task.requested_task_id = next_task_id
  -> status.task.task_select_request = 1
```

如果是蓝牙选题，链路也一样，只是 `requested_task_id` 来自蓝牙命令解析：

```text
蓝牙收到选题命令
  -> 解析出目标 TASK_X
  -> status.task.requested_task_id = TASK_X
  -> status.task.task_select_request = 1
```

随后在 20ms 控制周期中：

```text
update_status()
  -> update_task(&status)
    -> 发现 task_running == 0
    -> 发现 task_select_request == 1
    -> task_select(status, requested_task_id)
      -> status.task.task_id = TASK_X
      -> 如果 TASK_X 是 TASK1/TASK4，start_pose 强制 START_AB
      -> 如果 TASK_X 是 TASK2/TASK3，保留当前 start_pose
      -> 刷新 LED 编码
    -> task_select_request = 0
```

到这里为止，只是“选中了第 X 问”，车还没有动：

```text
status.task.task_id = TASK_X
status.task.task_running = 0
status.state.motion = STOP
```

### 2. 短按 PD2 请求进入任务入口

PD2 短按不是直接启动任务，也不直接修改 `task_running`。它只提出“我要进入当前选中任务”的请求：

```text
status.task.armed == 0
status.task.task_running == 0
```

PD2 短按后：

```text
PD2 按下并释放
  -> driver_button() 识别 BUTTON_UP
  -> server_button() 判断本次没有触发长按
  -> 如果 armed == 0 且 task_running == 0
       status.task.start_request = 1
       蜂鸣器短响一次
```

随后在 20ms 控制周期中：

```text
update_status()
  -> update_task(&status)
    -> 先处理 stop_request
    -> 再处理空闲状态下的 task_select_request / pose_switch_request
    -> 发现 start_request == 1
    -> 如果 armed == 0 且 task_running == 0
         status.task.armed = 1
         task_start(status)
    -> start_request = 0
```

`task_start(status)` 负责进入当前任务前的统一初始化，但不要把 `task_running` 置 1：

```text
task_start(status):
  清 start_request / stop_request
  清 cross_cnt / road_buf / road_determine
  清 PID 积分和历史误差
  清 phase_mileage
  刷新 initial_angle
  左右轮目标速度清零

  根据 task_id 和 start_pose 设置 race_phase
  phase_start_time = 当前系统时间
  motion = STOP
  base_speed = 0
```

注意：`task_start()` 只负责“进入任务前初始化”和“设置第一个 race_phase”。它不能代表任务已经真的跑起来，所以不要在这里置 `task_running = 1`。真正让车怎么动、什么时候确认任务开始运行，放到对应题目的 update 函数里。

### 3. 任务开始运行

`task_start()` 执行完后，`armed == 1` 表示当前任务入口已经放行。随后 `update_task()` 进入题目分发：

```text
update_task(status):
  if armed == 1:
    switch (task_id):
      TASK_BASIC_1 -> task_basic_1_update(status)
      TASK_BASIC_2 -> task_basic_2_update(status)
      TASK_ADV_1   -> task_adv_1_update(status)
      TASK_ADV_2   -> task_adv_2_update(status)
```

对应的 `task_xxx_update()` 根据 `race_phase` 设置底层运动：

```text
task_basic_1_update(status):
  switch (race_phase):
    Q1_START_A_TURN:
      确认任务流程真正开始后 task_running = 1
      设置 motion / base_speed / tar_angle
      满足完成条件后 task_set_phase(Q1_SIDE_AD)

    Q1_SIDE_AD:
      设置 motion = FIND_LINE
      检测到有效路口后 task_set_phase(Q1_TURN_D)
```

最后由底层运动控制执行：

```text
update_status()
  -> 根据 status.state.motion 执行 follow_line() / keep_angle() / stop
  -> driver_wheel()
  -> 电机实际动作
```

所以完整链路可以压缩成一句话：

```text
PB11/蓝牙选中 TASK_X
  -> update_task() 消费 task_select_request，真正修改 task_id
  -> PD2 短按置 start_request
  -> update_task() 将 armed 置 1，并调用 task_start() 做入口初始化
  -> task_xxx_update() 进入题内流程
  -> 题内状态机确认任务真的开始后，才置 task_running = 1
  -> task_xxx_update() 根据 race_phase 设置 motion
  -> 底层 motion 控制电机运行
  -> task_finish()/task_stop() 清 task_running 和 armed，退出任务入口
```

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

    uint8_t armed;          // 是否已进入当前任务入口，原先讨论里的 cmd
    uint8_t task_running;   // 题目内部确认任务正在运行

    uint8_t task_select_request;  // 请求切换任务
    uint8_t requested_task_id;    // 任务切换的目标任务
    uint8_t pose_switch_request;  // 请求切换 AB/AD 发车模式

    uint8_t start_request;  // 按键或蓝牙提出进入任务入口请求
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
- `armed`：任务入口许可标志。`start_request` 被 `update_task()` 消费后置 1，表示已经进入当前选中任务的流程入口。
- `task_running`：任务内部确认“任务真的开始运行”的标志，必须由具体 `task_xxx_update()` 在合适阶段置 1，不能由按钮直接置 1，也不建议由 `task_start()` 直接置 1。
- `task_select_request`：请求切换任务。PB11 短按和蓝牙选题都走这个接口。
- `requested_task_id`：任务切换的目标值。PB11 短按时由按钮逻辑先算出“下一个任务”，再填到这里。
- `pose_switch_request`：请求切换 AB/AD 发车模式。只在第二问和第三问空闲状态下有效。
- `start_request`：进入任务入口请求。按钮或蓝牙只置 1，由 `update_task()` 消费，消费后置 `armed = 1` 并执行任务入口初始化。
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

建议从 1 开始，方便后续蓝牙命令参数直接对应题号。具体蓝牙命令还待定，不在这里写死。

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

`armed` 对应之前讨论里的 `cmd`，但名字更清楚。它不是按钮直接改出的“任务已经运行”，而是 `update_task()` 消费 `start_request` 后确认“当前选中任务允许进入流程入口”。

```text
armed = 0  未进入任务入口
armed = 1  已进入当前选中任务的流程入口
```

`start_request`、`armed`、`task_running` 的关系必须分清：

```text
start_request:
  外部输入提出“进入任务入口”的一次性请求
  按钮/蓝牙只置这个请求，不直接改 task_running

armed:
  update_task() 消费 start_request 后置 1
  用来说明当前任务入口已经放行
  可以用来判断“有没有真正进入任务流程”

task_running:
  只能由具体 task_xxx_update() 在确认任务真的开始执行后置 1
  不能由按钮直接置 1
  不建议由 task_start() 直接置 1，避免假启动
```

推荐生命周期：

```text
上电:
  armed = 0
  task_running = 0
  motion = STOP

蓝牙停车命令:
  stop_request = 1
  update_task() 消费后按远程停车语义停车

PD2 短按或蓝牙启动命令:
  start_request = 1

update_task() 消费 start_request:
  armed = 1
  task_start(status) 做入口初始化

题内状态机确认任务开始:
  task_running = 1

任务正常完成:
  task_running = 0
  armed = 0
  motion = STOP
```

上电默认 `armed = 0`，避免误触发启动。PD2 短按或蓝牙启动命令只置 `start_request = 1`，真正把 `armed` 置 1 的动作必须放在 `update_task()` 里统一完成。

## 七、按钮分配

按钮事件发生后只置 `status.task` 里的请求位，不直接切换任务。按钮和蓝牙必须共用同一套请求接口，后续统一由 `update_task()` 消费请求并真正修改 `task_id` / `start_pose`。

| 按键 | 事件 | 条件 | 动作 |
|---|---|---|---|
| PB11 短按 | `BUTTON_UP` 且本次没有触发长按 | `task_running == 0` | 计算下一个任务，写入 `requested_task_id`，再置 `task_select_request = 1` |
| PB11 长按 | `BUTTON_LONG` | `task_running == 0` | `pose_switch_request = 1` |
| PD2 短按 | `BUTTON_UP` 且本次没有触发长按 | `armed == 0` 且 `task_running == 0` | `start_request = 1` |
| PD2 长按 | `BUTTON_LONG` | 任意安全时刻 | 灰度校准 `correct_gw_analogue()` |

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

蓝牙命令的具体字符和参数暂时待定，不在本文档中写死。但是架构原则已经确定：

1. 蓝牙和按键共用同一组 `status.task` 请求字段，避免两套逻辑冲突。
2. 蓝牙不直接修改 `task_id` / `start_pose` / `task_running`，只置请求位。
3. 具体请求仍由 `update_task()` 在 20ms 控制周期中统一消费。
4. 最终命令格式沿用 `main.c` 里 `pid_tune()` 的风格：以 `C` 开头，以换行结尾。

也就是说，最终形式应该类似：

```text
C... \r\n
```

或：

```text
C... \n
```

这里的 `...` 是后续再定的命令体。命令体可以表示“选第几问”“切换 AB/AD 发车点”“武装”“启动”“停车”等语义，但无论命令体怎么设计，解析后都应该落到下面这些统一请求接口上：

| 语义 | 解析后的动作 |
|---|---|
| 选择指定任务 | `requested_task_id = 目标任务; task_select_request = 1` |
| 切换到下一个任务 | 先算出下一个任务，写入 `requested_task_id`，再置 `task_select_request = 1` |
| 切换发车模式 | `pose_switch_request = 1` |
| 请求进入/启动当前任务 | `start_request = 1` |
| 停车 | 置 `stop_request = 1` |

这样后续即使蓝牙协议改几次，任务调度层也不用跟着改，只需要改蓝牙解析层。

## 九、LED 与蜂鸣器反馈

声光提示以 3 个 LED 的常亮编码为主，蜂鸣器只做少数关键动作确认。

三个 0/1 从左到右分别表示：

```text
板载 LED, LED1, LED2
```

任务状态显示规则：

| 状态 | LED 编码 |
|---|---|
| TASK1 | `1 0 1` |
| TASK2，AB 发车 | `1 1 1` |
| TASK2，AD 发车 | `0 1 1` |
| TASK3，AB 发车 | `1 0 0` |
| TASK3，AD 发车 | `0 0 0` |
| TASK4 | `1 1 0` |

也就是说，LED 常亮状态由 `task_id + start_pose` 决定。每次切换任务或切换发车姿态后，都应该刷新一次 LED 显示。

蜂鸣器规则：

| 事件 | 蜂鸣器反馈 |
|---|---|
| PD2 短按请求进入当前任务入口 | 短响一次 |
| PB11 长按切换 TASK2/TASK3 发车姿态 | 常响一段时间 |

蜂鸣器不要用阻塞延时。推荐用系统时间做非阻塞关闭：

```text
触发蜂鸣器:
  status.device.buzzer.on = 1
  buzzer_off_time = status.T + 持续时间

周期检查:
  if (status.device.buzzer.on && status.T >= buzzer_off_time)
      status.device.buzzer.on = 0
```

这里的 `status.T` 指工程里已有的系统时间字段；如果实际字段名是 `status.state.time`，就用实际字段名。核心原则是：蜂鸣器只是被置为“响到某个截止时间”，控制周期继续正常跑，不使用 `HAL_Delay()`，也不需要 `PERIODIC` 宏。

## 十、update_task() 任务调度入口

`update_task()` 每 20ms 在 `update_status()` 中调用。

推荐调用顺序：

```text
update_status():
  1. 读取灰度、轮速、陀螺仪等传感器
  2. driver_button() 处理按钮事件，按钮事件只置 status.task 请求位
  3. 解析蓝牙/串口命令，蓝牙命令只置 status.task 请求位
  4. update_task(&status)
  5. 根据 status.state.motion 执行底层控制
       FIND_LINE  -> follow_line()
       KEEP_ANGLE -> keep_angle()
       STOP       -> 目标速度清零
  6. driver_LED / driver_BUZZER / driver_wheel
```

`update_task()` 不负责实时扫描按键。按钮和蓝牙事件发生时已经直接置好了 request，`update_task()` 只负责消费这些 request。

## 十一、update_task() 内部逻辑

推荐顺序：

```text
update_task(status):

  // 1. 远程停车请求优先级最高
  if (status.task.stop_request) {
      task_stop(status);
      return;
  }

  // 2. 未运行任务时，先处理任务选择/发车模式请求
  if (!status.task.task_running && !status.task.armed) {
      if (status.task.task_select_request) {
          task_select(status, status.task.requested_task_id);
          status.task.task_select_request = 0;
      }

      if (status.task.pose_switch_request) {
          task_switch_start_pose(status);
          status.task.pose_switch_request = 0;
      }
  }

  // 3. 处理进入任务入口请求
  if (status.task.start_request) {
      if (!status.task.task_running && !status.task.armed) {
          status.task.armed = 1;
          task_start(status);
      }
      status.task.start_request = 0;
  }

  // 4. 没有进入任务入口则不推进题目状态机
  if (!status.task.armed) {
      return;
  }

  // 5. 根据 task_id 分发
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

任务选择请求只建议在 `task_running == 0` 且 `armed == 0` 时生效。任务已经进入入口或正在运行时，普通 PB11/PD2 操作和蓝牙选题命令都应忽略，只有 `stop_request` 始终有效。

推荐把具体切换逻辑封成几个小函数：

```text
task_select(status, id):
  如果 id 合法，则 task_id = id
  如果 id 是 Q1/Q4，start_pose 强制 START_AB

button_pb11_short(status):
  先计算 next_id，让 task_id 在 1 -> 2 -> 3 -> 4 -> 1 之间轮换
  requested_task_id = next_id
  task_select_request = 1

task_switch_start_pose(status):
  只有 task_id 是 Q2 或 Q3 时，切换 START_AB/START_AD
  Q1/Q4 下收到该请求，可以忽略或蜂鸣器提示无效
```

## 十二、任务启动 task_start()

每次进入任务入口都必须统一初始化，避免上一轮状态污染下一轮。

```text
task_start(status):
  start_request = 0
  stop_request = 0
  不设置 task_running

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
  armed = 0
  state.motion = STOP
  state.base_speed = 0
  左右轮 tar_speed = 0
  完成声光提示
```

`task_start()` 只做入口初始化，不代表任务已经真正运行。`task_running = 1` 应该由对应的 `task_xxx_update()` 在确认任务真的开始执行时设置。

`armed` 表示“已经进入任务入口”，所以任务正常完成后建议清 0。这样下一次必须重新按 PD2 或发送蓝牙启动命令，避免旧入口状态残留。

## 十四、远程停车 task_stop()

远程停车用于蓝牙停车命令，具体命令字符串待定。语义应尽量接近现有 `main.c` 里 `Cz` 指令的停车效果，但要额外阻止任务状态机在下一周期继续推进。

```text
task_stop(status):
  task_running = 0
  armed = 0
  start_request = 0
  stop_request = 0
  state.motion = STOP
  state.base_speed = 0
  左右轮 tar_speed = 0
```

也就是：

```c
status.task.task_running = 0;
status.task.armed = 0;
status.task.start_request = 0;
status.task.stop_request = 0;
status.state.motion = STOP;
status.state.base_speed = 0;
status.motor.wheel[0].tar_speed = 0;
status.motor.wheel[1].tar_speed = 0;
```

`task_stop()` 不清 `task_id`，不清 `start_pose`，不清 `race_phase`，不重置 PID，也不清路口计数。它会清 `armed`，因为远程停车意味着退出当前任务入口，避免下一周期继续分发题内状态机。

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

## 十六、题内小状态机约定

每一道题都应该有自己的 `race_phase` 小状态机。`update_task()` 只负责根据 `task_id` 分发，真正的阶段判定和阶段切换放在对应的 `task_xxx_update()` 里。

```text
task_basic_1_update(status):
  switch (status.task.race_phase):
    case Q1_START_A_TURN:
      执行起点 A 特化逻辑
      如果确认任务已经真正开始执行:
        status.task.task_running = 1
      如果完成:
        task_set_phase(status, Q1_SIDE_AD)
      break

    case Q1_SIDE_AD:
      设置 motion = FIND_LINE
      如果检测到 D 点有效路口:
        task_set_phase(status, Q1_TURN_D)
      break

    case Q1_TURN_D:
      执行陀螺仪转弯 + 低速找线
      如果转弯完成且重新找到线:
        task_set_phase(status, Q1_SIDE_DC)
      break
```

每个 `race_phase` 里面只关心三件事：

```text
阶段动作：当前阶段让车怎么动
阶段判据：什么条件说明本阶段完成
阶段切换：完成后进入哪个阶段
```

推荐所有阶段切换都通过一个统一入口：

```text
task_set_phase(status, next_phase):
  status.task.race_phase = next_phase
  status.task.phase_start_time = status.state.time
  status.task.phase_mileage = 0
  清本阶段专用临时计数/连续帧计数
```

这样可以避免阶段切换时忘记清时间、里程或连续帧计数。

路口检测模块只负责产生事件，不能自己决定任务含义。同样一个 `LeftRoad`，在不同阶段含义完全不同：

```text
Q1_SIDE_AD:
  LeftRoad 可能表示到达 D 点，需要准备转弯

Q1_BA_FINAL:
  LeftRoad 可能表示回到 A 点附近，需要停车兜底

Q1_START_A_TURN:
  起点 A 本来就在左侧，普通 LeftRoad 应该被特化逻辑忽略
```

这也是 `cross_cnt` 应该放进 `TASK` 的原因：路口计数不是底层传感器看到黑线就加，而是题内状态机确认“这个路口在当前阶段有效”之后才加。

## 十七、落地原则

1. 按钮和蓝牙事件发生时，统一置 `status.task` 中的请求位。
2. 不需要 `task_input()` 实时扫描按钮。
3. `update_task()` 每 20ms 消费 request，启动、停止、推进题目状态机。
4. `task_running` 决定任务是否推进，不能用 `motion != STOP` 代替。
5. `race_phase` 必须独立存在，不能只靠 `cross_cnt` 推导。
6. `motion` 只表示底层运动方式，不表示第几问。
7. 每次进入任务入口必须统一清理路口计数、PID、里程、阶段时间等状态。
8. 蓝牙和按钮共用同一套状态字段，避免双系统冲突。
