# 小车控制工程架构

本文档描述当前工程的推荐架构。核心思想是：整车状态统一挂在全局 `STATUS status` 树上；比赛题目流程统一由 `TASK` 结构体管理；底层 `motion` 只负责执行运动方式，不负责理解“现在是第几问、第几个路口、该不该停车”。

一句话版本：

```text
输入事件/传感器
  -> status 状态树
  -> update_task() 推进比赛任务状态机
  -> status.state.motion 选择底层运动模式
  -> driver_xxx() 驱动硬件
```

---

## 一、工程分层

```text
Core/
  CubeMX 生成的 HAL 初始化代码，例如 MX_GPIO_Init()、MX_TIMx_Init()

Drivers/
  STM32G4 HAL/CMSIS 标准库

User/
  Status/
    整车状态树、TASK 调度、运动控制入口
  Sensor/
    灰度、陀螺仪等传感器驱动与数据解析
  Motor/
    电机、舵机等执行器驱动
  Device/
    按键、LED、蜂鸣器等板载外设
  It/
    定时器、串口等中断服务
  Tool/
    PID、数学宏、日志、数组工具等通用组件
  Middle/
    跨平台兼容层
```

硬件平台：STM32G474，系统主频约 150 MHz。

---

## 二、状态树 STATUS

工程里只有一个全局状态树实例：

```c
STATUS status;
```

它声明在 `User/Status/status.h`，定义在 `User/Status/status.c`。所有模块都应该通过 `status.xxx.xxx` 读写状态，避免再散落新的全局变量。

当前顶层结构：

```text
status
  ├── state     车辆运行状态、底层运动控制参数、路口观测缓存、状态层 PID
  ├── sensor    陀螺仪、数字灰度、模拟灰度等传感器数据
  ├── motor     轮子、舵机等执行器状态
  ├── device    LED、按键、蜂鸣器等外设状态
  └── task      比赛任务状态机
```

### 2.1 state：底层运动状态

`state` 描述“车现在怎么动”，不描述“现在跑第几问”。

关键字段：

```text
state.T                 控制周期，当前 update_status() 每 20ms 调用一次
state.time              系统运行时间，由 TIM5 1ms 中断递增
state.motion            STOP / FIND_LINE / KEEP_ANGLE / MOTOR_TEST
state.initial_angle     进入任务或初始化时记录的参考 yaw
state.cur_angle         当前 yaw
state.tar_angle         目标相对角度
state.base_speed        底层运动基础速度
state.road_determine    路口观测缓存
state.status_pid        follow_line_pid / keep_angle_pid
```

`motion` 是执行层枚举：

```c
typedef enum MOTION_STATION {
  STOP,
  KEEP_ANGLE,
  FIND_LINE,
  MOTOR_TEST,
} MOTION_STATION;
```

注意：`motion` 不能代替 `task_id` 或 `race_phase`。例如第一问和第二问都可能处于 `FIND_LINE`，但它们的任务含义完全不同。

### 2.2 sensor：传感器状态

```text
sensor.gy901        GY901 陀螺仪
sensor.gw_8bit      8 路数字灰度
sensor.gw_analogue  8 路模拟灰度
```

模拟灰度当前承担主要循迹任务：

```text
gw_analogue.channel[8]              原始 ADC 值
gw_analogue.correction_data_w[8]    白底校准数据
gw_analogue.correction_data_b[8]    黑底校准数据
gw_analogue.digital_8bit            二值化后的 8 位灰度
gw_analogue.diff                    归一化线偏差
```

### 2.3 motor：执行器状态

```text
motor.wheel[4]   直流电机
motor.servo[2]   舵机
```

当前主运动控制只使用左右两个轮子：

```text
wheel[0].cur_speed   左轮当前编码器速度
wheel[0].tar_speed   左轮目标速度
wheel[0].trust       左轮 PWM 推力
wheel[0].dir         左轮方向校准

wheel[1].cur_speed   右轮当前编码器速度
wheel[1].tar_speed   右轮目标速度
wheel[1].trust       右轮 PWM 推力
wheel[1].dir         右轮方向校准
```

`driver_wheel()` 内部根据 `tar_speed - cur_speed` 做速度环 PID，最后输出 TB6612 的方向 GPIO 和 PWM。

### 2.4 device：板载外设

```text
device.led_on_board
device.led1
device.led2
device.button_D2
device.button_B11
device.buzzer
```

按键事件由 `driver_button()` 识别，再交给 `server_button()` 写入 `status.task` 的请求位。

蜂鸣器采用非阻塞自动关闭：

```text
触发蜂鸣器:
  buzzer.on = 1
  buzzer.off_time = status.state.time + 持续时间

周期检查:
  if (buzzer.on && status.state.time >= buzzer.off_time)
      buzzer.on = 0
```

---

## 三、TASK 任务层

`TASK` 是现在比赛架构的核心。它描述“当前选择第几问、是否进入任务入口、题内跑到哪一步”。

```c
typedef struct TASK {
  uint8_t task_id;
  uint8_t start_pose;
  uint8_t race_phase;
  uint8_t cross_cnt;

  uint8_t armed;
  uint8_t task_running;

  uint8_t task_select_request;
  uint8_t requested_task_id;
  uint8_t pose_switch_request;

  uint8_t start_request;
  uint8_t stop_request;

  uint32_t phase_start_time;
  float phase_mileage;
} TASK;
```

### 3.1 task_id

```c
typedef enum TASK_ID {
  TASK_BASIC_1 = 1,
  TASK_BASIC_2 = 2,
  TASK_ADV_1   = 3,
  TASK_ADV_2   = 4,
} TASK_ID;
```

含义：

```text
TASK_BASIC_1  基础部分第一问
TASK_BASIC_2  基础部分第二问
TASK_ADV_1    发挥部分第一问
TASK_ADV_2    发挥部分第二问
```

### 3.2 start_pose

```c
typedef enum START_POSE {
  START_AB = 0,
  START_AD = 1,
} START_POSE;
```

规则：

```text
TASK_BASIC_1  固定 START_AB
TASK_BASIC_2  支持 START_AB / START_AD
TASK_ADV_1    支持 START_AB / START_AD
TASK_ADV_2    固定 START_AB
```

`task_select()` 中，选择第一问或第四问时会强制回到 `START_AB`。第二问和第三问保留当前 `start_pose`，允许通过长按 PB11 切换。

### 3.3 armed、start_request、task_running

这三个字段必须分清楚：

```text
start_request
  外部输入提出“我要进入当前任务入口”的一次性请求。
  按键和蓝牙只能置这个请求，不能直接启动任务。

armed
  update_task() 消费 start_request 后置 1。
  表示当前任务入口已经放行，允许进入对应 task_xxx_update()。

task_running
  只有具体 task_xxx_update() 在确认任务真的开始运行后才能置 1。
  它不能由按键直接置 1，也不应该由 task_start() 直接置 1。
```

推荐生命周期：

```text
上电:
  armed = 0
  task_running = 0
  motion = STOP

PD2 短按或蓝牙启动命令:
  start_request = 1

update_task() 消费 start_request:
  armed = 1
  task_start(status)

题内状态机确认任务真正开始:
  task_running = 1

任务完成:
  task_finish(status)
  armed = 0
  task_running = 0
  motion = STOP
```

这个设计的好处是：按下启动键不等于假启动。只有任务入口真正跑通，并且题内状态机确认开始执行后，`task_running` 才代表“任务真的在跑”。

### 3.4 race_phase

`race_phase` 是每道题内部的小状态机。

第一问当前已经有阶段枚举：

```c
typedef enum Q1_RACE_PHASE {
  Q1_START_A_TURN,
  Q1_SIDE_AD,
  Q1_TURN_D,
  Q1_SIDE_DC,
  Q1_TURN_C,
  Q1_SIDE_CB,
  Q1_TURN_B,
  Q1_BA_FINAL,
} Q1_RACE_PHASE;
```

推荐理解：

```text
task_id      表示现在选的是第几问
race_phase   表示这道题内部走到哪一步
motion       表示这一小步让底层怎么运动
```

例如：

```text
task_id = TASK_BASIC_1
race_phase = Q1_SIDE_AD
motion = FIND_LINE
```

含义是：当前跑第一问，题内处于 AD 边，底层正在循迹。

### 3.5 cross_cnt

`status.task.cross_cnt` 表示“当前任务状态机已经确认并消费过几个有效路口/角点”。

它不是传感器原始量，也不应该由 `road.c` 自己随便加。路口检测模块只能说“我好像看到了某种路口事件”，是否有效、是否计数、是否触发阶段切换，必须由具体任务状态机决定。

短期为了兼容旧代码，`road.c` 里仍有 legacy/global `cross_cnt`。长期目标是把任务流程计数统一迁移到 `status.task.cross_cnt`。

---

## 四、update_status() 控制周期

`update_status()` 每 20ms 调用一次，是整车控制主循环。当前推荐顺序如下：

```text
1. 读取传感器和轮速
   get_gw_raw_data()
   get_wheel_speed()
   get_gyr_raw_data()
   get_gyr_value()

2. 处理按键事件
   driver_button(button_D2)
   driver_button(button_B11)
   按键事件只写 status.task 的请求位

3. 推进任务层
   update_task(status)
   消费 start/stop/select/pose 请求
   根据 task_id 分发到 task_xxx_update()
   由题内状态机设置 motion/base_speed/tar_angle 等

4. 执行底层 motion
   FIND_LINE   -> follow_line(status)
   KEEP_ANGLE  -> keep_angle(status)
   STOP        -> wheel tar_speed = 0
   MOTOR_TEST  -> 固定测试速度

5. 驱动硬件
   driver_LED()
   driver_servo()
   蜂鸣器超时自动关闭
   driver_BUZZER()
   driver_wheel()
```

这条顺序很关键：按钮先产生请求，`update_task()` 再消费请求，最后底层 `motion` 才根据任务层给出的状态真正算轮速。

---

## 五、update_task() 调度逻辑

`update_task()` 是任务层唯一入口。它不扫描按键，只消费已经被按键或蓝牙置好的请求位。

推荐逻辑：

```text
update_task(status):

  1. stop_request 优先级最高
     if stop_request:
       task_stop(status)
       return

  2. 空闲状态下处理任务选择和发车姿态切换
     条件: !task_running && !armed

     if task_select_request:
       task_select(status, requested_task_id)
       task_select_request = 0

     if pose_switch_request:
       如果当前是 TASK_BASIC_2 或 TASK_ADV_1:
         START_AB <-> START_AD
         update_task_led(status)
       pose_switch_request = 0

  3. 处理进入任务入口请求
     if start_request:
       if !task_running && !armed:
         armed = 1
         task_start(status)
       start_request = 0

  4. 没有 armed 就不推进题内状态机
     if !armed:
       return

  5. 根据 task_id 分发
     TASK_BASIC_1 -> task_basic_1_update(status)
     TASK_BASIC_2 -> task_basic_2_update(status)
     TASK_ADV_1   -> task_adv_1_update(status)
     TASK_ADV_2   -> task_adv_2_update(status)
```

当前 `task_xxx_update()` 仍是占位骨架，后续每道题的真正流程都应该写在对应函数里。

---

## 六、任务入口与退出

### 6.1 init_task()

上电默认状态：

```text
task_id = TASK_BASIC_1
start_pose = START_AB
race_phase = 0
cross_cnt = 0

armed = 0
task_running = 0

所有 request = 0
phase_start_time = 0
phase_mileage = 0
```

上电必须 `armed = 0`，否则车可能直接进入任务。

### 6.2 task_start()

`task_start()` 只做“进入任务入口前的统一初始化”，不代表任务已经真的运行。

它负责清理：

```text
start_request / stop_request
status.task.cross_cnt
legacy cross_cnt / left_cnt / cross_delay
road_buf / road_determine
phase_mileage
follow_line_pid / keep_angle_pid 的历史误差和积分
initial_angle
左右轮 tar_speed
motion / base_speed
```

然后根据当前 `task_id` 和 `start_pose` 设置初始 `race_phase`，并记录：

```text
phase_start_time = status.state.time
```

注意：`task_start()` 不设置 `task_running = 1`。

### 6.3 task_finish()

正常完成任务时调用：

```text
task_running = 0
armed = 0
start_request = 0
stop_request = 0
task_select_request = 0
pose_switch_request = 0
motion = STOP
base_speed = 0
左右轮 tar_speed = 0
蜂鸣器短响
```

完成后必须重新按 PD2 或发蓝牙启动命令，才能再次进入任务入口。

### 6.4 task_stop()

远程停车/急停语义，效果接近旧 `Cz` 停车命令：

```text
task_running = 0
armed = 0
start_request = 0
stop_request = 0
motion = STOP
base_speed = 0
左右轮 tar_speed = 0
```

`task_stop()` 不清 `task_id`，不清 `start_pose`。这样停车后仍然保留当前选题和发车模式。

---

## 七、按键与蓝牙输入

按键和蓝牙都只写请求位，不直接修改任务核心状态。这样以后蓝牙命令改格式，也不会影响任务调度层。

### 7.1 PB11

```text
PB11 短按:
  条件: task_running == 0
  动作:
    next = task_id + 1
    如果 next > 4，则 next = 1
    requested_task_id = next
    task_select_request = 1

PB11 长按:
  条件:
    task_running == 0
    当前 task_id 是 TASK_BASIC_2 或 TASK_ADV_1
  动作:
    pose_switch_request = 1
    蜂鸣器长响约 1000ms
```

### 7.2 PD2

```text
PD2 短按:
  条件:
    armed == 0
    task_running == 0
  动作:
    start_request = 1
    蜂鸣器短响约 200ms

PD2 长按:
  动作:
    correct_gw_analogue()
    进入灰度校准
```

### 7.3 短按和长按互斥

`BUTTON` 结构体中使用 `long_triggered` 标志位：

```text
BUTTON_DOWN:
  long_triggered = 0

BUTTON_LONG:
  long_triggered = 1
  执行长按动作

BUTTON_UP:
  if long_triggered == 0:
    执行短按动作
```

这样长按释放时不会再误触发短按。

### 7.4 蓝牙接口原则

具体蓝牙命令格式仍可后续确定，但最终建议沿用 `main.c` 里 `pid_tune()` 的风格：

```text
C... \r\n
```

解析后统一落到这些请求接口：

```text
选择任务:
  requested_task_id = 目标任务
  task_select_request = 1

切换 AB/AD:
  pose_switch_request = 1

请求进入当前任务:
  start_request = 1

远程停车:
  stop_request = 1
```

---

## 八、LED 与蜂鸣器反馈

三个 LED 的常亮编码表示当前选题和发车姿态。

顺序为：

```text
板载 LED, LED1, LED2
```

编码表：

| 状态 | LED 编码 |
|---|---|
| TASK1 | `1 0 1` |
| TASK2 AB 发车 | `1 1 1` |
| TASK2 AD 发车 | `0 1 1` |
| TASK3 AB 发车 | `1 0 0` |
| TASK3 AD 发车 | `0 0 0` |
| TASK4 | `1 1 0` |

LED 由 `update_task_led(status)` 统一刷新。切换任务或切换发车姿态后都应该调用它。

蜂鸣器规则：

```text
PD2 短按请求进入任务入口:
  短响约 200ms

PB11 长按切换 TASK2/TASK3 发车姿态:
  长响约 1000ms

task_finish():
  短响约 200ms
```

蜂鸣器不能使用 `HAL_Delay()` 阻塞控制周期。

---

## 九、运动控制层

### 9.1 FIND_LINE

`follow_line(status)` 当前负责：

```text
读取模拟灰度二值化结果
计算模拟灰度 diff
调用 get_road_type() 更新路口观测
根据 diff 计算左右轮差速
```

需要特别注意：旧代码里 `follow_line()` 仍混有路口转向逻辑，例如 `LeftRoad` 直接原地转、`cross_cnt == 4` 特化右转等。这些属于旧赛题残留，长期应该迁移到 `task_xxx_update()`。

目标架构中：

```text
road.c / get_road_type()
  只负责告诉任务层“我观察到了什么路口事件”

task_xxx_update()
  决定这个事件在当前 race_phase 是否有效
  决定是否计数、转弯、停车、进入下一阶段

follow_line()
  只负责在需要循迹时输出左右轮目标速度
```

### 9.2 KEEP_ANGLE

`keep_angle(status)` 根据陀螺仪 yaw 做闭环保角：

```text
target = initial_angle + tar_angle
diff_angle = target - cur_angle
角度误差处理 360° 环绕
keep_angle_pid 计算修正量
左轮 = base_speed + diff
右轮 = base_speed - diff
```

后续直角转弯建议由任务状态机设置 `tar_angle`，再用 `KEEP_ANGLE` 或专门的转弯阶段完成，而不是在路口检测函数里开环写死轮速。

### 9.3 STOP

`STOP` 是底层停车模式：

```text
wheel[0].tar_speed = 0
wheel[1].tar_speed = 0
```

但“为什么停车”不由 `STOP` 判断。是完成任务、远程停车、阶段等待，还是保护逻辑，都应该由任务状态机决定。

---

## 十、路口识别层的边界

当前 `road.c` 里仍有旧架构残留：

```text
global cross_cnt
global left_cnt
global cross_delay
serve_road() 里直接改 base_speed / PID / 计数
follow_line() 里直接根据路型转弯
```

这些逻辑和新 `TASK` 架构不兼容。新边界应该是：

```text
灰度数据
  -> road.c 判断当前像 Straight / LeftRoad / RightRoad / CrossRoad
  -> 只产生“观测结果”
  -> task_xxx_update() 结合 race_phase、里程、时间、start_pose 判断是否采用
```

同一个 `LeftRoad` 在不同阶段含义不同：

```text
Q1_SIDE_AD:
  可能表示到达 D 点，准备左转

Q1_BA_FINAL:
  可能表示回到 A 点附近，应该停车

Q1_START_A_TURN:
  起点本来就靠近 AD，普通 LeftRoad 可能需要忽略
```

所以路口计数、阶段切换、转弯动作都必须放到任务状态机里。

---

## 十一、题内状态机写法

每道题都应该有自己的 `task_xxx_update()`。

一个阶段只做三件事：

```text
1. 阶段动作
   当前阶段让车怎么动，例如 FIND_LINE、KEEP_ANGLE、STOP。

2. 阶段判据
   什么条件说明本阶段完成，例如检测到有效路口、角度到位、里程达到、超时保护。

3. 阶段切换
   完成后进入哪个 race_phase。
```

推荐统一封装阶段切换：

```text
task_set_phase(status, next_phase):
  race_phase = next_phase
  phase_start_time = status.state.time
  phase_mileage = 0
  清理本阶段临时计数
```

第一问示意：

```text
Q1_START_A_TURN
  起点 A 特化处理，避免刚上电就把 AD 误当普通左路口
  完成后 -> Q1_SIDE_AD

Q1_SIDE_AD
  FIND_LINE
  检测到 D 点有效路口 -> Q1_TURN_D

Q1_TURN_D
  陀螺仪闭环左转 + 低速找线
  完成后 -> Q1_SIDE_DC

Q1_SIDE_DC
  FIND_LINE
  检测到 C 点有效路口 -> Q1_TURN_C

Q1_TURN_C
  完成后 -> Q1_SIDE_CB

Q1_SIDE_CB
  FIND_LINE
  检测到 B 点有效路口 -> Q1_TURN_B

Q1_TURN_B
  完成后 -> Q1_BA_FINAL

Q1_BA_FINAL
  BA 边循迹
  编码器里程为主，A 点灰度路口为辅
  到停车条件 -> task_finish()
```

第二问示意：

```text
根据 start_pose 决定方向:
  START_AB: 从 AB 出发，跑 3/4 圈到 B，掉头返回
  START_AD: 从 AD 出发，跑 3/4 圈到 D，掉头返回

BC 干扰 A4 区域:
  灰度横线可能制造假路口
  需要由 race_phase + 里程门限判断是否采用路口事件
  不能让 road.c 的全局 cross_cnt 自动决定任务进度
```

---

## 十二、中断与时间

`User/It/timer_it.c` 中 TIM5 是系统心跳：

```text
TIM5 每 1ms 中断:
  status.state.time += 1

每 20ms:
  update_status(&status)
```

因此：

```text
LONG_PRESS_CNT = 50
真实长按时间约为 50 * 20ms = 1000ms
```

需要非阻塞定时的地方优先使用 `status.state.time`，不要在控制周期里使用阻塞延时。

---

## 十三、串口资源

| 串口 | 当前用途 |
|---|---|
| USART1 | WiFi/蓝牙透传、PID 调参、日志打印 |
| USART2 | K230/视觉模块、步进电机、绝对编码器预留 |
| USART3 | 步进电机/MaixCam 预留 |
| UART4 | VOFA 预留 |

蓝牙命令最终应该和按键走同一套 `status.task` 请求接口。

---

## 十四、定时器资源

| 定时器 | 用途 |
|---|---|
| TIM1 | 编码器模式 |
| TIM2 | 编码器模式 |
| TIM3 | 编码器模式 |
| TIM4 | 编码器模式 |
| TIM5 | 系统心跳，驱动 20ms 控制周期 |
| TIM6 | CCD 驱动预留 |
| TIM8 | 电机 PWM，TB6612 驱动 |
| TIM15 | 舵机 PWM |

---

## 十五、添加新模块的原则

新增模块时按这个节奏接入：

```text
1. 在对应 User/Sensor、User/Motor 或 User/Device 下定义结构体
2. 写 init_xxx()
3. 写 get_xxx_raw_data() 或 driver_xxx()
4. 把结构体挂到 STATUS 对应分支
5. 在 init_status() / update_status() 中接入
```

控制参数和运行状态优先挂到 `status` 树，不要随手新增孤立全局变量。

---

## 十六、当前最重要的架构原则

1. `TASK` 管比赛流程，`motion` 管底层动作。
2. 按键和蓝牙只置请求位，不直接改 `task_id`、`armed`、`task_running`。
3. `update_task()` 是任务请求的唯一消费者。
4. `task_start()` 只做入口初始化，不设置 `task_running`。
5. `task_running` 只能由具体题目的 `task_xxx_update()` 在确认真正运行后置 1。
6. `road.c` 只能作为路口观测层，不能接管转弯、停车、任务计数。
7. `status.task.cross_cnt` 才是任务有效路口计数，旧全局 `cross_cnt` 只能作为过渡兼容。
8. LED 常亮编码由 `task_id + start_pose` 决定，蜂鸣器必须非阻塞。
9. 第一问、第二问这种比赛逻辑必须写成 `race_phase` 小状态机。
10. 上电默认 `armed = 0`、`task_running = 0`、`motion = STOP`，保证车不会自己跑。

---

## 十七、推荐阅读顺序

1. `User/ARCHITECTURE.md`：先理解本文档。
2. `User/Status/status.h`：看完整 `STATUS` 类型。
3. `User/Status/Defect.h`：看 `TASK` 字段和任务枚举。
4. `User/Status/Defect.c`：看 `init_task()`、`update_task()`、`task_start()`、`task_finish()`。
5. `User/Device/button.c`：看按键如何写任务请求位。
6. `User/Status/status.c`：看 `update_status()` 的 20ms 调用链。
7. `User/Status/road.c`：看当前旧路口识别残留，后续逐步清理。
