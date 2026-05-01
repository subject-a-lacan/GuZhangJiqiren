# 小车工程结构说明

这份文档写给刚接触本工程的队友。目标不是讲设计思想，而是让你能快速回答三个问题：

1. 程序从哪里开始跑？
2. 整车状态都存在哪里？
3. 一次 20ms 控制周期里，代码怎么从“读传感器”走到“电机真正转起来”？

先记住一句话：

```text
所有数据挂在全局 status 树上；
update_status() 每 20ms 跑一次；
update_task() 决定当前任务要干什么；
motion 层算出轮子目标速度；
driver_xxx() 把目标值真正写进硬件。
```

---

## 1. 先看工程目录

```text
Core/
  CubeMX 生成的初始化代码。
  例如 GPIO、TIM、USART、ADC、I2C 的 MX_xxx_Init() 都在这里。

Drivers/
  STM32 HAL 库和 CMSIS。
  平时基本不用改。

User/
  Status/
    整车控制核心。
    status.h/status.c 定义全局状态树和 update_status()。
    Defect.h/Defect.c 定义比赛 TASK 状态机。
    road.h/road.c 做路口识别相关逻辑。

  Sensor/
    传感器。
    例如 GY901 陀螺仪、灰度传感器。

  Motor/
    执行器。
    例如 wheel.c 控制 TB6612 + 电机，servo.c 控制舵机。

  Device/
    板载外设。
    例如 button.c、led.c、buzzer.c。

  It/
    中断服务。
    timer_it.c 负责定时调用 update_status()。

  Tool/
    通用工具。
    例如 PID、数学宏、串口日志。
```

新队友最应该先看这几个文件：

```text
User/Status/status.h      看 status 这棵状态树长什么样
User/Status/status.c      看 init_status() 和 update_status()
User/Status/Defect.h      看 TASK 结构体
User/Status/Defect.c      看 update_task() 怎么调度任务
User/Device/button.c      看按键怎么给 TASK 发请求
User/Motor/wheel.c        看轮子目标速度怎么变成 PWM
User/It/timer_it.c        看 20ms 控制周期怎么触发
```

---

## 2. 程序是怎么跑起来的

主入口在 `Core/Src/main.c`。大致流程是：

```text
main()
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO_Init()
  -> MX_TIMx_Init()
  -> MX_USARTx_UART_Init()
  -> init_status(&status, 20)
  -> after_init_state()
  -> 启动定时器 / PWM / 编码器 / 串口接收等
  -> while (1)
```

真正的控制逻辑不主要写在 `while (1)` 里，而是靠 TIM5 定时器周期性触发。

当前控制节拍：

```text
TIM5 每 1ms 进一次中断
  -> status.state.time += 1

每累计到 20ms
  -> update_status(&status)
```

所以调车时要有一个概念：大部分控制代码每 20ms 执行一次，不是一直连续执行。

---

## 3. 全局状态树 STATUS

工程里最重要的变量是：

```c
STATUS status;
```

它定义在 `User/Status/status.c`，声明在 `User/Status/status.h`。

整车几乎所有状态都挂在这棵树上。不要随便新建全局变量保存运行状态，优先考虑挂进 `status`。

当前结构：

```text
status
  ├── state     小车当前运行状态和控制参数
  ├── sensor    传感器数据
  ├── motor     电机、舵机等执行器数据
  ├── device    LED、按键、蜂鸣器等外设数据
  └── task      比赛任务状态机数据
```

可以把它理解成整车的“内存面板”：

```text
传感器读到什么          -> 写进 status.sensor
任务当前跑到哪一步       -> 写进 status.task
底层现在应该怎么运动     -> 写进 status.state
轮子目标速度是多少       -> 写进 status.motor
LED / 蜂鸣器 / 按键状态  -> 写进 status.device
```

---

## 4. STATUS 树的详细结构

下面这棵树不是每个字段都要立刻背下来，但你要知道去哪里找。

### 4.1 status.state

`state` 表示小车底层运行状态。

```text
status.state
  ├── T                 控制周期，当前是 20ms
  ├── time              系统运行时间，TIM5 每 1ms 加 1
  ├── motion            当前底层运动模式
  ├── initial_angle     参考 yaw 角
  ├── cur_angle         当前 yaw 角
  ├── tar_angle         目标相对角度
  ├── base_speed        底层基础速度
  ├── road_determine    路口识别缓存
  ├── gw_8bit           预留灰度状态
  └── status_pid
      ├── follow_line_pid
      └── keep_angle_pid
```

`motion` 的取值：

```text
STOP        停车，左右轮目标速度清零
FIND_LINE   循迹，调用 follow_line()
KEEP_ANGLE  保角，调用 keep_angle()
MOTOR_TEST  电机测试模式
```

重点：`motion` 只表示“车怎么动”，不表示“现在是第几问”。

### 4.2 status.sensor

`sensor` 保存传感器读数。

```text
status.sensor
  ├── gy901
  │   ├── data_buf[24]          陀螺仪原始数据
  │   ├── device_addr           I2C 地址
  │   └── data_start_addr       数据寄存器起始地址
  │
  ├── gw_8bit
  │   ├── data_buf              8 位数字灰度原始位图
  │   ├── gw_bit_weight[8]      权重
  │   ├── cross                 当前识别道路类型
  │   └── gw_diff               数字灰度偏差
  │
  └── gw_analogue
      ├── channel[8]            8 路 ADC 原始值
      ├── correction_data_w[8]  白底校准数据
      ├── correction_data_b[8]  黑底校准数据
      ├── digital_8bit          二值化后的 8 位灰度
      └── diff                  模拟灰度循迹偏差
```

目前循迹主要看 `gw_analogue.digital_8bit` 和 `gw_analogue.diff`。

### 4.3 status.motor

`motor` 保存电机和舵机状态。

```text
status.motor
  ├── wheel[4]
  │   ├── which       轮子编号
  │   ├── trust       当前 PWM 推力
  │   ├── cur_speed   编码器测得的当前速度
  │   ├── tar_speed   控制层给出的目标速度
  │   ├── dir         电机方向校准
  │   └── wheel_pid   单轮速度 PID
  │
  └── servo[2]
      ├── which
      ├── angle
      └── max_angle
```

当前主要使用：

```text
wheel[0]  左轮
wheel[1]  右轮
```

控制层只需要设置：

```c
status.motor.wheel[0].tar_speed = 左轮目标速度;
status.motor.wheel[1].tar_speed = 右轮目标速度;
```

真正的 PWM 输出由 `driver_wheel()` 完成。

### 4.4 status.device

`device` 保存板载外设状态。

```text
status.device
  ├── led_on_board
  ├── led1
  ├── led2
  ├── button_D2
  ├── button_B11
  └── buzzer
```

LED 和蜂鸣器的使用方式很简单：

```c
status.device.led1.on = 1;       // LED1 亮
status.device.buzzer.on = 1;     // 蜂鸣器响
```

但这只是改状态，真正写 GPIO 要等后面的：

```c
driver_LED(&status->device.led1);
driver_BUZZER(&status->device.buzzer);
```

### 4.5 status.task

`task` 是比赛任务状态机。

```text
status.task
  ├── task_id                当前选择第几问
  ├── start_pose             AB / AD 发车姿态
  ├── race_phase             当前题目内部跑到哪一阶段
  ├── cross_cnt              当前任务确认过的有效路口数
  │
  ├── armed                  是否已经进入任务入口
  ├── task_running           任务是否真的开始运行
  │
  ├── task_select_request    请求切换任务
  ├── requested_task_id      想切到哪个任务
  ├── pose_switch_request    请求切换 AB/AD
  ├── start_request          请求进入当前任务
  ├── stop_request           请求停车
  │
  ├── phase_start_time       当前阶段开始时间
  └── phase_mileage          当前阶段累计里程，预留
```

`task_id`：

```text
1  TASK_BASIC_1
2  TASK_BASIC_2
3  TASK_ADV_1
4  TASK_ADV_2
```

`start_pose`：

```text
START_AB = 0
START_AD = 1
```

---

## 5. 最核心调用链：update_status()

新人最需要看懂 `User/Status/status.c` 里的 `update_status()`。

它每 20ms 执行一次，顺序大概是：

```text
update_status(status)

  1. 读取传感器
     get_gw_raw_data()
     get_wheel_speed()
     get_gyr_raw_data()
     get_gyr_value()

  2. 扫描按键
     driver_button(button_D2)
     driver_button(button_B11)

  3. 更新比赛任务状态机
     update_task(status)

  4. 根据 motion 计算左右轮目标速度
     FIND_LINE   -> follow_line(status)
     KEEP_ANGLE  -> keep_angle(status)
     STOP        -> 左右轮 tar_speed = 0
     MOTOR_TEST  -> 左右轮 tar_speed = 40

  5. 把状态真正写到硬件
     driver_LED()
     driver_servo()
     蜂鸣器超时自动关闭
     driver_BUZZER()
     driver_wheel()
```

这就是整个工程最重要的数据流。

更直观一点：

```text
传感器硬件
  -> get_xxx_raw_data()
  -> status.sensor

按键硬件
  -> driver_button()
  -> server_button()
  -> status.task 的 request 字段

比赛任务
  -> update_task()
  -> status.task.race_phase
  -> status.state.motion / base_speed / tar_angle

底层运动
  -> follow_line() / keep_angle()
  -> status.motor.wheel[x].tar_speed

硬件输出
  -> driver_wheel()
  -> TB6612 + 电机
```

---

## 6. update 层和 driver 层的区别

这是新队友最容易混的地方。

### 6.1 update 层：算“应该是什么”

update 层主要负责读数据、判断状态、计算目标值。

典型函数：

```text
update_status()
update_task()
follow_line()
keep_angle()
get_gw_raw_data()
get_gyr_raw_data()
get_wheel_speed()
```

例子：

```text
follow_line() 不直接输出 PWM。
它只是根据灰度偏差算出左右轮目标速度 tar_speed。
```

### 6.2 driver 层：把目标值写进硬件

driver 层主要负责把 status 里的目标状态变成 GPIO/PWM。

典型函数：

```text
driver_wheel()
driver_LED()
driver_BUZZER()
driver_servo()
driver_button()
```

例子：

```text
driver_wheel() 读取 wheel.tar_speed 和 wheel.cur_speed，
做速度 PID，
算出 wheel.trust，
最后写 TB6612 方向 GPIO 和 TIM8 PWM。
```

所以代码里经常是两步：

```c
status.motor.wheel[0].tar_speed = 100;  // 先改目标值
driver_wheel(&status.motor.wheel[0]);   // 后写硬件
```

---

## 7. 一条完整的电机控制链

以循迹为例：

```text
update_status()
  -> get_gw_raw_data()
     读取模拟灰度 ADC

  -> update_task()
     任务状态机决定当前应该 FIND_LINE
     status.state.motion = FIND_LINE
     status.state.base_speed = 某个速度

  -> follow_line()
     get_gw_analoge_digital_data()
     get_gw_analogue_analogue_diff()
     compute_pid(follow_line_pid, 灰度偏差)
     status.motor.wheel[0].tar_speed = base_speed - diff
     status.motor.wheel[1].tar_speed = base_speed + diff

  -> driver_wheel(wheel[0])
     根据 tar_speed - cur_speed 做速度 PID
     输出左轮 PWM 和方向

  -> driver_wheel(wheel[1])
     根据 tar_speed - cur_speed 做速度 PID
     输出右轮 PWM 和方向
```

所以电机最终为什么这么转，要沿着这条链往上查：

```text
PWM 不对
  看 driver_wheel()

tar_speed 不对
  看 follow_line() / keep_angle() / STOP 分支

motion 不对
  看 update_task() 和当前 race_phase

任务没启动
  看 button.c 有没有置 start_request，update_task() 有没有置 armed
```

---

## 8. TASK 层是干什么的

旧代码里很多逻辑直接写在 `follow_line()` 或 `road.c` 里，例如看到 `LeftRoad` 就立刻原地转弯。

现在的新架构不这样干。

新架构分三层：

```text
TASK 层
  决定“当前第几问、跑到哪一阶段、下一步该干什么”

motion 层
  决定“用循迹、保角、停车还是电机测试”

driver 层
  决定“怎么把目标速度/LED 状态/PWM 写到硬件”
```

比如同样看到 `LeftRoad`：

```text
第一问 AD 边看到 LeftRoad
  可能表示到 D 点，要准备左转

第一问 BA 最后看到 LeftRoad
  可能表示回到 A 点附近，要停车

刚发车时看到 LeftRoad
  可能是起点旁边的 AD，需要忽略
```

所以路口的意义不能由 `road.c` 自己决定，必须由 `task_xxx_update()` 根据 `race_phase` 判断。

---

## 9. TASK 的启动链路

当前按键逻辑在 `User/Device/button.c`。

### 9.1 PB11 短按：切换任务

```text
PB11 释放
  -> driver_button() 识别 BUTTON_UP
  -> server_button()
  -> 计算下一个 task_id
  -> status.task.requested_task_id = next
  -> status.task.task_select_request = 1
```

然后下一个 20ms：

```text
update_status()
  -> update_task()
  -> 发现 task_select_request == 1
  -> task_select(status, requested_task_id)
  -> status.task.task_id 改变
  -> update_task_led(status)
```

### 9.2 PB11 长按：切换 AB/AD 发车

只对第二问和第三问有效：

```text
PB11 长按
  -> pose_switch_request = 1
  -> 蜂鸣器长响约 1000ms

update_task()
  -> 如果当前是 TASK_BASIC_2 或 TASK_ADV_1
  -> start_pose 在 START_AB / START_AD 之间切换
  -> update_task_led(status)
```

### 9.3 PD2 短按：请求进入当前任务

```text
PD2 短按
  -> start_request = 1
  -> 蜂鸣器短响约 200ms

update_task()
  -> 如果当前没有 armed，也没有 task_running
  -> armed = 1
  -> task_start(status)
  -> 根据 task_id 设置 race_phase
```

注意：

```text
PD2 短按不会直接把 task_running 置 1。
task_start() 也不会直接把 task_running 置 1。

task_running 必须由具体 task_xxx_update() 在确认任务真的开始后置 1。
```

这可以防止“按了启动键，但任务内部没跑通，却显示任务已经在跑”的假启动。

---

## 10. TASK 几个关键字段怎么理解

```text
task_id
  当前选的是第几问。

start_pose
  当前发车姿态，AB 或 AD。

race_phase
  当前题目内部的小阶段。
  例如第一问可以分成起点转弯、AD 边、D 点转弯、DC 边等。

cross_cnt
  当前任务确认过的有效路口数。
  它应该由任务状态机更新，不应该由 road.c 自动决定。

start_request
  外部输入提出“我要启动当前任务”的请求。

armed
  update_task() 已经接受启动请求，任务入口已经放行。

task_running
  任务内部确认“真的开始跑了”。

stop_request
  蓝牙或其他紧急输入请求停车。
```

三者关系最容易混：

```text
start_request 是请求
armed 是入口放行
task_running 是任务真的运行
```

---

## 11. LED 和蜂鸣器怎么看状态

三个 LED 的顺序：

```text
板载 LED, LED1, LED2
```

当前编码：

| 状态 | LED |
|---|---|
| TASK1 | `1 0 1` |
| TASK2 AB | `1 1 1` |
| TASK2 AD | `0 1 1` |
| TASK3 AB | `1 0 0` |
| TASK3 AD | `0 0 0` |
| TASK4 | `1 1 0` |

LED 不是在按键函数里直接最终决定的，而是：

```text
按键置 request
  -> update_task() 消费 request
  -> update_task_led()
  -> driver_LED()
```

蜂鸣器：

```text
PD2 短按启动请求      短响约 200ms
PB11 长按切换 AB/AD   长响约 1000ms
task_finish()         短响约 200ms
```

蜂鸣器通过 `off_time` 自动关闭，不用 `HAL_Delay()` 阻塞。

---

## 12. road.c 现在应该怎么看

`road.c` 目前还有旧架构残留。新队友看它时要特别小心。

它里面有一些旧逻辑：

```text
全局 cross_cnt
全局 left_cnt
全局 cross_delay
serve_road() 里直接改 base_speed
follow_line() 里看到 LeftRoad/RightRoad 直接转弯
cross_cnt == 4 的特殊右转
```

这些以后应该逐步清理。

新架构希望 `road.c` 只做一件事：

```text
根据灰度数据判断当前像不像某种路口：
Straight / LeftRoad / RightRoad / CrossRoad
```

至于这个路口是否有效、要不要计数、要不要转弯、要不要停车，应该交给：

```text
task_basic_1_update()
task_basic_2_update()
task_adv_1_update()
task_adv_2_update()
```

也就是说：

```text
road.c 负责“看见了什么”
TASK 负责“这件事在当前题目里代表什么”
```

---

## 13. 常见修改应该去哪改

```text
想改任务切换、启动、停止逻辑
  -> User/Status/Defect.c
  -> update_task() / task_start() / task_finish()

想改某一道题怎么跑
  -> User/Status/Defect.c
  -> task_basic_1_update() 等 task_xxx_update()

想改按键功能
  -> User/Device/button.c
  -> server_button()

想改循迹
  -> User/Status/status.c
  -> follow_line()
  -> User/Sensor/gw_analogue.c

想改直角转弯/保角
  -> User/Status/status.c
  -> keep_angle()
  -> 以及具体 task_xxx_update() 里如何设置 tar_angle

想改电机 PID 或方向
  -> User/Motor/wheel.c
  -> init_motor() 里的 init_wheel() 参数

想改 LED/蜂鸣器状态提示
  -> User/Status/Defect.c
  -> update_task_led()
  -> User/Device/buzzer.c / led.c

想看控制周期怎么触发
  -> User/It/timer_it.c
```

---

## 14. 新队友调试时的读代码顺序

建议按这个顺序，不要一上来钻进某个细节函数里迷路。

```text
1. User/Status/status.h
   先看 STATUS 树有哪些分支。

2. User/Status/status.c
   看 init_status() 初始化了哪些东西。
   再看 update_status() 每 20ms 做了什么。

3. User/Status/Defect.h
   看 TASK 结构体有哪些字段。

4. User/Status/Defect.c
   看 update_task() 怎么消费请求。

5. User/Device/button.c
   看 PB11 / PD2 怎么把按键动作变成 TASK 请求。

6. User/Motor/wheel.c
   看 tar_speed 最后怎么变成 PWM。

7. User/Status/road.c
   最后再看路口识别，并注意里面有旧逻辑残留。
```

---

## 15. 最后记住这几条

1. `status` 是整车唯一核心状态树。
2. `update_status()` 是 20ms 控制主循环。
3. 按键和蓝牙不直接改任务状态，只置 request。
4. `update_task()` 消费 request，并推进比赛状态机。
5. `TASK` 管第几问和题内阶段，`motion` 管底层运动方式。
6. `follow_line()` / `keep_angle()` 算 `tar_speed`。
7. `driver_wheel()` 才真正输出 PWM。
8. `road.c` 未来只应该做路口观测，不应该接管转弯和停车。
