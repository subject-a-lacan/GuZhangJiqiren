# 小车控制代码架构介绍

## 一、整体分层

```
main.c                    ← 系统入口：初始化 + 主循环（仅负责 VOFA 调参打印）
    │
    ▼
User/                     ← 全部应用层代码（你写的代码基本都在这里）
    ├── Status/           ← ★ 核心：状态树，整车唯一全局状态
    ├── Sensor/           ← 传感器驱动（陀螺仪、灰度、CCD、激光雷达等）
    ├── Motor/            ← 执行器驱动（直流电机、舵机、闭环步进）
    ├── Device/           ← 板载外设（LED、按键、蜂鸣器）
    ├── It/               ← 中断服务函数（定时器中断、串口中断）
    ├── Tool/             ← 通用工具库（PID、数学宏、日志、数组运算）
    └── Middle/           ← 跨平台兼容层（STM32 / MSPM0 切换）
    │
    ▼
Core/                     ← CubeMX 自动生成的 HAL 初始化代码（MX_xxx_Init）
    │
    ▼
Drivers/                  ← STM32G4xx HAL 标准库（CMSIS + HAL Driver）
```

**硬件平台**: STM32G474 (Cortex-M4)，系统主频约 150 MHz。

---

## 二、核心设计：状态树（Status Tree）

### 2.1 概念

状态树是把整车的所有状态（传感器数据、电机数据、运动状态、外设状态）全部封装进一个全局结构体 `STATUS status`。关于小车的任何参数读取和修改，全部通过这个结构体进行。

```c
// 声明在 User/Status/status.h，定义在 User/Status/status.c
STATUS status;   // 全局唯一的状态树实例
```

### 2.2 树形结构（完整层次）

```
status                                // 根节点
  ├── state                           // 运行状态与控制参数
  │   ├── T                           // 系统控制周期 (ms)
  │   ├── time                        // 系统累计运行时间 (tick)
  │   ├── motion                      // 运动模式: STOP / KEEP_ANGLE / FIND_LINE / MOTOR_TEST
  │   ├── initial_angle               // 上电时的初始陀螺仪角度
  │   ├── cur_angle                   // 当前 yaw 角度
  │   ├── tar_angle                   // 目标 yaw 角度
  │   ├── base_speed                  // 基础速度（直行时的参考速度）
  │   ├── road_determine              // 路口判定状态
  │   │   ├── data_buf                // 当前帧灰度二值化数据
  │   │   ├── integral                // 多帧 OR 累计结果（去抖）
  │   │   ├── maybe                   // 候选路口倒计时
  │   │   ├── cross_cnt               // 已通过的路口个数
  │   │   ├── cross                   // 当前识别到的道路类型
  │   │   └── integral_times          // 判路累计帧数（默认 6）
  │   ├── gw_8bit                     // 8 位数字灰度当前值
  │   └── status_pid                  // 状态层 PID
  │       ├── follow_line_pid         // 巡线角度修正 PID
  │       └── keep_angle_pid          // 保角转向修正 PID
  │
  ├── sensor                          // 传感器数据
  │   ├── gy901                       // GY901 九轴陀螺仪 (I2C1, 0xA1)
  │   │   ├── data_buf[24]            // 24 字节原始数据缓冲
  │   │   ├── device_addr             // I2C 设备地址
  │   │   ├── data_start_addr         // 寄存器起始地址 (0x34)
  │   │   └── gy901_keep_angle_pid    // 陀螺仪自带保角 PID（预留）
  │   ├── gw_8bit                     // 8 路数字灰度 (I2C2, 0x98)
  │   │   ├── data_buf                // 8 位原始位图
  │   │   ├── gw_bit_weight[8]        // 各通道权重
  │   │   ├── integral                // 路口判定累计
  │   │   ├── maybe                   // 候选路口倒计时
  │   │   ├── cross_cnt               // 路口计数
  │   │   ├── cross                   // 道路类型
  │   │   ├── gw_find_line_pid        // 数字灰度巡线 PID
  │   │   └── gw_diff                 // 线偏差
  │   └── gw_analogue                 // 8 路模拟灰度 (ADC3, GPIO MUX 选通)
  │       ├── channel[8]              // 8 通道原始 ADC 值
  │       ├── sta                     // 0=工作 1=校准
  │       ├── correction_data_w[8]    // 白底校准数据
  │       ├── correction_data_b[8]    // 黑底校准数据
  │       ├── digital_8bit            // 二值化后的 8 位数据
  │       ├── digital_high_threshold[8] // 高阈值（迟滞比较）
  │       ├── digital_low_threshold[8]  // 低阈值（迟滞比较）
  │       └── diff                    // 模拟巡线偏差值
  │
  ├── motor                           // 执行器
  │   ├── wheel[4]                    // 4 个直流电机
  │   │   ├── which                   // 硬件编号 1-4
  │   │   ├── trust                   // PWM 推力输出值
  │   │   ├── cur_speed               // 当前编码器实测速度
  │   │   ├── tar_speed               // 控制层给出的目标速度
  │   │   ├── dir                     // 正方向校准 (-1 或 +1)
  │   │   └── wheel_pid               // 单轮速度环 PID
  │   └── servo[2]                    // 2 个舵机
  │       ├── which                   // 编号 1/2
  │       ├── angle                   // 目标角度
  │       └── max_angle               // 最大行程 (180° 或 270°)
  │
  └── device                          // 板载外设
      ├── led_on_board                // 板载 LED
      ├── led1                        // 外接 LED1
      ├── led2                        // 外接 LED2
      │   （以上三个 LED 结构: which, High_level_is_on, on）
      ├── button_D2                   // 按键 D2
      ├── button_B11                  // 按键 B11
      │   （以上两个按键结构: last, now, Press_is_high_level, which, long_press_cnt）
      └── buzzer                      // 蜂鸣器
          （结构: which, High_level_is_on, on）
```

### 2.3 状态树的访问方式

所有读取和修改都直接通过 `status.xxx.xxx` 完成：

```c
// 读：获取左轮当前速度
int16_t speed = status.motor.wheel[0].cur_speed;

// 写：设置左轮目标速度
status.motor.wheel[0].tar_speed = 100;

// 写：打开板载 LED
status.device.led_on_board.on = 1;

// 读：获取当前 yaw 角
float yaw = status.state.cur_angle;

// 写：切换运动模式
status.state.motion = FIND_LINE;
```

> **关键规则：不要自己新建全局变量来存传感器数据或控制参数，全部挂到 status 树里。**

---

## 三、封装格式：对外设的统一接口

每个外设模块遵循统一的命名规范，包含三类函数：

| 函数 | 命名格式 | 职责 |
|---|---|---|
| 初始化 | `init_xxx(XXX *xxx)` | 设置外设的默认参数、初始化 GPIO/通信接口 |
| 获取数据 | `get_xxx_raw_data(XXX *xxx)` | 从硬件读取原始数据（I2C/ADC/编码器），写入 status 树 |
| 解析数据 | `get_xxx_value(XXX *xxx, ...)` | 将原始数据解析为有意义的物理量（角度、偏差等） |
| 驱动输出 | `driver_xxx(XXX *xxx)` | 将 status 树中的目标值写入硬件（PWM/GPIO） |

### 具体示例

#### 传感器（以陀螺仪为例）

```c
// 1. 初始化：设定 I2C 地址和寄存器地址
init_gyr(&status.sensor.gy901);

// 2. 获取原始数据：通过 I2C 突发读取 24 字节到 data_buf
get_gyr_raw_data(&hi2c1, &status.sensor.gy901);

// 3. 解析原始数据：从 data_buf 中提取 yaw 角，转为 -180°~+180°
status.state.cur_angle = get_gyr_value(&status.sensor.gy901, gyr_z_yaw);
```

#### 执行器（以直流电机为例）

```c
// 1. 初始化：绑定硬件编号和方向校准
init_wheel(&status.motor.wheel[0], 1, -1);    // 左轮，硬件编号 1，方向取反

// 2. 获取原始数据：读取编码器 TIM->CNT，计算当前速度
status.motor.wheel[0].cur_speed = get_wheel_speed(&status.motor.wheel[0]);

// 3. 驱动输出：根据 tar_speed 做 PID 计算，输出 PWM + 方向 GPIO
driver_wheel(&status.motor.wheel[0]);
```

#### 外设（以按键为例）

```c
// 1. 初始化
init_button(&status.device.button_D2, 1, 0);  // 编号 1，低电平按下

// 2. 驱动（轮询边沿检测 + 长按计时）
driver_button(&status.device.button_D2);
```

---

## 四、更新层与驱动层

### 4.1 系统节拍

所有控制逻辑由 **TIM5 硬件定时器** 驱动：

```
TIM5 每 1ms 触发一次中断
    │
    ├── status.state.time += 1    （系统时间递增）
    │
    └── 每 20ms 执行一次：
          update_status(&status)
```

### 4.2 update_status() —— 更新层（读传感器 → 算控制量）

`update_status()` 是整车的核心控制循环，每 20ms 执行一次。位于 `User/Status/status.c`。

```c
void update_status(STATUS *status) {
    // === 第一步：读取所有传感器原始数据 ===
    get_gw_raw_data(&status->sensor.gw_analogue);              // ADC3 读 8 路模拟灰度
    status->motor.wheel[0].cur_speed = get_wheel_speed(...);   // 读 4 个编码器
    status->motor.wheel[1].cur_speed = get_wheel_speed(...);
    status->motor.wheel[2].cur_speed = get_wheel_speed(...);
    status->motor.wheel[3].cur_speed = get_wheel_speed(...);
    get_gyr_raw_data(&hi2c1, &status->sensor.gy901);           // I2C 读陀螺仪
    status->state.cur_angle = get_gyr_value(...);               // 解析陀螺仪角度

    // === 第二步：根据运动模式计算目标速度 ===
    if (motion == FIND_LINE)  follow_line(status);    // 巡线控制
    if (motion == KEEP_ANGLE) keep_angle(status);     // 保角控制
    if (motion == STOP)       tar_speed = 0;           // 停车
    if (motion == MOTOR_TEST) tar_speed = 40;          // 电机测试

    // === 第三步：驱动所有执行器和外设 ===
    driver_button(...);   // 按键状态机
    driver_LED(...);      // LED 输出
    driver_servo(...);    // 舵机 PWM
    driver_BUZZER(...);   // 蜂鸣器
    driver_wheel(...);    // 电机速度环 PID + PWM 输出
}
```

**数据流**：

```
传感器硬件 ──get_raw──▶ status.sensor.xxx    （原始数据存入状态树）
                              │
                              ▼  get_value / 控制算法
                        状态更新层（follow_line / keep_angle）
                              │
                              ▼
                        status.motor.xxx.tar_speed    （目标值写入状态树）
                              │
                              ▼  driver_xxx
                         执行器硬件（PWM / GPIO）
```

### 4.3 驱动层

驱动层就是各个 `driver_xxx()` 函数，它们不关心控制逻辑，只负责把 status 树里的目标值变成硬件动作：

- `driver_wheel()`: `tar_speed` → PID 计算 → `trust` → TIM8 PWM + GPIO 方向
- `driver_servo()`: `angle` → 脉宽计算 → TIM15 PWM
- `driver_LED()`: `on` → GPIO 高低电平
- `driver_BUZZER()`: `on` → GPIO 高低电平
- `driver_button()`: GPIO 读电平 → 边沿检测 → 更新 `last/now/long_press_cnt`

---

## 五、运动控制详解

### 5.1 巡线控制 `follow_line()`

```
1. 读取模拟灰度 → digital_8bit（二值化）+ diff（线偏差）
2. 路口判定 get_road_type()
   - 多帧 OR 累计去抖（integral_times=6）
   - 输出: Straight / LeftRoad / RightRoad / CrossRoad / ...
3. 根据路型生成轮速差:
   - 直道: diff → follow_line_pid → 左右轮差速
   - 左转: 左轮 +20, 右轮 -20（原地转向）
   - 右转: 左轮 -20, 右轮 +20
   - 第 4 个十字路口: 右转
```

### 5.2 保角控制 `keep_angle()`

```
1. 计算角度误差 = tar_angle + initial_angle - cur_angle
   （注意 360° 环绕处理）
2. 误差 → keep_angle_pid → diff（限幅 ±25）
3. 左轮 = base_speed + diff
   右轮 = base_speed - diff
4. 角度稳定在 ±1° 以内后，自动提速到 base_speed=40
```

### 5.3 PID 级联结构

```
巡线/保角 PID（状态层）           速度环 PID（电机层）
follow_line_pid / keep_angle_pid    wheel_pid[0~3]
       │                                  │
  计算轮速差 → tar_speed          tar_speed - cur_speed → trust (PWM)
```

默认参数均为纯 P 控制（ki=0, kd=0），后续可调。

---

## 六、中断服务

### 6.1 定时器中断 `User/It/timer_it.c`

- **TIM5**: 1ms 周期，递增 `status.state.time`，每 20ms 调用 `update_status()`

### 6.2 串口中断 `User/It/uart_it.c`

| 串口 | 用途 | 接收方式 |
|---|---|---|
| USART1 | WiFi(ESP8266) / PID调参 / VOFA打印 | 单字节中断 + `UART_PID_Tune()` 解析 |
| USART2 | 视觉(K230) / 步进电机 / 绝对编码器 | DMA + IDLE 空闲中断 |
| USART3 | 步进电机 / MaixCam | DMA + IDLE 空闲中断 |
| UART4 | VOFA 预留 | DMA + IDLE 空闲中断 |

---

## 七、工具库 `User/Tool/`

| 文件 | 功能 |
|---|---|
| `pid.h/pid.c` | 通用 PID 控制器：`init_pid()` / `compute_pid()` |
| `task.h` | 非阻塞延时宏：`PERIODIC_START(name, ms)` / `PERIODIC_END` |
| `log.h/log.c` | 格式化串口输出：`log_uprintf(&huart, format, ...)` |
| `math_tool.h` | 数学宏：`ABS`, `CONFINE`, `CLAMP`, `SIGN` 等 |
| `array.h/array.c` | 数组运算：求和、拷贝、卷积、差分、均值等 |
| `eeprom.h/eeprom.c` | I2C EEPROM 读写 |

---

## 八、如何添加新模块

假设你要添加一个 **超声波模块**：

### Step 1: 定义状态结构体

在 `User/Sensor/ultrasonic.h`:

```c
typedef struct {
    uint8_t data_buf[4];   // 原始数据
    float distance;        // 解析后的距离 (cm)
} ULTRASONIC;
```

### Step 2: 实现 init/get/driver 函数

```c
void init_ultrasonic(ULTRASONIC *u);
void get_ultrasonic_raw_data(ULTRASONIC *u);
float get_ultrasonic_distance(ULTRASONIC *u);
```

### Step 3: 挂载到状态树

在 `User/Status/status.h` 的 `SENSOR` 结构体中添加：

```c
typedef struct SENSOR {
    GYR gy901;
    GW_8BIT gw_8bit;
    GW_ANALOGUE gw_analogue;
    ULTRASONIC ultrasonic;    // ← 新增
} SENSOR;
```

### Step 4: 在 update_status() 中调用

```c
get_ultrasonic_raw_data(&status->sensor.ultrasonic);
float dist = get_ultrasonic_distance(&status->sensor.ultrasonic);
```

---

## 九、定时器资源分配

| 定时器 | 用途 | 说明 |
|---|---|---|
| TIM1 | 编码器模式 | 左前轮速度测量 |
| TIM2 | 编码器模式 | 右前轮速度测量 |
| TIM3 | 编码器模式 | 左后轮速度测量 |
| TIM4 | 编码器模式 | 右后轮速度测量 |
| TIM5 | 系统心跳 | 1ms 周期，控制循环节拍 |
| TIM6 | CCD 驱动 | μs 级时序 |
| TIM8 | 电机 PWM | 4 通道，TB6612 驱动 |
| TIM15 | 舵机 PWM | 2 通道，50Hz |

---

## 十、串口资源分配

| 串口 | 波特率 | 当前用途 |
|---|---|---|
| USART1 | 115200 | WiFi 透传(ESP8266) + PID 调参 + VOFA 打印 |
| USART2 | -- | 视觉模块 / 步进电机 / 绝对编码器 |
| USART3 | -- | 步进电机 / MaixCam |
| UART4 | -- | VOFA 预留 |

---

## 十一、快速上手 check list

如果你是第一次接触这套代码，建议按以下顺序阅读：

1. `User/ARCHITECTURE.md`（本文档）
2. `User/Status/status.h` — 状态树的完整类型定义
3. `User/Status/status.c` — `update_status()` 控制循环
4. `User/It/timer_it.c` — 中断如何驱动控制循环
5. `Core/Src/main.c` — 初始化流程和主循环
6. 再看你需要改的外设模块（`User/Sensor/`, `User/Motor/`, `User/Device/`）

关键原则就三条：
- **状态全部挂树** — 不要自己定义全局变量存数据
- **init → get_raw → parse → driver** — 按这个流程读写外设
- **所有控制逻辑在 `update_status()` 里** — main() 的 while(1) 基本是空的