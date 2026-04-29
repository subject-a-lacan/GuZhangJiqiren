# 云台闭环步进移植架构说明

## 1. 当前要借鉴的云台工程是什么结构

原工程是一个裸机控制工程，虽然你习惯上把它理解成 STM32 下位机代码，但从文件和 API 看，实际芯片是 TI MSPM0G3507：`ti_msp_dl_config.h` 中定义了 `CONFIG_MSPM0G3507`，外设调用也都是 `DL_UART_Main_*`、`DL_TimerA_*`、`DL_TimerG_*` 这一类 TI DriverLib 接口。

它的核心思想不是复杂的分层框架，而是非常典型的“主循环 + 中断 + 全局状态变量”结构：

```text
main.c
  ├─ 初始化系统时钟、SysTick、PWM、定时器、UART 中断、OLED、PID
  ├─ while(1)
  │    ├─ Timer_work()
  │    ├─ key_work()
  │    ├─ uart_work()
  │    └─ oled_show()
  ├─ UART_0_INST_IRQHandler()
  │    └─ 串口按帧接收 K230 发来的视觉坐标
  └─ TIMER_0_INST_IRQHandler()
       └─ 根据 x_site/y_site 做双轴 PID，并输出给两个步进电机
```

工程中的文件职责大致如下：

| 文件 | 作用 |
| --- | --- |
| `main.c` | 应用入口、主循环、定时器中断控制逻辑 |
| `alldata.h` | 全局状态变量集中声明，类似一个比较松散的状态树 |
| `Uart.c/.h` | 串口收包状态机，接收 K230 视觉数据 |
| `work.c/.h` | 上层任务封装，包括串口解析、按键、OLED、简单动作函数 |
| `PID.c/.h` | PID 参数初始化和 PID 计算 |
| `StepMotor.c/.h` | 把控制量换算为 PWM 周期，并设置步进电机方向/使能/PWM |
| `ti_msp_dl_config.c/.h` | SysConfig 生成的底层外设初始化代码 |

原工程虽然没有你现在工程里的 `status` 状态树，但它事实上也有“全局状态中心”：`alldata.h`。里面保存了 `x_site`、`y_site`、`L_PWM`、`R_PWM`、`mode`、`keycnt`、`Motor.button`、`Tick`、`x_stepPid`、`y_stepPid` 等变量。也就是说，它把各种模块的状态散放在全局变量里，而你的工程可以把这些东西规整地挂进 `status`。

## 2. 原工程中 K230 和下位机如何联动

K230 在原工程里承担“视觉上位机/视觉协处理器”的角色。它负责摄像头识别、目标检测、计算误差，然后通过串口把误差发给下位机。下位机不处理图像，只消费 K230 给出的坐标偏差。

原工程启用的是 `UART_0`：

```c
NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
```

`UART_0` 的参数来自 SysConfig：

```text
UART_0_BAUD_RATE = 115200
UART_0_RX = GPIOA.11
UART_0_TX = GPIOA.10
```

K230 发送的数据帧格式可以理解为：

```text
[xxxxxxxx*]
```

其中 `xxxxxxxx` 是 8 字节有效数据：

```text
前 4 字节：x 坐标误差
后 4 字节：y 坐标误差
```

例如：

```text
[-010+018*]
[+035+020*]
```

`Uart.c` 的中断函数是一个非常小的串口接收状态机：

```text
等待 '['
  ↓
开始缓存数据
  ↓
遇到 '*'
  ↓
等待 ']'
  ↓
一帧完成，Serial_RxFlag = 1
```

中断只负责“收完整帧”，不负责解析业务含义。业务解析放在 `work.c` 的 `uart_work()` 中：

```c
if (Serial_RxFlag == 1) {
    if (strlen(Serial_RxPacket) != 8) {
        Serial_RxFlag = 0;
        return -1;
    }

    char x_str[5] = {0};
    char y_str[5] = {0};

    strncpy(x_str, Serial_RxPacket, 4);
    strncpy(y_str, Serial_RxPacket + 4, 4);

    x_site = atoi(x_str);
    y_site = atoi(y_str);

    Serial_RxFlag = 0;
}
```

所以原工程的数据链路是：

```text
K230 视觉识别
  ↓ UART 发送 [xxyyyy*] 类型数据帧
UART 接收中断缓存一帧
  ↓ Serial_RxFlag = 1
主循环 uart_work() 解析 x_site/y_site
  ↓
定时器中断读取 x_site/y_site 做 PID
```

这套方式值得保留的点是：

1. 串口中断只收包，不做复杂计算。
2. 主循环或周期任务负责解析协议。
3. 控制中断只读取已经解析好的误差值。
4. 视觉数据和电机控制之间通过状态变量解耦。

## 3. 原工程中下位机如何控制云台

原工程的云台是两个普通步进电机，所以下位机需要自己产生脉冲、方向和使能信号。两个轴的关系是：

```text
X 轴电机：L_PWM，对应 PWM_0_INST = TIMA1，输出 GPIOB.17
Y 轴电机：R_PWM，对应 PWM_1_INST = TIMG6，输出 GPIOB.7
X 轴方向：DIR_A_PIN = GPIOB.21
Y 轴方向：DIR_B_PIN = GPIOB.27
X 轴使能：EN_MA_PIN = GPIOB.14
Y 轴使能：EN_MB_PIN = GPIOB.18
```

`StepMotor.c` 中定义了电机和定时器参数：

```c
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPS 16
#define PWM_CLOCK_FREQ 10000000
```

也就是 1.8 度步进电机，16 细分，PWM 定时器时钟 10 MHz。

`Calculate_target()` 的职责是把目标转速转换为 PWM 定时器周期：

```text
目标转速 rpm
  ↓
每圈步数 = 200 * 16
  ↓
每秒脉冲数 stepsPerSecond
  ↓
PWM 频率
  ↓
定时器 period = PWM_CLOCK_FREQ / pwmFrequency - 1
```

函数返回值带符号：

```text
正数：一个方向
负数：另一个方向
0：停止输出脉冲
```

`Set_PWM(L_Target, R_Target)` 则是真正的底层驱动函数：

```text
L_Target > 0：X 轴使能，DIR_A 置位，PWM_0 输出 50% 占空比
L_Target < 0：X 轴使能，DIR_A 清零，PWM_0 输出 50% 占空比
L_Target = 0：X 轴 PWM 比较值置 0，停止脉冲

R_Target > 0：Y 轴使能，DIR_B 置位，PWM_1 输出 50% 占空比
R_Target < 0：Y 轴使能，DIR_B 清零，PWM_1 输出 50% 占空比
R_Target = 0：Y 轴 PWM 比较值置 0，停止脉冲
```

原工程的云台控制闭环在 `TIMER_0_INST_IRQHandler()` 里。定时器配置为 1ms 周期，但代码中 `Tick.steppid++ >= 2` 后才做一次 PID，因此实际云台闭环大约每 2ms 执行一次：

```c
if (Motor.button != 0) {
    if (Tick.steppid++ >= 2) {
        Tick.steppid = 0;
        Tick.step_flag = 1;

        PID_caculate(&x_stepPid, x_site, 0);
        PID_caculate(&y_stepPid, y_site, 18);

        L_PWM = Calculate_target(x_stepPid.output + x_buchang);
        R_PWM = Calculate_target(y_stepPid.output);

        Set_PWM(L_PWM, R_PWM);
    }
}
```

这里的控制含义是：

```text
x_site 当前视觉 X 偏差，目标值为 0
 y_site 当前视觉 Y 偏差，目标值为 18
```

Y 轴目标不是 0，而是 18，说明代码里做了一个安装偏差或瞄准偏差补偿。也就是视觉中心和实际激光落点可能不完全重合，所以让目标保持在图像中的 y=18 附近。

`Pid_Init(kp, ki, kd)` 有一个关键细节：

```c
x_stepPid.Kp = kp;
x_stepPid.Ki = ki;
x_stepPid.Kd = kd;

y_stepPid.Kp = -kp;
y_stepPid.Ki = -ki;
y_stepPid.Kd = kd;
```

Y 轴 PID 参数取反，说明两个轴的机械安装方向、图像坐标方向或电机方向定义并不一致。这个思想移植时要保留：不要假设两个轴正方向天然一致，应该在结构体里给每个轴留一个 `dir` 或 `reverse` 参数。

## 4. 原工程的模式逻辑

原工程的控制开关不是单纯一个函数调用，而是通过 `mode`、`keycnt`、`Motor.button` 共同决定。

主循环中有几个关键分支：

```text
mode == 1 && keycnt == 4：进入视觉闭环瞄准
mode == 2 && keycnt == 4：X 轴正向扫描，发现目标后切回闭环
mode == 4 && keycnt == 4：X 轴反向扫描，发现目标后切回闭环
mode == 5：清零 mode/keycnt
```

闭环瞄准状态下，如果误差进入阈值：

```c
if (x_site >= -2 && x_site <= 2 && y_site <= 20 && y_site >= 15) {
    Motor.button = 0;
    LED2_ON();
    L_PWM = R_PWM = 0;
    Set_PWM(L_PWM, R_PWM);
    mode = 0;
}
```

也就是：

```text
视觉误差进入目标窗口
  ↓
关闭云台控制
  ↓
停止两个步进电机
  ↓
点亮 LED2
  ↓
退出当前瞄准模式
```

扫描模式下，代码先让 X 轴以固定速度转动：

```c
L_PWM = Calculate_target(30);
R_PWM = 0;
Set_PWM(L_PWM, R_PWM);
```

或者反向：

```c
L_PWM = Calculate_target(-30);
R_PWM = 0;
Set_PWM(L_PWM, R_PWM);
```

当检测到 `x_site > 0 && x_site <= 80`，说明目标进入视野，再停止扫描并切回 `mode = 1` 进入精瞄。

可以把原工程抽象成三个功能层：

```text
视觉输入层：UART 收 K230 坐标
控制决策层：根据 mode/keycnt/x_site/y_site 决定扫描、闭环、停止
执行输出层：PID -> 转速 -> PWM/方向/使能
```

## 5. 你的工程应该如何嵌入这套功能

你的条件和原工程有两点本质不同：

1. 你用的是 STM32 HAL 工程，并且已经有 `status` 状态树。
2. 你用的是自带闭环的步进电机，不需要 STM32 自己产生 STEP/DIR 脉冲，只需要通过 `huart2`、`huart3` 发串口命令。

因此不要把原工程的 `StepMotor.c` 原样搬过来。真正应该照搬的是“函数职责划分”和“控制流程”，不是 PWM 底层实现。

推荐的新数据流：

```text
K230 / MaixCam 视觉
  ↓ 视觉串口帧
status.sensor.vision.x_error / y_error / valid
  ↓ 周期控制任务
status.motor.gimbal.x_axis / y_axis
  ↓ driver_gimbal()
huart2 控制 X 轴闭环步进
huart3 控制 Y 轴闭环步进
```

推荐在 `status` 中新增云台子树：

```c
typedef enum {
    GIMBAL_STOP = 0,
    GIMBAL_SCAN_LEFT,
    GIMBAL_SCAN_RIGHT,
    GIMBAL_TRACK,
    GIMBAL_LOCKED,
    GIMBAL_ERROR,
} GIMBAL_MODE;

typedef struct {
    int16_t x;
    int16_t y;
    uint8_t valid;
    uint32_t last_update_time;
} VISION_TARGET;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t id;
    int8_t reverse;
    float tar_speed;
    float cur_speed;
    float tar_angle;
    float cur_angle;
    uint8_t enable;
} CLOSED_STEP_AXIS;

typedef struct {
    CLOSED_STEP_AXIS x_axis;
    CLOSED_STEP_AXIS y_axis;
    PID x_pid;
    PID y_pid;
    GIMBAL_MODE mode;
    uint8_t enable;
    uint8_t locked;
    int16_t target_x;
    int16_t target_y;
    int16_t deadband_x;
    int16_t deadband_y;
    float scan_speed;
    float max_track_speed;
    uint32_t last_cmd_time;
} GIMBAL;
```

然后把它挂到现有状态树：

```c
typedef struct SENSOR {
    GYR gy901;
    GW_8BIT gw_8bit;
    GW_ANALOGUE gw_analogue;
    VISION_TARGET vision;
} SENSOR;

typedef struct MOTOR {
    WHEEL wheel[4];
    SERVO servo[2];
    GIMBAL gimbal;
} MOTOR;
```

如果你不想把视觉放在 `SENSOR` 里，也可以把 `VISION_TARGET` 放到 `GIMBAL` 内部。但从状态树语义看，K230 识别结果属于传感器输入，放在 `status.sensor.vision` 更干净。

## 6. 建议的文件封装

为了仿照原工程，同时适配你的工程，建议在 `User/Motor` 和 `User/Sensor` 下这样分：

```text
User/Sensor/vision_uart.c/.h
  ├─ init_vision_uart()
  ├─ vision_uart_rx_callback()
  └─ update_vision_target()

User/Motor/gimbal_step.c/.h
  ├─ init_closed_step_axis()
  ├─ driver_closed_step_speed()
  ├─ driver_closed_step_stop()
  ├─ init_gimbal()
  ├─ update_gimbal()
  └─ driver_gimbal()
```

和原工程的对应关系如下：

| 原工程函数/变量 | 你的工程中建议的替代 |
| --- | --- |
| `Serial_RxPacket` | `status.sensor.vision.rx_packet` 或 vision 模块静态缓存 |
| `Serial_RxFlag` | `status.sensor.vision.new_frame` |
| `x_site/y_site` | `status.sensor.vision.x/y` |
| `Pid_Init()` | `init_gimbal()` 内初始化 `x_pid/y_pid` |
| `PID_caculate()` | 复用你现有 `compute_pid()` 或封装 `gimbal_compute_pid()` |
| `Calculate_target()` | 不再需要 PWM 周期换算，改成速度限幅和方向转换 |
| `Set_PWM()` | `driver_closed_step_speed(&axis, speed)`，内部通过 UART 发命令 |
| `Motor.button` | `status.motor.gimbal.enable` |
| `mode/keycnt` | `status.motor.gimbal.mode` 或更高层任务状态 |
| `TIMER_0_IRQHandler()` | `TIM5` 周期中断里调用 `update_gimbal()`，再由 `driver_gimbal()` 发命令 |

## 7. 你的 huart2/huart3 如何承担闭环步进控制

你现有工程中 `huart2` 和 `huart3` 已经初始化，且 `User/Motor/lq_step.c` 已经有类似闭环步进命令函数：

```c
void trun_lq_step_abslute_angle(UART_HandleTypeDef *huart, float angle, float speed);
void trun_lq_step_angle(UART_HandleTypeDef *huart, float angle, uint8_t dir, float speed);
void trun_lq_step_speed(UART_HandleTypeDef *huart, float speed, uint8_t dir);
void trun_lq_step_current(UART_HandleTypeDef *huart, uint16_t current, uint8_t dir);
```

所以云台底层驱动不需要再碰 PWM。推荐这样定义两个轴：

```c
init_closed_step_axis(&status.motor.gimbal.x_axis, &huart2, 1, 1);
init_closed_step_axis(&status.motor.gimbal.y_axis, &huart3, 2, -1);
```

其中 `reverse` 用来修正方向：

```text
speed > 0：希望视觉误差向正方向修正
reverse = 1：直接按正方向发给电机
reverse = -1：反向发给电机
```

底层速度命令可以包一层：

```c
void driver_closed_step_speed(CLOSED_STEP_AXIS *axis, float speed) {
    uint8_t dir;
    float abs_speed;

    speed *= axis->reverse;

    if (speed >= 0) {
        dir = 1;
        abs_speed = speed;
    } else {
        dir = 0;
        abs_speed = -speed;
    }

    if (axis->enable == 0 || abs_speed < 0.01f) {
        trun_lq_step_speed(axis->huart, 0, dir);
        return;
    }

    trun_lq_step_speed(axis->huart, abs_speed, dir);
}
```

这样上层 `update_gimbal()` 不需要关心具体协议，只给 `x_axis.tar_speed` 和 `y_axis.tar_speed` 赋值。

## 8. 推荐的云台周期控制逻辑

原工程把 PID 放在定时器中断中直接算。你的工程已有 `TIM5` 每 20ms 调 `update_status(&status)`，建议把云台控制也放进这个节奏里，而不是单独开一个复杂中断。

推荐结构：

```c
void update_status(STATUS *status) {
    // 原有传感器更新
    get_gw_raw_data(&status->sensor.gw_analogue);
    get_gyr_raw_data(&hi2c1, &status->sensor.gy901);

    // 原有运动控制
    if (status->state.motion == FIND_LINE) {
        follow_line(status);
    }

    // 新增云台控制决策
    update_gimbal(status);

    // 原有设备驱动
    driver_LED(&status->device.led1);
    driver_wheel(&status->motor.wheel[0]);

    // 新增云台输出
    driver_gimbal(&status->motor.gimbal);
}
```

`update_gimbal()` 建议写成状态机：

```text
GIMBAL_STOP
  └─ 两轴目标速度 = 0

GIMBAL_SCAN_LEFT / GIMBAL_SCAN_RIGHT
  └─ X 轴固定速度扫描，Y 轴停止
  └─ 如果 vision.valid 且 x 进入可跟踪范围，则切到 GIMBAL_TRACK

GIMBAL_TRACK
  └─ 对 x/y 误差做 PID
  └─ PID 输出限幅后变成两轴目标速度
  └─ 如果误差进入 deadband，则切到 GIMBAL_LOCKED

GIMBAL_LOCKED
  └─ 两轴停止
  └─ 置位 locked，可通知激光/蜂鸣器/上层任务
```

核心伪代码：

```c
void update_gimbal(STATUS *status) {
    GIMBAL *g = &status->motor.gimbal;
    VISION_TARGET *v = &status->sensor.vision;

    if (!g->enable) {
        g->mode = GIMBAL_STOP;
    }

    switch (g->mode) {
    case GIMBAL_STOP:
        g->x_axis.tar_speed = 0;
        g->y_axis.tar_speed = 0;
        break;

    case GIMBAL_SCAN_LEFT:
        g->x_axis.tar_speed = -g->scan_speed;
        g->y_axis.tar_speed = 0;
        if (v->valid && v->x > 0 && v->x <= 80) {
            g->mode = GIMBAL_TRACK;
        }
        break;

    case GIMBAL_SCAN_RIGHT:
        g->x_axis.tar_speed = g->scan_speed;
        g->y_axis.tar_speed = 0;
        if (v->valid && v->x > 0 && v->x <= 80) {
            g->mode = GIMBAL_TRACK;
        }
        break;

    case GIMBAL_TRACK: {
        float x_out = compute_pid(&g->x_pid, g->target_x - v->x);
        float y_out = compute_pid(&g->y_pid, g->target_y - v->y);

        x_out = CONFINE(x_out, -g->max_track_speed, g->max_track_speed);
        y_out = CONFINE(y_out, -g->max_track_speed, g->max_track_speed);

        g->x_axis.tar_speed = x_out;
        g->y_axis.tar_speed = y_out;

        if (ABS(v->x - g->target_x) <= g->deadband_x &&
            ABS(v->y - g->target_y) <= g->deadband_y) {
            g->mode = GIMBAL_LOCKED;
            g->locked = 1;
        }
        break;
    }

    case GIMBAL_LOCKED:
        g->x_axis.tar_speed = 0;
        g->y_axis.tar_speed = 0;
        break;

    default:
        g->mode = GIMBAL_ERROR;
        g->x_axis.tar_speed = 0;
        g->y_axis.tar_speed = 0;
        break;
    }
}
```

注意：你现有 `compute_pid()` 的参数形式需要以实际 `pid.c/h` 为准。如果它是“传入实际值，内部 target 固定”的风格，就把 `target_x/target_y` 写进 PID 结构；如果它是“传入误差”的风格，就按上面这样传误差。

## 9. 中断里应该更新什么

你的工程设计原则是“中断里更新结构体”。建议按轻重分工：

### UART 视觉接收中断

中断里只做这些事：

1. 逐字节接收 K230 数据。
2. 用状态机拼帧。
3. 一帧完成后更新 `status.sensor.vision.raw_packet` 或临时缓存。
4. 设置 `status.sensor.vision.new_frame = 1`。
5. 重新开启 `HAL_UART_Receive_IT()`。

不要在 UART 中断里跑 PID，也不要直接发电机命令。

### TIM5 周期中断

TIM5 可以继续做系统节拍：

```text
1ms：status.state.time += T
20ms：update_status(&status)
```

如果后续云台响应不够快，可以单独让云台每 5ms 更新一次：

```c
if (status.state.time % 5 == 0) {
    update_gimbal(&status);
    driver_gimbal(&status.motor.gimbal);
}
```

但第一版建议直接跟着现有 20ms 跑，先保证架构干净。

### huart2/huart3 电机反馈中断

如果你的闭环步进会回传位置、到位状态、错误码，建议分别加：

```text
status.motor.gimbal.x_axis.cur_angle
status.motor.gimbal.x_axis.cur_speed
status.motor.gimbal.x_axis.arrived
status.motor.gimbal.x_axis.error

status.motor.gimbal.y_axis.cur_angle
status.motor.gimbal.y_axis.cur_speed
status.motor.gimbal.y_axis.arrived
status.motor.gimbal.y_axis.error
```

但如果当前只需要速度闭环控制，第一版可以不解析电机反馈，只发送命令。

## 10. 移植时最重要的取舍

原工程的云台闭环是：

```text
视觉误差 -> PID -> rpm -> PWM 周期 -> STEP/DIR 驱动器
```

你的工程应该变成：

```text
视觉误差 -> PID -> 目标速度 -> UART 闭环步进命令
```

因此：

1. `Uart.c` 的 K230 收包状态机可以借鉴。
2. `work.c` 的 `uart_work()` 解析方式可以借鉴。
3. `PID.c` 的双轴 PID 思路可以借鉴。
4. `main.c` 的扫描、跟踪、锁定模式可以借鉴。
5. `StepMotor.c` 的 PWM 换算不应该照搬，只保留“底层电机驱动函数单独封装”的思想。
6. 原来的 `mode/keycnt/Motor.button` 应该被规整成 `status.motor.gimbal.mode/enable/locked`。
7. 原来的 `x_site/y_site` 应该挂到 `status.sensor.vision.x/y`。
8. 原来的 `L_PWM/R_PWM` 应该变成 `status.motor.gimbal.x_axis.tar_speed/y_axis.tar_speed`。

## 11. 推荐最终调用关系

初始化阶段：

```text
main.c
  ├─ HAL_Init/SystemClock/MX_USART2/MX_USART3/...
  ├─ init_status(&status, 1)
  │    ├─ init_sensor()
  │    │    └─ init_vision_target(&status.sensor.vision)
  │    └─ init_motor()
  │         └─ init_gimbal(&status.motor.gimbal)
  ├─ HAL_UART_Receive_IT(视觉串口, &vision_rx_byte, 1)
  └─ HAL_TIM_Base_Start_IT(&htim5)
```

运行阶段：

```text
K230 UART RX 中断
  └─ vision_uart_rx_callback()
       └─ 更新 status.sensor.vision 原始帧/new_frame

TIM5 20ms 周期
  └─ update_status(&status)
       ├─ update_vision_target(&status.sensor.vision)
       ├─ update_gimbal(&status)
       └─ driver_gimbal(&status.motor.gimbal)
              ├─ huart2 -> X 轴闭环步进
              └─ huart3 -> Y 轴闭环步进
```

上层任务只需要改状态：

```c
status.motor.gimbal.enable = 1;
status.motor.gimbal.mode = GIMBAL_TRACK;
```

或者启动扫描：

```c
status.motor.gimbal.enable = 1;
status.motor.gimbal.mode = GIMBAL_SCAN_RIGHT;
```

命中后读取：

```c
if (status.motor.gimbal.locked) {
    status.device.led2.on = 1;
}
```

这就是把原工程“能跑的云台逻辑”移植到你现在状态树架构里的最自然方式：视觉作为传感器输入，云台作为电机子模块，PID 和状态机作为 `update_gimbal()`，串口闭环步进命令作为 `driver_gimbal()`。
