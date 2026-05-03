# xiao → xvshen 完整移植设计文档

## 项目概述

| 项目 | 路径 | 角色 |
|------|------|------|
| **xiao** (原始) | `C:\Users\kakak\Desktop\xiao\xiaosaishenche` | 稳定运行的小车，2轮差速，巡线+转弯+一圈自动停 |
| **xvshen** (移植) | `C:\Users\kakak\Desktop\xvshen\GuZhangJiqiren` | 电赛故障机器人，4轮平台，多传感器，任务系统架构 |

**移植目标**：让 xvshen 的 Q1 基础任务完全复现 xiao 的行为 — 巡线、路口左转、一圈后自动停车。

---

## 一、架构对比

### 1.1 整体架构差异

```
┌────────── xiao (原始) ──────────┐    ┌────────── xvshen (移植) ──────────┐
│                                  │    │                                    │
│  main.c                          │    │  main.c                            │
│    └─ init_status()              │    │    └─ init_status()                │
│    └─ while(1) { }  // 空的      │    │    └─ while(1) { }  // 空的        │
│                                  │    │                                    │
│  timer_it.c (254行, 核心)        │    │  timer_it.c (50行, 薄层)           │
│    ├─ 传感器读取                  │    │    ├─ driver_gw_analogue() (10ms)  │
│    ├─ 丢线/有线计数               │    │    └─ update_status()    (20ms)    │
│    ├─ 状态机 (FIND_LINE/         │    │                                    │
│    │         KEEP_ANGLE/STOP)    │    │  status.c (调度中心)                │
│    ├─ diff_drive() 差速输出      │    │    ├─ 感知层: 读传感器 读编码器     │
│    ├─ find_line() 巡线PID        │    │    ├─ 策略层: update_task()         │
│    ├─ 路口检测 + 转弯             │    │    ├─ 执行层: motion_execute()     │
│    └─ driver_status() →          │    │    └─ 驱动层: driver_wheel()等     │
│        驱动轮子/LED/蜂鸣器        │    │                                    │
│                                  │    │  Defect.c (任务状态机)              │
│                                  │    │    ├─ TASK 结构体                  │
│                                  │    │    ├─ 按钮 → armed → 阶段推进      │
│                                  │    │    └─ task_basic_1_update()        │
│                                  │    │       └─ Q1 赛道状态机             │
│                                  │    │                                    │
│  PID参数 (kp/ki/kd):             │    │  PID参数 (kp/ki/kd):               │
│  find_line:  0.8 / 0.03 / 2.5   │    │  follow_line: 0.8 / 0.03 / 2.5    │
│  keep_angle: 1.2 / 0.0  / 1.2   │    │  keep_angle:  1.2 / 0.0  / 1.2    │
│  wheel:      25  / 0.08 / 8     │    │  wheel:       25  / 0.08 / 8      │
└──────────────────────────────────┘    └────────────────────────────────────┘
```

### 1.2 核心区别：状态机在哪里

| 对比维度 | xiao | xvshen |
|----------|------|--------|
| 状态机位置 | `timer_it.c` 的 `HAL_TIM_PeriodElapsedCallback` 内 | `Defect.c` 的 `task_basic_1_update()` + `status.c` 的 `motion_execute()` |
| 状态机粒度 | 单一 switch 包含一切 | 分层：阶段机(race_phase) → 运动机(motion) → 执行层(motion_execute) |
| 传感器读取 | 状态机内局部变量 `yaw`, `line` | `status.c` 写入 `status.state.cur_angle`，`Defect.c` 读取 |
| 电机控制 | `find_line()` / `diff_drive()` 直接设 tar_speed | `motion_execute()` 设 tar_speed，与任务逻辑分离 |
| 转弯触发 | 丢线 OR 路口 → KEEP_ANGLE | 同上（逻辑一致） |
| 转弯完成 | 重新看到线 LINE_BACK_CNT 次 → back_to_find_line() | 同上 |
| 停车判断 | corner_count ≥ 4 → final_stop | 同上有 corner_count，但 Q1 用阶段机推进 |
| 缺少 | — | 任务系统(armed/startup_release)、阶段机(Q1_*)、多任务框架 |

---

## 二、模块映射

### 2.1 传感器层

| xiao 模块 | xvshen 模块 | 状态 | 说明 |
|-----------|-------------|------|------|
| `Sensor/gy901.c` | `Sensor/gy901.c` | 已移植 | GYR 结构体不同(多了 PID 字段少了 angle 字段)，但核心读写逻辑一致 |
| `Sensor/gw_analogue.c` | `Sensor/gw_analogue.c` | 已移植 | MUX 引脚需修正为 AD0/AD1/AD2 |
| `Sensor/maxicam.c` | 无直接对应 | 未移植 | MaixCam 视觉模块，xvshen 用 ms_find_line 替代 |

### 2.2 执行层

| xiao 模块 | xvshen 模块 | 状态 | 说明 |
|-----------|-------------|------|------|
| `Motor/wheel.c` | `Motor/wheel.c` | 已移植 | xvshen 支持 4 轮(只用 0-1)，`cur_speed` 在外部读取，`set_wheel_dir` 用 `wheel->trust` 而非参数 |
| `Motor/servo.c` | `Motor/servo.c` | 已移植 | 角度范围参数不同(270/180 vs 180/270) |

### 2.3 设备层

| xiao 模块 | xvshen 模块 | 状态 |
|-----------|-------------|------|
| `Device/button.c` | `Device/button.c` | 已移植 |
| `Device/led.c` | `Device/led.c` | 已移植 |
| `Device/buzzer.c` | `Device/buzzer.c` | 已移植 |

### 2.4 工具层

| xiao 模块 | xvshen 模块 | 状态 |
|-----------|-------------|------|
| `Tool/pid.c` | `Tool/pid.c` | 一致 |
| `Tool/math_tool.h` | `Tool/math_tool.h` | 一致 |
| `Tool/log.c` | `Tool/log.c` | 一致 |
| `Tool/array.c` | `Tool/array.c` | 一致 |

### 2.5 控制层（核心差异）

| xiao | xvshen | 说明 |
|------|--------|------|
| `timer_it.c` (254行) | `timer_it.c` (50行) + `status.c` + `Defect.c` (440行) | xiao 的单文件被拆成三层 |
| `status.state.motion` | `status.state.motion` | 枚举值一致: STOP=0, KEEP_ANGLE=1, FIND_LINE=2 |
| `status.move.find_line_pid` | `status.state.status_pid.follow_line_pid` | PID 参数一致但路径不同 |
| `status.move.keep_angle_pid` | `status.state.status_pid.keep_angle_pid` | 同上 |
| `corner_count` (static) | `corner_count` (static in Defect.c) | 一致 |
| `final_stop` (bool) | `final_stop` (uint8_t) | 类型不同，语义一致 |
| 无阶段机 | `race_phase` (Q1_WAIT_KEY2 → ... → Q1_BA_FINAL) | xvshen 新增的阶段推进 |
| 无任务系统 | `TASK` 结构体 + `update_task()` | xvshen 新增的任务调度 |

---

## 三、数据流对比

### 3.1 xiao 数据流 (20ms 周期)

```
HAL_TIM_PeriodElapsedCallback (htim5, 每1ms)
  │
  ├─ time % 10 == 0:
  │    driver_gw_analogue()          ← 灰度传感器采集
  │
  └─ time % 20 == 0:
       │
       ├─ get_gyr_raw_data(&hi2c1)   ← I2C 读陀螺仪
       ├─ yaw = get_gyr_value(gyr_z_yaw)
       ├─ line = digital_8bit         ← 8-bit 线状态
       │
       ├─ 丢线/有线计数
       │    if (line == 0x00) lost_line_cnt++
       │    else              line_seen_cnt++
       │
       ├─ switch (motion):
       │    case FIND_LINE:
       │      ├─ 丢线→STOP(if final_stop) 或 enter_keep_angle
       │      ├─ 路口→enter_keep_angle
       │      └─ find_line()         ← PID 巡线 → 设 tar_speed
       │    case KEEP_ANGLE:
       │      ├─ 重见线→corner_count++, back_to_find_line
       │      └─ PID 保持角度 → 设 tar_speed
       │    case STOP:
       │      └─ 刹车 300ms → 停车
       │
       └─ driver_status(&status)     ← 驱动轮子/LED/蜂鸣器
            ├─ driver_wheel(0)       ← PID 速度环 → PWM
            ├─ driver_wheel(1)
            └─ driver_LED / driver_BUZZER
```

### 3.2 xvshen 数据流 (20ms 周期)

```
HAL_TIM_PeriodElapsedCallback (htim5, 每1ms)
  │
  ├─ time % 10 == 0:
  │    driver_gw_analogue()          ← 灰度传感器采集
  │
  └─ time % 20 == 0:
       update_status(&status)
       │
       ├─ [感知层]
       │    ├─ get_wheel_speed(0)    ← 编码器读数 → cur_speed
       │    ├─ get_wheel_speed(1)
       │    ├─ get_gyr_raw_data()    ← I2C 读陀螺仪
       │    ├─ cur_angle = get_gyr_value(gyr_z_yaw)
       │    └─ driver_button() ×2    ← 按钮扫描
       │
       ├─ [策略层] update_task(&status)
       │    └─ task_basic_1_update(s)
       │         ├─ line = digital_8bit
       │         ├─ yaw = cur_angle
       │         ├─ 丢线/有线计数
       │         └─ switch (race_phase):
       │              case Q1_WAIT_KEY2: → 等长按
       │              case Q1_START_A_TURN ... Q1_BA_FINAL:
       │                ├─ 初始转弯 (initial_turn_done)
       │                └─ switch (motion):
       │                     case FIND_LINE:  路口/丢线/巡线
       │                     case KEEP_ANGLE: 转弯完成/角度PID
       │                     case STOP:       刹车逻辑
       │
       ├─ [执行层] motion_execute(s)
       │    ├─ FIND_LINE:  PID(diff) → diff_drive → tar_speed
       │    ├─ KEEP_ANGLE: PID(err)  → diff_drive → tar_speed
       │    └─ STOP:       (空 — 刹车由 Defect.c 处理)
       │
       └─ [驱动层]
            ├─ driver_wheel(0)       ← PID 速度环 → PWM
            ├─ driver_wheel(1)
            ├─ driver_LED / driver_BUZZER
            └─ driver_servo ×2
```

### 3.3 关键差异点

| 差异 | xiao | xvshen | 影响 |
|------|------|--------|------|
| cur_speed 读取时机 | `driver_wheel()` 内部读取（PID计算前） | `update_status()` 开头读取（整个周期前） | 时序有微小差异，不影响功能 |
| yaw 变量作用域 | 20ms 块级局部变量 | `status.state.cur_angle`（update_status 写入），`task_basic_1_update` 用局部 `yaw` 副本 | 值一致 |
| PID 调用位置 | 状态机内直接调用 `compute_pid()` | `motion_execute()` 中集中调用 | PID 更新时机一致 |
| find_line 调用时机 | 仅 `line != 0x00` 时调用 | `motion_execute()` 无条件调用 FIND_LINE（即使 line==0x00） | 丢线时 xvshen 会驱动直行(base_speed)，xiao 保持上次速度 |
| 阶段推进 | 无，纯靠 corner_count | race_phase 自动推进（每完成一个弯就推进） | xvshen 的阶段机更可靠 |

---

## 四、Q1 赛道阶段机设计

### 4.1 赛道布局

```
      A ─────────────────── B
      │                     │
      │    ┌───────────┐    │
      │    │           │    │
      │    │  赛道内部  │    │
      │    │           │    │
      │    └───────────┘    │
      │                     │
      D ─────────────────── C
```

小车从 A 点出发 → AB 直道 → B 点左转 → BC 直道 → C 点左转 → CD 直道 → D 点左转 → DA 直道 → A 点停车。

### 4.2 阶段枚举

```c
typedef enum Q1_RACE_PHASE {
    Q1_WAIT_KEY2,      // 0: 等待 PD2 长按释放
    Q1_START_A_TURN,   // 1: A 点起步 左转90°
    Q1_SIDE_AD,        // 2: A→D 直道巡线（注意：方向是从A到D）
    Q1_TURN_D,         // 3: D 点左转
    Q1_SIDE_DC,        // 4: D→C 直道巡线
    Q1_TURN_C,         // 5: C 点左转
    Q1_SIDE_CB,        // 6: C→B 直道巡线
    Q1_TURN_B,         // 7: B 点左转
    Q1_BA_FINAL,       // 8: B→A 最后直道 → 丢线停车
} Q1_RACE_PHASE;
```

### 4.3 阶段推进图

```
                    ┌─────────────┐
                    │ Q1_WAIT_KEY2│  等 PD2 长按 → startup_release
                    └──────┬──────┘
                           ↓
                    ┌──────────────┐
                    │Q1_START_A_TURN│  A点初始左转 (initial_turn_done)
                    └──────┬───────┘
                           ↓ (KEEP_ANGLE 完成)
                    ┌──────────────┐
                    │  Q1_SIDE_AD  │  A→D 直道巡线
                    └──────┬───────┘
                           ↓ (检测到路口)
                    ┌──────────────┐
                    │  Q1_TURN_D   │  D点左转
                    └──────┬───────┘
                           ↓ (KEEP_ANGLE 完成)
                    ┌──────────────┐
                    │  Q1_SIDE_DC  │  D→C 直道巡线
                    └──────┬───────┘
                           ↓ (检测到路口)
                    ┌──────────────┐
                    │  Q1_TURN_C   │  C点左转
                    └──────┬───────┘
                           ↓ (KEEP_ANGLE 完成)
                    ┌──────────────┐
                    │  Q1_SIDE_CB  │  C→B 直道巡线
                    └──────┬───────┘
                           ↓ (检测到路口)
                    ┌──────────────┐
                    │  Q1_TURN_B   │  B点左转
                    └──────┬───────┘
                           ↓ (KEEP_ANGLE 完成)
                    ┌──────────────┐
                    │  Q1_BA_FINAL │  B→A 最后直道 → 丢线 → STOP
                    └──────────────┘
```

### 4.4 阶段切换时机

| 当前阶段 | 切换条件 | 下一阶段 |
|----------|----------|----------|
| Q1_WAIT_KEY2 | PD2 长按释放 (`startup_release`) | Q1_START_A_TURN |
| Q1_START_A_TURN | 首次进入: `!initial_turn_done` → 直接进入 KEEP_ANGLE | (转弯中) |
| Q1_START_A_TURN | KEEP_ANGLE 完成 (`line_seen_cnt >= 3`) | Q1_SIDE_AD |
| Q1_SIDE_AD | 检测到路口 (`r != Straight`) → 进入 KEEP_ANGLE | Q1_TURN_D |
| Q1_TURN_D | KEEP_ANGLE 完成 | Q1_SIDE_DC |
| Q1_SIDE_DC | 检测到路口 → 进入 KEEP_ANGLE | Q1_TURN_C |
| Q1_TURN_C | KEEP_ANGLE 完成 | Q1_SIDE_CB |
| Q1_SIDE_CB | 检测到路口 → 进入 KEEP_ANGLE | Q1_TURN_B |
| Q1_TURN_B | KEEP_ANGLE 完成 | Q1_BA_FINAL |
| Q1_BA_FINAL | 丢线 (`lost_line_cnt >= 3`) + `final_stop==1` | STOP (刹车) |

---

## 五、已修复的问题清单

### 5.1 灰度传感器 MUX 引脚

**文件**: `User/Sensor/gw_analogue.c` → `select_channel()`

**问题**: 移植版使用了错误的 GPIO 名 (IO2/IO3/IO4)，这些不是灰度传感器 MUX 的实际引脚。

**修复**: 改为 AD0(PF9) / AD1(PE6) / AD2(PC3)，与 xiao 的硬件一致。

```c
// 错误
if (channel & 0x01) HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, ...);
// 正确
if (channel & 0x01) HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, ...);
```

### 5.2 CCD 编译错误

**文件**: `Core/Inc/main.h`, `User/It/timer_it.c`, `User/Sensor/ccd.c`

- `main.h` 缺少 `CCD_CLK_Pin` / `CCD_SI_Pin` 宏定义 → 添加 GPIO PB2/PB1
- `timer_it.c` 多余 `#include "ccd.h"` → 删除
- `ccd.c` 中 `HAL_ADC_Start_DMA` 参数类型不匹配 → 添加 `(uint32_t *)` 转换

### 5.3 初始状态破坏 arming 流程

**文件**: `Core/Src/main.c`

曾错误地将初始 `motion` 设为 `FIND_LINE`，导致上电就转。恢复为 `STOP`，保留 armed → PD2 长按 → startup_release 的正常启动流程。

### 5.4 自动停车（corner_count 依赖陀螺仪）

**文件**: `User/Status/Defect.c`

**根因**: corner_count 原本在 KEEP_ANGLE 退出时依赖 `ABS(yaw_delta) > 10.0f` 来确认转弯完成。陀螺仪数据不可靠时，delta 可能不超过 10 度，导致 corner_count 永远到不了 4。

**修复**: corner_count 改为在**检测到路口决定转弯时**计数，完全脱离陀螺仪依赖。

```c
// 初始转弯 (Q1_START_A_TURN)
corner_count++;  // cc: 0 → 1

// 路口检测 (FIND_LINE → junction)
corner_count++;  // cc: 1→2, 2→3, 3→4
if (corner_count >= 4) final_stop = 1;
```

---

## 六、GYR 结构体差异及潜在风险

### 6.1 结构体对比

```c
// xiao: GYR 结构体 (28 + 12 = 40 bytes)
typedef struct GYR {
    uint8_t data_buf[24];      // 24B
    uint8_t device_addr;       // 1B
    uint8_t data_start_addr;   // 1B
    // padding 2B
    float cur_angle;           // 4B
    float initial_angle;       // 4B
    float tar_angle;           // 4B
} GYR;

// xvshen: GYR 结构体 (28 + PID size)
typedef struct GYR {
    uint8_t data_buf[24];      // 24B
    uint8_t device_addr;       // 1B
    uint8_t data_start_addr;   // 1B
    // padding 2B
    PID gy901_keep_angle_pid;  // ~32B (kp,ki,kd,integral,error,...)
} GYR;
```

### 6.2 影响

xvshen 的 GYR 多了 `gy901_keep_angle_pid` 但缺少 `cur_angle/initial_angle/tar_angle`。当前代码中：

- `cur_angle` → 改用 `status.state.cur_angle`
- `initial_angle` → 改用 `status.state.initial_angle`
- `tar_angle` → 改用 `status.state.turn.target_yaw`
- `gy901_keep_angle_pid` → 未使用（`motion_execute` 用的是 `status.state.status_pid.keep_angle_pid`）

**无功能影响**，但 `init_gyr()` 中创建的 PID 是无用的，浪费内存。

---

## 七、wheel.c 差异

| 差异点 | xiao | xvshen | 影响 |
|--------|------|--------|------|
| 轮数 | 2 (wheel[0-1]) | 4 (wheel[0-3]) | xvshen 只驱动 0-1 |
| cur_speed 读取 | `driver_wheel()` 内部读取 | `update_status()` 提前读取 | 时序微小差异 |
| set_wheel_dir 参数 | `trust` 参数 | `wheel->trust` 成员 | 值相同，无影响 |
| 起步限幅条件 | `cur_speed <= 10` | `ABS(cur_speed) < 10` | 负速度时行为不同，但正常行驶不影响 |
| 制动时 trust | 无特殊处理 | 无特殊处理 | 一致的 |

---

## 八、color_count 计数方案对比

### 8.1 原始方案（依赖陀螺仪——已废弃）

```
KEEP_ANGLE 中:
  重见线 LINE_BACK_CNT 次 →
    计算 yaw_delta = yaw - entry_yaw →
      如果 ABS(yaw_delta) > 10° → corner_count++
```

**问题**: 陀螺仪不可靠时 delta 偏小，corner_count 漏计。

### 8.2 新方案（路口检测——当前方案）

```
检测到路口 (r != Straight, junction_cnt >= 1) →
  决定转弯 (final_stop == 0) →
    corner_count++
    如果 corner_count >= 4 → final_stop = 1

初始转弯 (Q1_START_A_TURN, !initial_turn_done) →
  corner_count++
```

**优势**: 完全脱离陀螺仪，依赖灰度传感器（稳定可靠）。

### 8.3 计数流程

```
corner_count 初始值: 0

Q1_START_A_TURN  初始左转     → cc = 1
Q1_SIDE_AD       检测到路口    → cc = 2
Q1_SIDE_DC       检测到路口    → cc = 3
Q1_SIDE_CB       检测到路口    → cc = 4 → final_stop = 1

Q1_BA_FINAL     丢线          → STOP (刹车300ms → 停车)
```

---

## 九、调试日志说明

在 `Defect.c` 中添加了 USART1 串口日志（需 `#include "log.h"`）：

| 日志格式 | 触发时机 | 含义 |
|----------|----------|------|
| `START_TURN cc=N ph=N` | 初始转弯开始 | 第1个弯开始 |
| `JUNC_TURN cc=N fs=N ph=N` | 检测到路口转弯 | 第2-4个弯开始 |
| `TURN_END cc=N fs=N ph=N` | KEEP_ANGLE 退出 | 转弯完成，回到巡线 |
| `STOP cc=N fs=N ph=N` | 丢线+final_stop | 刹车开始 |
| `LOST_TURN cc=N ph=N` | 丢线+!final_stop | 兜底转弯 |

`ph` 值对应 `Q1_RACE_PHASE` 枚举：
```
0=Q1_WAIT_KEY2, 1=Q1_START_A_TURN, 2=Q1_SIDE_AD, 3=Q1_TURN_D,
4=Q1_SIDE_DC, 5=Q1_TURN_C, 6=Q1_SIDE_CB, 7=Q1_TURN_B, 8=Q1_BA_FINAL
```

---

## 十、后续工作

### 10.1 待验证

- [ ] 实车测试 corner_count 是否稳定到达 4
- [ ] 确认 Q1_BA_FINAL 最后的丢线是否能触发 STOP
- [ ] 验证刹车力度（-50 × 300ms）是否足够停稳

### 10.2 可选优化

| 项目 | 说明 |
|------|------|
| 移除 GYR 中无用 PID | `gy901_keep_angle_pid` 从未使用，浪费约32字节 RAM |
| 统一 PID 命名 | xiao 的 `status.move.*_pid` vs xvshen 的 `status.state.status_pid.*_pid` |
| 清理 road.h/.c | 路口判断已完全迁移到 gw_analogue.c 的 `get_road_type()` |
| 完善 Q2/Q3/Q4 任务 | `task_basic_2_update()` 等目前是空桩 |
| CCD 验证 | ccd.c 标注「未验证」，需要实际测试 |
