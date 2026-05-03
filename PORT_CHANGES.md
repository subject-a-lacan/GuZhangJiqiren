# 移植修改记录

从 `xiao\xiaosaishenche` 移植到 `xvshen\GuZhangJiqiren` 的所有修改。

---

## 1. 灰度传感器 MUX GPIO 引脚修正

**文件**: `User/Sensor/gw_analogue.c`

**问题**: 移植版 `select_channel()` 使用了错误的 GPIO 引脚名 (IO2/IO3/IO4)，导致灰度传感器无法正确切换通道，小车不巡线只转圈。

**修改**: 将引脚名改为与硬件对应的 AD0/AD1/AD2。

```c
// 修改前（错误）
if (channel & 0x01) HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, ...);
if (channel & 0x02) HAL_GPIO_WritePin(IO3_GPIO_Port, IO3_Pin, ...);
if (channel & 0x04) HAL_GPIO_WritePin(IO4_GPIO_Port, IO4_Pin, ...);

// 修改后（正确）
if (channel & 0x01) HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, ...);
if (channel & 0x02) HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, ...);
if (channel & 0x04) HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, ...);
```

---

## 2. CCD 编译错误修复

### 2a. 添加 CCD 引脚定义

**文件**: `Core/Inc/main.h`

**问题**: `ccd.c` 引用了 `CCD_CLK_GPIO_Port`、`CCD_CLK_Pin` 等未定义的宏，导致 20 个编译错误。

**修改**: 在 main.h 的 Private defines 区域添加：

```c
#define CCD_CLK_Pin GPIO_PIN_2
#define CCD_CLK_GPIO_Port GPIOB
#define CCD_SI_Pin GPIO_PIN_1
#define CCD_SI_GPIO_Port GPIOB
```

### 2b. 移除 timer_it.c 中未使用的 CCD 引用

**文件**: `User/It/timer_it.c`

**修改**: 删除 `#include "ccd.h"`（该文件未使用 CCD 相关代码）。

### 2c. 修正 ADC DMA 类型转换

**文件**: `User/Sensor/ccd.c`

**问题**: `HAL_ADC_Start_DMA` 参数类型不匹配导致编译警告。

**修改**: 添加 `(uint32_t *)` 强制类型转换。

```c
// 修改前
HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&BUFF_DATA_1[(cnt - 10) / 3], 1);
// 这行原来缺少 (uint32_t *) 转换
```

---

## 3. 初始状态恢复（保留 arming 流程）

**文件**: `Core/Src/main.c`

**问题**: 曾将初始状态改为 `motion = FIND_LINE, base_speed = 50`，导致一上电就开始转，跳过了 armed → PD2 长按的启动流程。

**修改**: 恢复为原始行为，初始状态保持 STOP。

```c
// main.c 中 after_init_state() 之后
status.state.motion = STOP;  // 保持 STOP，等待 armed + PD2 长按
```

---

## 4. 自动停车修复（核心改动）

**文件**: `User/Status/Defect.c`

**问题**: 移植版 corner_count 依赖陀螺仪 yaw 角度差 `ABS(delta) > 10.0f` 来判断转弯完成。陀螺仪数据不可靠，导致 corner_count 无法到达 4，final_stop 永远不会置 1，小车跑完一圈不会停。

**修改**: 将 corner_count 的计数时机从「转弯完成时（依赖陀螺仪）」改为「检测到路口决定转弯时（依赖灰度传感器）」。

### 4a. 初始转弯处计 corner_count

```c
// Q1_START_A_TURN 首次进入时
if (s->task.race_phase == Q1_START_A_TURN && !initial_turn_done) {
    initial_turn_done = 1;
    turn_dir = -90.0f;
    s->state.turn.entry_yaw = yaw;
    lost_line_cnt = 0;
    corner_count++;  // ← 新增：初始转弯也计 corner_count
    enter_keep_angle(s, yaw + turn_dir);
    break;
}
```

### 4b. 路口检测时计 corner_count

```c
// FIND_LINE 中检测到路口时
if (r != Straight) {
    junction_cnt++;
    if (junction_cnt >= JUNCTION_CONFIRM) {
        if (final_stop) {
            turn_dir = 0.0f;
        } else {
            turn_dir = -90.0f;
            s->state.turn.entry_yaw = yaw;
            junction_cnt = 0;
            reset_cross_state(s);
            corner_count++;                    // ← 新增：检测到路口就计
            if (corner_count >= 4) final_stop = 1;  // ← 新增：第4个路口置 final_stop
            // 阶段过渡 ...
            enter_keep_angle(s, yaw + turn_dir);
            break;
        }
        junction_cnt = 0;
        reset_cross_state(s);
    }
}
```

### 4c. 移除 KEEP_ANGLE 中的陀螺仪依赖

```c
// 修改前：依赖 ABS(yaw_delta) > 10.0f
if (line_seen_cnt >= LINE_BACK_CNT) {
    float delta = yaw - s->state.turn.entry_yaw;
    if (ABS(delta) > 10.0f) {
        corner_count++;
        if (corner_count >= 4) final_stop = 1;
    }
    back_to_find_line(s);
}

// 修改后：不再依赖陀螺仪，仅保留阶段推进
if (line_seen_cnt >= LINE_BACK_CNT) {
    back_to_find_line(s);
    // 阶段过渡 ...
}
```

### 4d. 添加调试日志

在 `Defect.c` 顶部添加 `#include "log.h"`，并在关键节点输出串口日志（USART1）：

| 日志 | 含义 |
|------|------|
| `START_TURN cc=N ph=N` | 初始转弯 |
| `JUNC_TURN cc=N fs=N ph=N` | 检测到路口转弯 |
| `TURN_END cc=N fs=N ph=N` | 转弯完成 |
| `STOP cc=N fs=N ph=N` | 进入刹车 |
| `LOST_TURN cc=N ph=N` | 丢线但 final_stop=0（兜底转弯） |

---

## corner_count 计数流程

```
corner_count = 0

START_A_TURN 初始转弯    → cc = 1  (起点左转)
Q1_SIDE_AD 检测到路口    → cc = 2  (第一个路口)
Q1_SIDE_DC 检测到路口    → cc = 3  (第二个路口)
Q1_SIDE_CB 检测到路口    → cc = 4  → final_stop = 1  (第三个路口)

Q1_BA_FINAL 巡线 → 丢线 → STOP (刹车 300ms → 停车)
```

---

## 修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `User/Sensor/gw_analogue.c` | MUX 引脚 IO2/IO3/IO4 → AD0/AD1/AD2 |
| `Core/Inc/main.h` | 添加 CCD_CLK、CCD_SI 引脚定义 |
| `User/It/timer_it.c` | 移除未使用的 `#include "ccd.h"` |
| `User/Sensor/ccd.c` | 添加 `(uint32_t *)` 类型转换 |
| `Core/Src/main.c` | 初始状态保持 STOP（保留 arming 流程） |
| `User/Status/Defect.c` | corner_count 改为路口检测计数 + 调试日志 |

---

## 5. 同步 xiao 更新 (2026-05-03)

xiao 项目更新后，将以下改进同步到 xvshen。

### 5a. GYR 结构体修复

**文件**: `User/Sensor/gy901.h`, `User/Sensor/gy901.c`

**问题**: xvshen 的 GYR 结构体包含从未使用的 `gy901_keep_angle_pid`（约32字节 RAM 浪费），且缺少 `cur_angle/initial_angle/tar_angle` 字段。

**修改**: 删除 PID 字段，添加 angle 字段（与 xiao 一致）。

```c
// 修改前
typedef struct GYR {
  uint8_t data_buf[24];
  uint8_t device_addr;
  uint8_t data_start_addr;
  PID gy901_keep_angle_pid;  // 无用
} GYR;

// 修改后
typedef struct GYR {
  uint8_t data_buf[24];
  uint8_t device_addr;
  uint8_t data_start_addr;
  float cur_angle;
  float initial_angle;
  float tar_angle;
} GYR;
```

`init_gyr()` 中移除 PID 创建，改为初始化 angle 字段为 0。

### 5b. 舵机 angle_error 参数

**文件**: `User/Motor/servo.h`, `User/Motor/servo.c`, `User/Status/status.c`

**问题**: xvshen 的舵机缺少角度误差补偿参数。

**修改**:
- SERVO 结构体添加 `float angle_error` 字段
- `init_servo(SERVO *s, uint8_t which, float max_angle, float angle_error)` → 4 参数
- `driver_servo` 使用 `angle + angle_error` 计算实际角度
- `init_motor()` 中舵机参数改为 `servo[0]=(1, 270, 35)`, `servo[1]=(2, 180, 0)`

### 5c. 轮子起步限幅条件

**文件**: `User/Motor/wheel.c`

**修改**: 起步限幅条件从 `ABS(cur_speed) < 10` 改为 `cur_speed <= 10`（与 xiao 一致）。

### 5d. 初始转弯 pivot turn

**文件**: `User/Status/Defect.c`, `User/Status/status.h`

**问题**: xvshen 的初始转弯使用 `TURN_BASE_SPEED=35` 前进+转弯，xiao 使用 `base_speed=0` 原地转（pivot turn），更快更稳定。

**修改**:
- 新增静态变量 `initial_corner_counted`, `initial_turn_ticks`
- 新增常量 `INIT_TURN_DIFF 30`
- KEEP_ANGLE 增加初始转弯分支：原地转（左右轮反向），等待 `ticks≥5 && line_seen≥3` 后切巡线

```c
// KEEP_ANGLE 中新增的初始转弯分支
if (!initial_corner_counted) {
    float dir = (turn_dir < 0) ? -1.0f : 1.0f;
    s->motor.wheel[0].tar_speed = (int16_t)(dir * INIT_TURN_DIFF);
    s->motor.wheel[1].tar_speed = (int16_t)(-dir * INIT_TURN_DIFF);
    initial_turn_ticks++;
    if (initial_turn_ticks >= 5 && line_seen_cnt >= 3) {
        initial_corner_counted = 1;
        back_to_find_line(s);
        task_set_phase(s, Q1_SIDE_AD);
    }
    break;
}
```

### 5e. 路口直接 STOP + 丢线兜底

**文件**: `User/Status/Defect.c`

**问题**: 之前 corner_count 到达 4 后设置 `final_stop=1`，然后依赖 Q1_BA_FINAL 丢线触发 STOP。xiao 更新后改为路口直接 STOP。

**修改**:
- 路口检测：`corner_count >= 4` → 直接 STOP（不等丢线），日志 `STOP_JUNC`
- 丢线：`corner_count++` → `if >= 5` → STOP（兜底），日志 `STOP_LOST`
- `corner_count` 在初始转弯时**赋值** 1（不再递增），在路口和丢线时递增
- `final_stop` 变量变为死代码（仅保留结构兼容）

### 5f. 新增工具文件

| 新文件 | 说明 |
|--------|------|
| `User/Status/para.h` | 类型无关参数访问（`get_cur_val` 等） |
| `User/Status/para.c` | 实现 |
| `User/Status/task_defer.h` | 通用延迟任务调度器（`TASK_DEFER`，避免与 `TASK` 冲突） |
| `User/Status/task_defer.c` | 实现（条件触发、超时、任务池） |

### corner_count 新流程

```
corner_count = 0

Q1_START_A_TURN 初始转弯    → cc = 1  (赋值)
Q1_SIDE_AD 检测到路口       → cc = 2  (递增)
Q1_SIDE_DC 检测到路口       → cc = 3  (递增)
Q1_SIDE_CB 检测到路口       → cc = 4  → STOP_JUNC (直接刹车)
(N/A 丢线)                 → cc = 5  → STOP_LOST (兜底)
```

### 修改文件清单 (本次)

| 文件 | 修改内容 |
|------|----------|
| `User/Sensor/gy901.h` | GYR 结构体：删除无用 PID，添加 angle 字段 |
| `User/Sensor/gy901.c` | init_gyr() 移除 PID 创建，初始化 angle 字段 |
| `User/Motor/servo.h` | SERVO 添加 angle_error 字段，init_servo 改为4参数 |
| `User/Motor/servo.c` | driver 使用 angle+angle_error，init 4参数+PWM启动 |
| `User/Motor/wheel.c` | cur_speed 条件 ABS()→直接比较 |
| `User/Status/status.h` | 添加 INIT_TURN_DIFF 常量 |
| `User/Status/status.c` | init_motor 舵机参数更新 |
| `User/Status/Defect.c` | pivot 初始转弯 + 路口直接STOP + 丢线兜底 |
| `User/Status/para.h` | **新文件** — 参数工具 |
| `User/Status/para.c` | **新文件** |
| `User/Status/task_defer.h` | **新文件** — 通用延迟任务调度器 |
| `User/Status/task_defer.c` | **新文件** |
