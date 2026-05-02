# 本次迭代优化总结

## 一、路口判断

### 1.1 `init_road_determine()` 补全缺失字段

| 文件 | `User/Sensor/gw_analogue.c` |
|------|------|
| 函数 | `init_road_determine()` |

**问题**：`cross->maybe` 字段未初始化，上电后可能是随机值，导致 `get_road_type()` 的 maybe 倒计时状态机在首次触发前行为不确定。

**修改**：添加 `cross->maybe = 0;`，与 integral/data_buf/cross/cross_cnt 一并清零。

---

### 1.2 `serve_road()` 统一管理路口计数

| 文件 | `User/Sensor/gw_analogue.c` |
|------|------|
| 函数 | `serve_road()` |

**问题**：全局 `cross_cnt` 和结构体 `cross->cross_cnt` 在路口确认后未同步递增，导致 TASK 状态机无法正确获取路口通过次数。

**修改**：在 `serve_road()` 中对非 Straight/UnknowRoad 路口同时递增两个计数器：
```c
if (road != Straight && road != UnknowRoad) {
    cross->cross_cnt++;
    cross_cnt++;
}
```

---

### 1.3 `follow_line()` 简化——纯 PID + 差速

| 文件 | `User/Status/status.c` |
|------|------|
| 函数 | `follow_line()` |

**问题**：旧的 `follow_line()` 内嵌了路口判断和开环转弯逻辑（LeftRoad/RightRoad/CrossRoad 的 20/-20 tank turn），与 TASK 架构冲突——传感器观测、TASK 决策、运动执行三层职责混淆。

**修改**：`follow_line()` 精简为只做两件事：PID 计算 + 差速输出。路口观测由 `driver_gw_analogue` → `cross.cross` 提供，转弯决策交由 `update_task` 的 race_phase 状态机负责。

**修改前**（~26 行，含路口判断和 tank turn 分支）：
```c
// 包含 Turn_or_Straight()、cross_delay、LeftRoad/RightRoad/CrossRoad 分支
```

**修改后**（4 行）：
```c
void follow_line(STATUS *status) {
  float diff = compute_pid(&status->state.status_pid.follow_line_pid, status->sensor.gw_analogue.diff);
  status->motor.wheel[0].tar_speed = status->state.base_speed - (int16_t)diff;
  status->motor.wheel[1].tar_speed = status->state.base_speed + (int16_t)diff;
}
```

**附带清理**：`Turn_or_Straight()` 定义保留但已从所有调用链中移除（待后续彻底删除）。

---

### 1.4 路口返回直行条件诊断（已知问题，暂未修改）

`get_road_type()` 中退出路口状态的条件仅有 `0x18 / 0x10 / 0x08` 三种位图，实测可能需要扩展（如 `0x1C, 0x38` 等），否则车辆可能卡在路口状态无法恢复 Straight。已记录，待实车测试后调整。

---

### 1.5 默认阈值诊断（已知问题，需校准解决）

`init_gw_analogue()` 中默认 `digital_high_threshold` = 46~47，对 12 位 ADC（0~4095）而言过小，导致 `get_gw_analogue_analogue_diff()` 中 `channel[i] < high_threshold` 条件永远不成立，diff 恒为 0，车辆直走不巡线。

**解决方式**：长按 PD2 进行白/黑校准后，阈值会被写入实际量级。

---

## 二、停止逻辑

### 2.1 新增硬停止标志 `stop_cmd`

| 涉及文件 |
|------|
| `User/Status/Defect.h` — TASK 结构体 |
| `User/Status/Defect.c` — init_task / task_start / task_finish / task_stop |
| `User/Status/status.c` — update_status |
| `User/Motor/wheel.c` — driver_wheel |

**问题**：旧停止逻辑仅将 `wheel[].tar_speed` 置 0，依赖 PID 死区（`tar_speed==0 && ABS(cur_speed)<3` → trust=0）自然收敛。但编码器噪声和残留积分会导致 PID 在死区附近仍有小量输出，表现为停车后车轮抖动。

**方案**：新增 `stop_cmd` 硬停止标志，在 PWM 输出层面直接切断，跳过 PID 计算。

**语义严格区分**：
- `stop_cmd`：电机层硬停止。1 = 禁止 PWM 输出，0 = 允许。
- `stop_request`：TASK 层逻辑请求。由按钮/蓝牙设置，由 `update_task` 消费。本次不修改其行为。

**修改细节**：

| 位置 | 操作 | 说明 |
|------|------|------|
| `TASK` 结构体 | 新增 `uint8_t stop_cmd` | 硬停止命令字段 |
| `init_task()` | `stop_cmd = 1` | 上电默认禁止 PWM，防误动 |
| `task_start()` | `stop_cmd = 0` | 任务启动后允许 PWM 输出 |
| `task_finish()` | `stop_cmd = 1` | 任务完成，切断 PWM |
| `task_stop()` | `stop_cmd = 1` | 急停，切断 PWM |
| `update_status()` — FIND_LINE | `stop_cmd = 0` | 巡线状态允许 PWM |
| `update_status()` — KEEP_ANGLE | `stop_cmd = 0` | 保角状态允许 PWM |
| `update_status()` — STOP | `stop_cmd = 1` | 停止状态切断 PWM |
| `update_status()` — MOTOR_TEST | `stop_cmd = 0` | 电机测试允许 PWM |
| `driver_wheel()` 顶部 | if stop_cmd → trust=0, PWM写0, return | **在 compute_pid() 之前**阻断，防止积分累积 |

**修改前**：STOP → tar_speed=0 → PID 仍在计算 → 死区附近抖动

**修改后**：STOP → stop_cmd=1 → driver_wheel 立即 PWM=0 + return → PID 完全跳过，无抖动

---

## 三、校准时的声光提示

| 文件 | `User/Sensor/gw_analogue.c` |
|------|------|
| 函数 | `correct_gw_analogue()` |

**问题**：校准过程无感官反馈。白校准时仅亮板载 LED，黑校准时仅灭板载 LED，无法直观区分两个阶段，也无法确认采集是否完成。

**修改**：

| 阶段 | 板载 LED | LED1 | LED2 | 蜂鸣器 |
|------|:---:|:---:|:---:|------|
| 白校准 (sta=0) | ○ | ○ | ● | 每路 ADC Stop 后触发，合成一声长响（off_time = now + 500ms） |
| 黑校准 (sta=1) | ○ | ● | ○ | 同上 |
| 校准完成 | ○ | ○ | ○ | — |

LED 组合（0,0,1 和 0,1,0）与 6 种 TASK 任务 LED 编码不冲突。

---

## 涉及文件汇总

| 文件 | 改动类型 |
|------|------|
| `User/Status/Defect.h` | TASK 结构体新增 stop_cmd 字段 |
| `User/Status/Defect.c` | init_task / task_start / task_finish / task_stop 增加 stop_cmd 处理 |
| `User/Status/status.c` | follow_line 简化；update_status 四个 motion 分支增加 stop_cmd 设置 |
| `User/Motor/wheel.c` | driver_wheel 顶部增加硬停止判断（主动写 PWM=0 + return） |
| `User/Sensor/gw_analogue.c` | init_road_determine 补 maybe=0；serve_road 统一计数；correct_gw_analogue 增加 LED + 蜂鸣器反馈 |
