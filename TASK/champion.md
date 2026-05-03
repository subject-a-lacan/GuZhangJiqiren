# 第一问全流程与实现方案

> 任务架构（选题、按键、蓝牙、cmd、三层分层）见 [jiegou.md](jiegou.md)。本文档仅负责第一问内部逻辑。

## 路线

```
A (左上) ●═══════════● B (右上)
         ║           ║
         ║  逆时针   ║
         ║           ║
D (左下) ●═══════════● C (右下)
```

- 发车：车身在 AB 上，车头朝左（A），前沿中心在 AD 线外沿
- 路径：A(左转) → D(左转) → C(左转) → B(左转) → A(停车)
- 全程 400cm，≤15 秒，停车 D1 ≤ 3cm

---

## 完整状态机

```
[上电初始化] → motion = STOP
     ↓ 按键触发
[FIND_LINE 状态] → 正常巡线 + 拐角检测
     ↓ cross_cnt: 0→1(A) →2(D) →3(C) →4(B)
[BA_FINAL 状态] → 拐角 4 转完，BA 边编码器测距停车
     ↓ 里程达标
[STOP] → 声光提示，结束
```

cross_cnt 从 0 开始，起点的拐角 A 正常检测——车踩油门蠕动不到 1cm，传感器扫到 LeftRoad，左转进入 AD 边，cross_cnt 变为 1。

---

## 各阶段详细实现

### 阶段 0：STOP（初始）

```
motion = STOP, task_running = 0
base_speed = 0
task_id 已由 jiegou.md 中的按键/蓝牙机制设定
PD2 双击或蓝牙 'G' → cmd = 1
调度器检测 cmd==1 → task_running = 1, motion = FIND_LINE
蜂鸣器长鸣一声
cross_cnt = 0
```

### 阶段 1：FIND_LINE（正常巡线 + 拐角响应）

**逻辑**：

```
每周期：
  编码器累加里程（用于分段调速）
  get_road_type() 正常调用 → 修改 cross_cnt

  // === 分段速度 ===
  if (编码器里程 < SPRINT_THRESHOLD):    // 初值 80cm，实测校准
      base_speed = 30-35  ← 冲刺
  else:
      base_speed = 10-12  ← 降速准备转弯

  // === 拐角响应 ===
  if (检测到 Straight):
      PID 循线直走

  if (检测到 LeftRoad):
      执行转弯（见下方"转弯方案（待定）"）
      cross_cnt++
      清零编码器里程 ← 为下一条边重新计
      base_speed = 30 ← 恢复高速
```

**拐角序列**：

| 拐角 | cross_cnt 变化 | 位置 | 物理含义 |
|---|---|---|---|
| A | 0→1 | 左上 | 左转，车头朝左→朝下，驶入 AD |
| D | 1→2 | 左下 | 左转，车头朝下→朝右，驶入 DC |
| C | 2→3 | 右下 | 左转，车头朝右→朝上，驶入 CB |
| B | 3→4 | 右上 | 左转，车头朝上→朝左，驶入 BA |

**退出条件**：cross_cnt == 4 且 B 点转弯完成 → motion = BA_FINAL

---

### 阶段 2：BA_FINAL（最后直道 + 精确停车）

**进入条件**：拐角 4（B 点）转弯完成，cross_cnt == 4

**目的**：BA 边上以低速巡线，编码器测距触发停车。

**逻辑**：

```
进入时：
  清零编码器里程
  base_speed = 12  ← 低速精确

每周期：
  编码器累加里程

  // === 主逻辑：编码器测距停车 ===
  if (编码器里程 >= STOP_TARGET):  // 实测校准值
      motion = STOP
      task_running = 0
      声光提示

  // === 兜底 1：灰度传感器（加最低距离门槛） ===
  if (检测到 LeftRoad && 编码器里程 >= MIN_FINAL_DIST):
      motion = STOP
      task_running = 0

  // === 兜底 2：蓝牙核武器 ===
  if (蓝牙收到 'S'):
      motion = STOP
      task_running = 0
      cmd = 0

  其他情况：PID 巡线 Straight
```

**MIN_FINAL_DIST**（初值 ~90cm）：比 STOP_TARGET 略短。BA 边前半段即使灰度误判 LeftRoad 也不会触发停车，只有车确实接近 A 时才允许灰度兜底生效。

**STOP_TARGET 校准步骤**：
1. 车手动放理想停车位（前沿在 AD 外沿，车身在 AB）
2. 读编码器里程值 → STOP_TARGET
3. 赛前微调 ±1cm 至 D1 ≤ 3cm 稳定满足

**退出条件**：motion == STOP

---

## 转弯方案（待定）

### 问题

只靠陀螺仪角度闭环转 90° 不够——角度到了但车身可能不在线上，切回巡线 PID 会在盲区里乱拉。

### 思路：两段式转弯

```
检测到 LeftRoad
     ↓
[阶段 A：陀螺仪硬转]
  记录起始角度，目标 = 起始 - 90°
  左右轮反向 pivot
  陀螺仪误差 < 2° 持续 3 帧 → 硬转完成
     ↓
[阶段 B：低速探线]
  base_speed = 8-10（非常慢）
  直走，同时监控灰度传感器
  等 gray_straight_cnt >= 3（连续 3 帧看到 Straight）
     ↓
  咬线成功 → 恢复 base_speed，正常巡线
```

阶段 B 不盲走——低速下 FIND_LINE 的 PID 有能力把车从线旁边拉回来。关键是硬转完先确认传感器不处于全白，再开 PID。

如果阶段 B 超时（~500ms 还没看到 Straight）：做小的左右摆动扩大传感器扫描范围，还扫不到线就停车报警。

**注意**：Kp_turn 和 TURN_SPEED 需实测校准。由于转弯需要迅速，角度环的 KP 和 KI 可能需要给得激进一些。

---

## 与现有代码的映射

| 新增/修改内容 | 涉及文件 | 改动性质 |
|---|---|---|
| `TASK_ID`、`START_POSE` 枚举，`task_id`/`start_pose`/`cmd` 字段 | `status.h` 的 `STATE` | 新增 |
| `MOTION_STATION` 增加 `BA_FINAL` | `status.h` | 新增枚举值 |
| `mileage_cm` 变量 | `status.c` | 新增全局变量 |
| `update_mileage()` / `reset_mileage()` | `status.c` | 新增函数 |
| `task_basic_1_update()` 状态机 | `status.c` 或新文件 | 新增函数 |
| `update_status()` 中按 `task_id` 分派 | `status.c` 的 `update_status()` | 新增 switch |
| 按键逻辑重写 `server_button()` | `button.c` | 重写 PB11 相关逻辑 |
| 陀螺仪闭环转弯 + 低速探线 | 新文件或 `status.c` | 新增函数（待定） |
| 蓝牙 UART 命令解析 | `main.c` 或独立文件 | 新增 |
| `follow_line()` 中 LeftRoad 分支 | `status.c` 的 `follow_line()` | 改为调用两段式转弯 |
| 分段速度逻辑 | `status.c` | 新增在状态机中 |
| `STOP_TARGET` / `MIN_FINAL_DIST` / `CM_PER_PULSE` 等常量 | `status.h` 或新 `config.h` | 新增宏定义 |

---

---
## 移植与实现思路

### 移植基础：xiao 项目已合入的部分

xiao 项目的巡线/转弯/停车状态机已通过 [Defect.c](../User/Status/Defect.c) 的 `task_basic_1_update()` 移植到本项目。移植过程中发现并修复了以下关键差异：

| 差异 | xiao | GuZhangJiqiren | 修复 |
|------|------|----------------|------|
| 左轮 dir | `wheel[0].dir = 1` | `wheel[0].dir = -1` | 改为 1，否则正 trust 驱左轮反转 → 转圈 |
| 陀螺仪 I2C 地址 | `0xA0` | `0xA1` | 改为 0xA0，GY901 7 位地址 0x50 << 1 |
| 陀螺仪读取 | 双读比较 yaw/roll 防撕裂 | 单读无校验 | 改为双读，失败保留上次值 |
| 陀螺仪公式 | `value/32768*2000` | 部分公式缺 `/32768` | 已修正 |
| LED 电平 | 低有效 `(1, 0)` | 高有效 `(1, 1)` | 维持各自硬件配置 |
| 灰度 mux 引脚 | AD0/AD1/AD2 | IO2/IO3/IO4 | 维持各自硬件（引脚不同但功能等价） |
| I2C 时序 | `0x10E32879` | `0x00E063FF` | 维持各自（均能正常通信） |

### 当前代码状态

xiao 移植后的 `task_basic_1_update()` 已经实现了：
- `Q1_WAIT_KEY2` → `Q1_START_A_TURN` → `Q1_SIDE_AD` → `Q1_TURN_D` → `Q1_SIDE_DC` → `Q1_TURN_C` → `Q1_SIDE_CB` → `Q1_TURN_B` → `Q1_BA_FINAL`
- 陀螺仪 KEEP_ANGLE 转弯 + 丢线触发 STOP
- `corner_count` 机制：累计 4 个真转角后 `final_stop = 1`

**与 champion.md 设计的主要差异**：

| 设计点 | xiao 移植版（现有） | champion.md（目标） |
|--------|---------------------|---------------------|
| 转弯 | 纯陀螺仪角度闭环 KEEP_ANGLE | 两段式：陀螺仪硬转 + 低速探线 |
| 停车 | 丢线触发 + 反向制动 300ms | 编码器测距 + 灰度兜底 |
| 速度 | 恒定 50 | 分段：冲刺 30-35 / 降速 10-12 |
| 拐角计数 | corner_count 累计 yaw 变化 | 同 corner_count，或直接用 cross_cnt |
| 里程 | 无 | 编码器里程（cm） |

---

### 实现：champion.md 方案落地

#### 1. 编码器里程

利用现有 `get_wheel_speed()` 每 20ms 返回的编码器差值累加里程：

```c
// 在 Defect.c 中新增静态变量
static float mileage_cm = 0;

// 每 20ms 在 task_basic_1_update() 开头调用
static void update_mileage(STATUS *s) {
    // 取两轮平均速度，20ms 周期换算 cm
    float avg_speed = (ABS(s->motor.wheel[0].cur_speed) + 
                       ABS(s->motor.wheel[1].cur_speed)) / 2.0f;
    mileage_cm += avg_speed * CM_PER_PULSE * 0.02f;  // 20ms = 0.02s
}

static void reset_mileage(void) { mileage_cm = 0; }
```

`CM_PER_PULSE` 是标定值，通过直走 100cm 读取 encoder 累计脉冲数反算。

**注意**：`cur_speed` 是 20ms 内的编码器差值（单位：脉冲/20ms），不是实际速度。可以直接累加脉冲：
```c
// 更简单的方式：直接累加脉冲数
static int32_t total_pulses = 0;
total_pulses += ABS(s->motor.wheel[0].cur_speed) + ABS(s->motor.wheel[1].cur_speed);
float mileage_cm = total_pulses * CM_PER_PULSE;
```

#### 2. 两段式转弯替代 KEEP_ANGLE

当前 `enter_keep_angle()` 直接被路口检测触发。改为两段式：

```c
// 转弯阶段枚举
typedef enum { TURN_HARD, TURN_SEARCH } TURN_STAGE;
static TURN_STAGE turn_stage = TURN_HARD;
static int turn_frame_cnt = 0;
static int straight_seen_cnt = 0;

static void enter_two_phase_turn(STATUS *s, float target_yaw) {
    s->state.turn.target_yaw = target_yaw;
    s->state.motion = KEEP_ANGLE;  // 复用现有 motion
    s->state.status_pid.keep_angle_pid.integral = 0;
    s->state.status_pid.keep_angle_pid.last_error = 0;
    s->state.status_pid.keep_angle_pid.is_first = 1;
    turn_stage = TURN_HARD;
    turn_frame_cnt = 0;
    straight_seen_cnt = 0;
    line_seen_cnt = 0;
}

// 在 motion_execute() 的 KEEP_ANGLE 分支中：
case KEEP_ANGLE: {
    float err = s->state.turn.target_yaw - s->state.cur_angle;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    if (turn_stage == TURN_HARD) {
        // 阶段 A：陀螺仪角度闭环
        float out = compute_pid(&s->state.status_pid.keep_angle_pid, err);
        diff_drive(s, TURN_BASE_SPEED, out, TURN_DIFF_LIMIT);

        // |err| < 2° 持续 3 帧 → 进入阶段 B
        if (ABS(err) < 2.0f) {
            turn_frame_cnt++;
            if (turn_frame_cnt >= 3) {
                turn_stage = TURN_SEARCH;
                s->state.base_speed = 8;  // 低速探线
            }
        } else {
            turn_frame_cnt = 0;
        }
        break;
    }

    // 阶段 B：低速直走探线
    diff_drive(s, 8, 0, 0);  // 低速直走
    uint8_t line = s->sensor.gw_analogue.digital_8bit;
    if ((line & 0x3C) != 0) {  // 中间四路有黑线
        straight_seen_cnt++;
    } else {
        straight_seen_cnt = 0;
    }

    if (straight_seen_cnt >= 3) {
        // 咬线成功 → 回巡线
        back_to_find_line(s);
        turn_stage = TURN_HARD;
    }

    // 超时 500ms（25 帧）→ 左右摆动探线
    if (++turn_search_timeout > 25) {
        // 小幅左右摆动扩大搜索范围
        static int wiggle = 0;
        float dir = (wiggle / 10 < 5) ? -1.0f : 1.0f;
        diff_drive(s, 8, dir * SEARCH_DIFF, SEARCH_DIFF);
        wiggle = (wiggle + 1) % 100;
        // 摆动 100 帧还找不到 → 停车报警
        if (wiggle == 99) {
            s->state.motion = STOP;
            s->device.buzzer.on = 1;
        }
    }
    break;
}
```

**和 xiao KEEP_ANGLE 的区别**：xiao 是角度到位后给 SEARCH_DIFF 差速继续压线，champion 是到位后低速直走等传感器自然扫到线。champion 的方式更保守但更稳妥。

#### 3. 分段速度

在 FIND_LINE 巡线时根据里程动态调速：

```c
// 在 task_basic_1_update 中每个 SIDE 阶段：
if (s->task.race_phase == Q1_SIDE_AD || Q1_SIDE_DC || Q1_SIDE_CB || Q1_BA_FINAL) {
    if (mileage_cm < SPRINT_THRESHOLD) {
        s->state.base_speed = 30;   // 冲刺
    } else {
        s->state.base_speed = 10;   // 降速准备转弯
    }
}
```

每进入新的 SIDE 阶段调用 `reset_mileage()`。

#### 4. BA_FINAL 编码器停车

替代现有的丢线触发 STOP：

```c
case Q1_BA_FINAL:
    s->state.base_speed = 12;
    update_mileage(s);

    if (mileage_cm >= STOP_TARGET) {
        s->state.motion = STOP;
        s->task.task_running = 0;
        brake_until = 0;
        s->device.buzzer.on = 1;
        s->device.buzzer.off_time = s->state.time + 500;
        break;
    }

    // 灰度兜底：距离够了才允许灰度触发
    if (mileage_cm >= MIN_FINAL_DIST) {
        Road r = s->sensor.gw_analogue.cross.cross;
        if (r == LeftRoad || r == CrossRoad || r == TBRoad || r == TLRoad) {
            s->state.motion = STOP;
            s->task.task_running = 0;
            break;
        }
    }
    // 正常巡线（同 FIND_LINE）
    break;
```

**和 xiao 的区别**：xiao 是回到 A 点检测到路口 → 不转 → 丢线 → STOP。champion 是用编码器精确测距停车，更准确但不依赖路口检测。

#### 5. 路口阶段的阶段推进

champion.md 用 `cross_cnt` 计数，xiao 移植版用 `corner_count`。两者等价，映射关系：

```
cross_cnt:  0 → 1(A) → 2(D) → 3(C) → 4(B)
corner_count 同理（在 KEEP_ANGLE 退出时 yaw 变化 > 10° 才计数）
```

---

### 移植注意事项

1. **`cur_speed` 符号**：左轮 dir 改为 1 后，前进时 cur_speed 为正值。里程计算用 `ABS()` 兼容两者。

2. **灰度阈值**：每个传感器硬件不同，阈值 (`digital_high_threshold[8]`) 需要实地校准（先白纸再黑胶带各调用一次 `correct_gw_analogue()`）。

3. **陀螺仪初始化**：上电后 yaw 值任意，`after_init_state()` 记录 `initial_angle`。所有转弯 target_yaw 是相对值，不依赖绝对方向。

4. **Wheel PID 积分**：进入转弯时建议清零 wheel PID 积分项，防止上一段巡线的积分残余导致转弯起步异常。

5. **MOTOR_TEST**：调试时用 `status.state.motion = MOTOR_TEST` 让两轮同速直走，验证方向正确性。如果改了 dir 还转圈，检查物理接线。

6. **I2C 超时**：陀螺仪读取在 TIM5 ISR 上下文中（20ms 周期），HAL_I2C_Mem_Read timeout 设 5ms，确保不会阻塞下一个 20ms 周期。

---

## 实测校准清单

| 参数 | 说明 | 校准方法 |
|---|---|---|
| `CM_PER_PULSE` | 每编码器脉冲对应的厘米数 | 直走 100cm，读脉冲总数，反算 |
| `STOP_TARGET` | BA 边停车距离阈值 | 手动放车到理想停车位，读编码器值 |
| `MIN_FINAL_DIST` | BA 边允许灰度兜底的最低距离 | 比 STOP_TARGET 略短（~90cm），过滤前半段误判 |
| `Kp_turn` / `Ki_turn` | 陀螺仪闭环转弯 PID | 从小到大调，转到不振荡且够快 |
| `TURN_SPEED` | 转弯最大轮速 | 实测，保证能转过去但不甩尾 |
| `SPRINT_THRESHOLD` | 每边冲刺距离（初值 80cm） | 实测，确保降速后有足够距离稳定转弯 |

