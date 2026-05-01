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

## 实测校准清单

| 参数 | 说明 | 校准方法 |
|---|---|---|
| `CM_PER_PULSE` | 每编码器脉冲对应的厘米数 | 直走 100cm，读脉冲总数，反算 |
| `STOP_TARGET` | BA 边停车距离阈值 | 手动放车到理想停车位，读编码器值 |
| `MIN_FINAL_DIST` | BA 边允许灰度兜底的最低距离 | 比 STOP_TARGET 略短（~90cm），过滤前半段误判 |
| `Kp_turn` / `Ki_turn` | 陀螺仪闭环转弯 PID | 从小到大调，转到不振荡且够快 |
| `TURN_SPEED` | 转弯最大轮速 | 实测，保证能转过去但不甩尾 |
| `SPRINT_THRESHOLD` | 每边冲刺距离（初值 80cm） | 实测，确保降速后有足够距离稳定转弯 |

