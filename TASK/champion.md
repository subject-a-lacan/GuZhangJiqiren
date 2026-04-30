# 第一问全流程与实现方案

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
[START 状态] → cross_cnt = 1（预设），起步，过拐角 A
     ↓ 左转完成 + Straight 稳定 + 编码器达标
[FIND_LINE 状态] → 正常巡线 + 拐角检测
     ↓ cross_cnt: 1→2(D) →3(C) →4(B)
[BA_FINAL 状态] → 拐角 4 转完，BA 边编码器测距停车
     ↓ 里程达标
[STOP] → 声光提示，结束
```

---

## 各阶段详细实现

### 阶段 0：STOP（初始）

```
motion = STOP
base_speed = 0
等待按键（button_D2 或 button_B11）
按键 → motion = START，声光提示启动
```

### 阶段 1：START（处理起步拐角）

**目的**：让车从发车位置进入 AD 直道，同时不破坏拐角计数。

**进入条件**：按键触发

**逻辑**：

```
cross_cnt = 1  ← 预设，拐角 A 视作已发生
base_speed = 15  ← 低速起步

每周期：
  get_road_type() 照常调用（但不修改 cross_cnt）
  
  if (检测到 LeftRoad):
      执行陀螺仪闭环左转 90°
      // 车头：朝左 → 朝下，驶入 AD 边
  
  if (LeftRoad 触发过 && 连续 3 帧 Straight && 编码器里程 > 阈值):
      motion = FIND_LINE
      cross_cnt 从现在开始正常响应 get_road_type()
      base_speed = 30  ← 进入高速
```

**退出条件**：左转完成 + Straight 稳定 3 帧 + 编码器里程达标

**注意**：如果实体车测试时传感器在 A 点判出的是 CrossRoad 而非 LeftRoad（因为 AB 残段干扰），在 START 状态里加 200ms 盲走延迟再开始检测。

---

### 阶段 2：FIND_LINE（正常巡线 + 拐角响应）

**进入条件**：START 阶段退出

**逻辑**：

```
每周期：
  编码器累加里程（用于分段调速）
  get_road_type() 正常调用 ← 修改 cross_cnt

  // === 分段速度 ===
  if (编码器里程 < 80cm):
      base_speed = 30-35  ← 冲刺
  else:
      base_speed = 10-12  ← 降速准备转弯

  // === 拐角响应 ===
  if (检测到 Straight):
      PID 循线直走

  if (检测到 LeftRoad):
      if (cross_cnt >= 4):
          motion = STOP  ← 左转陷阱保护（兜底）
      else:
          记录当前陀螺仪角度
          目标角度 = 当前 - 90°
          陀螺仪闭环转 90°
          误差 < 2° 持续 3 帧 → 转弯完成
          cross_cnt++
          清零编码器里程 ← 为下一条边重新计
          base_speed = 30 ← 恢复高速
```

**拐角序列**：

| 拐角 | cross_cnt 变化 | 位置 | 物理含义 |
|---|---|---|---|
| D | 1→2 | 左下 | 左转，车头朝下→朝右 |
| C | 2→3 | 右下 | 左转，车头朝右→朝上 |
| B | 3→4 | 右上 | 左转，车头朝上→朝左 |

**退出条件**：cross_cnt == 4 且 B 点左转完成 → motion = BA_FINAL

---

### 阶段 3：BA_FINAL（最后直道 + 精确停车）

**进入条件**：拐角 4（B 点）左转完成，cross_cnt == 4

**目的**：BA 边上以低速巡线，编码器测距触发停车。

**逻辑**：

```
进入时：
  清零编码器里程
  base_speed = 12  ← 低速精确

每周期：
  编码器累加里程
  
  // === 主逻辑：编码器测距停车 ===
  if (编码器里程 >= STOP_TARGET):  // 实测校准值，约 97cm
      motion = STOP
      声光提示

  // === 兜底 1：灰度传感器 ===
  if (检测到 LeftRoad):
      motion = STOP  // 此时不应转弯，应停车

  // === 兜底 2：蓝牙核武器 ===
  if (蓝牙收到 'S'):
      motion = STOP

  其他情况：PID 巡线 Straight
```

**STOP_TARGET 校准步骤**：
1. 车手动放理想停车位（前沿在 AD 外沿，车身在 AB）
2. 读编码器里程值 → STOP_TARGET
3. 赛前微调 ±1cm 至 D1 ≤ 3cm 稳定满足

**退出条件**：motion == STOP

---

## 陀螺仪闭环左转

取代现有开环 pivot：

```
turn_start_angle = 当前陀螺仪角度
turn_target_angle = turn_start_angle - 90.0  // 逆时针左转

转弯子循环（每周期）：
  error = turn_target_angle - 当前陀螺仪角度
  
  // 角度归一化到 [-180, 180]
  if (error > 180) error -= 360
  if (error < -180) error += 360
  
  // 简单 P 控
  turn_power = Kp_turn * error
  turn_power = CONFINE(turn_power, -TURN_SPEED, TURN_SPEED)
  
  左轮 = -turn_power
  右轮 = +turn_power
  
  if (ABS(error) < 2.0):
      stable_cnt++
      if (stable_cnt >= 3):
          转弯完成，退出
  else:
      stable_cnt = 0
```

Kp_turn 和 TURN_SPEED 需实测校准。Kp_turn 不需要激进——目标是固定值，没有追踪延迟。

---

## 编码器里程计算

```
// 每周期在 update_status() 中调用
void update_mileage() {
    static int32_t last_left_pulse = 0;
    static int32_t last_right_pulse = 0;
    
    int32_t left_now = 读左轮编码器累计脉冲;
    int32_t right_now = 读右轮编码器累计脉冲;
    
    int32_t left_delta = left_now - last_left_pulse;
    int32_t right_delta = right_now - last_right_pulse;
    
    float avg_cm = (left_delta + right_delta) / 2.0 * CM_PER_PULSE;
    mileage_cm += avg_cm;
    
    last_left_pulse = left_now;
    last_right_pulse = right_now;
}

// 转弯完成后重置
void reset_mileage() {
    mileage_cm = 0;
    刷新 last_left_pulse / last_right_pulse 基准;
}
```

CM_PER_PULSE 取决于轮径和编码器线数。假设轮径 6.5cm、20 线编码器：每脉冲 ≈ π×6.5/20 ≈ 1.02cm。**赛前必须实测校准**。

---

## 与现有 status.c 的映射

| 新增/修改内容 | 涉及文件 | 改动性质 |
|---|---|---|
| `MOTION_STATION` 增加 `START`、`BA_FINAL` | `status.h` | 新增枚举值 |
| `mileage_cm` 变量 | `status.c` | 新增全局变量 |
| `update_mileage()` / `reset_mileage()` | `status.c` | 新增函数 |
| `update_status()` 状态机分支 | `status.c` 的 `update_status()` | 扩充分支 |
| 陀螺仪闭环转弯函数 `gyro_turn_left()` / `gyro_turn_right()` | 新文件或 `status.c` | 新增函数 |
| 蓝牙 UART 接收 + STOP 注入 | `main.c` 或独立文件 | 新增 |
| `follow_line()` 中 LeftRoad 分支 | `status.c` 的 `follow_line()` | 改为调用陀螺仪闭环 |
| 分段速度逻辑 | `status.c` | 新增在 `update_status()` 中 |
| STOP_TARGET / CM_PER_PULSE 等常量 | `status.h` 或新 `config.h` | 新增宏定义 |

---

## 实测校准清单

| 参数 | 说明 | 校准方法 |
|---|---|---|
| `CM_PER_PULSE` | 每编码器脉冲对应的厘米数 | 直走 100cm，读脉冲总数，反算 |
| `STOP_TARGET` | BA 边停车距离阈值 | 手动放车到理想停车位，读编码器值 |
| `Kp_turn` | 陀螺仪闭环转弯 P 值 | 从小到大调，转到不振荡且够快 |
| `TURN_SPEED` | 转弯最大轮速 | 实测，保证能转过去但不甩尾 |
| `SPRINT_THRESHOLD` | 每边冲刺距离（初值 80cm） | 实测，确保降速后有足够距离稳定转弯 |
| 起步盲走延迟 | 如传感器在 A 点误判才需要 | 实测是否需要 |
