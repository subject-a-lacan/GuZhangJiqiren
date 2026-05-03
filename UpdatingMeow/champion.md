# AI 提示词：优化 TASK1 转弯找线与直线速度策略

你是一个嵌入式 STM32 小车工程代码 agent。请阅读本项目的 `TASK/jiegou.md`、`TASK/route.md`，再重点修改 `User/Status/Defect.c` 里的 `driver_task1()` 相关逻辑。目标是优化第一问 TASK1：转弯后不容易丢线，并加入基于编码器里程的直线分段速度策略。

请遵守以下原则：

1. 保持现有 TASK 架构，不重写整个工程。
2. 优先小改、局部改，避免把无关任务 Q2/Q3/Q4 搅乱。
3. 不要把 8 路灰度永久加入普通循迹 `diff`。正常巡线仍以当前 `gw_analogue.c` 的中间 4 路 `diff` 为主。
4. 转弯后允许使用 `digital_8bit` 的中间 6 路做“找线/接管”判断，但不要改坏传感器层路口识别。
5. 不使用时间阈值推进 TASK1 阶段，阶段距离优先用 `phase_mileage` 和 `encoder_pulse_to_cm()`。

## 当前问题

TASK1 现在的转弯结束条件主要是角度误差：

```c
ABS(diff_angle) < Q1_TURN_TOLERANCE_DEG
```

实车上可能出现：角度已经转到位，但灰度传感器没有压住下一条线，马上切回 `FIND_LINE` 后因为 `diff` 不可靠导致循不上线。

另一个已验证事实：编码器里程比较准，实测一段约 `102cm` 的路程换算为 `101.63cm`，可以用于 TASK1 直线阶段速度切换和最后停车前降速。

## 需要实现的总体方案

把 TASK1 的转弯流程拆成：

```text
TURN_x：角度环转弯
REACQUIRE_x：转弯后低速找线/接管
SIDE_x：正常直线巡线
```

不要用“角度误差 < 10 度 OR 最外侧传感器黑”直接结束转弯。正确语义是：

```text
角度误差 < 10°：只表示允许进入找线接管阶段
中间 6 路看到线：允许从找线阶段切回巡线阶段
中间 4 路稳定看到线：恢复正常直线速度策略
```

## 建议新增宏

放在 `User/Status/Defect.c` TASK1 参数区附近：

```c
#define Q1_TURN_TO_FIND_TOLERANCE_DEG  10.0f
#define Q1_TURN_LINE_MASK_6            0x7E  /* bit1~bit6 */
#define Q1_TURN_LINE_MASK_4            0x3C  /* bit2~bit5 */
#define Q1_LINE_STABLE_CNT             3

#define Q1_FLASH_SPEED                 80
#define Q1_CRUISE_SPEED                50
#define Q1_FINAL_SLOW_SPEED            20
#define Q1_STRAIGHT_FLASH_CM           80.0f
#define Q1_FINAL_SLOW_CM               90.0f
```

如果已有同名宏，按含义合并，不要重复定义。

## 需要新增的 TASK1 阶段

在 `Q1_RACE_PHASE` 中加入转弯后的找线阶段。建议使用明确名字：

```c
Q1_FIND_AD,  /* A 转完后找 AD 线 */
Q1_FIND_DC,  /* D 转完后找 DC 线 */
Q1_FIND_CB,  /* C 转完后找 CB 线 */
Q1_FIND_BA,  /* B 转完后找 BA 线 */
```

对应流程：

```text
Q1_TURN_A -> Q1_FIND_AD -> Q1_SIDE_AD
Q1_TURN_D -> Q1_FIND_DC -> Q1_SIDE_DC
Q1_TURN_C -> Q1_FIND_CB -> Q1_SIDE_CB
Q1_TURN_B -> Q1_FIND_BA -> Q1_BA_FINAL
```

`task1_enter_phase()` 继续负责：

```c
status->task.race_phase = next_phase;
status->task.phase_mileage = 0;
status->task.phase_start_time = status->state.time;
```

注意：不要在 `task1_enter_phase()` 里清 `cnt_seen`。`cnt_seen` 必须只在真实检测到 `Straight` 且处于巡线语义时清零，避免同一个路口被消费两次。

## 转弯结束与找线接管

建议拆两个 helper：

```c
static float task1_angle_error(STATUS *status);
static uint8_t task1_turn_angle_ready(STATUS *status);
static uint8_t task1_middle6_seen(STATUS *status);
static uint8_t task1_middle4_seen(STATUS *status);
```

语义：

```c
task1_turn_angle_ready:
  ABS(angle_error) < Q1_TURN_TO_FIND_TOLERANCE_DEG

task1_middle6_seen:
  (status->sensor.gw_analogue.digital_8bit & Q1_TURN_LINE_MASK_6) != 0

task1_middle4_seen:
  (status->sensor.gw_analogue.digital_8bit & Q1_TURN_LINE_MASK_4) != 0
```

在 `Q1_TURN_A/D/C/B` 中：

```text
motion = KEEP_ANGLE
base_speed = Q1_TURN_SPEED
如果角度误差 < 10°，进入对应 Q1_FIND_x
```

在 `Q1_FIND_x` 中：

```text
motion = FIND_LINE
base_speed = Q1_FINAL_SLOW_SPEED 或 Q1_TURN_SPEED 中较稳的值
如果中间 6 路看到线，进入对应正常直线阶段
```

`Q1_FIND_x` 是转弯后的低速接管阶段，不要直接高速冲。它的目的只是让灰度重新压住下一条边。

## 中间 4 路稳定后恢复速度

在每个正常直线阶段内，建议维护一个 TASK1 内部静态计数或 TASK 字段：

```c
static uint8_t q1_mid4_stable_cnt;
```

每次进入新 phase 时清零。运行时：

```text
如果中间 4 路看到线，stable_cnt++
否则 stable_cnt = 0
```

当 `stable_cnt >= Q1_LINE_STABLE_CNT` 后，认为巡线已经稳定接管，可以执行正常直线速度策略。

如果不想新增静态变量，也可以新增 `TASK` 字段，但要保证初始化和切阶段时清零。

## 直线阶段速度策略

所有普通直线阶段，包括：

```text
Q1_SIDE_AD
Q1_SIDE_DC
Q1_SIDE_CB
```

速度按当前阶段里程切换：

```text
phase_mileage 换算距离 <= 80cm:
  base_speed = Q1_FLASH_SPEED    // 80

phase_mileage 换算距离 > 80cm:
  base_speed = Q1_CRUISE_SPEED   // 50
```

用：

```c
float cm = encoder_pulse_to_cm((int32_t)status->task.phase_mileage);
```

注意：只有在中间 4 路稳定看到线后再允许冲刺速度。如果刚从 `Q1_FIND_x` 切进普通直线，前几帧还没有稳定压线，先用低速，避免刚接管就冲出去。

建议封装：

```c
static void task1_apply_side_speed(STATUS *status);
```

语义：

```text
未稳定压线：低速，例如 Q1_FINAL_SLOW_SPEED 或 Q1_TURN_SPEED
稳定压线且 cm <= 80：Q1_FLASH_SPEED
稳定压线且 cm > 80：Q1_CRUISE_SPEED
```

## 最后停车阶段速度策略

`Q1_BA_FINAL` 阶段：

```text
如果当前阶段里程 <= 90cm:
  base_speed = Q1_CRUISE_SPEED

如果当前阶段里程 > 90cm:
  base_speed = Q1_FINAL_SLOW_SPEED   // 20
```

然后停车条件继续由 `task1_final_stop_condition()` 管理。可以优先使用编码器里程停车，因为编码器精度已经验证不错。若保留灰度终点兜底，需要写成辅助条件，不要散落在 `switch` 里。

建议封装：

```c
static void task1_apply_final_speed(STATUS *status);
```

## 重要：status.c 速度覆盖问题

当前 `User/Status/status.c` 的 `update_status()` 在 `FIND_LINE` 分支里有：

```c
status->state.base_speed = cmd_speed;
follow_line(status);
```

这会覆盖 `driver_task1()` 设置的 `Q1_FLASH_SPEED / Q1_CRUISE_SPEED / Q1_FINAL_SLOW_SPEED`。实现本任务时必须解决这个问题，否则 TASK1 速度策略不会生效。

做法：

```text
当 status.task.armed && status.task.task_running 时，FIND_LINE 不要用 cmd_speed 覆盖 base_speed。
空闲手动调试或 MOTOR_TEST 时再使用 cmd_speed。
```

保持改动小，不要大范围重写 `update_status()`。

## 不要做的事

1. 不要把普通循迹 `diff` 改成永久 8 路参与。
2. 不要用最左/最右传感器黑作为直接切回巡线的唯一条件。
3. 不要使用时间阈值推进阶段。
4. 不要在 `task1_enter_phase()` 里清 `cnt_seen`。
5. 不要把 `Q1_FLASH_SPEED=80` 用在转弯刚结束、尚未稳定压线的阶段。
6. 不要让传感器层直接修改 `race_phase`。

## 验收标准

1. TASK1 转弯阶段不会因为刚到角度阈值就直接高速巡线。
2. 每次转弯后必须经过 `Q1_FIND_x` 找线阶段。
3. 中间 6 路看到线后，才允许进入下一条边的巡线阶段。
4. 中间 4 路连续稳定看到线后，才允许直线阶段按 80cm 内冲刺。
5. 普通直线阶段 80cm 内速度为 `Q1_FLASH_SPEED=80`，80cm 后为 `Q1_CRUISE_SPEED=50`。
6. BA 最后停车阶段 90cm 后速度降为 `Q1_FINAL_SLOW_SPEED=20`。
7. `status.c` 不再无条件用 `cmd_speed` 覆盖 TASK1 的 `base_speed`。
8. 工程能通过当前可用的编译检查；如果全工程因已有 `ccd.c` 引脚问题失败，需要在最终说明里明确这不是本次改动引入的。

