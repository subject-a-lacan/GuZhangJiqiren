# AI 提示词：设计并实现 TASK2 基本第二问状态机

你是一个嵌入式 STM32 小车工程代码 agent。请先阅读 `TASK/jiegou.md`、`TASK/timu.md`、`TASK/route.md`、`TASK/tupianjieshi.md`，再阅读当前代码里的 `User/Status/Defect.c`、`User/Status/Defect.h`、`User/Status/status.c`、`User/Sensor/gw_analogue.c`。

目标是为基本要求第二问 `TASK_BASIC_2` 设计并实现状态机：小车从 `AB` 发车点或 `AD` 发车点出发，由裁判选择发车姿态，沿带有 `BC` 边干扰 A4 纸的轨迹行驶 `3/4` 圈，到达对侧角点后掉头，沿原路返回发车点停车。启动和停车需要保留声光提示。

请保持改动小而清晰，不要重写整个工程，不要破坏已经调好的 `TASK_BASIC_1`。

## 题意和路线

基本第二问有两个发车姿态，由 `status.task.start_pose` 区分：

```text
START_AB:
  出发后按 TASK 文档规定方向跑 3/4 圈，到 B 点附近掉头，再原路返回 AB 发车点。
  可以理解为：A -> D -> C -> B，然后 B -> C -> D -> A。

START_AD:
  出发后按 TASK 文档规定方向跑 3/4 圈，到 D 点附近掉头，再原路返回 AD 发车点。
  可以理解为：A -> B -> C -> D，然后 D -> C -> B -> A。
```

如果 `TASK/route.md` 对绝对方向有更精确说明，以 `route.md` 为准，但状态机思路不变：先 3/4 圈，再 180 deg 掉头，再沿相同三条边反向返回。

`BC` 边中部会覆盖干扰 A4 纸。干扰纸的纵向黑线与原 `BC` 轨迹线重合，横向黑条会污染灰度巡线和路口识别。第二问最重要的难点不是普通 90 deg 转弯，而是不要把 `BC` 干扰纸误当成真实路口。

## 当前工程可复用基础

当前工程已经有：

1. `TASK.task_id/start_pose/race_phase/cnt_seen/phase_mileage` 这些任务状态字段。
2. `update_task()` 中统一累计 `phase_mileage`。
3. `encoder_pulse_to_cm()` 可把编码器脉冲换算为 cm。
4. `TASK_BASIC_1` 已经实现了 `TURN -> FIND -> SIDE` 的转弯后找线结构。
5. `status.c` 已经避免任务运行时 `cmd_speed` 无条件覆盖 `base_speed`。
6. `gw_analogue.c` 里路口识别属于传感器层，只给出观测结果，任务有效路口应由 `Defect.c` 的状态机消费。

实现 TASK2 时优先复用 TASK1 的思想：角度环负责 90 deg 或 180 deg 转向，转完后低速找线，中间 6 路看到线再切回巡线，中间 4 路稳定后再恢复正常速度。

## 建议状态机架构

推荐把 TASK2 拆成这些基本动作块，而不是直接写一大堆时间判断：

```text
SIDE_NORMAL          普通边巡线
SIDE_BC_INTERFERE    BC 干扰边巡线
TURN_90              真实角点 90 deg 转弯
FIND_AFTER_TURN      转弯后低速找线
UTURN_180            3/4 圈终点 180 deg 掉头
FIND_AFTER_UTURN     掉头后低速找回原路
FINAL_SIDE           返回发车点前的最后一条边
FINISH               停车
```

具体实现可以有两种方式：

```text
方式 A：显式 enum 阶段
  Q2_AB_SIDE_AD, Q2_AB_TURN_D, Q2_AB_SIDE_DC, ...
  Q2_AD_SIDE_AB, Q2_AD_TURN_B, Q2_AD_SIDE_BC, ...

方式 B：路线表 + leg_index
  用一个 route table 描述每条边、下一个转向、是否 BC 干扰边、是否终点掉头。
```

为了少犯错，第一版建议用方式 A，虽然 enum 多一点，但每个阶段语义清楚，调车时容易定位。等跑稳后再考虑路线表抽象。

## START_AB 推荐阶段

START_AB 可以按下面的物理阶段理解：

```text
Q2_AB_START_TO_A
Q2_AB_TURN_A
Q2_AB_FIND_AD
Q2_AB_SIDE_AD
Q2_AB_TURN_D
Q2_AB_FIND_DC
Q2_AB_SIDE_DC
Q2_AB_TURN_C
Q2_AB_FIND_CB
Q2_AB_SIDE_CB_INTERFERE
Q2_AB_UTURN_B
Q2_AB_FIND_BC_RETURN
Q2_AB_SIDE_BC_INTERFERE_RETURN
Q2_AB_TURN_C_RETURN
Q2_AB_FIND_CD_RETURN
Q2_AB_SIDE_CD_RETURN
Q2_AB_TURN_D_RETURN
Q2_AB_FIND_DA_FINAL
Q2_AB_SIDE_DA_FINAL
```

前半程到 B 是 3/4 圈，B 点不再执行普通 90 deg 转弯，而是执行 `UTURN_180`。掉头后返回时，转弯方向与出发方向相反。

## START_AD 推荐阶段

START_AD 可以按下面的物理阶段理解：

```text
Q2_AD_START_TO_A
Q2_AD_TURN_A
Q2_AD_FIND_AB
Q2_AD_SIDE_AB
Q2_AD_TURN_B
Q2_AD_FIND_BC
Q2_AD_SIDE_BC_INTERFERE
Q2_AD_TURN_C
Q2_AD_FIND_CD
Q2_AD_SIDE_CD
Q2_AD_UTURN_D
Q2_AD_FIND_DC_RETURN
Q2_AD_SIDE_DC_RETURN
Q2_AD_TURN_C_RETURN
Q2_AD_FIND_CB_RETURN
Q2_AD_SIDE_CB_INTERFERE_RETURN
Q2_AD_TURN_B_RETURN
Q2_AD_FIND_BA_FINAL
Q2_AD_SIDE_BA_FINAL
```

前半程到 D 是 3/4 圈，D 点执行 `UTURN_180`，再返回 A 附近的 AD 发车点。

如果实际发车时第一段已经在线上，不需要 `START_TO_A` 的单独阶段，也可以直接进入第一条边巡线阶段。但不要把发车阶段写成固定时间，优先用路口事件或编码器里程推进。

## 转弯设计

90 deg 转弯建议继续使用陀螺仪闭环：

```text
进入 TURN_90:
  initial_angle = cur_angle
  tar_angle = +90 或 -90
  motion = KEEP_ANGLE
  base_speed = Q2_TURN_SPEED

角度误差进入宽容范围:
  切到 FIND_AFTER_TURN
```

掉头建议先用 180 deg 陀螺仪闭环，不要第一版就做纯开环：

```text
进入 UTURN_180:
  initial_angle = cur_angle
  tar_angle = +180 或 -180
  motion = KEEP_ANGLE
  base_speed = Q2_UTURN_SPEED

角度误差 < 12~15 deg:
  切到 FIND_AFTER_UTURN

中间 6 路看到线:
  切回返回边巡线
```

如果实车 180 deg 闭环很慢或容易扫过线，可以把掉头拆成两个 90 deg 小阶段，但第一版不要同时引入太多复杂度。

## 转弯后找线

复用 TASK1 的思想：

```text
角度误差进入宽容范围:
  只表示允许进入低速找线阶段

中间 6 路看到线:
  允许从 FIND 阶段切回 SIDE 阶段

中间 4 路连续稳定看到线:
  才恢复正常巡线速度
```

建议 helper：

```c
static float task2_angle_error(STATUS *status);
static uint8_t task2_turn_angle_ready(STATUS *status);
static uint8_t task2_middle6_seen(STATUS *status);
static uint8_t task2_middle4_seen(STATUS *status);
```

不要用最外侧传感器看到黑线作为“已经找回主线”的唯一条件。最外侧在 BC 干扰纸上很容易被骗。

## BC 干扰段处理

这是第二问的核心难点。

不要依赖传感器层自动判断“这是干扰纸”。状态机本来就知道当前是不是在 `BC` 边，所以应由 TASK2 在 `SIDE_BC_INTERFERE` 阶段主动降速和屏蔽假路口。

建议做法：

```text
当 phase 是 BC 干扰边:
  base_speed 使用 Q2_INTERFERE_SPEED，比如 30~40
  不消费 LeftRoad / RightRoad / CrossRoad / TBRoad 作为角点
  只有 phase_mileage 换算距离超过接近边尾阈值后，才允许接受真实角点
```

推荐增加类似判断：

```text
cm = encoder_pulse_to_cm(phase_mileage)

BC 边中段干扰窗口:
  25cm <= cm <= 75cm

在这个窗口内:
  ignore road event
  cnt_seen 不因为假路口推进阶段

超过 80cm 后:
  才允许检查真实角点并进入 TURN_90 或 UTURN_180
```

阈值后续按实车调，第一版可以：

```c
#define Q2_BC_IGNORE_START_CM  20.0f
#define Q2_BC_IGNORE_END_CM    80.0f
#define Q2_CORNER_ENABLE_CM    75.0f
```

干扰纸会让普通 `diff` 被横线拉偏。第一版先低速通过，不要大改 `gw_analogue.c`。如果低速仍然被横线拉飞，再考虑在 TASK2 的 BC 干扰阶段做更小的保护：

```text
检测到横线态或大面积黑:
  限制循迹 PID 输出
  或短暂使用上一次可信 diff
  或用陀螺仪保持大方向低速穿过
```

这类保护应尽量写在任务层或状态层，不要让传感器层直接修改 `race_phase`。

## 路口消费和 cnt_seen

保持 TASK1 的经验：

```text
只有在真正离开路口，检测到 Straight，并且处于正常巡线语义时，才允许 cnt_seen = 0。
```

在以下阶段不要随便清 `cnt_seen`：

```text
FIND_AFTER_TURN
FIND_AFTER_UTURN
SIDE_BC_INTERFERE 的干扰窗口内
UTURN_180
```

原因是 BC 干扰纸会制造大量假路口和假 Straight，过早清零会让同一个物理路口或干扰横线被重复消费。

建议封装：

```c
static uint8_t task2_accept_expected_corner(STATUS *status, Road road);
static uint8_t task2_in_bc_ignore_window(STATUS *status);
static uint8_t task2_corner_enabled_by_distance(STATUS *status);
```

普通边可以依赖路口事件进入下一转弯；BC 边必须加距离门槛。

## 速度策略

第二问时间要求是 `t2 <= 25s`，但稳定性优先。建议先用保守速度跑通：

```c
#define Q2_FAST_SPEED        60
#define Q2_CRUISE_SPEED      50
#define Q2_INTERFERE_SPEED   35
#define Q2_TURN_SPEED        30
#define Q2_FIND_SPEED        20
#define Q2_FINAL_SLOW_SPEED  20
```

普通边可以做简单分段：

```text
前 60~70cm: Q2_FAST_SPEED
后段:       Q2_CRUISE_SPEED
```

BC 干扰边不要冲刺：

```text
整段或干扰窗口: Q2_INTERFERE_SPEED
快到真实角点前: 维持 Q2_CRUISE_SPEED 或更低
```

最终返回发车点前建议降速：

```text
final side cm > 85~90cm:
  base_speed = Q2_FINAL_SLOW_SPEED
```

停车优先使用编码器里程加路口兜底，不要只用时间。

## 停车策略

返回发车点时，最后一条边不要再做 90 deg 转弯。进入 `FINAL_SIDE` 后：

```text
motion = FIND_LINE
先正常巡线
距离接近终点后降速
满足最终停车条件后 task_finish()
```

最终停车条件建议第一版用编码器：

```text
final side cm >= Q2_FINAL_STOP_CM
```

可加灰度兜底：

```text
如果接近终点距离后检测到预期起点角点或横向线，也允许停车
```

但兜底条件必须有距离门槛，例如 `cm > 85cm`，避免 BC 干扰纸造成提前停车。

## 不要做的事

1. 不要把 BC 干扰纸的横线当成真实路口消费。
2. 不要在 `gw_analogue.c` 里直接推进 TASK2 状态机。
3. 不要用固定时间跑完整路线。
4. 不要破坏 TASK1 当前的 `TURN -> FIND -> SIDE` 结构。
5. 不要让 `task1` 和 `task2` 共用会互相污染的静态计数器，除非每次进入任务和切阶段都明确清零。
6. 不要在 `task1_enter_phase()` 这类 TASK1 helper 里塞 TASK2 逻辑。
7. 不要在 BC 干扰窗口内清零 `cnt_seen` 或接受任意 Road 事件。

## 验收标准

1. `TASK_BASIC_2` 能根据 `status.task.start_pose` 选择 AB 或 AD 两套路线。
2. AB 发车能跑到 B 点附近掉头，再原路返回 AB 发车点停车。
3. AD 发车能跑到 D 点附近掉头，再原路返回 AD 发车点停车。
4. 每次 90 deg 转弯后都有低速找线阶段，不会刚到角度就高速巡线。
5. 180 deg 掉头后也有低速找线阶段。
6. 经过 BC 干扰纸时不会把横向黑条误判为真实角点。
7. BC 干扰边只有超过距离门槛后才允许消费真实角点。
8. 返回路上再次经过 BC 干扰纸时同样能屏蔽假路口。
9. 最后停车不是靠固定时间，而是靠编码器距离或带距离门槛的灰度兜底。
10. `TASK_BASIC_1` 行为不被改坏。

## 推荐实现顺序

1. 先在 `Defect.h` 增加 TASK2 所需 enum 阶段。
2. 在 `Defect.c` 增加 TASK2 参数宏和 helper，不动 TASK1 helper。
3. 在 `task_start()` 中根据 `TASK_BASIC_2` 和 `start_pose` 设置初始 Q2 phase。
4. 实现 `driver_task2()` 的 AB 路线，先不追求高速。
5. 加入 BC 干扰窗口屏蔽。
6. 实现 AD 路线。
7. 最后微调速度、距离阈值和停车阈值。

