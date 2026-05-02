# TASK1 第一问注意点

本文是给 AI agent 的 TASK1 小状态机实现提示词。实现时要适配现有 TASK 架构，不要去重写传感器层。

第一问路线：

```text
AB 发车点出发
车头朝 A
逆时针跑一圈
A -> D -> C -> B -> A
最后停回 AB 发车点附近
```

关键注意点：

1. 第一问所有有效拐角都是左转。
2. 发车点不一定严格就在 A 点转弯处，所以不能一进入 TASK1 就直接原地左转。必须增加“从发车点走到第一个转弯处 A”的阶段。
3. 起步时车可能已经压着/靠近一个路口，暂时先不写特殊屏蔽逻辑。如果实车出现一启动就误判路口，再单独加起步时间或起步里程屏蔽。
4. Q1 只需要把 T 型路口合并成普通左右路口使用，不改 `get_road_type()`，不改 `serve_road()`，不改 `road_new_from_bit()`。
5. BA 最后一段停车可以考虑用里程数判定，但里程精度还没实测好，所以先把它作为一个方案保留，不要把整个状态机强绑定到唯一的里程停车方案。

# T 型路口合并规则

Q1 只有左转/右转语义，传感器层检测出的 `TLRoad / TRRoad` 在 `driver_task1` 或 `task_basic_1_update()` 内部合并即可。

不要改动传感器层：

```text
User/Sensor/gw_analogue.c
get_road_type()
serve_road()
road_new_from_bit()
Road 枚举左右映射
```

TASK1 内部读取原始路口后做一层映射：

```c
Road road = status->sensor.gw_analogue.cross.cross;

if (road == TLRoad) {
  road = LeftRoad;
}
if (road == TRRoad) {
  road = RightRoad;
}
```

映射表：

| 传感器报 | TASK1 状态机当成 |
|----------|------------------|
| `LeftRoad` | `LeftRoad` |
| `RightRoad` | `RightRoad` |
| `TLRoad` | `LeftRoad` |
| `TRRoad` | `RightRoad` |
| `TBRoad` | Q1 暂时忽略 |
| `CrossRoad` | Q1 暂时忽略 |
| `UnknowRoad` | 忽略 |
| `Straight` | 正常直线 |

# driver_task1 整体架构

实现位置建议仍然放在 `User/Status/Defect.c` 的 TASK1 内部函数里，也就是当前的 `task_basic_1_update()`。如果想改名成 `driver_task1()` 也可以，但不要破坏 `update_task()` 分发链路。

## 需要的状态字段

现有字段：

```c
status->task.race_phase;
status->task.cross_cnt;
status->task.phase_mileage;
status->task.phase_start_time;
status->state.motion;
status->state.base_speed;
```

建议新增一个 TASK 层字段：

```c
uint8_t cnt_seen;
```

`cnt_seen` 用来防止同一个物理路口在连续多帧里被重复消费。

语义：

```text
cnt_seen = 0:
  当前传感器路口事件还没有被 TASK 消费。

cnt_seen = 1:
  当前路口事件已经被 TASK 消费，直到恢复 Straight 前都不能重复计数。

road == Straight:
  cnt_seen 清零，允许下一个路口被消费。
```

注意：建议只在接受到“有效路口”时把 `cnt_seen` 置 1。不要看到任意非 Straight 就置 1，否则如果路口前几帧误报成 `CrossRoad / TBRoad`，后面稳定成 `LeftRoad` 时反而会被挡掉。

## 建议重新整理 Q1_RACE_PHASE

当前枚举里只有：

```c
Q1_START_A_TURN,
Q1_SIDE_AD,
Q1_TURN_D,
Q1_SIDE_DC,
Q1_TURN_C,
Q1_SIDE_CB,
Q1_TURN_B,
Q1_BA_FINAL,
```

建议拆成更清楚的阶段：

```c
Q1_START_TO_A,  // 从发车点先沿 AB/起步方向走到 A 点附近
Q1_TURN_A,      // A 点左转，进入 AD
Q1_SIDE_AD,     // AD 边巡线，等待 D 点
Q1_TURN_D,      // D 点左转，进入 DC
Q1_SIDE_DC,     // DC 边巡线，等待 C 点
Q1_TURN_C,      // C 点左转，进入 CB
Q1_SIDE_CB,     // CB 边巡线，等待 B 点
Q1_TURN_B,      // B 点左转，进入 BA
Q1_BA_FINAL,    // BA 边回到 A，最后停车
```

如果不想大改命名，也至少要保证第一阶段不是直接原地转弯，而是“先向前走到 A 点附近”。

## 阶段切换通用动作

每次切换 `race_phase` 时，统一做这些事：

```c
status->task.race_phase = next_phase;
status->task.phase_start_time = status->state.time;
status->task.phase_mileage = 0;
status->task.cnt_seen = 0;
```

`phase_mileage` 当前单位是编码器脉冲，不是 cm。需要真实距离判断时，再调用：

```c
encoder_pulse_to_cm(status->task.phase_mileage)
```

## 路口消费通用逻辑

每次进入 TASK1 内部，先获取映射后的路口：

```c
Road road = status->sensor.gw_analogue.cross.cross;
if (road == TLRoad) road = LeftRoad;
if (road == TRRoad) road = RightRoad;
```

然后做 `cnt_seen` 维护：

```c
if (road == Straight) {
  status->task.cnt_seen = 0;
}
```

在等待有效左路口的阶段中：

```c
if (road == LeftRoad && status->task.cnt_seen == 0) {
  status->task.cnt_seen = 1;
  status->task.cross_cnt++;
  // 切换到对应转弯阶段
}
```

不要让传感器层的全局 `cross_cnt` 直接替代 `status.task.cross_cnt`。TASK 自己的 `cross_cnt` 只在小状态机确认“这是本题有效路口”后增加。

# TASK1 阶段设计

## 1. Q1_START_TO_A

目标：从发车点先向 A 点附近走一小段，而不是上来就原地左转。

动作建议：

```text
motion = FIND_LINE
base_speed = 低速或中速
```

完成条件先用简单方案：

```text
phase_mileage 达到一个标定脉冲阈值
```

或者：

```text
运行时间达到一个短时间阈值
```

进入下一阶段：

```text
Q1_TURN_A
```

注意：这一阶段暂时不要因为检测到路口就立刻转弯。起步处可能已经压着边线或路口，先用距离/时间把车带离起步不稳定区域。

## 2. Q1_TURN_A

目标：A 点左转 90 度，进入 AD 边。

动作建议：

```text
motion = KEEP_ANGLE
base_speed = 低速
目标角度 = 当前起始角度基础上左转 90 度
```

完成条件：

```text
角度误差进入允许范围
并且保持一小段时间
```

进入下一阶段：

```text
Q1_SIDE_AD
```

## 3. Q1_SIDE_AD

目标：沿 AD 边巡线，等待 D 点有效左路口。

动作：

```text
motion = FIND_LINE
base_speed = 巡线速度
```

有效路口判断：

```text
road == LeftRoad
并且 cnt_seen == 0
```

确认后：

```text
task.cross_cnt++
进入 Q1_TURN_D
```

## 4. Q1_TURN_D

目标：D 点左转 90 度，进入 DC 边。

动作和完成条件同 `Q1_TURN_A`。

进入下一阶段：

```text
Q1_SIDE_DC
```

## 5. Q1_SIDE_DC

目标：沿 DC 边巡线，等待 C 点有效左路口。

动作同 `Q1_SIDE_AD`。

有效左路口确认后：

```text
task.cross_cnt++
进入 Q1_TURN_C
```

## 6. Q1_TURN_C

目标：C 点左转 90 度，进入 CB 边。

动作和完成条件同前面的转弯阶段。

进入下一阶段：

```text
Q1_SIDE_CB
```

## 7. Q1_SIDE_CB

目标：沿 CB 边巡线，等待 B 点有效左路口。

动作同 `Q1_SIDE_AD`。

有效左路口确认后：

```text
task.cross_cnt++
进入 Q1_TURN_B
```

## 8. Q1_TURN_B

目标：B 点左转 90 度，进入 BA 最后一段。

动作和完成条件同前面的转弯阶段。

进入下一阶段：

```text
Q1_BA_FINAL
```

进入 `Q1_BA_FINAL` 时一定要清零 `phase_mileage`，因为最后停车可能要用从 B 转完之后开始算的阶段里程。

## 9. Q1_BA_FINAL

目标：沿 BA 回到 A 附近，停回发车点。

动作：

```text
motion = FIND_LINE
base_speed = 可先高速，接近终点前降速
```

停车策略暂时保留两种，不要一开始写死：

方案 A：里程停车

```text
从 B 转弯完成进入 BA 开始累计 phase_mileage
达到标定脉冲数后 task_finish()
```

优点：不依赖最后 A 点路口的极限检测。
缺点：编码器里程精度还没验证，轮子打滑会影响停车点。

方案 B：最终路口辅助停车

```text
BA 段巡线时检测到最终 A 点对应路口后停车或急停
```

优点：直接利用终点特征。
缺点：灰度探头检测到 AD 线时，车头距离限制可能已经非常紧，容易刹不住。

建议实现时先把 BA 停车封装成清晰判断，不要散落在多个地方：

```c
if (q1_final_stop_condition(status, road)) {
  task_finish(status);
}
```

具体最终采用里程、路口，还是二者结合，等实测 `phase_mileage` 精度后再定。

# 实现边界

1. 不要修改 `gw_analogue.c` 的路口判断。
2. 不要修改 `Road` 枚举左右映射。
3. 不要删除全局 `cross_cnt`。
4. 不要让传感器层直接推进 `race_phase`。
5. `status.task.cross_cnt` 只由 TASK1 内部确认有效路口后增加。
6. `phase_mileage` 保持脉冲单位，只有需要和 cm 阈值比较时再换算。
7. 第一版先追求状态机清楚可调，不要把起步屏蔽、停车策略、A4 干扰处理全部揉进一个大判断。
