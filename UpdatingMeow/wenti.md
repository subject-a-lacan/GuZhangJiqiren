# get_road_type 恢复 Straight 条件太窄问题

## 问题位置

问题在 `User/Sensor/gw_analogue.c` 的 `get_road_type()` 里。

当前逻辑大致是：

```c
if (cross->cross == Straight) {
  // 只有当前状态是 Straight，才会检测新路口
  // 检测到边缘探头 bit7 或 bit0 后，开始多帧积分并判断路口
} else if ((road_data == 0x18) || (road_data == 0x10) || (road_data == 0x08)) {
  // 当前已经处于非 Straight 路口状态时，只有看到这三种形态才恢复 Straight
  serve_road(cross, Straight);
}
```

## 为什么这是问题

`get_road_type()` 是一个带锁存的路口状态机：

```text
Straight
  -> 检测到路口
  -> cross->cross 变成 LeftRoad / RightRoad / CrossRoad 等非 Straight 状态
  -> 等待重新回到 Straight
  -> 才能检测下一个路口
```

也就是说，识别到一个路口之后，必须先恢复成 `Straight`，后续路口才会继续被识别。

但当前恢复条件只认：

```text
0x18 = 0001 1000
0x10 = 0001 0000
0x08 = 0000 1000
```

这要求小车离开路口后，灰度传感器必须刚好只有中间一两个探头压线。

实车上直线状态不一定这么理想，可能出现：

```text
0x1C = 0001 1100
0x38 = 0011 1000
0x0C = 0000 1100
0x30 = 0011 0000
0x3C = 0011 1100
```

这些从人的角度看仍然是“已经回到直线附近”，但当前代码不会承认它们是 `Straight`。

## 可能表现

实车现象可能是：

```text
第一个路口能识别
离开路口后 cross->cross 仍然卡在 CrossRoad / LeftRoad / RightRoad
后续路口不再触发新判断
cross_cnt 不再增加
TASK 状态机等不到新的路口
```

这不是左右路口枚举的问题，也不是 `road_new_from_bit()` 的问题。

这个问题只和“非 Straight 状态如何恢复成 Straight”有关。

## 推荐修复方向

不要只用三个固定值判断直线：

```c
road_data == 0x18 || road_data == 0x10 || road_data == 0x08
```

更合理的语义是：

```text
中间循迹区有线
并且左右两侧路口区没有线
就认为已经回到 Straight
```

按照当前 8 路灰度 bit 分区：

```text
左侧路口区: bit7 bit6 -> 0xC0
中间循迹区: bit5 bit4 bit3 bit2 -> 0x3C
右侧路口区: bit1 bit0 -> 0x03
```

可以考虑类似逻辑：

```c
uint8_t middle = road_data & 0x3C;
uint8_t side = road_data & 0xC3;

if (middle && !side) {
  serve_road(cross, Straight);
}
```

这样可以覆盖更多正常直线形态，例如：

```text
0x18
0x10
0x08
0x1C
0x38
0x0C
0x30
0x3C
```

## 注意事项

1. 不要修改 `LeftRoad / RightRoad` 枚举值。
2. 不要修改 `road_new_from_bit()` 的左右映射。
3. 不要删除全局 `cross_cnt`。
4. 不要让 `gw_analogue.c` 直接修改 TASK 阶段。
5. 这里只解决“路口状态恢复 Straight 条件太窄”的问题。

# TASK 小状态机缺少 race_phase / cross_cnt 维护问题

## 问题背景

当前 TASK 外层架构已经能跑通：

```text
按钮 / 蓝牙请求
  -> update_task()
  -> task_start()
  -> armed = 1
  -> 分发到 task_basic_1_update() / task_basic_2_update() 等任务内部函数
```

但是接下来要写每一题内部的小状态机时，必须先明确两个核心字段的维护责任：

```c
status.task.race_phase;
status.task.cross_cnt;
```

这两个字段不能只初始化，必须在 `update_task()` 分发进去的具体 `task_xxx_update()` 内部持续更新。

## race_phase 目前的问题

`race_phase` 现在只在 `task_start()` 里根据 `task_id` 赋初值，例如第一问：

```c
status->task.race_phase = Q1_START_A_TURN;
```

但是进入 `task_basic_1_update()` 之后，还没有根据实际运行情况推进阶段。

这意味着后续第一问如果写成：

```text
起步
  -> A 点转弯
  -> AD 段巡线
  -> D 点转弯
  -> DC 段巡线
  -> C 点转弯
  -> CB 段巡线
  -> B 点转弯
  -> BA 段停车
```

就必须在 `task_basic_1_update()` 内部写成真正的阶段机：

```c
switch (status->task.race_phase) {
  case Q1_START_A_TURN:
    // 判断 A 点起步/转弯是否完成
    // 完成后切到 Q1_SIDE_AD
    break;

  case Q1_SIDE_AD:
    // 巡线，等待 D 点有效路口
    // 确认后切到 Q1_TURN_D
    break;
}
```

每次阶段切换时，建议同时刷新阶段起点信息：

```c
status->task.race_phase = next_phase;
status->task.phase_start_time = status->state.time;
// phase_mileage 暂时先不定，后面单独讨论是按阶段清零还是整任务累计。
```

## cross_cnt 目前的问题

工程里现在存在三类“路口计数”：

```text
1. 全局 cross_cnt
   由 gw_analogue.c 的传感器层增加，用于兜底和调试。

2. status.sensor.gw_analogue.cross.cross_cnt
   也是传感器层的观测计数，表示传感器认为自己识别到了几次路口。

3. status.task.cross_cnt
   TASK 层自己的有效路口计数，应该表示“当前任务承认通过了几个有效路口”。
```

前两个是传感器视角，第三个才是任务视角。

所以 `status.task.cross_cnt` 不能让 `gw_analogue.c` 自动增加，也不能简单等于全局 `cross_cnt`。它必须在具体任务小状态机里，根据当前 `race_phase` 判断“这个路口对当前任务是否有效”，确认有效后再增加。

例如：

```c
if (当前阶段正在等待 D 点路口 &&
    传感器检测到 CrossRoad / LeftRoad / RightRoad &&
    这个传感器路口事件还没有被消费) {
  status->task.cross_cnt++;
  status->task.race_phase = Q1_TURN_D;
}
```

这样 TASK 计数才是干净的，不会被干扰 A4、误判路口、起步时 AD 线等无效事件污染。

## 需要 cnt_seen 保证路口“纯洁性”

如果只写：

```c
if (status->sensor.gw_analogue.cross.cross != Straight) {
  status->task.cross_cnt++;
}
```

会有严重问题。

因为小车压在同一个路口上时，传感器状态可能连续很多个控制周期都是非 Straight。这样同一个物理路口会被重复加很多次。

所以 TASK 层需要一个类似 `cnt_seen` 的字段或局部静态逻辑，用来记录“传感器层这一次路口事件是否已经被 TASK 消费过”。

推荐语义：

```c
uint8_t cnt_seen;
```

含义：

```text
cnt_seen = 0:
  当前传感器路口事件还没被 TASK 层消费，可以根据 race_phase 判断是否有效。

cnt_seen = 1:
  当前这个路口已经处理过了，即使传感器还保持非 Straight，也不能重复增加 task.cross_cnt。

当传感器恢复 Straight 后:
  cnt_seen 清零，允许下一次路口事件被处理。
```

伪代码：

```c
Road road = status->sensor.gw_analogue.cross.cross;

if (road == Straight) {
  status->task.cnt_seen = 0;
} else if (status->task.cnt_seen == 0) {
  status->task.cnt_seen = 1;

  if (当前 race_phase 认为这个 road 是有效路口) {
    status->task.cross_cnt++;
    status->task.race_phase = next_phase;
  }
}
```

## 建议修改方向

1. 在 `TASK` 结构体里增加一个 `cnt_seen` 字段，用于 TASK 层防止重复消费同一个路口事件。
2. `init_task()` 里初始化 `cnt_seen = 0`。
3. `task_start()` 里也清零 `cnt_seen = 0`。
4. 每个 `task_xxx_update()` 内部，根据 `race_phase + 当前 road + cnt_seen` 判断是否增加 `status.task.cross_cnt`。
5. `gw_analogue.c` 仍然只负责传感器观测，不直接修改 `status.task.cross_cnt`，也不直接推进 `race_phase`。
6. `phase_mileage` 的策略暂时不在这里定，后面单独讨论：到底是每个阶段单独计数，还是整个任务累计计数。
