# 路口判断恢复条件过窄问题

本文档只记录当前 `gw_analogue.c` 里最需要注意的一个问题：路口判断从非 `Straight` 状态恢复到 `Straight` 的条件太窄。

## 1. 这个问题是什么意思

当前 `get_road_type()` 的逻辑大概是：

```c
if (cross->cross == Straight) {
  // 当前认为自己在直线状态时，才会检测新路口
  // 如果边缘探头看到黑线，就开始多帧积分并判断路口类型
} else if ((road_data == 0x18) || (road_data == 0x10) || (road_data == 0x08)) {
  // 当前已经认为自己在路口状态时，只有看到这三种直线形态，才恢复 Straight
  serve_road(cross, Straight);
}
```

关键点是：代码只有在 `cross->cross == Straight` 时，才会开始检测新的路口。

一旦检测到了 `LeftRoad / RightRoad / CrossRoad / TLRoad / TRRoad / TBRoad`，`cross->cross` 就会变成非 `Straight`。

此后它不会继续检测新路口，而是先等待自己恢复到 `Straight`。

也就是说，当前路口识别是一个带锁存的状态机：

```text
Straight
  -> 检测到边缘黑线
  -> 多帧积分
  -> 判定为某个路口
  -> cross->cross 变成非 Straight
  -> 等待直线形态
  -> 恢复 Straight
  -> 才能检测下一个路口
```

问题就在“等待直线形态”这里。

## 2. 为什么说条件太窄

当前代码只接受三种 `road_data` 作为“已经回到直线”：

```c
road_data == 0x18
road_data == 0x10
road_data == 0x08
```

如果把 8 路灰度看成 8 个 bit，它大概要求只有中间一两个探头压在线上：

```text
0x18 = 0001 1000
0x10 = 0001 0000
0x08 = 0000 1000
```

这是非常理想的直线状态。

但实车循迹时，直线不一定只会长这样。由于线宽、车身偏移、传感器高度、阈值、地面反光，直线很可能读成：

```text
0x1C = 0001 1100
0x38 = 0011 1000
0x0C = 0000 1100
0x30 = 0011 0000
0x3C = 0011 1100
```

这些从人的角度看仍然是“已经回到直线附近”，但是当前代码不会承认它们是 `Straight`。

## 3. 会导致什么现象

假设车刚刚识别到一个路口：

```c
cross->cross = LeftRoad;
cross_cnt++;
```

然后车离开路口，回到赛道主线。

如果灰度读数刚好是：

```c
road_data = 0x1C;
```

那么当前条件不满足：

```c
road_data == 0x18  // false
road_data == 0x10  // false
road_data == 0x08  // false
```

结果：

```c
cross->cross
```

会继续保持 `LeftRoad`，不会恢复成 `Straight`。

后果是下一次遇到路口时，`get_road_type()` 仍然走非 `Straight` 分支，不会重新进入路口检测流程。

最终表现可能是：

```text
第一个路口能识别
离开路口后状态卡在 LeftRoad/RightRoad/CrossRoad
后续路口不再计数
cross_cnt 卡住
TASK 状态机等不到新的路口
```

这就是“恢复条件太窄”的意思。

## 4. 这个问题和左右编码无关

不要把这个问题和 `LeftRoad/RightRoad` 左右反的问题混在一起。

以下内容不要动：

```c
LeftRoad = 0b001
RightRoad = 0b100
```

以及：

```c
uint8_t left = L ? 0b100 : 0;
uint8_t right = R ? 0b001 : 0;
```

当前实车左右判断已经靠历史代码和硬件方向抵消跑通了。

这个文档说的问题只和“路口状态能不能恢复到 Straight”有关。

## 5. 推荐修复方向

不要只用三个固定值判断直线：

```c
road_data == 0x18 || road_data == 0x10 || road_data == 0x08
```

更合理的判断应该是：

```text
中间区域看到线
并且左右路口区域没有明显分支
```

根据当前 `road_decision()` 的分区，8 路可以粗略分成：

```text
左侧路口区: bit7 bit6 -> 0xC0
中间循迹区: bit5 bit4 bit3 bit2 -> 0x3C
右侧路口区: bit1 bit0 -> 0x03
```

因此可以考虑把恢复条件改成类似语义：

```c
uint8_t middle = road_data & 0x3C;
uint8_t side = road_data & 0xC3;

if (middle && !side) {
  serve_road(cross, Straight);
}
```

这表示：

```text
只要中间 4 个探头有线，并且左右两侧路口区没有线，就认为已经回到直线。
```

它能覆盖：

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

这样更适合实车，因为直线状态不可能每一帧都刚好只有 `0x18/0x10/0x08`。

## 6. 修复时要注意

1. 不要删除全局 `cross_cnt`。
2. 不要修改 `status.task.cross_cnt` 的语义。
3. 不要让 `gw_analogue.c` 直接修改 TASK 阶段。
4. 不要修正左右枚举和 `road_new_from_bit()`。
5. 只改“非 Straight 状态恢复到 Straight 的判断条件”。

目标是让路口状态机能稳定完成这个循环：

```text
Straight -> 路口 -> Straight -> 下一个路口
```

而不是识别一次路口后卡死在非 `Straight`。
