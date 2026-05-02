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
