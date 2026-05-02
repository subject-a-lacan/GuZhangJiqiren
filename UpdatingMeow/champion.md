# T 型路口合并

Q1 只有左转/右转，Q2 的 A4 纸干扰另有方案处理。TLRoad/TRRoad 在 `driver_task1` 中当作 LeftRoad/RightRoad 处理即可，**不改动传感器层**。

## 不需要改 get_road_type / serve_road

`serve_road()` ([gw_analogue.c:364](User/Sensor/gw_analogue.c#L364)) 对所有非 Straight/UnknowRoad 类型都会 `cross_cnt++`，TLRoad/TRRoad 已自动计数，不存在漏计。

## 在 driver_task1 中合并

`cross.cross` 保留原始值，`driver_task1` 读的时候自己做映射：

```c
Road road = status->sensor.gw_analogue.cross.cross;
if (road == TLRoad) road = LeftRoad;
if (road == TRRoad) road = RightRoad;
// 后续用映射后的 road 做 race_phase 推进判断
```

| 传感器报 | 状态机当什么处理 |
|----------|:---:|
| TLRoad (0b011) | LeftRoad |
| TRRoad (0b110) | RightRoad |
| TBRoad / CrossRoad | 忽略，Q1 不出现 |
