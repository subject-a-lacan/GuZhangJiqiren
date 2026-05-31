# TASK2 输出占比目标

主循环当前打印：

```text
左trust,右trust,balance.out,position_out,mileage.kd*car_speed,balance_D_out,ADC
```

## 1. balance 内部：P 和 D 的比例

近似：

```text
balance_P_out = balance.out - balance_D_out
balance_D_out = 最后一列中的 balance_D_out
```

理想目标：

```text
静止或慢速靠近 0 度时：
D 项应该很小，最好低于 balance.out 的 10%~20%

摆杆快速穿过 0 度时：
D 项应该占 balance.out 的 30%~60%

摆杆明显快速倒下时：
D 项可以短时间占到 60%~80%，但不应该长期吃满限幅
```

如果静止时 D 占比很高，甚至吃满限幅：

```text
先不要加 KD
增大 TASK2_D_DEADBAND
或减小 TASK2_D_ALPHA
或减小 KD
```

如果过 0 后完全刹不住：

```text
增大 KD
或略微增大 TASK2_D_ALPHA
```

## 2. position_out 内部：位置项和速度项比例

当前：

```text
position_out = -(mileage.kp * task2_car_position + mileage.kd * car_speed)
```

打印里有：

```text
speed_out = mileage.kd * car_speed
```

所以近似：

```text
pos_out = -position_out - speed_out
```

理想目标：

```text
刚进入稳定阶段：
speed_out 应该是 position_out 的主要部分，占 60%~90%
pos_out 先很小，占 10%~40%

小车速度明显变大时：
speed_out 应该明显变大，用来压住匀速逃跑

摆已经比较稳、车离起点较远时：
pos_out 再慢慢变大，可以占 position_out 的 40%~70%
```

如果一进入稳定 position_out 主要由 pos_out 贡献：

```text
mileage.kp 太大
先减小 mileage.kp
```

如果小车越跑越快但 speed_out 很小：

```text
mileage.kd 太小
```

## 3. balance.out 和 position_out 的比例

这是最关键的总比例。

理想目标：

```text
刚捕获摆杆、角度还不稳：
|position_out| <= |balance.out| 的 20%~40%

摆杆接近稳定，但小车开始跑远：
|position_out| 可以到 |balance.out| 的 40%~80%

摆杆已经比较稳、主要在回位置：
|position_out| 可以接近 |balance.out|
但不建议长期超过 balance.out
```

如果刚进稳定阶段：

```text
|position_out| >= |balance.out|
```

通常说明后两项太强，会抢救摆控制权。

如果车明显跑飞：

```text
|position_out| 仍然小于 |balance.out| 的 20%
```

说明速度/位置反馈太弱，优先加 `mileage.kd`。

## 4. 调参顺序

```text
1. 先让 D 静止不乱跳
2. 调 balance.kp / balance.kd，让 balance.out 能救摆
3. 加 mileage.kd，让小车不匀速跑飞
4. 最后加 mileage.kp，让车慢慢回中
```

一句话目标：

```text
救摆阶段 balance.out 主导；
刹摆瞬间 D 项明显参与；
防跑飞阶段 speed_out 主导 position_out；
回中阶段 pos_out 才慢慢变大。
```
