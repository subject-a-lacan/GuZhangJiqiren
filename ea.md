# TASK2 LQR 调参顺序

当前 TASK2 第二阶段输出：

```c
pwm = angle_out + position_out;

angle_out = compute_pid(balance_pid, angle_error);
position_out = -(mileage_pid.kp * task2_car_position
               + mileage_pid.kd * car_speed);
```

参数对应关系：

```text
Cd -> balance_pid.kp -> K_angle
Cf -> balance_pid.kd -> K_angle_diff
Cx -> balance_pid.ki -> 暂时不用，保持 0
Cj -> mileage_pid.kp -> K_pos
Cl -> mileage_pid.kd -> K_vel
```

## VOFA 建议观察量

优先打印这些量，按逗号分隔：

```text
task2_state,roll,angle_error,angle_out,task2_car_position,car_speed,position_out,pwm,wheel0_trust,wheel1_trust
```

看波形时重点判断：

- `roll/angle_error` 是否能被拉回 0 附近。
- `pwm/wheel_trust` 是否经常打满到 `+-3000`。
- `car_speed` 是否长期同号，长期同号就是小车在匀速跑。
- `task2_car_position` 是否单方向越积越大。
- `task2_state` 是否频繁从 2 掉回 1。

## 第 0 步：先关掉位置和速度项

先只调角度环，不要四个参数一起动：

```text
Cx0
Cj0
Cl0
```

## 第 1 步：调 K_angle，也就是 Cd

目标：第二阶段能明显接杆，杆偏了小车会立刻往正确方向追。

建议范围：

```text
Cd3000 ~ Cd12000
```

推荐从这里开始：

```text
Cd6000
Cf0
Cj0
Cl0
```

现象判断：

- 杆进入第二阶段后还是软软倒下：`Cd` 太小，往 `8000、10000、12000` 加。
- 杆一进第二阶段小车方向明显反了：`Cd` 符号反了，先试负值。
- 小车一进第二阶段就满 PWM 猛抽，而且杆快速反向越过：`Cd` 太大，往 `4000、3000` 降。
- 能把杆拉住一点，但过冲明显：进入下一步调 `Cf`。

注意：`Cd` 比 `Cf` 数字大是正常的。因为 `compute_pid()` 里 D 项是 `(error - last_error) / T`，当前 `T = 8`，D 项实际输入会被除小。

## 第 2 步：调 K_angle_diff，也就是 Cf

目标：减少角度过冲，让杆过 0 度附近时不要冲太远。

建议范围：

```text
Cf1000 ~ Cf8000
```

在一个能接杆的 `Cd` 上，从小到大试：

```text
Cf1000
Cf2000
Cf3000
Cf5000
Cf8000
```

现象判断：

- `roll` 大幅越过 0 度，来回摆很大：`Cf` 太小。
- 加 `Cf` 后过冲变小，波形更钝：方向基本对。
- 加 `Cf` 后小车变慢、接不住，甚至比不加 D 更软：`Cf` 可能在抵消 P，试负号。
- 高频抖动、PWM 正负快速跳：`Cf` 太大。

这一阶段的合格标准：杆能在第二阶段被接住一小段时间，即使小车会跑，也先接受。

## 第 3 步：调 K_vel，也就是 Cl

目标：解决第二阶段小车向一个方向匀速跑的问题。

保持 `Cj0`，先只加速度项：

```text
Cj0
```

建议范围：

```text
Cl1 ~ Cl80
```

从小到大试：

```text
Cl2
Cl5
Cl10
Cl20
Cl40
Cl80
```

现象判断：

- `car_speed` 长时间同号，小车一直溜走：`Cl` 太小。
- `car_speed` 被压住，速度波形开始回到 0 附近：`Cl` 有效。
- 一加 `Cl` 小车反向乱冲，或者 `roll` 立刻更难稳：`Cl` 太大。
- `Cl` 越大车越跑，方向更错：速度项符号反了，需要单独改符号，不能继续硬调。

这一阶段的合格标准：小车速度不会长时间保持同一个方向，能看到明显刹车和回拉。

## 第 4 步：最后调 K_pos，也就是 Cj

目标：让小车不要离起点越来越远，慢慢回到起点附近。

`task2_car_position` 是编码器脉冲累计值，会越积越大，所以 `Cj` 必须很小。

建议范围：

```text
Cj0.0005 ~ Cj0.02
```

从很小开始：

```text
Cj0.0005
Cj0.001
Cj0.002
Cj0.005
Cj0.01
Cj0.02
```

现象判断：

- 杆能稳，小车位置慢慢漂走：`Cj` 太小。
- `task2_car_position` 能慢慢往 0 拉：`Cj` 有效。
- 小车为了回原点大幅冲，反而把杆推倒：`Cj` 太大。
- 位置越修越远：`Cj` 符号反了。

## 推荐起始流程

第一组：

```text
Cd6000
Cx0
Cf0
Cj0
Cl0
```

如果软，逐步加：

```text
Cd8000
Cd10000
Cd12000
```

能接杆后加 D：

```text
Cf2000
Cf3000
Cf5000
```

能接住但小车跑，再加速度：

```text
Cl5
Cl10
Cl20
Cl40
```

最后再加位置：

```text
Cj0.001
Cj0.002
Cj0.005
```

## 调参原则

- 一次只动一个参数。
- `Cx` 先保持 0，不要加积分。
- 先让杆站住，再让小车速度不跑，最后再管位置回零。
- 如果某个参数越加越坏，先怀疑符号，不要无限加大。
- 如果 `pwm` 长时间饱和到 `+-3000`，说明参数太大或状态切换太晚，继续加参数没有意义。
