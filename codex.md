# TASK2 左右轮 PWM 开环补偿公式

数据来源：`SHIT.md`

结论：同样 PWM 下，`wheel1` 明显比 `wheel0` 快，低 PWM 时差距更大。所以 TASK2 不能继续：

```c
wheel0_pwm = u;
wheel1_pwm = u;
```

而应该把控制器输出的统一控制量 `u` 映射成左右轮不同 PWM：

```c
wheel0_pwm = compensate_wheel0(u);
wheel1_pwm = compensate_wheel1(u);
```

## 1. 实测平均速度

单位：8ms 内编码器累计值。

```text
PWM    wheel0_avg    wheel1_avg
500      5.776         8.845
800     12.346        15.769
1100    18.621        22.448
1400    24.255        27.766
1700    31.739        36.217
2000    36.821        42.464
2500    47.538        51.846
```

可以看到 `wheel1` 始终更快，所以它应该被压低，`wheel0` 应该被补高。

## 2. PWM-速度线性拟合

拟合形式：

```text
speed = k * pwm + b
```

得到：

```text
wheel0_speed = 0.02081293 * pwm - 4.433174
wheel1_speed = 0.02173805 * pwm - 1.717761
```

反解：

```text
wheel0_pwm = 48.047064 * speed + 213.000983
wheel1_pwm = 46.002297 * speed + 79.020959
```

## 3. 把统一控制量 u 映射到左右轮 PWM

把原来的 `u` 理解成“平均轮特性下的等效 PWM”。平均轮拟合为：

```text
avg_speed = 0.02127549 * u - 3.075467
```

代入左右轮反解，得到正向补偿公式：

```text
wheel0_pwm = 1.022225 * u + 65.233801
wheel1_pwm = 0.978721 * u - 62.457607
```

直观含义：

```text
wheel0 比较弱，需要加 PWM
wheel1 比较强，需要减 PWM
```

## 4. 启动 PWM

从 `SHIT.md` 记录：

```text
wheel0 正转可靠启动 PWM: 约 201
wheel0 反转可靠启动 PWM: 约 180

wheel1 正转可靠启动 PWM: 约 255
wheel1 反转可靠启动 PWM: 约 173
```

注意：`wheel1` 正转 233 时只是“有时能动/碰一下又停”，不算可靠启动 PWM；第一版用 255 更稳。

## 5. 推荐补偿函数

当前只有正向完整速度曲线；反向只有最小启动 PWM，没有完整反向 PWM-速度曲线。所以：

- 正向用拟合公式。
- 反向先用同一套斜率补偿，再按反向可靠启动 PWM 做下限保护。
- 后续最好补测反向 `PWM-speed` 曲线，再替换反向公式。

```c
static int16_t task2_compensate_wheel0(int16_t u)
{
    float abs_u = ABS(u);
    float pwm_abs;

    if (u == 0) return 0;

    pwm_abs = 1.022225f * abs_u + 65.233801f;

    if (u > 0 && pwm_abs < 201.0f) pwm_abs = 201.0f;
    if (u < 0 && pwm_abs < 180.0f) pwm_abs = 180.0f;

    pwm_abs = CONFINE(pwm_abs, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm_abs : -(int16_t)pwm_abs;
}

static int16_t task2_compensate_wheel1(int16_t u)
{
    float abs_u = ABS(u);
    float pwm_abs;

    if (u == 0) return 0;

    pwm_abs = 0.978721f * abs_u - 62.457607f;

    if (u > 0 && pwm_abs < 255.0f) pwm_abs = 255.0f;
    if (u < 0 && pwm_abs < 173.0f) pwm_abs = 173.0f;

    pwm_abs = CONFINE(pwm_abs, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm_abs : -(int16_t)pwm_abs;
}
```

## 6. `task2_set_pwm()` 应该改成这样

```c
static void task2_set_pwm(int16_t pwm)
{
    int16_t wheel0_pwm = task2_compensate_wheel0(pwm);
    int16_t wheel1_pwm = task2_compensate_wheel1(pwm);

    status.motor.wheel[0].trust = wheel0_pwm;
    status.motor.wheel[1].trust = wheel1_pwm;

    set_wheel_dir(&status.motor.wheel[0], wheel0_pwm);
    set_wheel_dir(&status.motor.wheel[1], wheel1_pwm);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ABS(wheel0_pwm));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ABS(wheel1_pwm));
}
```

## 7. 重要注意

这个补偿只解决“左右轮同 PWM 下速度/驱动力不一致”的静态差异。

它不能完全解决：

```text
轮胎打滑
地面摩擦变化
电池电压变化
摆杆反作用力突变
车体原地旋转时平均速度欺骗控制器
```

如果补偿后仍然转圈，下一步加一个很弱的左右轮同步修正：

```c
sync_error = status.motor.wheel[0].cur_speed - status.motor.wheel[1].cur_speed;
sync_trim = K_sync * sync_error;

wheel0_pwm -= sync_trim;
wheel1_pwm += sync_trim;
```

这个不是完整速度环，只是防止左右轮差速太大导致原地转圈。
