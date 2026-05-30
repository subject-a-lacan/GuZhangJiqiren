# TASK2 左右轮 PWM 开环补偿公式

数据来源：`SHIT.md`

结论：同 PWM 下，`wheel1` 明显比 `wheel0` 快，小车有右转趋势。TASK2 不能继续直接：

```c
wheel0_pwm = u;
wheel1_pwm = u;
```

应该把统一控制量 `u` 映射成左右轮不同 PWM：

```c
wheel0_pwm = compensate_wheel0(u);
wheel1_pwm = compensate_wheel1(u);
```

## 1. 实测平均速度

单位：8ms 内编码器累计值。

正转：

```text
PWM    wheel0_avg    wheel1_avg
500      5.826         9.163
800     12.390        15.780
1100    18.723        22.787
1400    24.475        28.443
1700    31.739        36.217
2000    36.821        42.464
2500    46.063        50.938
```

反转：

```text
PWM     wheel0_avg    wheel1_avg
-800     -10.318      -16.227
-1200    -18.400      -25.960
-1600    -26.111      -34.667
-2000    -33.389      -41.889
-2400    -39.518      -50.071
```

`800` 正转数据里有一个明显异常点 `13,1`，拟合时剔除了这个坏点。

## 2. PWM-速度拟合结果

正转拟合：

```text
wheel0_speed = 0.0202345 * pwm - 3.7604
wheel1_speed = 0.0211757 * pwm - 0.7993
avg_speed    = 0.0207051 * pwm - 2.2798
```

反转按绝对值拟合：

```text
wheel0_speed_abs = 0.0183470 * pwm_abs - 3.8080
wheel1_speed_abs = 0.0209039 * pwm_abs + 0.3165
avg_speed_abs    = 0.0196254 * pwm_abs - 1.7457
```

## 3. 推荐补偿公式

把输入 `u` 理解为“平均轮特性下的等效 PWM”，补偿到左右轮相同平均速度。

正转：

```text
wheel0_pwm = 1.0232 * u + 71.80
wheel1_pwm = 0.9778 * u - 68.61
```

反转：

```text
wheel0_pwm = -(1.0697 * abs(u) + 112.40)
wheel1_pwm = -(0.9388 * abs(u) - 98.65)
```

直观含义：

```text
wheel0 比较弱，需要加 PWM
wheel1 比较强，需要减 PWM
反转时两轮差异更大，所以必须正反转分开补偿
```

## 4. 新死区

这次重新测得的可靠启动死区：

```text
wheel0 正转: 185
wheel0 反转: 182
wheel1 正转: 175
wheel1 反转: 173
```

## 5. 可直接使用的代码

```c
static int16_t task2_compensate_wheel0(int16_t u)
{
    float abs_u = ABS(u);
    float pwm_abs;

    if (u == 0) return 0;

    if (u > 0) {
        pwm_abs = 1.0232f * abs_u + 71.80f;
        if (pwm_abs < 185.0f) pwm_abs = 185.0f;
    } else {
        pwm_abs = 1.0697f * abs_u + 112.40f;
        if (pwm_abs < 182.0f) pwm_abs = 182.0f;
    }

    pwm_abs = CONFINE(pwm_abs, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm_abs : -(int16_t)pwm_abs;
}

static int16_t task2_compensate_wheel1(int16_t u)
{
    float abs_u = ABS(u);
    float pwm_abs;

    if (u == 0) return 0;

    if (u > 0) {
        pwm_abs = 0.9778f * abs_u - 68.61f;
        if (pwm_abs < 175.0f) pwm_abs = 175.0f;
    } else {
        pwm_abs = 0.9388f * abs_u - 98.65f;
        if (pwm_abs < 173.0f) pwm_abs = 173.0f;
    }

    pwm_abs = CONFINE(pwm_abs, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm_abs : -(int16_t)pwm_abs;
}
```

## 6. `task2_set_pwm()` 使用方式

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

## 7. 注意

这个补偿只能解决“左右轮同 PWM 下速度不同”的静态差异，不能完全解决：

```text
轮胎打滑
地面摩擦变化
电池电压变化
摆杆反作用力突变
车体原地旋转时平均速度欺骗控制器
```

如果补偿后仍然明显转圈，下一步应该加一个很弱的左右轮同步修正：

```c
sync_error = status.motor.wheel[0].cur_speed - status.motor.wheel[1].cur_speed;
sync_trim = K_sync * sync_error;

wheel0_pwm -= sync_trim;
wheel1_pwm += sync_trim;
```

这不是完整速度环，只是防止左右轮差速太大导致原地旋转。
