# 左右轮 PWM 开环补偿公式

数据来源：`SHIT.md`，速度单位：8ms 编码器累计值。

同一 PWM 下 `wheel1`（左）比 `wheel0`（右）快，所以不能 `wheel0_pwm = wheel1_pwm = u`，必须分开补偿。

## 1. speed = k * PWM + b

```
wheel0_speed = 0.02083798 * pwm - 4.543297    (R²=0.9894)
wheel1_speed = 0.02179245 * pwm - 1.917124    (R²=0.9874)
```

## 2. 反解：PWM = (speed - b) / k

```
wheel0_pwm = 47.989305 * speed + 218.029684
wheel1_pwm = 45.887447 * speed +  87.971904
```

## 3. 平均轮 + 统一控制量 u → 左右 PWM

平均轮：`avg_speed = 0.02131521 * u - 3.230210`

代入反解：

```
wheel0_pwm = 1.022902 * u + 63.014
wheel1_pwm = 0.978101 * u - 60.254
```

## 4. 验证

| u | wheel0_pwm | wheel1_pwm |
|---|-----------|-----------|
| 200 | 268 | 135 |
| 500 | 574 | 429 |
| 800 | 881 | 722 |
| 1100 | 1188 | 1016 |
| 1400 | 1495 | 1309 |
| 2000 | 2109 | 1896 |
| 2500 | 2620 | 2385 |
| -500 | -448 | -549 |
| -2000 | -1983 | -2016 |

## 5. 死区保护

```
wheel0 正转可靠启动: ~201 PWM
wheel1 正转可靠启动: ~255 PWM
wheel0 反转启动: ~180 PWM
wheel1 反转启动: ~173 PWM
```

## 6. 代码

```c
static int16_t task2_compensate_wheel0(int16_t u) {
    if (u == 0) return 0;
    float pwm = 1.022902f * ABS(u) + 63.014f;
    if (u > 0 && pwm < 201.0f) pwm = 201.0f;
    if (u < 0 && pwm < 180.0f) pwm = 180.0f;
    pwm = CONFINE(pwm, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm : -(int16_t)pwm;
}

static int16_t task2_compensate_wheel1(int16_t u) {
    if (u == 0) return 0;
    float pwm = 0.978101f * ABS(u) - 60.254f;
    if (u > 0 && pwm < 255.0f) pwm = 255.0f;
    if (u < 0 && pwm < 173.0f) pwm = 173.0f;
    pwm = CONFINE(pwm, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm : -(int16_t)pwm;
}
```
