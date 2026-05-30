# 左右轮 PWM 开环补偿公式（更新版）

数据来源：`SHIT.md`，速度单位：8ms 编码器累计值。

## 1. 死区

| 轮子 | 正转 | 反转 |
|------|------|------|
| Wheel0 (右) | +185 PWM | -182 PWM |
| Wheel1 (左) | +175 PWM | -173 PWM |

## 2. speed = k * PWM + b

```
W0_speed = 0.02024784 * pwm - 3.928370   (R²=0.9834, n=334)
W1_speed = 0.02118590 * pwm - 1.076538   (R²=0.9792, n=334)
```

## 3. 平均轮 + 统一控制量 u → 左右 PWM

```
avg_speed = 0.02071687 * u - 2.502454

W0_pwm = 1.023164 * u + 70.423
W1_pwm = 0.977861 * u - 67.305
```

## 4. 代码

```c
static int16_t task2_compensate_wheel0(int16_t u) {
    if (u == 0) return 0;
    float pwm = 1.023164f * ABS(u) + 70.423f;
    if (u > 0 && pwm < 185.0f) pwm = 185.0f;
    if (u < 0 && pwm < 182.0f) pwm = 182.0f;
    pwm = CONFINE(pwm, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm : -(int16_t)pwm;
}

static int16_t task2_compensate_wheel1(int16_t u) {
    if (u == 0) return 0;
    float pwm = 0.977861f * ABS(u) - 67.305f;
    if (u > 0 && pwm < 175.0f) pwm = 175.0f;
    if (u < 0 && pwm < 173.0f) pwm = 173.0f;
    pwm = CONFINE(pwm, 0, TRUST_CONFINE);
    return (u > 0) ? (int16_t)pwm : -(int16_t)pwm;
}
```
