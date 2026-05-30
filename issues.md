# TASK2 最小有效 PWM 补偿方案

这个补偿不是电机刚能空转的死区 PWM，而是：

```text
让小车在落地、带摆杆负载时产生明显救摆加速度的最小 PWM
```

现在的问题是：P 给了以后，车速度会变大，但摆已经倒得很厉害时才有明显动作，加速度不足以救回摆。说明很多时候最终控制输出 `u` 没有跨过“有效救摆加速度阈值”，或者跨得太晚。

所以应该加的是最终输出 `u` 的最小有效 PWM 补偿，不是角度前馈。

## 1. 它和角度前馈的区别

不要做：

```text
angle -> 某个固定 PWM 前馈
```

因为后面完整控制会变成：

```text
u = K1*angle + K2*angle_speed + K3*position + K4*car_speed
```

如果前馈只关于角度，会和角速度、位置、车速项打架。

应该做：

```text
先让 PID/LQR 算出最终输出 u
再对最终 u 做 deadzone/min_pwm 补偿
```

这样它只是执行器非线性补偿，可以和 PID、PD、LQR 共存。

## 2. 怎么测 min_pwm

完整装好车和摆杆，不跑 TASK2。用串口直接给两轮同向 PWM，记录 8ms 编码器速度序列。

推荐测：

```text
正向 PWM: 300, 400, 500, 600, 700, 800
反向 PWM: -300, -400, -500, -600, -700, -800
```

每个 PWM 记录：

```text
PWM: wheel0_speed, wheel1_speed
连续记录至少 10 个控制周期
```

更好的是看速度增量：

```text
accel ~= speed_now - speed_last
```

判断标准：

```text
连续 3 个控制周期速度同方向增长
并且平均增量 >= 2~3 个编码器脉冲/8ms
```

例子：

```text
300: 0,1,1,1,2     不算
400: 0,1,2,2,3     勉强
500: 0,2,5,8,11    算
```

如果 500 第一次满足条件，那么：

```text
task2_balance_min_pwm = 500
```

正反向可以分开测。如果差别不大，第一版可以先用统一值；如果差别很大，就做正反分开的 min_pwm。

## 3. 第一版代码接入方式

加在最终控制输出之后：

```c
float u = balance_out + position_out;

if (ABS(u) > 1.0f) {
    if (u > 0) {
        u += task2_balance_min_pwm;
    } else {
        u -= task2_balance_min_pwm;
    }
}

task2_set_pwm((int16_t)u);
```

注意：不要只加在 `balance_out` 上。应该加在最终输出 `u` 上，因为后面位置项、车速项也会一起参与控制。

## 4. 推荐初值

如果还没有测完，可以先试：

```text
task2_balance_min_pwm = 400
```

然后按现象调：

```text
太小：
摆倒下去时车还是慢半拍，没有明显冲击。

合适：
摆刚开始倒时，车能立刻给一脚，有明显救摆加速度。

太大：
角度刚有一点误差就猛冲、抽搐、打滑、原地转圈。
```

可调范围建议：

```text
300 ~ 700
```

## 5. 更平滑的版本

如果硬加 min_pwm 导致 0 附近突跳太明显，可以改成渐进补偿：

```c
static float task2_apply_min_pwm(float u)
{
    float min_pwm = task2_balance_min_pwm;
    float blend_pwm = 300.0f;

    if (u > 0) {
        if (u < blend_pwm) {
            return u + min_pwm * (u / blend_pwm);
        }
        return u + min_pwm;
    }

    if (u < 0) {
        if (-u < blend_pwm) {
            return u - min_pwm * ((-u) / blend_pwm);
        }
        return u - min_pwm;
    }

    return 0;
}
```

第一版建议先用硬加，方便判断有没有效果。

## 6. 和其他问题的关系

这个补偿解决的是：

```text
控制器已经判断出方向，但 PWM 太小，没有产生有效加速度
```

它不能解决：

```text
控制方向反了
左右轮严重不同步
轮胎打滑
P/D 符号错误
角度零点错误
前冲阶段切稳定太晚
```

如果加了 min_pwm 后仍然原地转圈，优先看左右轮速度差，并考虑加左右轮同步修正。
