# 前馈融合版

参考文件：

- `qiankui.md`：原始标定数据，格式为 `PWM: left_speed,right_speed`
- `claudeqk.md`：按所有原始采样点做线性拟合
- `codexqk.md`：按各 PWM 档位均值做拟合，但采样周期说明有问题

## 对 codexqk.md 的检查

`codexqk.md` 里的整体方向是对的：

- 左右轮应该分开算前馈。
- 右轮同 PWM 下速度更大，所以同目标速度下右轮 PWM 应该比左轮小。
- 前馈形式用 `PWM = ff_offset + ff_k * speed` 是合理的。
- 起转 PWM 应该单独保留。

但是有两个问题：

1. 它写的“采样周期约 200ms”不对。
   - 当前 `qiankui.md` 里的速度数据来自 8ms 内编码器累计值。
   - 所以后续直接用这些参数时，`target_speed` 也必须是“8ms 编码器累计值”。

2. 它写的周期缩放公式不适合当前用法。
   - 如果仍然用 8ms 内的速度量，就不需要缩放 `ff_k`。
   - 如果以后改成 `T ms` 内编码器累计值，应该先换算回 8ms 等效速度：

```c
speed_8ms = speed_Tms * 8.0f / T_ms;
```

而不是直接套 `ff_k_new = ff_k * (T_control_ms / 200)`。

## 融合采用的计算方式

为了避免某一个 PWM 档位样本特别多而把拟合拉偏，融合版采用：

1. 先对每个 PWM 档位分别求左右轮平均速度。
2. 再用这些档位均值拟合：

```c
PWM = ff_offset + ff_k * speed_8ms
```

得到：

| 参数 | 左轮 | 右轮 |
|---|---:|---:|
| `ff_offset` | 200.68 | 107.12 |
| `ff_k` | 47.24 | 46.27 |
| `ff_min` | 250 | 260 |

## 推荐第一版前馈参数

```c
#define LEFT_FF_OFFSET   200.68f
#define LEFT_FF_K        47.24f
#define LEFT_FF_MIN      250.0f

#define RIGHT_FF_OFFSET  107.12f
#define RIGHT_FF_K       46.27f
#define RIGHT_FF_MIN     260.0f
```

这里 `speed` 的单位是：8ms 内编码器累计值。

## 推荐实现

```c
static float signf_simple(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

static float wheel_ff(float speed, float offset, float k, float min_pwm)
{
    float abs_speed;
    float pwm;
    float sign;

    if (speed == 0.0f) {
        return 0.0f;
    }

    sign = signf_simple(speed);
    abs_speed = speed * sign;
    pwm = offset + k * abs_speed;

    if (pwm < min_pwm) {
        pwm = min_pwm;
    }

    return sign * pwm;
}

left_pwm = wheel_ff(target_left_speed,
                    LEFT_FF_OFFSET,
                    LEFT_FF_K,
                    LEFT_FF_MIN);

right_pwm = wheel_ff(target_right_speed,
                     RIGHT_FF_OFFSET,
                     RIGHT_FF_K,
                     RIGHT_FF_MIN);
```

## 常用目标速度表

| 目标 speed_8ms | 左轮 PWM | 右轮 PWM |
|---:|---:|---:|
| 5 | 437 | 338 |
| 6 | 484 | 385 |
| 8 | 579 | 477 |
| 10 | 673 | 570 |
| 15 | 909 | 801 |
| 20 | 1145 | 1033 |
| 25 | 1382 | 1264 |
| 30 | 1618 | 1495 |
| 35 | 1854 | 1726 |
| 40 | 2090 | 1958 |

## 使用注意

- `target_speed == 0` 时必须输出 `0`，不能输出起转 PWM。
- `target_speed` 如果是负数，前馈 PWM 也应该是负数，方向交给电机方向函数处理。
- 这只是前馈，后面可以叠加速度闭环 PID：

```c
pwm = ff_pwm + speed_pid_out;
```

- 上车后先验证 `speed_8ms = 10, 20, 30` 三档。如果稳态速度偏差有固定方向，再微调对应轮子的 `ff_k`。
