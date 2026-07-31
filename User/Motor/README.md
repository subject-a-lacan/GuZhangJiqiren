# 小球机构每 1 脉冲查找表

## 结论

本表完全使用相对脉冲，不绑定任何绝对零点：

```c
int32_t absolute_target = zero_pulse + relative_target;
```

- `zero_pulse`：实物调平后得到的驱动器绝对脉冲，暂时未知，留在工程配置或状态结构中调节。
- `relative_target`：查表得到的相对水平位置脉冲。
- 修改 `zero_pulse` 时不需要重新生成本表。

## 表的范围

- 最小相对脉冲：`-2500`
- 最大相对脉冲：`6908`
- 间隔：`1` 脉冲
- 项数：`9409`

该范围来自已经核验的原表绝对脉冲 `P=0..9408`，转换关系是原标定阶段的 `n=P-2500`。正向理论临界点为 `n=6908.225...`，因此最后一个有效整数项是 `6908`。

调节 `zero_pulse` 后，这张相对表对应的驱动器绝对目标范围会整体平移为：

```text
[zero_pulse - 2500, zero_pulse + 6908]
```

还应当根据实物碰撞位置设置更保守的绝对软限位；表的几何边界不能代替实物软限位。

## 与原 50 脉冲表的小数差异

原表把两固定轴高度差写成了 CAD 四位小数 `35.7071 mm`，所以水平行残留了约 `0.000012°` 的数值误差。本版根据水平闭合关系使用：

```text
sqrt(50² - (200 + 30 - 195)²) = 35.7071421427 mm
```

因此 `n=0` 严格为 `θ=0`。端点与旧表相比只变化约 `0.000013°`，这是消除 CAD 小数截断误差，并不是机构关系发生了改变。

## 正方向和物理量

- 相对脉冲为正：水管靠近车头的一端抬高。
- 小球位置向车头为正。
- 因而水管车头端抬高时，小球受重力向车尾加速，加速度为负。

重力项使用：

```text
a0(n) = -(5/7) * g * sin(theta(n))
g = 9810 mm/s^2
```

小车加速度为 `car_accel_mm_s2` 时，完整模型是：

```text
a_model(n) = a0(n) - (5/7) * car_accel_mm_s2 * cos(theta(n))
```

## 文件说明

- `ball_mechanism_lut.h`：范围、定点格式、结构体和相对/绝对脉冲辅助函数。
- `ball_mechanism_lut.c`：9409 项、每 1 脉冲一项的单片机定点数表。
- `ball_mechanism_lut_reference.xlsx`：逐项完整浮点参考值和可审计公式，方便人工检查；不放入单片机。
- `generate_ball_mechanism_lut.py`：离线重生成脚本。

## 定点格式

每项只占 4 字节：

```c
typedef struct
{
    int16_t  gravity_accel_x10;
    uint16_t cos_theta_q15;
} BallMechanismLutEntry;
```

- `gravity_accel_x10 / 10` 得到 `a0`，单位为 `mm/s^2`，分辨率为 `0.1 mm/s^2`。
- `cos_theta_q15 / 32768` 得到 `cos(theta)`。
- 9409 项的数据区理论大小为 `9409 * 4 = 37636` 字节，约 `36.75 KiB`。
- 数组不重复保存脉冲编号；数组下标 `i` 对应 `n=-2500+i`。

若改用两列 `float`，仅数组就需要约 `9409 * 8 = 75272` 字节，因此本版本采用定点数。

## 接入示例

```c
/* 该变量应放在你的统一机构配置中，调平后再赋真实值。 */
int32_t zero_pulse;

int32_t relative_target = 500;
int32_t absolute_target =
    BallMechanismLut_ToAbsolute(zero_pulse, relative_target);

uint32_t index =
    BallMechanismLut_IndexFromRelative(relative_target);

int32_t gravity_accel_x10 =
    g_ball_mechanism_lut[index].gravity_accel_x10;
uint32_t cos_theta_q15 =
    g_ball_mechanism_lut[index].cos_theta_q15;
```

`zero_pulse`故意没有在查表模块中定义，也没有给一个假数值，避免尚未调平时误把占位值作为真实绝对坐标发送给电机。
