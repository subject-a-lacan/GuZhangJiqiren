目前的硬件方案大改：为了更容易起摆  我把摆长变得很长 由于题目要求摆的末端到铅垂线最少5cm 那么初始角度的正弦值就是5/摆长 摆长越长 初始角度越小（初始角度就是摆靠在支架上和铅垂线的夹角）这样一来由于初始角度差不多+7度 很小 只需要很小的角速度就能起摆到目标值0度  （摆向小车前方是正角度 最大为7度  摆向小车后方是负角度 最大为-27度左右）
这样一来现有的第二问逻辑是完全不对的：现在是初始向后慢行较长时间 然后短时向前冲刺利用惯性 起摆 但是这是建立在摆初始靠在前面的支架上的（我们车上只有一个支架 在车的前段 初始摆靠在支架上 角度为正7度） 如果摆靠到最后（也就是-27度）  向后慢行向前冲刺的逻辑就会失效 此时应该是向前再向后

而且更关键的是 由于支架到0度 只有7度的活动空间 向后再向前的起摆逻辑可能多余了 现在只需要向前给一个较大的PWM就能起摆  然后第二问角度绝对值大于某个特定值就重回swing_up的状态 这个逻辑是完全错误的 因为 如果是-27度  摆就是靠后的 但是swing是针对初始状态下摆靠前的逻辑。

总结：起摆逻辑多余 直接向前加速即可  保护机制需要修改 只需要小于某个负值时进入保护：向前冲再向后冲

## 接下来代码大体改动方案

### 1. TASK2 状态机改成前冲起摆 + LQR + 后侧补救

当前硬件摆长变长后，初始前支架角度只有约 `+7 度`，已经离直立很近，不需要继续使用原来的方波起摆。

建议 TASK2 状态改成：

```c
typedef enum {
  TASK2_FRONT_KICK = 0,
  TASK2_BALANCE_LQR,
  TASK2_BACK_RECOVER,
} TASK2_STATE;
```

含义：

```text
TASK2_FRONT_KICK:
  初始靠前支架时使用。
  直接给小车一个向前 PWM，持续一小段时间，然后进入 LQR。

TASK2_BALANCE_LQR:
  正常四项 LQR 稳摆。
  不再用 ABS(angle_error) 做保护。

TASK2_BACK_RECOVER:
  只有当 roll 小于某个负阈值时进入，说明摆已经靠到后侧。
  后侧补救动作使用“向前冲一段时间，再向后冲一段时间”，然后回 LQR。
```

### 2. 删除原来的通用方波 swing_up 思路

原来的：

```text
先向后慢行较长时间
再向前短时冲刺
反复方波起摆
```

只适合摆初始靠前支架、且需要较大起摆能量的旧结构。

现在硬件下，它会在两种情况下出错：

- 初始 `+7 度` 时，方波起摆动作多余，容易把系统带乱。
- 摆已经到后侧 `-27 度` 时，继续用针对前侧初始的方波逻辑，方向是错的。

所以 `TASK2_SWING_UP` 不再作为通用状态使用。

### 3. 保护条件改成只看负角度阈值

不要再用：

```c
if (ABS(angle_error) > TASK2_FALLBACK_ANGLE_DEG)
```

因为机械限位不对称：

```text
前侧约 +7 度
后侧约 -27 度
```

建议改成：

```c
if (roll < task2_back_recover_angle)
```

第一版参数建议：

```text
task2_back_recover_angle = -15.0f 或 -20.0f
```

如果正常 LQR 波形会短暂到 `-15 度`，就放宽到 `-20 度`。

### 4. PID_TUNE 中新增/保留的可调参数

TASK2 起摆和补救相关参数要能通过 `UART_PID_Tune()` 在线调：

```c
int16_t  task2_front_kick_pwm;
uint32_t task2_front_kick_time;

int16_t  task2_recover_fwd_pwm;
uint32_t task2_recover_fwd_time;
int16_t  task2_recover_bwd_pwm;
uint32_t task2_recover_bwd_time;

float task2_back_recover_angle;
```

推荐初始值：

```text
task2_front_kick_pwm      = 2500
task2_front_kick_time     = 80

task2_recover_fwd_pwm     = 2500
task2_recover_fwd_time    = 80
task2_recover_bwd_pwm     = 1500
task2_recover_bwd_time    = 120

task2_back_recover_angle  = -20.0
```

调参范围建议：

```text
front_kick_pwm:      1500 ~ 3000
front_kick_time:     40ms ~ 200ms

recover_fwd_pwm:     1500 ~ 3000
recover_fwd_time:    40ms ~ 200ms
recover_bwd_pwm:     800  ~ 2500
recover_bwd_time:    40ms ~ 250ms

back_recover_angle: -12deg ~ -25deg
```

### 5. PID_TUNE 中需要删除的旧可调参数

改完后，原来方波起摆的这些参数不再使用，应从 `UART_PID_Tune()` 里删掉，避免调参时混淆：

```text
task2_swing_speed_fwd
task2_swing_speed_bwd
task2_swing_time_fwd
task2_swing_time_bwd
```

对应的旧命令也应删除或改作新参数：

```text
Cp
Cr
Ct
Cv
```

建议重新分配：

```text
Cp -> task2_front_kick_pwm
Cr -> task2_front_kick_time
Ct -> task2_recover_fwd_pwm
Cv -> task2_recover_fwd_time
Ch -> task2_recover_bwd_pwm
Cy -> task2_recover_bwd_time
Co -> task2_back_recover_angle
```

LQR 参数继续保留：

```text
Cd -> balance_pid.kp
Cf -> balance_pid.kd
Cx -> balance_pid.ki，暂时保持 0
Cj -> mileage_pid.kp
Cl -> mileage_pid.kd
```

### 6. 调试顺序

第一步只调 `TASK2_FRONT_KICK`：

```text
先 Cj0 Cl0
用较小 LQR 参数保证不会乱冲
调 Cp/Cr，让摆能从 +7 度附近进入 LQR 可控范围
```

第二步调 LQR：

```text
Cd/Cf 先接住杆
Cl 再压住小车速度
Cj 最后拉回位置
```

第三步再调后侧补救：

```text
故意让摆掉到后侧，观察 roll 是否小于 Co
调 Ct/Cv/Ch/Cy，让它能从后侧回到 LQR 可控范围
```

不要在一开始就同时调前冲、LQR、后侧补救，否则波形很难判断是谁导致的问题。
