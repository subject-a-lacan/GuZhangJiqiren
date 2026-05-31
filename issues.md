就你现在这个场景，我推荐一个最小但靠谱的改法：

不要再用 compute_pid() 算 balance 的 D。
保留 balance 结构体，但它只存参数：

balance.kp = 角度 P
balance.kd = 手写 D 的系数
balance.ki = 继续不用
然后 TASK2 第二阶段手写：

angle_p = task2_update_pot_angle();        // 快角度，用于 P
d_raw = angle_p - last_angle;              // 每 8ms 角度变化量
d_deadband(d_raw);
d_filt += alpha_d * (d_raw - d_filt);      // 只滤 D
d_out = balance.kd * d_filt;
d_out = CONFINE(d_out, -D_LIMIT, D_LIMIT);

balance_out = balance.kp * angle_p + d_out;
你的代码结构里这样最合适，因为：

compute_pid() 的 D 是通用误差差分，太硬了，没法单独加死区、滤波、限幅。
你的 P 项必须快，不能跟 D 一起被重滤波拖慢。
你已经有 balance_pid、mileage_pid 两个结构体，继续用它们存参数最省事。
你主循环已经在打印 balance.out 和 D 输出，占比分析可以继续用。
我建议第一版具体参数/常量：

D_DEADBAND = 0.05 度/周期
D_ALPHA = 0.35
D_LIMIT = 800
PWM_RATE_LIMIT = 400 / 8ms
注意：你的 d_raw 不要除以 0.008，先用“每周期角度差”。
因为你现在的 compute_pid() 其实是除以 T，而 T=8，不是 0.008 秒。你之前调的 KD 量级也是建立在这个奇怪单位上的。为了少引入新变量，先保持“度/周期”的思路。

代码结构推荐这样放：

在 Defect.c 顶部加：

#define TASK2_D_DEADBAND 0.05f
#define TASK2_D_ALPHA    0.35f
#define TASK2_D_LIMIT    800.0f
#define TASK2_PWM_SLEW   400.0f
加静态变量：

static float task2_last_angle;
static float task2_d_filt;
static float task2_last_pwm;
task2_enter_back_swing() 或进入稳定阶段时清零：

task2_last_angle = task2_pot_angle;
task2_d_filt = 0;
task2_last_pwm = 0;
第二阶段替换：

balance_out = compute_pid(balance_param, angle_error);
为：

float d_raw = angle_error - task2_last_angle;
task2_last_angle = angle_error;

if (ABS(d_raw) < TASK2_D_DEADBAND) {
  d_raw = 0.0f;
}

task2_d_filt += TASK2_D_ALPHA * (d_raw - task2_d_filt);

task2_debug_balance_d_out = balance_param->kd * task2_d_filt;
task2_debug_balance_d_out = CONFINE(task2_debug_balance_d_out,
                                    -TASK2_D_LIMIT,
                                     TASK2_D_LIMIT);

balance_out = balance_param->kp * angle_error + task2_debug_balance_d_out;
balance_param->out = balance_out;
最终输出前加 slew rate：

pwm_out = task2_apply_min_pwm(balance_out + position_out);

float delta = pwm_out - task2_last_pwm;
delta = CONFINE(delta, -TASK2_PWM_SLEW, TASK2_PWM_SLEW);
pwm_out = task2_last_pwm + delta;
task2_last_pwm = pwm_out;
调试方法：

先 balance.kd = 0，确认 P 和后两项方向没坏。
加 balance.kd，看静止时最后一列 D 输出是否压在 ±100 左右。
如果静止还跳：
增大 D_DEADBAND 到 0.08 / 0.1
或减小 D_ALPHA 到 0.25
如果动态救不住：
增大 D_LIMIT 到 1000 / 1200
或增大 D_ALPHA 到 0.45
如果电机还是发热抖：
降低 PWM_SLEW 到 250~300
一句话方案：P 快，D 脏就单独清洗，PWM 再限斜率。
这比继续在 compute_pid() 上硬调 KD 更适合你现在这台车