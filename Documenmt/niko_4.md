你现在需要直接修改一个 STM32G4 电赛小车工程，实现 2026 年电赛 H 题第四问的“小车平滑缓启动”。

请先完整阅读工程中的以下文件，再开始修改：

- status.h
- status.c
- Defect.h
- Defect.c
- wheel.h
- wheel.c
- pid.h
- pid.c
- timer_it.c
- main.c
- log.c / log.h
- uart_it.c / uart_it.h
- ball_control.c / ball_control.h

不要凭空假设不存在的结构体成员、函数或串口接口。任何修改都必须基于工程中的实际代码。

==================================================
一、任务目标
==================================================

第四问使用的任务编号为：

TASK_ADV_4
对应：
driver_task7(status)

需要实现一个独立、非阻塞、可复用的五次 S 曲线速度规划器，使小车从静止平滑加速到巡航速度。

这个模块需要同时输出：

1. 小车公共目标速度
   单位：编码器 count / 5 ms

2. 对应的物理目标速度
   单位：mm/s

3. 小车规划纵向加速度 a_c
   单位：mm/s²

4. 实际四轮平均速度
   单位：
   - count / 5 ms
   - mm/s

5. 编码器累计里程
   单位：mm

规划得到的公共目标速度用于：

follow_line()
→ 左右轮差速叠加
→ 四轮速度闭环

规划得到的 a_c 用于：

ball_control_request(status, 0.0f, a_c)

本阶段不要重写小球估计器、机械查表、步进电机控制和 MaixCAM 协议。

==================================================
二、必须遵守的工程事实
==================================================

1. 控制周期

TIM5 硬件中断周期为 1 ms。

每 5 ms调用一次：

update_status(&status)

因此速度规划器的控制周期为：

Ts = 0.005 s

注意：

现有 PID 结构体中的 T 使用的是工程原有的毫秒标度，现有 PID 参数已经按这种写法使用。

不要顺手把整个 PID 模块的 T 改成秒，否则会破坏现有 PID 参数。

新增加的物理速度规划模块必须单独使用：

sample_time_s = 0.005f

2. 编码器速度单位

get_wheel_speed() 每 5 ms读取一次编码器定时器的计数增量并清零。

因此：

cur_speed 的单位是 count / 5 ms

wheel->tar_speed 是 float，且和 cur_speed 直接作差进入速度 PID，因此：

tar_speed 的单位也是 count / 5 ms

3. 编码器和轮子参数

电机减速比：

1:28

编码器线数：

13

定时器正交编码器四倍频：

4 倍频

轮子直径：

66.0 mm

车轮转一圈对应的计数为：

COUNTS_PER_WHEEL_REV = 13 × 4 × 28

每个编码器计数对应的距离为：

MM_PER_COUNT =
    π × 66.0 / COUNTS_PER_WHEEL_REV

不要在多个文件重复写这一公式。

应当让 pid.h 或一个统一配置头文件成为唯一数据源。

建议保留现有宏，并修改/增加为：

#define ENCODER_PPR 13.0f
#define ENCODER_QUADRATURE 4.0f
#define GEAR_RATIO 28.0f
#define WHEEL_DIAMETER_MM 66.0f

#define COUNTS_PER_WHEEL_REV \
    (ENCODER_PPR * ENCODER_QUADRATURE * GEAR_RATIO)

#define WHEEL_CIRCUMFERENCE_MM \
    (3.1415926f * WHEEL_DIAMETER_MM)

#define MM_PER_COUNT \
    (WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_WHEEL_REV)

如工程其他代码继续使用 CM_PER_PULSE，应保持兼容，例如：

#define CM_PER_PULSE (MM_PER_COUNT * 0.1f)

不要同时保留互相矛盾的 6.723 cm 和 6.6 cm。

4. 四轮方向

get_wheel_speed() 最终会乘 wheel->dir。

工程设计意图是：

小车向车头方向运动时，四个 cur_speed 都为正。

在实际代码中继续使用已经完成方向修正后的四个 cur_speed，不要再次乘 dir。

5. 当前循迹关系

现有 follow_line() 的基本形式为：

右轮目标 = base_speed - diff
左轮目标 = base_speed + diff

四轮分别为：

wheel[0] = base_speed - diff
wheel[1] = base_speed + diff
wheel[2] = base_speed - diff
wheel[3] = base_speed + diff

缓启动模块只生成一个公共速度。

不要分别给四个轮子单独生成四条缓启动曲线。

==================================================
三、设计方法：五次 S 曲线
==================================================

这里的“五次 S 曲线”不需要测量陀螺仪加速度，也不需要测量 jerk。

不要读取 IMU 加速度用于生成这条曲线。

不要对 IMU 加速度做差分。

不要在线测量加加速度。

它只是根据当前时间，计算一个平滑的目标速度和理论加速度。

1. 调参宏

在 Defect.c 顶部或一个专门的 task7_config.h 中集中定义：

#define TASK7_CRUISE_SPEED_UNIT       12.0f
#define TASK7_RAMP_DISTANCE_MM       300.0f
#define TASK7_AB_LENGTH_MM           1500.0f
#define TASK7_DEBUG_PERIOD_MS        100u
#define TASK7_DIFF_LIMIT_RATIO       1.0f
#define TASK7_ENABLE_DEBUG           1

说明：

TASK7_CRUISE_SPEED_UNIT：
巡航目标速度，单位 count/5 ms。

初始值可以使用 12.0f，后续实车调整。

TASK7_RAMP_DISTANCE_MM：
计划完成缓启动所使用的距离，单位 mm。

初始值可以使用 300.0f，后续实车调整。

这些只是可调初值，不代表理论最优参数。

2. 巡航物理速度

设：

U = TASK7_CRUISE_SPEED_UNIT

U 的单位是 count/5 ms。

物理巡航速度：

V = U × MM_PER_COUNT / Ts

其中：

V 单位为 mm/s；
Ts = 0.005 s。

3. 根据缓启动距离计算缓启动时间

设：

Sr = TASK7_RAMP_DISTANCE_MM

Sr 单位为 mm。

五次 S 曲线从 0 加速到 V 的平均速度正好为 V/2，因此：

Tr = 2 × Sr / V

其中：

Tr 为缓启动总时间，单位 s。

不要使用“等待固定时间后突然切速”的方法。

虽然曲线以时间为自变量，但时间只用于连续计算当前速度，整个缓启动期间每 5 ms都会生成一个新的速度目标，不存在突然切换。

4. 归一化时间

当前经过时间：

elapsed_s =
    (now_ms - start_time_ms) × 0.001f

归一化进度：

tau = elapsed_s / Tr

把 tau 限制在：

0 ≤ tau ≤ 1

5. 五次曲线

定义：

q(tau) =
    10 × tau³
  - 15 × tau⁴
  +  6 × tau⁵

目标速度：

target_speed_unit =
    U × q(tau)

单位：

count / 5 ms

目标物理速度：

target_speed_mm_s =
    V × q(tau)

单位：

mm/s

不要使用 powf()。

应使用连续乘法：

tau2 = tau * tau
tau3 = tau2 * tau
tau4 = tau3 * tau
tau5 = tau4 * tau

6. 规划加速度

q 对 tau 的导数为：

dq =
    30 × tau²
  - 60 × tau³
  + 30 × tau⁴

规划小车纵向加速度：

accel_mm_s2 =
    V / Tr × dq

单位：

mm/s²

这就是需要传给小球控制器的 a_c。

不要使用：

base_speed[k] - base_speed[k-1]

作为加速度。

也不要使用整数速度差分。

7. 曲线结束

当：

elapsed_s >= Tr

设置：

phase = CAR_SPEED_CRUISE
progress = 1.0f
target_speed_unit = U
target_speed_mm_s = V
accel_mm_s2 = 0.0f

之后保持巡航速度。

==================================================
四、新建速度规划模块
==================================================

新建：

car_speed_profile.h
car_speed_profile.c

结构体建议如下，名称可以略作调整，但字段意义必须保留：

typedef enum
{
    CAR_SPEED_IDLE = 0,
    CAR_SPEED_RAMP,
    CAR_SPEED_CRUISE
} CAR_SPEED_PHASE;

typedef struct
{
    uint8_t enabled;
    CAR_SPEED_PHASE phase;

    uint32_t start_time_ms;

    float sample_time_s;
    float mm_per_count;

    float cruise_speed_unit;
    float cruise_speed_mm_s;

    float ramp_distance_mm;
    float ramp_time_s;

    float progress;
    float target_speed_unit;
    float target_speed_mm_s;
    float accel_mm_s2;

    float measured_speed_unit;
    float measured_speed_mm_s;

    int64_t sum_wheel_counts;
    float mileage_mm;

    float ideal_ab_time_s;
    float peak_accel_mm_s2;

} CAR_SPEED_PROFILE;

不需要在结构体中保存测量 jerk。

不需要保存 IMU 加速度。

至少实现以下接口：

void car_speed_profile_init(
    CAR_SPEED_PROFILE *profile,
    float sample_time_s,
    float mm_per_count);

void car_speed_profile_reset(
    CAR_SPEED_PROFILE *profile);

uint8_t car_speed_profile_start(
    CAR_SPEED_PROFILE *profile,
    uint32_t now_ms,
    float cruise_speed_unit,
    float ramp_distance_mm);

void car_speed_profile_update_measurement(
    CAR_SPEED_PROFILE *profile,
    int16_t wheel0_count,
    int16_t wheel1_count,
    int16_t wheel2_count,
    int16_t wheel3_count);

void car_speed_profile_step(
    CAR_SPEED_PROFILE *profile,
    uint32_t now_ms);

要求：

- 所有函数必须非阻塞；
- 不允许 HAL_Delay；
- 不允许 while 等待；
- 不允许串口发送；
- 不允许动态内存；
- 不允许把运行状态放进函数内部 static 局部变量；
- 参数非法时 start() 返回 0；
- reset() 后所有目标速度、加速度、里程和累计计数归零；
- reset() 时保留 sample_time_s 和 mm_per_count，或者在重置后重新初始化；
- 对 NULL 指针进行保护。

==================================================
五、实际速度和里程计算
==================================================

update_status() 每 5 ms已经读取四个轮子的 cur_speed。

在四个 get_wheel_speed() 全部执行完成后，立即调用：

car_speed_profile_update_measurement(...)

四轮平均计数：

average_count =
    (wheel0_count
   + wheel1_count
   + wheel2_count
   + wheel3_count) / 4

其中：

average_count 单位为 count/5 ms。

实测物理速度：

measured_speed_mm_s =
    average_count × MM_PER_COUNT / Ts

其中 Ts = 0.005 s。

里程不要再对速度做一次梯形积分。

因为 cur_speed 本身就是当前 5 ms 内累计的编码器计数。

累计四轮总计数：

sum_wheel_counts +=
    wheel0_count
  + wheel1_count
  + wheel2_count
  + wheel3_count

车辆中心累计里程：

mileage_mm =
    sum_wheel_counts × MM_PER_COUNT / 4

仅在 profile->enabled 时累计第四问里程。

profile 禁用时可以继续更新 measured_speed，但不要累计第四问里程。

不要使用四轮速度绝对值。

小车向前时应累计正值；如果车辆倒退，应允许里程减少。

==================================================
六、挂接状态树
==================================================

在 status.h 中包含：

#include "car_speed_profile.h"

当前 CONTROL 结构体中已经有 BALL_CONTROL。

修改为：

typedef struct
{
    BALL_CONTROL ball;
    CAR_SPEED_PROFILE car_speed;
} CONTROL;

不要把 CAR_SPEED_PROFILE 放在 driver_task7() 的静态局部变量中。

不要新建独立全局变量保存同一份速度规划状态。

所有运行状态统一挂在：

status.control.car_speed

==================================================
七、初始化位置
==================================================

在 init_status() 的现有初始化流程中，增加：

car_speed_profile_init(
    &status->control.car_speed,
    (float)T * 0.001f,
    MM_PER_COUNT);

注意：

传入的 T 是 5 ms。

必须转换为：

0.005 s

不要直接把 5 传给物理速度规划器。

不要修改原有 PID 初始化中 T=5 的逻辑。

==================================================
八、PID 状态复位
==================================================

现有 compute_pid() 保存：

- error
- last_error
- integral
- derivative
- out
- is_first

当前 driver_wheel() 在 stop_cmd 有效时直接关闭 PWM并返回，但不会自动清理 PID 历史。

因此在 pid.h / pid.c 新增：

void pid_reset_state(PID *pid);

实现要求：

pid->error = 0.0f;
pid->last_error = 0.0f;
pid->integral = 0.0f;
pid->derivative = 0.0f;
pid->out = 0.0f;
pid->is_first = 1;

不要改变：

kp
ki
kd
T
integral_max
InteralCoef

第四问开始和停止时都要复位：

- follow_line_pid
- 四个 wheel_pid

固定执行四次的 for 循环是允许的，它不是阻塞等待。

==================================================
九、修改 task_start()
==================================================

不要在 driver_task7() 中反复设置 task_running。

task_start() 已经负责设置：

armed
task_running
stop_cmd
phase_start_time

在 task_start() 中：

1. 每次任务开始前，先保证旧的 car_speed 状态已复位。

2. 如果 task_id == TASK_ADV_4：

- 复位 follow_line_pid；
- 复位四个 wheel_pid；
- 把四个 wheel[i].tar_speed 清零；
- 调用 car_speed_profile_start()；
- 如果启动失败，调用 task_stop(status) 并 return；
- 把第四问需要的一次性 MaixCAM 命令放到这里发布；
- 不要使用 phase_mileage 充当一次性布尔标志。

示意逻辑：

if (status->task.task_id == TASK_ADV_4)
{
    pid_reset_state(
        &status->state.status_pid.follow_line_pid);

    for (uint8_t i = 0; i < 4; i++)
    {
        pid_reset_state(
            &status->motor.wheel[i].wheel_pid);

        status->motor.wheel[i].tar_speed = 0.0f;
    }

    if (!car_speed_profile_start(
            &status->control.car_speed,
            (uint32_t)status->state.time,
            TASK7_CRUISE_SPEED_UNIT,
            TASK7_RAMP_DISTANCE_MM))
    {
        task_stop(status);
        return;
    }

    task7_cmd_type = T7_CMD_CDA;
}

第一次进入 driver_task7() 时 elapsed_s 应为 0，因此第一个速度指令必须为 0。

==================================================
十、替换 driver_task7()
==================================================

删除当前 driver_task7() 中的以下逻辑：

- 强制 task_running = 1；
- 强制 motion = STOP；
- 强制 base_speed = 0；
- 使用 phase_mileage == 0 判断是否发送命令；
- 把 phase_mileage 写成 1；
- 固定给 ball_control_request() 传 0 加速度。

新的 driver_task7() 只做以下事情：

1. 获取：

CAR_SPEED_PROFILE *profile =
    &status->control.car_speed;

2. 如果 profile 未启用：

- motion = STOP；
- base_speed = 0；
- return。

3. 调用：

car_speed_profile_step(
    profile,
    (uint32_t)status->state.time);

4. 设置：

status->state.motion = FIND_LINE;
status->task.stop_cmd = 0;

5. 为兼容旧界面，可把浮点目标速度转换成整数镜像：

status->state.base_speed =
    (int16_t)profile->target_speed_unit;

但这个 int16_t 只用于旧界面显示和兼容。

follow_line() 真正使用的必须是：

profile->target_speed_unit

不能把浮点速度先转成 int16_t 后再用于四轮控制，否则小于一个 count/5 ms 的缓慢变化会被量化掉。

6. 恢复 phase_mileage 的真实语义：

status->task.phase_mileage =
    profile->mileage_mm;

单位为 mm。

7. 调用小球控制请求：

ball_control_request(
    status,
    TASK7_BALL_TARGET_MM,
    profile->accel_mm_s2);

其中：

TASK7_BALL_TARGET_MM = 0.0f

==================================================
十一、修改 follow_line()
==================================================

不能直接把 status.state.base_speed 改成 float，因为这可能影响大量旧任务。

应让 follow_line() 根据 car_speed 是否启用选择公共速度。

逻辑：

float base_speed;

if (status->control.car_speed.enabled)
{
    base_speed =
        status->control.car_speed.target_speed_unit;
}
else
{
    base_speed =
        (float)status->state.base_speed;
}

随后计算循迹 PID 的 diff。

第四问缓启动初期 base_speed 很小，如果不限制 diff，可能出现：

右轮 = 小正数 - 较大 diff
左轮 = 小正数 + 较大 diff

导致一侧反转，车辆原地扭动，从而破坏纵向缓启动。

因此当 car_speed 启用时，对 diff 增加任务专用限制：

diff_limit =
    fabsf(base_speed) *
    TASK7_DIFF_LIMIT_RATIO

diff = clamp(
    diff,
    -diff_limit,
    +diff_limit)

TASK7_DIFF_LIMIT_RATIO 初始设置为 1.0f。

这样保证缓启动期间：

|diff| ≤ |base_speed|

所以任意一侧目标速度最低为 0，不会因为循迹修正而反向旋转。

最终保持：

wheel[0].tar_speed = base_speed - diff;
wheel[1].tar_speed = base_speed + diff;
wheel[2].tar_speed = base_speed - diff;
wheel[3].tar_speed = base_speed + diff;

不要分别独立缓升四个轮子。

不要修改其他任务未启用 car_speed 时的循迹行为。

==================================================
十二、修改 task_stop()
==================================================

task_stop() 中必须：

1. car_speed_profile_reset(
       &status->control.car_speed);

2. 复位 follow_line_pid。

3. 遍历四个轮子：

- tar_speed = 0.0f；
- pid_reset_state(&wheel_pid)。

4. phase_mileage = 0.0f。

5. 保留原有：

- ball_control_disable(status)；
- task_running = 0；
- armed = 0；
- stop_cmd = 1；
- motion = STOP；
- base_speed = 0。

虽然 stop_cmd 会让四个 driver_wheel() 直接关闭 PWM，但仍然必须清除四个 tar_speed 和 PID 历史，避免下一次启动继承旧状态。

==================================================
十三、修复 TIM5 系统时间更新位置
==================================================

当前 timer_it.c 中 status.state.time += 1 位于 htim5 判断之前。

这会导致其他定时器如果也进入 HAL_TIM_PeriodElapsedCallback()，系统时间被错误增加。

改为：

void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim)
{
    if (htim != &htim5)
    {
        return;
    }

    status.state.time += 1;

    if (status.state.time % 5u == 0u)
    {
        update_status(&status);
    }
}

不要在定时器中断中增加任何：

- HAL_Delay；
- UART 等待；
- printf；
- 阻塞发送；
- 电机等待到位；
- while 等待。

==================================================
十四、前馈和低速输出
==================================================

现有 wheel.c 中的电机前馈为：

正转：
ff = offset + k × tar_speed

因此只要 tar_speed 从 0 变成一个很小的正数，前馈就会加入非零 offset。

本次第一阶段不要擅自删除现有前馈截距，也不要重新标定前馈。

保留当前前馈，先通过调试观察实际启动是否出现明显跳变。

但是请在代码注释中明确：

如果目标速度平滑而实际速度突然起跳，重点排查：

- 前馈 offset；
- 电机静摩擦；
- PID 积分；
- 低速 PWM 输出。

当前 wheel.c 中：

先限幅到 ±8499，
随后低速时再次限幅到 ±8500。

后一个限幅不会产生实际效果。

本次可以只增加注释，不要在没有实车数据时擅自改变低速限幅值。

==================================================
十五、低频调试输出
==================================================

需要以 100 ms 周期输出以下数据：

1. target_speed_unit
   单位：count/5 ms

2. measured_speed_unit
   单位：count/5 ms

3. target_speed_mm_s
   单位：mm/s

4. measured_speed_mm_s
   单位：mm/s

5. mileage_mm
   单位：mm

6. accel_mm_s2
   单位：mm/s²

7. progress
   范围：0~1

8. 四轮 cur_speed，可选。

禁止在 driver_task7() 或 update_status() 中直接阻塞发送。

请检查 UART_send_justfloat() 的实际实现：

- 如果它只是写入 DMA/发送队列，可以在主循环服务函数中使用；
- 如果它内部使用 HAL_UART_Transmit、等待 TXE 或其他阻塞方法，不得直接使用。

建议新增：

void task7_debug_service(STATUS *status);

并在 main.c 主循环中调用：

task7_debug_service(&status);

位置可以放在：

task7_flush();

附近。

task7_debug_service() 必须：

- 只在 TASK7_ENABLE_DEBUG 开启时工作；
- 只在 TASK_ADV_4 运行时工作；
- 每 TASK7_DEBUG_PERIOD_MS 发送一次；
- 在主循环中执行，不在 5 ms中断中执行；
- 使用工程已有的非阻塞 UART DMA或发送队列；
- 如果发送队列忙，立即返回，本周期丢弃或下次重试；
- 不允许等待 UART 空闲；
- 不允许 printf。

如果工程目前没有 UART1 非阻塞发送接口，请实现最小的持久缓冲区 + DMA busy 标志 + 主循环服务，不要在中断中格式化长字符串。

JustFloat 数据可以直接发送 float 数组，避免 snprintf。

==================================================
十六、并发和状态一致性
==================================================

5 ms控制逻辑在 TIM5 中断中运行。

主循环只负责：

- MaixCAM 服务；
- 小球控制服务；
- 步进串口服务；
- 调试发送。

控制中断中写 car_speed 状态，主循环只读取调试数据。

32 位对齐 float 在 Cortex-M4 上可原子读取，但为了保证多个调试字段来自同一快照，可以：

- 在主循环复制调试数据时短暂关闭中断；
- 只复制几个 32 位字段；
- 复制完成后立即恢复中断；
- 不允许在关中断期间发送 UART。

不要长时间关闭中断。

控制计算本身不要使用临界区。

==================================================
十七、不要做的事情
==================================================

本次修改禁止：

1. 不要用 PWM 斜坡代替目标速度斜坡。

2. 不要绕过 driver_wheel() 的速度闭环。

3. 不要分别给四个轮子独立规划速度。

4. 不要读取 IMU 加速度来生成 S 曲线。

5. 不要测量或差分 jerk。

6. 不要使用整数 base_speed 作为最终缓启动速度。

7. 不要把控制状态藏在 driver_task7() 的 static 局部变量中。

8. 不要使用 HAL_Delay。

9. 不要在中断中调用 printf、snprintf、阻塞 UART。

10. 不要等待步进电机到位。

11. 不要重写 ball_control。

12. 不要修改 MaixCAM 协议。

13. 不要修改其他任务的控制逻辑。

14. 不要改变现有 PID 参数的时间单位。

15. 不要每 5 ms发送调试数据。

==================================================
十八、验收条件
==================================================

代码完成后必须满足以下检查：

1. 上电未启动任务时，四轮 PWM 为 0。

2. 第四问按键启动的第一个 5 ms周期：

target_speed_unit = 0
accel_mm_s2 = 0

3. 缓启动阶段：

target_speed_unit 单调不减。

4. target_speed_unit 从 0 平滑增长到 TASK7_CRUISE_SPEED_UNIT。

5. accel_mm_s2：

- 启动时为 0；
- 中间逐渐增大再减小；
- 进入巡航时回到 0。

6. progress 从 0 平滑增长到 1。

7. follow_line() 仍保持：

右轮 = base - diff
左轮 = base + diff

8. 第四问缓启动期间不允许因为 diff 过大使单侧轮子反转。

9. 四轮平均实测速度能够输出。

10. mileage_mm 使用编码器计数累计，不是用整数速度差分。

11. task_stop() 后：

- profile disabled；
- target speed = 0；
- acceleration = 0；
- mileage = 0；
- 四轮 tar_speed = 0；
- 四个速度 PID 状态清零；
- 循迹 PID 状态清零。

12. 第二次启动与第一次启动状态一致，不继承任何旧积分、旧里程或旧曲线进度。

13. 其他 TASK 不启用 car_speed 时，行为保持原样。

14. 中断中不存在阻塞操作。

15. 编译无新增 warning。

==================================================
十九、输出要求
==================================================

完成修改后，请按以下顺序输出：

1. 先列出实际修改和新增的文件。

2. 逐文件说明修改目的。

3. 给出完整的新文件内容：
   - car_speed_profile.h
   - car_speed_profile.c

4. 对已有文件给出精确 diff 或完整替换片段：
   - pid.h
   - pid.c
   - status.h
   - status.c
   - Defect.h
   - Defect.c
   - timer_it.c
   - main.c
   - 必要的 UART 调试服务文件

5. 标明每段代码应该插入到哪个函数、哪个已有语句之后。

6. 不要只给伪代码，必须给可编译 C 代码。

7. 不要一次性重构整个工程，只做实现第四问缓启动所需的最小修改。

8. 如果发现工程中的实际函数名或结构体和本文描述略有差异，应以工程源码为准，并在输出中说明如何适配，不得另外创造重复结构。