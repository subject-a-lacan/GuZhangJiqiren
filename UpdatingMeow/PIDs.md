# AI 提示词：用最小工程改动实现一二问 / 三四问两套 PID 和前馈

你是一个嵌入式 STM32 小车工程代码 agent。请阅读当前工程的 `User/Status/status.c`、`User/Status/status.h`、`User/Status/Defect.c`、`User/Motor/wheel.c`、`User/Tool/pid.c`、`User/Tool/pid.h`。

目标：在不推倒现有架构的前提下，让第一问、第二问使用一套空载控制参数，让第三问、第四问使用一套负重控制参数。两套参数包括：

```text
1. 巡线环 PID
2. 角度环 PID
3. 左右轮内环 PID
4. 轮速前馈参数
```

## 总原则

不要重写控制架构。

不要新增两套运行时 PID bank。

不要修改 `compute_pid()` 的数学逻辑。

不要大改 `follow_line()`、`keep_angle()`、`update_status()` 的调用链。

最简单的工程方案是：

```text
仍然只保留当前这一套运行时 PID 结构体。

定义两套参数模板：
  BASIC_PARAM：第一问、第二问使用
  ADV_PARAM：第三问、第四问使用

每次 task_start() 时，根据 task_id 把对应模板覆盖到当前运行时 PID 结构体和前馈参数里。
```

这样可以做到一二问和三四问参数分家，但不引入复杂 profile 系统。

## 当前问题

现有代码里 PID 和前馈基本是写死的：

```text
status.state.status_pid.follow_line_pid
status.state.status_pid.keep_angle_pid
status.motor.wheel[0].wheel_pid
status.motor.wheel[1].wheel_pid
```

这些运行时结构体只有一套。

`wheel.c` 里的前馈也写死：

```c
float ff_abs = 157.0f + 18.3f * ABS(wheel->tar_speed);
if (ff_abs < 254.0f) ff_abs = 254.0f;
```

第三问、第四问负重后，空载参数往往带不动；如果把参数调大，一二问空载又会太猛。因此需要两套参数。

## 最小实现方案

### 1. 新增控制参数结构体

建议放在 `status.h` 或单独新建一个很小的 `control_param.h`。为了少改文件，优先放在 `status.h` 附近。

```c
typedef struct CONTROL_PARAM {
  PID follow_line_pid;
  PID keep_angle_pid;
  PID wheel_left_pid;
  PID wheel_right_pid;

  float ff_offset;
  float ff_k;
  float ff_min;
} CONTROL_PARAM;
```

注意：这里直接存 `PID`，不是只存 `kp/ki/kd`。这样套参数时直接赋值即可，同时自动清零 `integral / last_error / is_first` 等运行状态。

### 2. 定义两套参数模板

建议放在 `status.c` 或新建 `control_param.c`。为了最小改动，可以先放在 `status.c`。

示例：

```c
static const CONTROL_PARAM basic_control_param = {
  .follow_line_pid = { /* 用 init_pid 生成更方便，见下一节 */ },
};
```

更推荐不要手写结构体字段，而是写一个函数生成模板：

```c
static CONTROL_PARAM make_basic_control_param(void) {
  CONTROL_PARAM p;
  p.follow_line_pid = init_pid(1.0f, 0.03f, 0.0f, 10.0f, 1.0f, 0.0f);
  p.keep_angle_pid  = init_pid(1.0f, 0.0f,  0.0f, 10.0f, 1.0f, 0.0f);
  p.wheel_left_pid  = init_pid(8.0f, 0.0f,  0.0f, 10.0f, 100.0f, 0.50f);
  p.wheel_right_pid = init_pid(8.0f, 0.0f,  0.0f, 10.0f, 100.0f, 0.50f);
  p.ff_offset = 157.0f;
  p.ff_k = 18.3f;
  p.ff_min = 254.0f;
  return p;
}

static CONTROL_PARAM make_adv_control_param(void) {
  CONTROL_PARAM p;
  p.follow_line_pid = init_pid(/* 三四问巡线 PID */);
  p.keep_angle_pid  = init_pid(/* 三四问角度 PID */);
  p.wheel_left_pid  = init_pid(/* 三四问左轮内环 PID */);
  p.wheel_right_pid = init_pid(/* 三四问右轮内环 PID */);
  p.ff_offset = /* 负重前馈 offset */;
  p.ff_k = /* 负重前馈 k */;
  p.ff_min = /* 负重起步最小 PWM */;
  return p;
}
```

第三问、第四问负重后，`ff_offset / ff_k / ff_min` 通常需要比 BASIC 更大，但具体值必须实车调。

### 3. 新增 apply_control_param()

建议放在 `status.c`。

```c
void apply_control_param(STATUS *status, CONTROL_PARAM p) {
  status->state.status_pid.follow_line_pid = p.follow_line_pid;
  status->state.status_pid.keep_angle_pid = p.keep_angle_pid;
  status->motor.wheel[0].wheel_pid = p.wheel_left_pid;
  status->motor.wheel[1].wheel_pid = p.wheel_right_pid;

  set_wheel_ff_param(p.ff_offset, p.ff_k, p.ff_min);
}
```

注意这里传值即可，不需要指针，不需要保存模板地址。

这个函数的效果是：

```text
切换任务时，把当前运行时 PID 整套换成对应参数。
由于 PID 是重新 init 出来的，积分、微分、last_error 都自然清零。
```

### 4. 修改 wheel.c 的前馈为可配置变量

在 `wheel.c` 顶部新增三个静态变量：

```c
static float wheel_ff_offset = 157.0f;
static float wheel_ff_k = 18.3f;
static float wheel_ff_min = 254.0f;
```

新增 setter：

```c
void set_wheel_ff_param(float offset, float k, float min_pwm) {
  wheel_ff_offset = offset;
  wheel_ff_k = k;
  wheel_ff_min = min_pwm;
}
```

在 `wheel.h` 声明：

```c
void set_wheel_ff_param(float offset, float k, float min_pwm);
```

把 `driver_wheel()` 里的硬编码前馈：

```c
float ff_abs = 157.0f + 18.3f * ABS(wheel->tar_speed);
if (ff_abs < 254.0f) ff_abs = 254.0f;
```

改成：

```c
float ff_abs = wheel_ff_offset + wheel_ff_k * ABS(wheel->tar_speed);
if (ff_abs < wheel_ff_min) ff_abs = wheel_ff_min;
```

这样一二问和三四问可以共用 `driver_wheel()`，但使用不同前馈。

### 5. 在 task_start() 中选择 BASIC 或 ADV

在 `Defect.c` 的 `task_start()` 开头或 PID 清零附近加入：

```c
if (status->task.task_id == TASK_BASIC_1 || status->task.task_id == TASK_BASIC_2) {
  apply_control_param(status, make_basic_control_param());
} else {
  apply_control_param(status, make_adv_control_param());
}
```

如果 `make_basic_control_param()` 放在 `status.c` 内部，则可以封装成：

```c
void apply_basic_control_param(STATUS *status);
void apply_adv_control_param(STATUS *status);
```

然后 `Defect.c` 只调用：

```c
if (status->task.task_id == TASK_BASIC_1 || status->task.task_id == TASK_BASIC_2) {
  apply_basic_control_param(status);
} else {
  apply_adv_control_param(status);
}
```

这个更干净，`Defect.c` 不需要知道每个 PID 的具体数值。

## 推荐最终接口

最小、清楚、好维护的接口是：

```c
void apply_basic_control_param(STATUS *status);
void apply_adv_control_param(STATUS *status);
```

调用点只在 `task_start()`。

这样任务层只表达：

```text
一二问 -> BASIC 控制参数
三四问 -> ADV 控制参数
```

而 PID 数值和前馈数值都集中放在 `status.c` 或独立参数文件里。

## UART 调参注意

当前 `main.c` 的 UART 调参直接改当前运行时 PID：

```c
status.state.status_pid.follow_line_pid.kp = val;
status.motor.wheel[0].wheel_pid.kp = val;
```

在这个最小方案下，UART 调参仍然能用，但它只会改“当前正在运行的那一套运行时 PID”。

注意：

```text
如果你调完参数后重新发车，task_start() 会重新 apply 模板，把 UART 临时改的值覆盖掉。
```

所以实车调参流程建议：

```text
1. 先用 UART 找到合适参数。
2. 把最终参数手动写回 make_basic_control_param() 或 make_adv_control_param()。
3. 再重新编译烧录。
```

不要为了 UART 调参去引入复杂的“在线修改模板”系统，第一版不需要。

## 不要做的事

1. 不要改 `compute_pid()`。
2. 不要复制一份 `compute_pid_basic()` / `compute_pid_adv()`。
3. 不要新增复杂的 profile manager。
4. 不要在 `follow_line()` 里判断 `task_id`。
5. 不要在 `keep_angle()` 里判断 `task_id`。
6. 不要在 `driver_wheel()` 里判断 `task_id`。
7. 不要让 `Defect.c` 直接散落一堆 PID 数值。

## 验收标准

1. 第一问、第二问启动时会应用 BASIC PID 和 BASIC 前馈。
2. 第三问、第四问启动时会应用 ADV PID 和 ADV 前馈。
3. `compute_pid()` 没有被改动。
4. `follow_line()` 和 `keep_angle()` 的主调用逻辑基本不变。
5. `driver_wheel()` 不再写死 `157 / 18.3 / 254`，而是使用当前前馈变量。
6. 切换任务后，PID 积分和历史误差不会沿用上一个任务。
7. TASK1 状态机不被破坏。
