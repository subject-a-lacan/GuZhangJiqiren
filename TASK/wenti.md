# 任务启动逻辑与声光提示修正提示词

请检查并修改本 STM32 工程中的任务启动逻辑和声光提示逻辑。注意严格遵守以下架构语义：

## 1. start_request / armed / task_running 的关系

- `start_request` 是外部输入提出的“一次性进入任务入口请求”。
- 按钮和蓝牙只允许置 `start_request`，不允许直接置 `armed`，也不允许直接置 `task_running`。
- `update_task()` 消费 `start_request`：
  - 如果 `armed == 0` 且 `task_running == 0`，则置 `armed = 1`，并调用 `task_start(status)` 做任务入口初始化。
  - 然后清 `start_request`。
- `armed` 表示“已经进入当前选中任务的流程入口”。
- `task_running` 表示“任务内部确认任务真的开始运行”。
- `task_running` 只能由 `task_basic_1_update()` / `task_basic_2_update()` / `task_adv_1_update()` / `task_adv_2_update()` 这些题内状态机设置。
- `task_start()` 只做入口初始化，不能设置 `task_running = 1`。

## 2. task_start()

`task_start()` 应该做：

```text
清 start_request / stop_request
清 cross_cnt / road_buf / road_determine
清 PID 积分和历史误差
清 phase_mileage
刷新 initial_angle
清左右轮目标速度
根据 task_id 和 start_pose 设置初始 race_phase
设置 phase_start_time
motion/base_speed 初始为 STOP/0
```

注意：不要设置 `task_running = 1`。

## 3. update_task()

推荐流程：

```text
stop_request 优先，调用 task_stop()

只有 armed == 0 且 task_running == 0 时，
才允许处理 task_select_request 和 pose_switch_request

start_request 被消费后：
  armed = 1
  task_start(status)
  清 start_request

如果 armed == 0：
  直接 return，不进入题内状态机

如果 armed == 1：
  根据 task_id 分发到对应 task_xxx_update()
```

题内状态机确认任务真的开始后，再设置：

```c
status->task.task_running = 1;
```

## 4. task_finish() / task_stop()

两者都应该让车停止：

```c
status->task.task_running = 0;
status->task.armed = 0;
status->task.start_request = 0;
status->task.stop_request = 0;
status->state.motion = STOP;
status->state.base_speed = 0;
status->motor.wheel[0].tar_speed = 0;
status->motor.wheel[1].tar_speed = 0;
```

`task_stop()` 不清 `task_id` / `start_pose` / `race_phase` / PID / `cross_cnt`，除非明确需要重置。

## 5. button.c 的按钮逻辑

PB11 短按：

```text
如果 armed == 0 且 task_running == 0：
  计算下一个任务 id
  写 requested_task_id
  置 task_select_request = 1
```

PB11 长按：

```text
如果 armed == 0 且 task_running == 0
并且当前 task_id 是 TASK_BASIC_2 或 TASK_ADV_1：
  置 pose_switch_request = 1
  蜂鸣器长响一段时间
```

PD2 短按：

```text
如果 armed == 0 且 task_running == 0：
  置 start_request = 1
  蜂鸣器短响
```

注意：不要在 `button.c` 里判断 `armed == 1` 才置 `start_request`。

PD2 长按：

```text
灰度校准 correct_gw_analogue()
```

`button.c` 不允许直接修改 `task_running`，不允许直接调用 `task_start()`。

## 6. LED 声光提示

需要实现一个统一的任务 LED 显示函数，例如：

```c
update_task_led(status);
```

三个 LED 顺序为：

```text
板载 LED, LED1, LED2
```

编码规则：

| 状态 | LED 编码 |
|---|---|
| TASK1 | `1 0 1` |
| TASK2 AB | `1 1 1` |
| TASK2 AD | `0 1 1` |
| TASK3 AB | `1 0 0` |
| TASK3 AD | `0 0 0` |
| TASK4 | `1 1 0` |

每次 `task_id` 或 `start_pose` 改变后，都要刷新 LED。

可以在 `task_select()` 和 `task_switch_start_pose()` 中调用，也可以在 `update_task()` 周期里统一根据 `task_id/start_pose` 刷新。

## 7. 蜂鸣器

蜂鸣器用非阻塞 `off_time`：

触发时：

```c
buzzer.on = 1;
buzzer.off_time = status.state.time + 持续时间;
```

周期检查时：

```c
if (buzzer.on && status.state.time >= buzzer.off_time) {
    buzzer.on = 0;
}
```

不要使用 `HAL_Delay()`。

核心原则：

```text
按钮只挂请求，update_task() 置 armed，题内状态机置 task_running。
```
