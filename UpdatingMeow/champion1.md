# TASK2 AB 发车状态机移植提示词

只讨论第二问 `TASK_BASIC_2` 的 **AB 发车** 情况。AD 发车之后再单独写，不要这次一起实现。

## 目标路线

AB 发车时先按第一问类似逻辑行驶：

```text
A -> D -> C -> B
```

到 B 点附近后不再普通左转，而是执行 180 度掉头，然后原路返回：

```text
B -> C -> D -> A
```

去程全是左转，回程全是右转。

## 状态机原则

可以仿照 TASK1 的结构，但不要直接复制 TASK1：

```text
TURN_x   角度环转弯
FIND_x   转完后低速找线
SIDE_x   正常巡线
UTURN_x  掉头
FINAL_x  最后一段回发车点停车
```

转弯后仍然采用 TASK1 的经验：

```text
角度误差进入阈值 -> 只允许进入 FIND
中间 6 路看到线 -> 允许从 FIND 切回 SIDE
中间 4 路稳定看到线 -> 才恢复正常速度
```

## 停车逻辑

最终停车时不要只写：

```c
status->task.stop_cmd = 1;
```

因为 `status.c` 里 `FIND_LINE` 分支会重新把 `stop_cmd` 置 0。

最终停车必须至少做到：

```c
status->state.motion = STOP;
status->task.stop_cmd = 1;
status->task.task_running = 0;
status->task.armed = 0;
status->motor.wheel[0].tar_speed = 0;
status->motor.wheel[1].tar_speed = 0;
```

更推荐直接调用：

```c
task_finish(status);
```

但如果为了响应更快，也必须同步切 `motion = STOP` 和清 `task_running`。

## BC 干扰段

BC 边有干扰 A4 纸，横向黑线会制造假路口。

AB 发车路线中，BC 边会经过两次：

```text
去程：C -> B
回程：B -> C
```

这两段都必须加“里程窗口屏蔽假路口”：

```text
在 BC 边中段一定距离范围内：
  忽略 LeftRoad / RightRoad / CrossRoad / TBRoad 等路口事件
  不推进 race_phase
  不清 cnt_seen

超过接近边尾的距离阈值后：
  才允许消费真实角点
```

建议先写成宏，方便实车调：

```c
#define Q2_BC_IGNORE_START_CM
#define Q2_BC_IGNORE_END_CM
#define Q2_CORNER_ENABLE_CM
```

## 掉头逻辑

到 B 点后执行 180 度掉头，不要普通 90 度转弯。

掉头建议：

```text
进入 UTURN_B:
  initial_angle = cur_angle
  tar_angle = Q2_UTURN_ANGLE
  motion = KEEP_ANGLE
  base_speed = 30

角度误差 < 12~15 度:
  进入 FIND_BC_RETURN

FIND_BC_RETURN:
  motion = FIND_LINE
  base_speed = 低速
  中间 6 路看到线后进入 SIDE_BC_RETURN
```

掉头时不需要完全停车，但必须提前降速，`base_speed` 先用 `30`。

## 角度宏

第二问不要复用 TASK1 的转角宏。

去程每个左转角度、回程每个右转角度、掉头角度都写成独立宏，方便实车逐点调：

```c
#define Q2_TURN_A_LEFT_ANGLE
#define Q2_TURN_D_LEFT_ANGLE
#define Q2_TURN_C_LEFT_ANGLE

#define Q2_UTURN_B_ANGLE

#define Q2_TURN_C_RIGHT_ANGLE
#define Q2_TURN_D_RIGHT_ANGLE

#define Q2_TURN_TO_FIND_TOLERANCE_DEG
```

右转角度的正负号按实车陀螺仪方向确认，不要想当然。

## 速度策略

先保守跑通，不追求极限速度。

建议宏：

```c
#define Q2_FLASH_SPEED
#define Q2_CRUISE_SPEED
#define Q2_TURN_SPEED
#define Q2_FIND_SPEED
#define Q2_UTURN_SPEED 30
#define Q2_FINAL_SLOW_SPEED
```

普通边可以用前段冲刺、后段巡航；BC 干扰段建议整体更慢或至少干扰窗口内更慢。

## 不要做

1. 不要这次实现 AD 发车。
2. 不要只靠时间推进阶段。
3. 不要在 BC 干扰窗口内消费路口。
4. 不要只设置 `stop_cmd=1` 就认为停车完成。
5. 不要让 TASK2 复用 TASK1 的左转角度宏。
6. 不要把去程左转和回程右转写成同一个 case 糊在一起。

