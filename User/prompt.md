/karpathy-guidelines

现在要修一个停车后电机死区抖动的问题。当前代码里 STOP 状态只是把 wheel[x].tar_speed 置 0，但 driver_wheel() 仍然会根据 tar_speed - cur_speed 计算 PID 输出，所以在编码器速度残留或死区附近可能继续输出 PWM，导致小车停车后抖动。

请按最小改动原则修改代码，不要重构无关逻辑。

目标：
新增一个 TASK 层的硬停止标志 stop_cmd，用于直接禁止轮子 PWM 输出。

语义必须严格区分：

1. stop_cmd
   - 电机层硬停止命令。
   - stop_cmd == 1 表示禁止 driver_wheel() 输出 PWM。
   - stop_cmd == 0 表示允许 driver_wheel() 正常根据 PID 输出 PWM。

2. stop_request
   - TASK 层“请求任务停止”的逻辑请求。
   - 本次修改不要依赖 stop_request 实现硬停止。
   - 不要在 update_status() 里根据 motion 去设置 stop_request。

具体修改要求：

1. 修改 User/Status/Defect.h
   在 TASK 结构体里新增：
   uint8_t stop_cmd;

2. 修改 User/Status/Defect.c

   init_task(TASK *task) 中：
   - 初始化 task->stop_cmd = 1;
   - 上电默认禁止电机 PWM 输出，防止误动。

   task_start(STATUS *status) 中：
   - 设置 status->task.stop_cmd = 0;
   - 任务开始后允许电机输出。
   - 注意：这里绝对不要把 stop_cmd 置 1。

   task_finish(STATUS *status) 中：
   - 设置 status->task.stop_cmd = 1;
   - 同时保持已有停止语义：
     task_running = 0;
     armed = 0;
     start_request = 0;
     stop_request = 0;
     motion = STOP;
     base_speed = 0;
     wheel[0].tar_speed = 0;
     wheel[1].tar_speed = 0;

   task_stop(STATUS *status) 中：
   - 设置 status->task.stop_cmd = 1;
   - 同时保持已有停止语义：
     task_running = 0;
     armed = 0;
     start_request = 0;
     stop_request = 0;
     motion = STOP;
     base_speed = 0;
     wheel[0].tar_speed = 0;
     wheel[1].tar_speed = 0;

3. 修改 User/Status/status.c

   在 update_status() 的运动状态分支中：
   - FIND_LINE / KEEP_ANGLE / MOTOR_TEST 这些会驱动车运动的状态，应确保 status->task.stop_cmd = 0;
   - STOP 状态应确保 status->task.stop_cmd = 1，并继续把 wheel[0].tar_speed 和 wheel[1].tar_speed 置 0。
   - 不要在这里设置 stop_request。
   - stop_request 不应该由 motion 自动生成。

4. 修改 User/Motor/wheel.c

   在 driver_wheel(WHEEL *wheel) 的最前面增加硬停止判断：

   if (status.task.stop_cmd) {
       wheel->trust = 0;
       根据 wheel->which，把对应 TIM8 通道 compare 写 0；
       return;
   }

   注意：
   - 不能只是“不更新 PWM”，必须主动把对应通道 compare 写 0。
   - 否则上一次 PWM 占空比可能残留。
   - 这个判断必须放在 compute_pid() 之前，避免 stop_cmd=1 时 PID 继续积分或输出。

5. 蓝牙/按钮急停语义

   如果后续蓝牙或按钮要做急停，应直接执行硬停止语义：
   status.task.stop_cmd = 1;
   status.task.task_running = 0;
   status.task.armed = 0;
   status.task.start_request = 0;
   status.task.stop_request = 0;
   status.state.motion = STOP;
   status.state.base_speed = 0;
   status.motor.wheel[0].tar_speed = 0;
   status.motor.wheel[1].tar_speed = 0;

   蓝牙/按钮急停不需要通过 stop_request 间接实现。

验收标准：

1. 上电后 init_task() 结束时 stop_cmd == 1，电机不会输出 PWM。
2. 短按启动任务并进入 task_start() 后 stop_cmd == 0，电机允许输出。
3. task_finish() 或 task_stop() 后 stop_cmd == 1，driver_wheel() 直接把 PWM compare 写 0。
4. update_status() 中 STOP 不会设置 stop_request，只会设置 stop_cmd。
5. driver_wheel() 在 stop_cmd == 1 时不会调用 compute_pid()。
6. 不修改无关 TASK 状态机逻辑，不删除 stop_request 字段。
