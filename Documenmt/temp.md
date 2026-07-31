我在运行任务三的同时打印什么值能有助于我调参
思考了 3m 26s

最有用的是下面这 12 路。不要只打印位置、速度和总输出，否则很难判断究竟该改 K
p
	​

、K
d
	​

 还是 K
i
	​

。

通道	打印值	用途
0	目标位置	看 Task3 何时从 +50 跳到 −50
1	视觉原始位置	判断视觉噪声
2	滤波后位置	判断位置滤波延迟
3	估计速度	判断速度、刹车时机及速度噪声
4	位置误差	判断响应快慢
5	P项	判断 K
p
	​

 是否足够
6	D项	判断 K
d
	​

 是否刹得住
7	I项	判断积分残留、饱和
8	最终期望加速度	实际送入查表前的总输出
9	相对目标脉冲	判断执行器是否饱和
10	积分确认计时	判断何时开始积分
11	hold_active	判断是否真正进入停止死区

其中：

P=K
p
	​

e
D=−K
d
	​

v
e：位置误差，单位 mm；
v：小球速度，单位 mm/s；
P、D、I以及最终输出的单位都是 mm/s²。

在 driver_task3() 最后，按照你现有的缓冲区发送方式写入：

{
  BALL_CONTROL *ball = &status->control.ball;
  static uint32_t last_dbg_seq = 0;

  /*
   * 每个经过控制器处理的新视觉帧只记录一次。
   * 不要按5 ms重复记录同一帧。
   */
  if (ball->estimator.control_ready &&
      ball->estimator.consumed_sample_seq != last_dbg_seq) {

    float p_term =
        ball->kp * ball->position_error_mm;

    float d_term =
        -ball->kd * ball->estimator.velocity_mm_s;

    last_dbg_seq = ball->estimator.consumed_sample_seq;

    task3_dbg_buf[0]  = ball->request.target_mm;
    task3_dbg_buf[1]  =
        (float)status->sensor.vision.ball.x10 * 0.1f;
    task3_dbg_buf[2]  = ball->estimator.position_mm;
    task3_dbg_buf[3]  = ball->estimator.velocity_mm_s;
    task3_dbg_buf[4]  = ball->position_error_mm;
    task3_dbg_buf[5]  = p_term;
    task3_dbg_buf[6]  = d_term;
    task3_dbg_buf[7]  = ball->integral_accel_mm_s2;
    task3_dbg_buf[8]  = ball->requested_accel_mm_s2;
    task3_dbg_buf[9]  = (float)ball->relative_target_pulse;
    task3_dbg_buf[10] = ball->stuck_timer_s;
    task3_dbg_buf[11] = (float)ball->hold_active;

    task3_dbg_ready = 1;
  }
}

然后在主循环的刷新函数中：

UART_send_justfloat(&huart1, 12,
                    local[0],  local[1],
                    local[2],  local[3],
                    local[4],  local[5],
                    local[6],  local[7],
                    local[8],  local[9],
                    local[10], local[11]);

注意：driver_task3() 在定时器中断链路里，不能直接调用 UART_send_justfloat()，仍然要像你现在的 Task4 一样，中断只写缓冲区，主循环负责发送。

调参时重点这样看：

运动太慢：误差很大但P项和脉冲较小，增加 K
p
	​

。
脉冲已经到 -2500 或 6908：输出已饱和，再加 K
p
	​

 没用。
过中心后刹不住：D项太小，可增加 K
d
	​

。
误差已经反号，I项还保持旧方向：积分残留正在造成过冲，应更快释放积分。
接近目标时，若“最终加速度”和速度同号，控制器正在给振荡补充能量，振幅就可能越来越大。
已经进入允许范围，但通道11始终为0：死区速度门槛或确认时间太苛刻。
原始位置变化明显，但滤波位置和速度反应很慢：是估计器延迟，不是PID太小。

还有一个非常关键的现象：你当前 Task3 只要位置进入 50±5 mm，哪怕速度还很大，也会立刻把目标从 +50 改成 -50。因此必须打印通道0；否则看到曲线突然反向时，很容易误以为PID自己发散了。若你想让球先在 +50 mm 真正停稳，切换条件还应加入：

fabsf(ball->estimator.velocity_mm_s) <= 12.0f

或者直接等待：

ball->hold_active == 1