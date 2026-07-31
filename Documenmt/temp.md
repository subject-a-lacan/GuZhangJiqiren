task6 控制小球停 0 点
调用 ball_control_request 之前，有一个关键的缺失环节还没接上：

MaixCAM 的 VL 帧解析器（maixcam.c:310-313）仍然把数据写到旧的 maixcam_loc，没有写到 status.sensor.vision.ball：


// maixcam.c:310 — 旧代码，写的是 maixcam_loc
maixcam_loc.x10          = x10;
maixcam_loc.timestamp_ms = ts;
maixcam_loc.valid        = 1;
maixcam_loc_new          = 1;
而 ball_control_service 读的是：


// ball_control.c:149
float measured_position_mm = (float)vision->x10 * 0.1f;
status.sensor.vision.ball 从未被写入，所以估计器永远收不到新帧，PD 永远不会输出，步进电机永远不会动。

需要做的事
在 parse_vl_frame 里加一句：把 x10、timestamp_ms 同步写到 status.sensor.vision.ball，递增 sample_seq
task6 本身只要：
启动时调一次 ball_control_request(&status, 0.0f, 0.0f)
设 motion = STOP, base_speed = 0
之后什么都不用做