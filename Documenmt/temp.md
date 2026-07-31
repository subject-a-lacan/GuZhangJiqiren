当前二分查表有个危险特性：

如果加入积分后 requested_accel_mm_s2 变成 NaN 或正无穷，查表同样会直接返回 -2500。

而且浮点数中：

0×NaN=NaN

所以即使 Ki=0，只要积分变量或 dt_s已经异常，也不能保证输出恢复正常。

立即在查表前增加：

if (!isfinite(ball->requested_accel_mm_s2)) {
    ball->integral_accel_mm_s2 = 0.0f;
    return;   /* 不发送新的异常目标，保持上一次命令 */
}

并且注意：如果运行过程中把 Ki改成0，已有积分不会自动消失，只会停止继续积累。必须显式清零：

ball->integral_accel_mm_s2 = 0.0f;

至少应在以下位置清零：

ball_control_init()；
ball_control_disable()；
Task4每次重新启动；
新的 session_seq开始时。
你现在的VOFA输出根本不足以判断

把没有意义的恒定目标值换掉，至少输出：

task4_dbg_buf[0] = ball->estimator.position_mm;
task4_dbg_buf[1] = ball->estimator.velocity_mm_s;
task4_dbg_buf[2] = ball->position_error_mm;
task4_dbg_buf[3] = ball->requested_accel_mm_s2;
task4_dbg_buf[4] = (float)ball->relative_target_pulse;

如果有积分变量，再加：

task4_dbg_buf[5] = ball->integral_accel_mm_s2;

判断规则非常明确：

requested_accel ≥ 465.3且脉冲为-2500：正常饱和，主要是P+D太大；
requested_accel = NaN/Inf且脉冲为-2500：积分或时间间隔计算出错；
requested_accel有限且小于465.3，却仍为-2500：执行一次 CCS 的 Clean Project + Rebuild，检查修改 status.h 后是否存在旧目标文件。