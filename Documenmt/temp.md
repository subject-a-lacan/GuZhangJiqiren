. 启动前必须复位 PID 状态

driver_wheel() 停车时直接返回，但没有清除：

integral；
last_error；
derivative；
is_first。

而 compute_pid() 会保存这些历史状态。

所以即使目标速度从零平滑增加，旧任务残留的积分和误差仍可能在重新启动时产生突变。

在每一问启动和停止时必须复位