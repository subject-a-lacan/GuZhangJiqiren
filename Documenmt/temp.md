返回值被丢了
gimbal_send() 虽然改成返回 uint8_t：[Emm_v5.c (line 4)](D:/KEIL_CUBE_VS/Episode1/User/Motor/Emm_v5.c:4)，但 Emm_V5_Pos_Control() 仍然返回 void：[Emm_v5.c (line 28)](D:/KEIL_CUBE_VS/Episode1/User/Motor/Emm_v5.c:28)。
当前实际链路是：
gimbal_send() 返回 0/1
→ Emm_V5_Pos_Control() 丢弃结果
→ stepper_request_move() 拿不到结果

用 huart3.gState 猜发送成功不可靠
[uart_it.c (line 72)](D:/KEIL_CUBE_VS/Episode1/User/It/uart_it.c:72) 现在通过检查 BUSY_TX 设置 stepper.busy。
正常情况下，13 字节命令刚启动 DMA 时确实通常还是 BUSY_TX，所以大概率表现正常。但它不是可靠依据：
BUSY_TX 只说明此刻串口正在发字节，不证明刚才那条位置命令成功；
如果发送完成中断恰好在检查前执行，成功命令会被误判为失败；
如果 UART 正在发送别的命令，也可能把本次失败误判为成功；
stepper.busy 定义的是“电机运动中”，而不是“串口正在发送”。
最小正确方向是让 Emm_V5_Pos_Control() 把 gimbal_send() 的结果返回给 stepper_request_move()：DMA 启动成功才置 busy=1，到位帧再清零；不需要反查 gState。

中断内异常死锁仍存在
[Emm_v5.c (line 6)](D:/KEIL_CUBE_VS/Episode1/User/Motor/Emm_v5.c:6) 仍在 TIM5 中断路径里依赖 HAL_GetTick() 忙等。正常完成时 USART3 能抢占 TIM5，所以通常没事；但 DMA/UART 真正异常时，SysTick 无法抢占 TIM5，100ms 超时不会推进，仍可能永久卡住。