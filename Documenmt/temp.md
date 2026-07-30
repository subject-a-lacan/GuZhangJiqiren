找到了。看 main.c 的主循环——当前只调了 maixcam_poll() 和 task7_flush()，完全没有调用 consume_uart_gyr()。

Read d:\KEIL_CUBE_VS\Episode1\Core\Src\main.c (lines 157-168)
根因：consume_uart_gyr() 从未被调用。

旧版（中断逐字节，正常）

HAL_UART_RxCpltCallback (每字节)
  → uart_gyr_rx_feed()        ← 逐字节解析 5 字节帧
  → gyro->gyro_z = ...         ← 直接写入状态树
  → gyro->yaw = ...
TASK5 读 status.sensor.uart_gyr.gyro_z/yaw → 有值。

新版（DMA，数据始终为 0）

HAL_UART_RxCpltCallback (10 字节满)
  → 解析 10 字节缓冲区
  → 找到 0x5A 0xAA 帧 + 0x5A 0xBB 帧
  → 写入 uart_gyr_snapshot[index]
  → uart_gyr_snapshot_ready = 1
  → ❌ 到此为止！gyro_z / yaw 从未更新
consume_uart_gyr() 里有 status.sensor.uart_gyr.gyro_z = ... 和 status.sensor.uart_gyr.yaw = ...，但主循环里没有调用它。main.c 的 while(1) 只有 maixcam_poll + task7_flush，没有 consume_uart_gyr()。中断里攒了快照，主循环从来不消费，gyro_z / yaw 永远是 0。