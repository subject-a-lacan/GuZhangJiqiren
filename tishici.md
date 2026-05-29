 
 先阅读，不准改代码：任务2的核心计算公式：base_speed = K_angle * pend_angle
          + K_gyro  * pend_angle_speed
          + K_pos   * car_position
          + K_vel   * car_speed;首先 在工作区主循环里按照pitch,roll,pitch角速度,roll角速度\r\n的格式打印这四个量，然后封装一个叫balance的PID结构体 这样一来它的error就是roll 它的微分项就是角速度 这样一来它的输出就是的base_speed的前两项了 再写一个叫mileage 的结构体  他的error就是负的编码器累加   微分x项就是两轮当前速度之和除以2并取负 所以这个结构体的PID输出就是base_speed的后两项  补全balance和mileage相关的初始化代码  请你再增加一个MOTION叫做BALANCE 然后写好keep_balance函数：里面就是先读取target为0 diff_balance为0减去roll ....总之模仿keep_angle写就行 但是注意现在只需要加公式里的前两项，也就是暂时不控制位置 然后· 更新keep_balance后也需要在update_status里更新对应的代码  然后 改写driver_task2：一直保持keep_balance状态  最后 在update_status里累加 mileage 先评估我说的对不对 我的方案行不行 然后不准改代码 告诉我你打算怎么改