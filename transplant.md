# TI Episode2 架构优化迁回 STM32 Episode1

> 对比基线：当前 `D:\KEIL_CUBE_VS\Episode1` 与 `C:\Users\19355\workspace_ccstheia\Episode2` 源码。
>
> 目标题目：`Documenmt/timu.md` 中的 H 题“车载平衡滚球运动控制系统”。
>
> 本文只记录值得迁移的完整架构变化和软件优化，不讨论由 MCU、编码器、PWM、GPIO、ADC 外设资源不同造成的普通适配。

## 1. 总结

| TI 中的变化 | 当前 STM32 状态 | 迁移决定 | 说明 |
|---|---|---|---|
| MaixCAM2 完整 UART 协议层 | 没有等价接收和命令状态机 | **迁移** | 为摄像头球位闭环提供稳定数据入口 |
| UART 单轴陀螺仪驱动 | 已有 I2C DMA GY901 | **作为第二路陀螺仪新增，禁止覆盖** | 两个陀螺仪可分别测车体航向和摆杆运动 |
| 灰度由部分通道扩展为完整 8 路，并切换 12 位 ADC 数据链路 | ADC3 仍为 8 位，灰度数组/阈值仍为 `uint8_t` | **迁移 12 位数据链路和 8 路采集** | 不迁移 TI 的路口/左支路专用逻辑 |
| 差速 `diff` 保持浮点并直接进入轮速目标 | `GW_ANALOGUE.diff` 已是 `float`，但 `follow_line()` 仍强转 `int16_t` | **删除差速输出处的整数截断** | 保留小数控制量，再在最终 PWM/轮速执行层按需要取整 |
| `Yuntai/Emm_v5/datou` 执行器层 | 已有 `lq_step` 基础驱动 | **按最终电机协议择需合并** | 提取使能、归零、停止和位置控制能力，不照搬双轴云台任务 |
| `server_button` 按键业务重构 | 仍是旧任务/位姿切换语义 | **迁移业务变化** | 任务选择、校准、启动/停止和蜂鸣反馈更适合现场操作 |
| 三个 LED 直接显示任务号二进制位 | 当前为按任务逐项 `switch` 映射 | **迁移** | 简单、无重复分支，支持 1～7 号测试档 |
| 设备自检和 JustFloat 遥测 | 没有统一自检；已有日志和在线调参 | **选择性迁移** | 作为调试工具，不进入正式控制架构 |
| TI `Defect.c` 中的所有 task | 都是阶段性测试代码 | **不迁移、不重写 STM32 task** | 新赛题任务应在后续按 H 题单独设计 |

迁移重点不是把 TI 工程覆盖到 STM32，而是把缺少的模块以独立边界接入现有 STM32 状态树。STM32 已有的 I2C 陀螺仪、轮速前馈、在线 PID 调参、`keep_angle()`、周期任务宏和现有 task 均保留。

## 2. MaixCAM2 通信层

TI 新增了 `User/Sensor/maixcam.c/.h`，这是完整的新模块，当前 STM32 没有等价实现。

### 2.1 TI 已实现的能力

- 128 字节 UART 环形接收缓冲，UART ISR 只调用 `maixcam_rx_feed(byte)` 投递数据。
- 主循环逐字节解析 `VD,...#` 检测帧和命令应答帧。
- 检测结果保存 `found`、`x10`、`y10`、`distance10`、更新时间和有效标志。
- `maixcam_cmd_T()`、`maixcam_cmd_D()`、`maixcam_cmd_M()` 和通用数值命令。
- 命令具有等待、成功、失败、超时状态，80 ms 超时并最多发送 3 次。
- 原始帧镜像和新数据标志，便于先测试协议再接控制器。

源码中的实际周期接口是：

```c
void maixcam_poll(uint32_t time_ms);
```

### 2.2 STM32 接入方式

保留 TI 模块的解析与状态机结构，仅替换发送和接收入口：

```c
void maixcam_init(void);
void maixcam_rx_feed(uint8_t byte);
void maixcam_poll(uint32_t time_ms);
void maixcam_cmd_T(uint8_t on_off);
void maixcam_cmd_D(uint8_t on_off);
void maixcam_cmd_M(uint8_t mode);
void maixcam_cmd_send_val(char type, float value);
```

- STM32 UART RX 中断、Receive-to-Idle 或 DMA 回调只负责调用 `maixcam_rx_feed()`。
- 主循环每 8 ms 调用一次 `maixcam_poll()`，解析不能放进 ISR。
- 球位控制器只读取已校验、未过期的检测结果；超时或丢失时停止更新摆杆目标。
- UART 球位数据只服务控制。题目要求的实时图传、录像和回放仍需独立的视频链路完成。

## 3. 第二路 UART 陀螺仪

### 3.1 为什么必须新增而不是替换

H 题同时包含小车循线和滚球摆杆控制，可能需要两个独立惯性测量位置：

1. 车体陀螺仪：提供小车航向或车体扰动信息，继续使用 STM32 当前的 I2C DMA GY901。
2. 摆杆/机构陀螺仪：提供摆杆角速度或独立姿态信息，新增 TI 已验证的 UART 单轴陀螺仪。

因此不得用 TI 的 `gy901.c/.h` 覆盖 STM32 的同名文件，也不能让两个设备共享同一个全局 `GYR` 或同名中断处理状态。

### 3.2 TI UART 驱动已有能力

- 230400 bps 主动上报 5 字节帧。
- `0x5A 0xAA` 解析 Z 轴角速度，`0x5A 0xBB` 解析 Z 轴角度。
- 帧头同步和校验和检查。
- `sendCaliYawCommand()` 完成航向归零。
- `performCaliBias()` 完成零偏校准。
- `GyroZ()`、`Yaw()` 和 `update_gyr()` 提供读取入口。

### 3.3 STM32 中的独立模块边界

建议新增独立文件，例如：

```text
User/Sensor/uart_gyro.c
User/Sensor/uart_gyro.h
```

接口使用独立命名，避免与现有 I2C GY901 冲突：

```c
typedef struct {
    float yaw;
    float gyro_z;
    uint32_t update_tick;
    uint8_t valid;
} UART_GYRO;

void uart_gyro_init(UART_GYRO *gyro);
void uart_gyro_rx_feed(UART_GYRO *gyro, uint8_t byte, uint32_t now_ms);
void uart_gyro_calibrate_yaw(void);
void uart_gyro_start_bias_calibration(void);
```

接入约束：

- 为 UART 陀螺仪分配独立 UART 实例，接收回调按 UART 句柄分发。
- 解析状态属于 UART 陀螺仪模块，不能复用现有 I2C GY901 的 DMA 缓冲和回调。
- 状态树中分别保存 `car_gyro` 与 `arm_gyro`，上层通过语义名称选用，不能依赖“gyro1/gyro2”猜测安装位置。
- 两路角度的零点、正方向、量程和更新时间分别标定。

TI 的 `performCaliBias()` 内部阻塞约 21 秒，不能原样放入 STM32 的按键业务或控制循环。迁移时应改成非阻塞校准状态：发送解锁和校准命令后记录截止时间，主循环到时再发送保存命令；校准期间禁止启动正式任务。

## 4. 灰度传感器：8 路与 12 位 ADC

本次不迁移 TI 的左支路保护、正方形路口识别、路口状态机或针对测试场地的阈值。灰度部分只处理完整 8 路和 12 位 ADC 数据链路。

### 4.1 需要迁移

- 8 路原始 ADC 数据全部参与黑白标定和归一化。
- 8 路通道顺序与实际从左到右的物理位置建立固定映射。
- 8 路黑线强度使用完整权重求重心：

```text
diff = sum(black[i] * weight[i]) / sum(black[i])
```

- `sum(black[i])` 太小时明确判为丢线，避免除零和无意义重心。
- 保留原始 8 位二值结果，供以后 H 题启停线识别使用，但本次不设计启停线 task。

### 4.2 12 位改动是必需的

STM32 当前 `ADC3` 配置为 `ADC_RESOLUTION_8B`，而 TI 版本已经将灰度数据链路改为 12 位。若只修改 CubeMX 分辨率、不改结构体，`HAL_ADC_GetValue()` 的 `0～4095` 结果会写入 `uint8_t` 并截断。

必须同步修改：

- `GW_ANALOGUE.channel[8]` 改为 `uint16_t`。
- `correction_data_w/b[8]` 改为 `uint16_t`。
- `digital_high_threshold/low_threshold[8]` 改为 `uint16_t`。
- `normalize_gray_data()` 的 `max/min/now` 参数改为 `uint16_t`。
- CubeMX 工程和生成代码均改为 `ADC_RESOLUTION_12B`。
- 重新执行八路白/黑标定；当前 8 位默认阈值不能直接作为最终 12 位阈值，最多乘 16 作为临时启动值。

ADC3 DMA 已使用 half-word 对齐，12 位结果可以直接保存；`digital_8bit` 仍保持 `uint8_t`，因为它只是八路二值位图。

### 4.3 ADC 调度与串口陀螺仪实时性

八路模拟开关切换、稳定等待和 `HAL_ADC_PollForConversion()` 属于阻塞轮询，不能放在 1 ms 定时器 ISR 或串口高优先级中断中。应仿照 TI 工程：

- 定时器 ISR 只更新时间、置 8 ms 标志并处理必要的短控制逻辑。
- 主循环每 8 ms 执行一次八路 ADC 轮询，完成采样、标定、二值化和重心计算。
- UART 陀螺仪使用普通 RX 中断逐字节接收，不使用 DMA；ISR 只取字节、推进 5 字节解析状态机并重新使能接收。
- 控制逻辑读取上一份完整灰度快照，避免主循环更新数组时与中断读取产生半更新数据。

### 4.2 不迁移

- `gw_left_guard_arm()`、`gw_left_guard_reset()` 等左支路保护接口。
- TI 为正方形左转测试加入的路口判断和状态缓存。
- TI 的 ADC 数值阈值、采样 API 和硬件通道选择代码。
- 任何因为 MSPM0 ADC 或 GPIO 资源产生的实现差异。

迁移后只要求 STM32 的 `GW_ANALOGUE` 能稳定输出完整八路归一化数据、八位二值结果和浮点重心 `diff`，不改变现有 task 行为。

## 5. 差速 `diff` 的浮点链路

TI 当前链路是：

```text
8 路归一化重心(float diff)
 -> follow_line PID(float)
 -> base_speed +/- diff(float tar_speed)
```

STM32 的 `GW_ANALOGUE.diff` 本身已经是 `float`，PID 也返回 `float`，但当前 `follow_line()` 在写入左右轮目标时使用了 `(int16_t)diff`。这会在进入轮速 PI 前丢掉小数部分，抵消 TI 这次改动的收益。

迁移时应：

- 保持 `GW_ANALOGUE.diff` 为 `float`。
- 保持循线 PID 输出为 `float`。
- 将左右轮 `tar_speed` 的计算改为浮点加减，不在 `follow_line()` 或 `keep_angle()` 中提前强转 `int16_t`。
- 仅在最终硬件 PWM、方向或整数编码器目标接口处按现有执行层要求取整/限幅。
- 重新标定循线 PID，因为浮点差速会改变小误差区的控制灵敏度。

这不是把轮子结构体全面改成浮点的要求；只需确保“传感器重心 -> PID -> 差速目标”这一段不发生整数截断。

## 6. 摆杆执行器架构

TI 新增的 `User/Yuntai/Emm_v5.c/.h`、`datou.c/.h` 和 `yuntai.c/.h` 构成新的执行器抽象：

- `Emm_v5`：串口协议层，提供使能、速度、位置、立即停止、同步运动、设置零点和回零。
- `datou`：单个电机对象，保存地址、模式、速度、加速度、方向和目标角度。
- `yuntai`：组合两个电机，根据二维目标进行双轴运动。

STM32 已有 `lq_step.c/.h`，所以不应直接并存两套重复的上层控制。按最终摆杆电机选择：

- 驱动器仍使用现有协议：扩展 `lq_step` 的使能、停止、归零和状态反馈，不迁移 `Emm_v5`。
- 驱动器使用 Emm_V5 协议：迁移 `Emm_v5 + datou`，并适配 HAL UART。
- H 题摆杆为单轴机构时，不迁移双轴 `yuntai` 的空间指向算法。

正式球位控制与底层协议之间保留统一接口：

```c
void balance_actuator_enable(uint8_t enable);
void balance_actuator_set_zero(void);
void balance_actuator_set_target_angle(float angle_deg);
void balance_actuator_stop(void);
```

这只是执行器边界。TI 代码没有完成 H 题的球位置闭环，后续仍需由 MaixCAM 球位误差计算目标摆角。

## 7. `server_button` / `server_key` 业务变化

TI 文件中的函数实际名为 `server_button()`；本文所说的 `server_key` 指同一层按键业务处理，不包括底层 GPIO 读法。

需要迁移的业务变化：

### KEY1 短按：任务选择

- 仅在 `task_running == 0` 时允许切换。
- 任务号按 `1 -> 2 -> ... -> 7 -> 1` 循环。
- 只设置 `requested_task_id` 和 `task_select_request`，由 `update_task()` 统一应用。
- 切换后蜂鸣器提示约 400 ms。

TI 的 1～7 号 task 当前全部视为测试档。迁移按键选择能力和任务号范围，不迁移这些 task 的内部实现。

### KEY1 长按：陀螺仪零偏校准

- 仅在任务空闲时允许触发第二路 UART 陀螺仪校准。
- 蜂鸣器给出较长提示。
- 必须调用非阻塞 `uart_gyro_start_bias_calibration()`，不能直接搬运 TI 中阻塞 21 秒的 `performCaliBias()`。

### KEY2 短按：启动/停止复用

- 空闲且未 armed：设置 `start_request = 1`。
- 任务运行中：设置 `stop_request = 1`。
- 两种操作均给出蜂鸣反馈。
- 按键层只发请求，不直接启动任务、写 PWM 或清控制器。

### KEY2 长按：灰度校准

- 仅在 `task_running == 0` 时允许执行灰度校准。
- 运行过程中忽略长按，防止标定覆盖正常采样数据。

不迁移 TI 的 `DL_GPIO_readPins()`；STM32 继续使用当前 HAL GPIO 和已有消抖/长按状态机。

## 8. 任务 LED 二进制编码

当前 STM32 的 `update_task_led()` 使用较长的 `switch`，并混入了起始位姿显示。TI 将三个 LED 直接映射到任务号的三个二进制位：

```c
void update_task_led(STATUS *status) {
    status->device.led_on_board.on = (status->task.task_id & 0x04) ? 1 : 0;
    status->device.led1.on         = (status->task.task_id & 0x02) ? 1 : 0;
    status->device.led2.on         = (status->task.task_id & 0x01) ? 1 : 0;
}
```

该变化应迁移：

- 任务 1～7 分别显示 `001`～`111`。
- 新增测试档不再需要扩展 `switch`。
- LED 只表达任务号，不再同时表达起始位姿；如果后续仍需要位姿提示，应交给显示屏或独立状态页面。
- 保留 STM32 当前 `driver_led()` 和 GPIO 实现，只替换 `update_task_led()` 的业务映射。

## 9. 调试工具的选择性迁移

### 8.1 设备自检

TI 的 `device_test.c/.h` 对 LED、蜂鸣器、按键、八路灰度、多路 UART 和电机做了统一测试。可以迁移测试入口和组织方式，但每项底层调用应复用 STM32 已有驱动。

自检函数只能由专用测试档调用，不应进入正式任务的控制周期。

### 8.2 JustFloat

TI 的 `UART_send_justfloat()` 适合用 VOFA 同时观察球位、两个陀螺仪、摆杆目标角和控制输出。STM32 已有格式化日志和在线 PID 调参，因此 JustFloat 只作为补充遥测：

- 固定通道含义和发送周期。
- 限制发送频率，避免影响 8 ms 控制循环。
- 正式运行时关闭高频文本日志。

## 10. 明确不迁移的内容

- TI `Defect.c` 中 TASK1～TASK7 的全部内部流程。它们是开发和外设测试代码，不是本次 H 题任务实现。
- TI 的正方形左转、路口停车、MaixCAM 测试 task 和灰度输出 task。
- 编码器实现、PWM 通道、轮号、轮径、GPIO 端口和其他由 MCU 硬件资源造成的差异。
- TI 的 ADC 阈值和 DriverLib 外设调用。
- TI UART 陀螺仪对现有 STM32 I2C GY901 的覆盖式替换。
- STM32 已经存在的轮速前馈、在线 PID 调参、`keep_angle()`、周期任务宏和日志系统。
- 与单轴滚球摆杆无关的双轴云台空间指向逻辑。

## 11. 推荐迁移顺序

1. **新增 UART 陀螺仪模块**：先做到双陀螺仪可同时更新、互不覆盖，并完成非阻塞校准。
2. **迁移 MaixCAM2 协议层**：独立验证命令应答、检测帧、超时和数据新鲜度。
3. **将 ADC3 改为 12 位并把八路轮询移到主循环**：完成 `uint16_t` 数据链路、八路标定和浮点重心。
4. **移除差速输出的整数截断**：确认小误差经过 PID 后仍能作用到目标速度。
5. **迁移 `server_button/server_key` 业务和 LED 编码**：保证测试档选择、启停和校准操作可靠。
6. **确定摆杆电机后整理执行器层**：在 `lq_step` 与 `Emm_v5/datou` 之间择一，建立统一摆杆接口。
7. **补充自检和 JustFloat**：为下一阶段实现 H 题正式 task 提供调试条件。

## 11. 验收标准

- 原有 I2C GY901 功能和接口保持不变，新增 UART 陀螺仪可独立接收、校准和报告数据。
- 两路陀螺仪在状态树中有明确安装语义、独立有效标志和更新时间。
- MaixCAM ISR 不解析协议，主循环能稳定得到带新鲜度的球位数据。
- ADC3 使用 12 位配置，灰度原始值、校准值和阈值使用 `uint16_t`，没有 8 位截断。
- 灰度模块输出 8 路归一化值、8 位二值数据和浮点重心 `diff`，没有引入 TI 路口逻辑。
- ADC 轮询位于主循环，UART 陀螺仪采用逐字节 RX 中断且不使用 DMA。
- 差速 `diff` 在进入轮速执行层前保持浮点，不被提前转成 `int16_t`。
- KEY1/KEY2 完成任务选择、非阻塞陀螺仪校准、启动/停止和空闲灰度校准。
- 任务 1～7 的 LED 显示分别为 `001`～`111`。
- STM32 现有 task 内部行为未因本轮架构迁移而改变。
- 文档中不再把编码器等硬件资源差异列为需要迁移的优化。

## 12. 当前 CubeMX 配置与运行时代码状态

CubeMX 配置已确认：ADC3 已为 12 位、右对齐、软件触发，DMA 为 half-word；USART2 已为 230400、8N1、开启 RX 中断且未配置 DMA；UART4 仍保留 DMA 供 MaixCAM 使用。

但仅修改 CubeMX 还不够，运行时代码必须同步完成：

- 将灰度 ADC 轮询从 `TIM5 -> update_status()` 的中断调用链移到主循环，避免 `HAL_ADC_PollForConversion()` 阻塞 UART2 陀螺仪 RX 中断。
- UART2 启动 `HAL_UART_Receive_IT(..., 1)`，在 `HAL_UART_RxCpltCallback()` 中逐字节解析并重新挂接下一字节；错误回调中恢复接收。
- 将灰度原始数据、白黑校准值和阈值从 `uint8_t` 改为 `uint16_t`，避免 12 位 ADC 结果截断。
- 保持 `digital_8bit` 为位图，保持差速 `diff` 从重心到 PID、再到轮速目标的浮点链路。

## 13. 控制周期统一为 5 ms

系统硬件中断仍为 1 ms，只将控制更新门限从 8 ms 改为 5 ms：

```c
if (status.state.time % 5 == 0) {
    update_status(&status);
}
```

所有控制周期相关的 `T` 必须同步改为 `5`，包括：

- `STATUS.state.T` 初始化值。
- 所有 `init_pid(..., T, ...)` 的 PID 采样周期参数。
- 依赖 `status.state.T` 的积分、微分、周期计数和调试发送周期换算。

不要修改 1 ms 系统时基，不要把 5 ms 误改成硬件定时器周期；只改变控制更新频率和控制器采样周期。
