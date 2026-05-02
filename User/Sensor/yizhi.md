# gw_analogue 融合移植提示词

本文档是给后续 AI agent 用的移植提示词。目标是参考：

```text
D:\KEIL_CUBE_VS\diansaishenche\User\Sensor\gw_analogue.c
D:\KEIL_CUBE_VS\diansaishenche\User\Sensor\gw_anagloge.h
```

把当前工程里分散的：

```text
User/Sensor/gw_analogue.c/h
User/Sensor/gw_find_line.c/h 的路口状态设计思路
User/Status/road.c/h 的路口判断逻辑
```

逐步融合到当前工程的：

```text
User/Sensor/gw_analogue.c
User/Sensor/gw_anagloge.h
```

最终目标不是无脑复制学长文件，而是让模拟灰度 `GW_ANALOGUE` 自己产出：

```text
原始 ADC 数据
二值化 digital_8bit
循迹 diff
路口观测结果 cross
```

然后由 TASK 状态机读取这些观测结果，决定是否计数、转弯、停车。

特别说明：

```text
最终目标是不再保留旧 road.c 和 gw_find_line.c 的路口判断链路。
但是 AI agent 不要删除这些文件，也不要帮用户移出工程。
旧文件由用户之后手动删除或从工程构建中移除。
```

---

## 0. 绝对禁止

1. 不要删除 `radar.c`、`abslute_angle_sensor.c`、`ccd.c` 等暂时没用的代码。它们是可能以后有用的僵尸代码，不是本次斩杀对象。
2. 不要整文件覆盖当前 `gw_analogue.c/h`。学长文件的硬件宏、ADC、串口和结构体都和当前工程不兼容。
3. 不要在 `gw_analogue.c` 里直接控制小车运动。传感器层不能直接改 `base_speed`、`wheel.tar_speed`、`motion`。
4. 不要让传感器层决定任务有效路口计数。有效路口计数属于 `status.task.cross_cnt`，必须由 `task_xxx_update()` 在具体 `race_phase` 中确认后更新。
5. 不要为了兼容旧 `road.c` / `gw_find_line.c` 而改学长融合方案的整体结构。最终路口判断应归入 `gw_analogue` 模块，旧文件由用户手动删除。
6. 绝对不要修正 `Road` 左右编码。当前实车路口判断已经完美运行，这里存在“两个 BUG 负负得正”的实际效果。移植时必须保持学长代码和当前工程一致的 `LeftRoad/RightRoad` 枚举值与 `road_new_from_bit()` 映射，不准改。

---

## 1. 学长待移植文件结构

学长目录下只有两个文件：

```text
D:\KEIL_CUBE_VS\diansaishenche\User\Sensor\gw_anagloge.h
D:\KEIL_CUBE_VS\diansaishenche\User\Sensor\gw_analogue.c
```

### 1.1 学长 gw_anagloge.h 的结构

它包含三部分：

```c
typedef enum Road { ... } Road;

typedef struct Cross {
  uint8_t data_buf;
  uint8_t integral;
  uint8_t maybe;
  uint8_t cross_cnt;
  Road cross;
  uint8_t integral_times;

  uint8_t CrossRoad_cnt;
  uint8_t TBRoad_cnt;
  uint8_t TLRoad_cnt;
  uint8_t TRRoadd_cnt;
  uint8_t LeftRoad_cnt;
  uint8_t RightRoad_cnt;
  uint8_t Straight_cnt;
  uint8_t UnknowRoad_cnt;
} Cross;

typedef struct GW_ANALOGUE {
  uint8_t channel[8];
  uint8_t sta;
  uint8_t correction_data_w[8];
  uint8_t correction_data_b[8];
  uint8_t digital_8bit;
  uint8_t digital_high_threshold[8];
  uint8_t digital_low_threshold[8];
  float diff;

  Cross cross;
} GW_ANALOGUE;
```

这说明学长把 `Road` 枚举和路口缓存 `Cross` 直接塞进了模拟灰度头文件。

### 1.2 学长 gw_analogue.c 的结构

从上到下大致是：

```text
1. 模拟灰度显示
   gw_analogue_gray_show()

2. 路口缓存初始化
   init_road_determine(Cross *cross)
   init_road_cnt(Cross *cross)

3. 模拟灰度初始化
   init_gw_analogue()
   同时初始化 channel/threshold/diff/cross

4. 硬件通道选择和 ADC 读取
   select_channel()
   get_gw_raw_data()

5. 灰度校准
   correct_gw_analogue()

6. 二值化
   get_gw_analoge_digital_data()

7. 模拟循迹偏差
   normalize_gray_data()
   normalize_gray_weight()
   get_gw_analogue_analogue_diff()

8. 路口判断
   road_new_from_bit()
   road_decision()
   serve_road()
   get_road_type()

9. 总驱动入口
   driver_gw_analogue()
```

其中最值得借鉴的是最后这个总入口：

```c
void driver_gw_analogue(GW_ANALOGUE *gw_analogue) {
  get_gw_raw_data(gw_analogue);
  get_gw_analoge_digital_data(gw_analogue);
  get_gw_analogue_analogue_diff(gw_analogue);
  get_road_type(&gw_analogue->cross, gw_analogue->digital_8bit);
}
```

目标是让当前工程也具备类似调用链。

---

## 2. 当前工程结构

当前工程里相关文件是：

```text
User/Sensor/gw_anagloge.h
User/Sensor/gw_analogue.c
User/Sensor/gw_find_line.h
User/Sensor/gw_find_line.c
User/Status/road.h
User/Status/road.c
User/Status/status.h
User/Status/status.c
User/Status/Defect.c
```

### 2.1 当前 gw_anagloge.h

当前 `GW_ANALOGUE` 只有模拟灰度本体字段：

```c
typedef struct GW_ANALOGUE {
  uint8_t channel[8];
  uint8_t sta;
  uint8_t correction_data_w[8];
  uint8_t correction_data_b[8];
  uint8_t digital_8bit;
  uint8_t digital_high_threshold[8];
  uint8_t digital_low_threshold[8];
  float diff;
} GW_ANALOGUE;
```

当前没有：

```text
Road enum
Cross/RoadDetermine
driver_gw_analogue()
```

### 2.2 当前 gw_analogue.c

当前 `gw_analogue.c` 只负责模拟灰度：

```text
gw_analogue_gray_show()
init_gw_analogue()
select_channel()
get_gw_raw_data()
correct_gw_analogue()
get_gw_analoge_digital_data()
normalize_gray_data()
normalize_gray_weight()
get_gw_analogue_analogue_diff()
```

它没有路口识别函数。

### 2.3 当前 road.h/road.c

当前 `road.h` 定义：

```c
typedef enum Road { ... } Road;

typedef struct RoadDetermine {
  uint8_t data_buf;
  uint8_t integral;
  uint8_t maybe;
  uint8_t cross_cnt;
  Road cross;
  uint8_t integral_times;
} RoadDetermine;
```

当前 `road.c` 定义：

```text
init_road_determine()
road_new_from_bit()
road_decision()
serve_road()
get_road_type()
```

注意：当前 `road.c` 是要斩杀的旧逻辑来源，因为它的 `serve_road()` 会直接改：

```text
global cross_cnt
left_cnt
cross_delay
rw_time_cur / rw_time_tar
speed_show_flag
status.state.base_speed
status.state.status_pid.follow_line_pid
```

这些不符合 TASK 架构。

### 2.4 当前 gw_find_line.c 的可借鉴点

当前 `gw_find_line.c` 是数字灰度模块，它有一个值得借鉴的设计：

```c
typedef struct GW_8BIT {
  uint8_t data_buf;
  int16_t gw_bit_weight[8];
  uint8_t integral;
  uint8_t maybe;
  uint8_t cross_cnt;
  Road cross;
  PID gw_find_line_pid;
  int32_t gw_diff;
} GW_8BIT;
```

也就是说：传感器结构体内部自己保存路口观测状态。

本次融合就是要把类似思想搬到模拟灰度：

```text
status.sensor.gw_analogue.cross.cross
```

---

## 3. 最关键兼容性问题

### 3.1 硬件宏不兼容

学长文件使用：

```c
hadc1
AD0_GPIO_Port / AD0_Pin
AD1_GPIO_Port / AD1_Pin
AD2_GPIO_Port / AD2_Pin
huart3
```

当前工程使用：

```c
hadc3
IO2_GPIO_Port / IO2_Pin
IO3_GPIO_Port / IO3_Pin
IO4_GPIO_Port / IO4_Pin
huart1
```

因此不能复制学长硬件读取部分。

移植时必须保留当前工程的：

```c
HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, ...)
HAL_GPIO_WritePin(IO3_GPIO_Port, IO3_Pin, ...)
HAL_GPIO_WritePin(IO4_GPIO_Port, IO4_Pin, ...)
HAL_ADC_Start(&hadc3)
HAL_ADC_PollForConversion(&hadc3, 1)
HAL_ADC_GetValue(&hadc3)
```

### 3.2 `Road` 归属问题

学长 `gw_anagloge.h` 自己定义了：

```c
typedef enum Road { ... } Road;
```

当前工程 `road.h` 也定义了：

```c
typedef enum Road { ... } Road;
```

如果直接把学长 `Road` 复制进当前 `gw_anagloge.h`，并且旧 `road.h` 仍被其它参与编译的文件 include，会出现重定义。

本次移植的最终目标不是兼容旧 `road.h`，而是像学长一样把 `Road` 枚举归入 `gw_analogue` 模块。

```text
Road enum 不再以 User/Status/road.h 为最终来源。
Road enum 应复制/迁入 User/Sensor/gw_anagloge.h。
后续旧 road.h/road.c 由用户手动删除或移出构建。
AI agent 不要删除 road.h/road.c。
```

也就是说，移植后推荐结构是严格接近学长版本：

```c
typedef enum Road {
  CrossRoad = 0b111,
  TBRoad = 0b101,
  TLRoad = 0b011,
  TRRoad = 0b110,
  LeftRoad = 0b001,
  RightRoad = 0b100,
  Straight = 0b010,
  UnknowRoad = 0b000,
} Road;

typedef struct Cross {
  ...
  Road cross;
  ...
} Cross;

typedef struct GW_ANALOGUE {
  ...
  Cross cross;
} GW_ANALOGUE;
```

兼容性提醒：

```text
在用户没有手动删除/移出旧 road.h/road.c 之前，同时编译新旧两套 Road/get_road_type/serve_road 可能报重定义或重复符号。
这是预期风险，不要为了规避这个风险把 Road 继续留在 road.h。
```

### 3.3 `Cross` 和 `RoadDetermine` 的关系

当前工程已有：

```c
RoadDetermine
```

学长新增：

```c
Cross
```

二者用途相近：

```text
RoadDetermine:
  原有 road.c 的路口判定状态

Cross:
  学长塞进 GW_ANALOGUE 的路口判定状态，额外带各种路口计数
```

移植时有两种路线。

推荐路线 ：新增 `Cross`，最终废弃 `RoadDetermine`

```text
优点：
  更接近学长结构。
  严格贴近学长结构。
  让路口观测状态归属于模拟灰度传感器。

做法：
  gw_anagloge.h 中新增 Road 和 Cross。
  GW_ANALOGUE 中新增 Cross cross。
  新逻辑使用 status.sensor.gw_analogue.cross。
  旧 status.state.road_determine 不作为新任务输入。
  旧 road.h/road.c 后续由用户手动删除或移出构建。
```



### 3.4 函数名冲突

当前 `road.c` 已经定义：

```text
init_road_determine()
road_new_from_bit()
road_decision()
serve_road()
get_road_type()
```

学长 `gw_analogue.c` 也定义同名函数：

```text
init_road_determine()
road_new_from_bit()
road_decision()
serve_road()
get_road_type()
```

如果直接把这些函数复制进当前 `gw_analogue.c`，但 `road.c` 仍参与编译，会出现重复定义。

本次最终目标是像学长一样把这些函数放进 `gw_analogue.c`，函数名也可以保持学长风格：

```text
init_road_determine()
init_road_cnt()
road_new_from_bit()
road_decision()
serve_road()
get_road_type()
```

重要：

```text
AI agent 不要删除旧 road.c和 gw_fine_line.c。
如果旧 road.c和gw_fine_line.c 仍参与编译，重复定义是预期兼容性问题。
由用户手动删除 road.c 或从工程构建中移除后，再进行最终编译验收。

```

### 3.5 `status.state.road_determine` 与 `status.sensor.gw_analogue.cross`

当前 `status.h` 里有：

```c
RoadDetermine road_determine;
```

当前 `follow_line()` 调用：

```c
get_road_type(&status->state.road_determine,
              status->sensor.gw_analogue.digital_8bit);
```

融合后希望改成：

```c
get_road_type(&status->sensor.gw_analogue.cross,
              status->sensor.gw_analogue.digital_8bit);
```

最终 TASK 也应该读：

```c
status.sensor.gw_analogue.cross.cross
```

而不是：

```c
status.state.road_determine.cross
```

但是 AI agent 不要删除 `status.state.road_determine` 字段所在文件。该旧字段后续由用户手动清理。

### 3.6 `serve_road()` 的语义必须改变

当前旧 `road.c` 的 `serve_road()` 是脏的，因为它直接影响车辆行为。

学长新版 `serve_road(Cross *cross, Road road)` 相对干净，只做：

```text
对应路口类型统计计数 +1
cross->cross = road
```

移植时应该严格采用学长这个方向，但还要注意：

```text
CrossRoad_cnt / LeftRoad_cnt 等只能作为传感器调试统计。
它们不能代替 status.task.cross_cnt。
```

语义：

```text
只更新传感器观测状态。
不改 motion。
不改 base_speed。
不改 wheel tar_speed。
不改 PID。
不改 status.task.cross_cnt。
```

### 3.7 `Road` 左右编码问题：不准修

学长文件和当前工程都有同一个语义问题：

```c
road_new_from_bit()
  left  -> 0b100
  front -> 0b010
  right -> 0b001
```

但是 enum 里写的是：

```c
LeftRoad  = 0b001
RightRoad = 0b100
TLRoad    = 0b011
TRRoad    = 0b110
```

从纯代码语义看左右是反的。但是当前实车路口判断已经完美运行，原因是其它地方也存在方向/位序反转，两个错误负负得正。

因此本文档明确要求：

```text
不要修 LeftRoad/RightRoad 枚举值。
不要修 TLRoad/TRRoad 枚举值。
不要修 road_new_from_bit() 中 left=0b100/right=0b001 的写法。
不要因为代码语义看起来反了就擅自改。
```

正确移植是保持学长文件和当前工程一致的映射。

### 3.8 回到 Straight 的条件太窄

学长代码从特殊路口回到 Straight 的条件是：

```c
digital_8bit == 0x18 ||
digital_8bit == 0x10 ||
digital_8bit == 0x08
```

这个条件可能太窄。实际回线时可能出现：

```text
0x18
0x10
0x08
0x1C
0x38
0x0C
0x3C
```

建议移植时先保留简单条件，调车时再扩展为更稳的“中间传感器看到线，外侧横线消失”判据。

### 3.9 日志串口和日志频率

学长代码大量使用：

```c
log_uprintf(&huart3, ...)
```

当前工程常用：

```c
log_uprintf(&huart1, ...)
```

移植时把打印相关的代码全部注释
建议：

```text
移植时把打印代码全部注释
```

### 3.10 校准 LED 冲突

学长代码校准时操作：

```c
status.device.led1.on = 1;
status.device.led1.on = 0;
```

当前工程校准时操作：

```c
status.device.led_on_board.on = 1;
status.device.led_on_board.on = 0;
```

而当前 TASK 架构已经用 3 个 LED 显示任务状态。

移植时要注意：

```text
把校准时的LED指示改成Defect.c目前没有采用的LED编码（Defect.c用了6个 还剩2个
自己挑一个 移植结束告诉用户用了哪个编码）
```

### 3.11 `get_gw_analogue_analogue_diff()` 写全局 status 的问题

学长代码最后写：

```c
status.sensor.gw_analogue.diff = diff;
```

当前工程也有类似写法。

更干净的写法应该是：

```c
gw_analogue->diff = diff;
```

因为函数已经传入了指针，没必要强绑全局 `status`。

移植时推荐顺手改成指针写法，减少传感器模块对全局状态的耦合。

---

## 4. 推荐最终结构

推荐移植完成后，当前 `gw_anagloge.h` 变成类似：

```c
#ifndef __GW_ANALOGUE_H
#define __GW_ANALOGUE_H

#include "main.h"

typedef enum Road {
  CrossRoad = 0b111,
  TBRoad = 0b101,
  TLRoad = 0b011,
  TRRoad = 0b110,
  LeftRoad = 0b001,
  RightRoad = 0b100,
  Straight = 0b010,
  UnknowRoad = 0b000,
} Road;

typedef struct Cross {
  uint8_t data_buf;
  uint8_t integral;
  uint8_t maybe;
  uint8_t cross_cnt;
  Road cross;
  uint8_t integral_times;

  uint8_t CrossRoad_cnt;
  uint8_t TBRoad_cnt;
  uint8_t TLRoad_cnt;
  uint8_t TRRoad_cnt;
  uint8_t LeftRoad_cnt;
  uint8_t RightRoad_cnt;
  uint8_t Straight_cnt;
  uint8_t UnknowRoad_cnt;
} Cross;

typedef struct GW_ANALOGUE {
  uint8_t channel[8];
  uint8_t sta;
  uint8_t correction_data_w[8];
  uint8_t correction_data_b[8];
  uint8_t digital_8bit;
  uint8_t digital_high_threshold[8];
  uint8_t digital_low_threshold[8];
  float diff;

  Cross cross;
} GW_ANALOGUE;

void init_gw_analogue(GW_ANALOGUE *gw_analogue);
void get_gw_raw_data(GW_ANALOGUE *gw_analogue);
void correct_gw_analogue(GW_ANALOGUE *gw_analogue);
void select_channel(uint8_t channel);
void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue);
void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue);
void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue);

void init_road_determine(Cross *cross);
void init_road_cnt(Cross *cross);
void get_road_type(Cross *cross, uint8_t road_data);
void driver_gw_analogue(GW_ANALOGUE *gw_analogue);

#endif
```

注意：

```text
Road 直接放在 gw_anagloge.h，严格贴近学长结构。
不要再让新 gw_analogue 模块依赖 road.h。
如果旧 road.h/road.c 仍存在，用户会手动删除或移出构建。
```

---

## 5. 推荐移植步骤

### Step 1：只扩展结构体，不改行为

修改 `gw_anagloge.h`：

```text
新增 Road enum，枚举值严格照学长代码，不修左右。
新增 Cross
GW_ANALOGUE 末尾新增 Cross cross
声明 init_road_determine / init_road_cnt / get_road_type / driver_gw_analogue
```

验证：

```text
工程能编译。
现有代码行为不变。
```

### Step 2：在 gw_analogue.c 中添加 Cross 初始化

在当前硬件代码基础上新增：

```text
init_road_determine()
init_road_cnt()
```

并在当前 `init_gw_analogue()` 开头或结尾调用：

```c
init_road_determine(&gw_analogue->cross);
init_road_cnt(&gw_analogue->cross);
```

验证：

```text
status.sensor.gw_analogue.cross.cross 上电后为 Straight。
计数字段清零。
```

### Step 3：移植 road 判断函数

从学长代码参考移植：

```text
road_new_from_bit()
road_decision()
serve_road()
get_road_type()
```

要求：

```text
serve_road() 只能更新 Cross。
不能改 status.state.base_speed。
不能改 status.state.motion。
不能改 wheel tar_speed。
不能改 PID。
不能改 status.task.cross_cnt。
```

验证：

```text
在旧 road.c 已经由用户手动移出构建后，编译没有 duplicate symbol。
灰度二值化后，get_road_type() 能更新 status.sensor.gw_analogue.cross.cross。
```

### Step 4：新增 driver_gw_analogue()

在当前 `gw_analogue.c` 中新增：

```c
void driver_gw_analogue(GW_ANALOGUE *gw_analogue) {
  get_gw_raw_data(gw_analogue);
  get_gw_analoge_digital_data(gw_analogue);
  get_gw_analogue_analogue_diff(gw_analogue);
  get_road_type(&gw_analogue->cross, gw_analogue->digital_8bit);
}
```

注意：

```text
这里必须用当前工程的 get_gw_raw_data()。
也就是 hadc3 + IO2/IO3/IO4。
不要复制学长 hadc1 + AD0/AD1/AD2。
```

验证：

```text
调用 driver_gw_analogue() 一次后：
  channel[0..7] 更新
  digital_8bit 更新
  diff 更新
  cross.cross 更新
```

### Step 5：调整 update_status()/follow_line() 调用链

当前调用链是：

```text
update_status()
  -> get_gw_raw_data()
  -> follow_line()
      -> get_gw_analoge_digital_data()
      -> get_gw_analogue_analogue_diff()
      -> get_road_type(&status.state.road_determine, digital_8bit)
```

目标调用链是：

```text
update_status()
  -> driver_gw_analogue(&status.sensor.gw_analogue)
  -> update_task()
  -> follow_line()
      -> 只使用 status.sensor.gw_analogue.diff 算 tar_speed
```

并且 TASK 后续读取：

```c
status.sensor.gw_analogue.cross.cross
```

注意：

```text
不要重复 ADC 读取。
如果 update_status() 已经 driver_gw_analogue()，follow_line() 就不要再 get_gw_raw_data()。
```

### Step 6：TASK 接管路口意义

移植后，路口观测结果位于：

```c
status.sensor.gw_analogue.cross.cross
```

具体任务中这样使用：

```text
task_basic_1_update():
  根据 race_phase 判断当前 road 是否有效。
  如果有效，再 status.task.cross_cnt++。
  然后切换 phase 或停车。

task_basic_2_update():
  根据 start_pose、race_phase、里程门限过滤干扰 A4。
```

不要用：

```c
status.sensor.gw_analogue.cross.LeftRoad_cnt
```

来代替任务计数。那些只是传感器调试统计。

### Step 7：最后再斩杀旧 road.c 逻辑

等新链路稳定后，再处理旧逻辑：

```text
Turn_or_Straight()
road_buf
status.state.road_determine
road.c 中会改 base_speed/PID/cross_delay 的旧 serve_road()
follow_line() 中 LeftRoad/RightRoad/CrossRoad 直接转弯
```

不要在移植第一步就全部删除。先让新观测链路跑通。

---

## 6. 推荐验收标准

移植完成后至少满足：

```text
1. 用户手动删除或移出旧 road.c/road.h 相关构建后，工程能编译，无 `Road` 重定义，无 `get_road_type/serve_road` 重复定义。
2. get_gw_raw_data() 仍然使用 hadc3 和 IO2/IO3/IO4。
3. driver_gw_analogue() 可以一次性更新 raw/digital/diff/cross。
4. follow_line() 不再调用旧 get_road_type(&status.state.road_determine, ...)。
5. 新路口观测结果在 status.sensor.gw_analogue.cross.cross。
6. road 观测不会直接改 base_speed、motion、wheel.tar_speed、PID。
7. status.task.cross_cnt 仍然只由 TASK 状态机更新。
8. 灰度校准、LED 任务编码、蜂鸣器逻辑没有被移植代码破坏。
```

---

## 7. 给 AI agent 的执行提示词

请按照以下要求移植，不要自由发挥：

```text
你正在把 D:\KEIL_CUBE_VS\diansaishenche\User\Sensor 下的 gw_analogue.c/h 思路移植进当前工程。

当前工程路径：
D:\KEIL_CUBE_VS\car_control_stm32_project-master

目标：
把模拟灰度读取、二值化、模拟循迹 diff、路口观测融合进 User/Sensor/gw_analogue.c/h。
最终让 status.sensor.gw_analogue.cross.cross 成为 TASK 可读取的路口观测结果。

禁止：
不要删除 radar.c、abslute_angle_sensor.c、ccd.c 等无关代码。
不要删除 road.c、road.h、gw_find_line.c、gw_find_line.h；这些旧文件由用户自己手动删除或移出构建。
不要整文件覆盖当前 gw_analogue.c/h。
不要复制学长的 hadc1、AD0/AD1/AD2、huart3 硬件配置。
不要修正 LeftRoad/RightRoad/TLRoad/TRRoad 的枚举值和 road_new_from_bit() 映射。
不要让传感器层修改 base_speed、motion、wheel.tar_speed、PID 或 status.task.cross_cnt。
不要为了兼容旧 road.c 而给新路口函数强行加 gw_ 前缀；最终目标是学长式融合到 gw_analogue 模块。

必须：
保留当前工程 hadc3 + IO2/IO3/IO4 的硬件读取。
在 gw_anagloge.h 中放入 Road enum，严格照学长代码。
在 GW_ANALOGUE 中新增 Cross cross。
新增 driver_gw_analogue()，使它统一完成 raw/digital/diff/road 观测更新。
路口判断函数名和结构尽量严格贴近学长代码：init_road_determine / init_road_cnt / road_new_from_bit / road_decision / serve_road / get_road_type。
将路口观测结果放在 status.sensor.gw_analogue.cross.cross。
让 TASK 状态机后续负责判断路口是否有效，以及是否更新 status.task.cross_cnt。

推荐步骤：
1. 扩展 gw_anagloge.h 的结构体。
2. 添加 Cross 初始化函数。
3. 添加 road_new_from_bit / road_decision / serve_road / get_road_type。
4. 添加 driver_gw_analogue。
5. 修改 update_status/follow_line 调用链，避免重复读取 ADC。
6. 用户会自己删除或移出旧 road.c/gw_find_line.c；AI agent 不要删除这些文件。
```

---

## 8. 关键心法

这次移植的本质不是“换一个 road.c”，而是改变归属：

```text
旧架构：
  road.c 识别路口，并偷偷决定停车、转弯、计数、改 PID。

目标架构：
  gw_analogue.c 只负责传感器观测。
  TASK 负责解释观测结果。
  motion/follow_line/keep_angle 负责底层运动。
  driver_wheel 负责真正输出 PWM。
```

记住：

```text
传感器只能说：我看到了什么。
TASK 才能说：这件事在当前题目阶段意味着什么。
```
