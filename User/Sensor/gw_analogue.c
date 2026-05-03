#include "adc.h"
#include "gpio.h"
#include "gw_anagloge.h"
#include "log.h"
#include "main.h"
#include "status.h"
#include "math_tool.h"
#include "Defect.h"
#include "stdbool.h"
float distance[8] = {-30, -20, -15, -10, 10, 15, 20, 30};

uint8_t cross_cnt = 0;
uint8_t left_cnt = 0;

/*
 * 调试用：将 digital_8bit 用 ASCII 字符打印到串口（# = 看到线，. = 没看到）。
 * 数据来源：status.sensor.gw_analogue.digital_8bit（由 get_gw_analoge_digital_data 更新）。
 * 调用时机：仅在调试时手动调用，不参与巡线/路口主链路。
 */
void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue) {
  uint8_t buf = gw_analogue->digital_8bit;
  char str[9];
  str[8] = '\0';
  for (int i = 0; i < 8; i++) {
    str[i] = buf & 0x80 ? '#' : '.';
    buf <<= 1;
  }
  PRINTLN("%s", str);
}

/*
 * 初始化路口判定缓存（status.sensor.gw_analogue.cross 中的判定字段）。
 * 将 integral / data_buf / maybe / cross / cross_cnt 置为安全初值，
 * integral_times 设为 70（需连续 70 帧确认路口，详见 get_road_type）。
 * 调用时机：init_gw_analogue() 上电初始化；task_start() 发车复位（Defect.c）。
 */
void init_road_determine(Cross *cross) {
  cross->integral = 0;
  cross->data_buf = 0;
  cross->maybe = 0;
  cross->cross = Straight;
  cross->cross_cnt = 0;
  cross->integral_times = 5;

  return;
}

/*
 * 清零路口类型调试计数器（CrossRoad_cnt ~ UnknowRoad_cnt）。
 * 这些计数器只记录传感器层观测到的各类型路口次数，仅用于调试/VOFA 打印，
 * 不能替代 status.task.cross_cnt（TASK 状态机确认的有效路口计数）。
 * 调用时机：init_gw_analogue() 上电初始化。
 */
void init_road_cnt(Cross *cross) {
  cross->CrossRoad_cnt = 0;
  cross->LeftRoad_cnt = 0;
  cross->RightRoad_cnt = 0;
  cross->Straight_cnt = 0;
  cross->TBRoad_cnt = 0;
  cross->TLRoad_cnt = 0;
  cross->TRRoad_cnt = 0;
  cross->UnknowRoad_cnt = 0;
  return;
}

/*
 * 模拟灰度传感器上电初始化。
 * 操作对象：status.sensor.gw_analogue（整个 GW_ANALOGUE 结构体）。
 * 1. 初始化路口判定缓存和调试计数器（调用 init_road_determine / init_road_cnt）。
 * 2. 清零 channel[] / correction_data_w[] / correction_data_b[]。
 * 3. 写入默认高低阈值（高 46-47 / 低 24-25，实车调出来的经验值）。
 * 4. 用默认阈值推算出校准数据的公式值（作为校准前兜底）。
 * 5. 清零 sta / digital_8bit / diff。
 * 6. 选中通道 0 作为初始 ADC 通道。
 * 调用时机：init_sensor() → status.c 上电初始化链路。
 */
void init_gw_analogue(GW_ANALOGUE *gw_analogue) {
  init_road_determine(&gw_analogue->cross);
  init_road_cnt(&gw_analogue->cross);

  // Initialize the ADC and GPIO for the analogue channels
  for (int i = 0; i < 8; i++) {
    gw_analogue->channel[i] = 0;  // Initialize channel values to 0
  }
  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 0;  // Initialize correction data to 0
    gw_analogue->correction_data_b[i] = 0;  // Initialize correction data to 0
  }
 gw_analogue->digital_high_threshold[0] = 46;
  gw_analogue->digital_high_threshold[1] = 46;
  gw_analogue->digital_high_threshold[2] = 47;
  gw_analogue->digital_high_threshold[3] = 47;
  gw_analogue->digital_high_threshold[4] = 47;
  gw_analogue->digital_high_threshold[5] = 47;
  gw_analogue->digital_high_threshold[6] = 46;
  gw_analogue->digital_high_threshold[7] = 46;

  gw_analogue->digital_low_threshold[0] = 24;
  gw_analogue->digital_low_threshold[1] = 24;
  gw_analogue->digital_low_threshold[2] = 24;
  gw_analogue->digital_low_threshold[3] = 25;
  gw_analogue->digital_low_threshold[4] = 25;
  gw_analogue->digital_low_threshold[5] = 25;
  gw_analogue->digital_low_threshold[6] = 25;
  gw_analogue->digital_low_threshold[7] = 24;

  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 2 * gw_analogue->digital_high_threshold[i] - gw_analogue->digital_low_threshold[i];  // Initialize high threshold to 0
    gw_analogue->correction_data_b[i] = 2 * gw_analogue->digital_low_threshold[i] - gw_analogue->digital_high_threshold[i];  // Initialize low threshold to 0
  }

  gw_analogue->sta = 0;           // Set the state to 0 (normal mode)
  gw_analogue->digital_8bit = 0;  // Initialize the 8-bit digital value to 0

  gw_analogue->diff = 0.0f;  // Initialize the difference value to 0.0f

  select_channel(0);  // Select channel 0 for initial setup
}

/*
 * 硬件层：通过 IO2/IO3/IO4 三根 GPIO 选择 8 选 1 模拟开关的通道。
 * channel 的低 3 位分别控制三根 IO：
 *   bit0 → IO2,  bit1 → IO3,  bit2 → IO4
 * 调用时机：get_gw_raw_data() 和 correct_gw_analogue() 遍历 8 通道时。
 */
void select_channel(uint8_t channel) {
  if (channel & 0x01) {
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_RESET);
  }
  if (channel & 0x02) {
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, GPIO_PIN_RESET);
  }
  if (channel & 0x04) {
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, GPIO_PIN_RESET);
  }
}

/*
 * 遍历 8 个通道，依次选择通道 → 启动 ADC3 → 等转换完成 → 读取 12bit ADC 值。
 * 结果写入 status.sensor.gw_analogue.channel[0..7]（0-4095 原始 ADC 值）。
 * 调用时机：driver_gw_analogue() 每个控制周期调用一次。
 */
void get_gw_raw_data(GW_ANALOGUE *gw_analogue) {
  // Read the ADC value for the selected channel
  for (int i = 0; i < 8; i++) {
    select_channel(i);                                   // Select the channel to read from
    for (volatile uint32_t _d = 0; _d < 300; _d++);     // ~3μs delay for mux settling
    HAL_ADC_Start(&hadc3);                               // Start the ADC conversion
    HAL_ADC_PollForConversion(&hadc3, 1);                // Wait for conversion to complete
    gw_analogue->channel[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
    HAL_ADC_Stop(&hadc3);                                // Stop the ADC conversion
  }
}

/*
 * 两阶段灰度校准，由外部校准流程手动驱动（不在 driver_gw_analogue 内自动调用）。
 * 操作对象：status.sensor.gw_analogue。
 * sta=0（白校准）：读取 8 通道 ADC → correction_data_w[]，点亮板载 LED 提示用户，
 *                  sta 切到 1，return 等下次调用。
 * sta=1（黑校准）：读取 8 通道 ADC → correction_data_b[]，熄灭 LED，
 *                 按白黑差值 ×0.33/0.66 计算 digital_low_threshold / digital_high_threshold，
 *                 sta 切回 0，串口打印校准结果。
 * 注意：校准时操作的 led_on_board 与 TASK LED 编码共享同一硬件，校准时 TASK 不应在运行。
 */
void correct_gw_analogue(GW_ANALOGUE *gw_analogue) {
  if (gw_analogue->sta == 0) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);                                             // Select the channel to read from
      for (volatile uint32_t _d = 0; _d < 300; _d++);               // ~3μs delay for mux settling
      HAL_ADC_Start(&hadc3);                                         // Start the ADC conversion
      HAL_ADC_PollForConversion(&hadc3, 1);                          // Wait for conversion to complete
      gw_analogue->correction_data_w[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
      HAL_ADC_Stop(&hadc3);                                // Stop the ADC conversion
      status.device.buzzer.on = 1;
      status.device.buzzer.off_time = status.state.time + 500;
    }
    status.device.led_on_board.on = 0;
    status.device.led1.on = 0;
    status.device.led2.on = 1;
    gw_analogue->sta = 1;  // Set the state to calibration mode 1
    return;
  }
  if (gw_analogue->sta == 1) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);                                             // Select the channel to read from
      for (volatile uint32_t _d = 0; _d < 300; _d++);               // ~3μs delay for mux settling
      HAL_ADC_Start(&hadc3);                                         // Start the ADC conversion
      HAL_ADC_PollForConversion(&hadc3, 1);                          // Wait for conversion to complete
      gw_analogue->correction_data_b[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
      HAL_ADC_Stop(&hadc3);                                          // Stop the ADC conversion
      status.device.buzzer.on = 1;
      status.device.buzzer.off_time = status.state.time + 500;
    }
    status.device.led_on_board.on = 0;
    status.device.led1.on = 1;
    status.device.led2.on = 0;
    gw_analogue->sta = 0;  // Set the state to calibration mode 2
    for (int i = 0; i < 8; i++) {
      gw_analogue->digital_low_threshold[i] = gw_analogue->correction_data_b[i] +
                                              (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.33;
      // Calculate the low threshold
      gw_analogue->digital_high_threshold[i] = gw_analogue->correction_data_b[i] +
                                               (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.66;
      // Calculate the high threshold
    }
    for (int i = 0; i < 8; i++) {
      log_uprintf(&huart1, "%d ", gw_analogue->digital_low_threshold[i]);
    }
    log_uprintf(&huart1, "\n\n");
    for (int i = 0; i < 8; i++) {
      log_uprintf(&huart1, "%d ", gw_analogue->digital_high_threshold[i]);
    }
    return;
  }
}

/*
 * 迟滞比较器：将 8 路原始 ADC 值转换为 8bit 数字量。
 * 输入：status.sensor.gw_analogue.channel[0..7]（原始 ADC）
 * 输出：status.sensor.gw_analogue.digital_8bit（bit=1 表示该路看到黑线）
 * 规则：
 *   channel[i] > digital_high_threshold[i] → bit 清零（白，没线）
 *   channel[i] < digital_low_threshold[i]  → bit 置 1（黑，有线）
 *   介于两者之间 → 保持上一次的 bit 值（迟滞带，防抖动）
 * digital_8bit 在 init_gw_analogue 时清零一次，之后每帧就地修改，不整体复位。
 * 调用时机：driver_gw_analogue() 每个控制周期调用。
 */
void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue) {
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->channel[i] > gw_analogue->digital_high_threshold[i]) {
      gw_analogue->digital_8bit &= ~(1 << i);
    } else if (gw_analogue->channel[i] < gw_analogue->digital_low_threshold[i]) {
      gw_analogue->digital_8bit |= (1 << i);
    }
  }
}

/*
 * 内部工具函数：将单个通道的 ADC 值线性映射到 0-100。
 * (now - min) / (max - min) * 100
 * 注意：调用方需保证 max != min，否则除零。
 */
float normalize_gray_data(uint8_t max, uint8_t min, uint8_t now) {
  return (((float)(now - min) / (float)(max - min)) * 100);
}

/*
 * 内部工具函数：将中间 4 路传感器的归一化值转为权重（4 路权重之和 = 1.0）。
 * 只处理通道 2-5（中间四路），通道 0,1,6,7 不参与循迹 diff 计算。
 * 若 4 路总和为 0（完全没看到线），直接 return 不除零。
 */
void normalize_gray_weight(float *raw_data) {
  float total = 0;
  for (int i = 2; i < 6; i++) {
    total += raw_data[i];
  }
  if (total == 0) {
    return;
  }
  for (int i = 2; i < 6; i++) {
    raw_data[i] = (raw_data[i] / total);
  }

  return;
}

/*
 * 计算黑线相对于传感器中心的偏差值（模拟循迹 diff）。
 * 只用中间 4 路（通道 2-5，对应 distance = {-15, -10, 10, 15}）：
 *   1. 对每路 ADC 取反归一化（越黑值越大 → channel 值低 → buff 值高）。
 *   2. 归一化为权重（4 路权重和=1）。
 *   3. 加权求和得到 diff（负值=线偏左，正值=线偏右，0=居中）。
 * 结果写入 status.sensor.gw_analogue.diff → follow_line() 用此值算 PID 差速。
 * 调用时机：driver_gw_analogue() 每个控制周期调用。
 */
void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue) {
  float buff[8] = {0};
  float diff = 0;
  for (int i = 2; i < 6; i++) {
    if (gw_analogue->channel[i] < gw_analogue->digital_high_threshold[i])
      buff[i] = 100 - normalize_gray_data(gw_analogue->correction_data_w[i], gw_analogue->correction_data_b[i], gw_analogue->channel[i]);
  }
  normalize_gray_weight(buff);
  for (int i = 2; i < 6; i++) {
    diff += buff[i] * distance[i];
  }

  gw_analogue->diff = diff;
}

/*
 * 内部工具函数：将三个布尔方向编码为 Road 枚举值。
 * L → bit2(0b100), F → bit1(0b010), R → bit0(0b001)
 * 注意：由于历史遗留的双 BUG 抵消，L 对应物理右侧传感器、R 对应物理左侧传感器，
 * 枚举值 LeftRoad/RightRoad 也随之交叉。实物路口判断已验证正确，禁止修改。
 */
enum Road road_new_from_bit(bool L, bool F, bool R) {
  uint8_t left = L ? 0b100 : 0;
  uint8_t font = F ? 0b010 : 0;
  uint8_t right = R ? 0b001 : 0;

  return left | font | right;
}

/*
 * 内部函数：根据累积的多帧积分值 + 当前帧 data_buf 判定路口类型。
 * left  = integral 高 2 位 == 0b11（物理左侧传感器连续看到线）
 * right = integral 低 2 位 == 0b11（物理右侧传感器连续看到线）
 * font  = data_buf 中间 4 位任一为 1（中间传感器当前看到线）
 * 通过 road_new_from_bit() 编码后返回 Road 枚举。
 * 仅由 get_road_type() 在 maybe 倒计时到 1 时调用。
 */
Road road_decision(Cross *cross) {
  bool left = (cross->integral >> 6) == 0x03;     // 0b1100_0000
  bool right = (cross->integral & 0x03) == 0x03;  // 0b0000_0011
  bool font = cross->data_buf & 0x3C;             // 0b0011_1100
  Road road = road_new_from_bit(left, font, right);
  return road;
}

/*
 * 内部函数：记录一次路口观测结果（传感器层，不决定运动）。
 * 1. 对应 road 类型的调试计数器 +1。
 * 2. 更新 cross->cross = road（写入 status.sensor.gw_analogue.cross.cross）。
 * 3. 若 road 不是 Straight/UnknowRoad，递增 cross->cross_cnt 和全局 cross_cnt。
 * 不会修改 base_speed / motion / wheel.tar_speed / PID / status.task.cross_cnt。
 * 全局 cross_cnt 仅供调试和 follow_line 兜底逻辑使用；正式任务以 TASK 的 cross_cnt 为准。
 * 仅由 get_road_type() 在路口确认或回到 Straight 时调用。
 */
void serve_road(Cross *cross, Road road) {
  switch (road) {
    case CrossRoad:
      cross->CrossRoad_cnt++;
      break;
    case TBRoad:
      cross->TBRoad_cnt++;
      break;
    case TLRoad:
      cross->TLRoad_cnt++;
      break;
    case TRRoad:
      cross->TRRoad_cnt++;
      break;
    case LeftRoad:
      cross->LeftRoad_cnt++;
      break;
    case RightRoad:
      cross->RightRoad_cnt++;
      break;
    case Straight:
      cross->Straight_cnt++;
      break;
    case UnknowRoad:
      cross->UnknowRoad_cnt++;
      break;
  }

  cross->cross = road;

  if (road != Straight && road != UnknowRoad) {
    cross->cross_cnt++;
    cross_cnt++;
  }
}

/*
 * 多帧路口检测状态机，每帧由 driver_gw_analogue() 调用一次。
 * 输入：road_data = status.sensor.gw_analogue.digital_8bit（当前帧二值化结果）
 * 输出：status.sensor.gw_analogue.cross.cross（路口观测结果）
 *
 * 状态机逻辑：
 * A. 当前为 Straight（正常巡线中）：
 *    - 若外侧传感器（bit7 或 bit0）看到线 → 启动 maybe 计数器（= integral_times=5）。
 *    - 每帧将 digital_8bit 按位 OR 累积到 integral。
 *    - maybe 从 5 减到 1 的过程中持续累积。
 *    - maybe 减到 1 时：调用 road_decision(integral, data_buf) 判定路口类型，
 *      调用 serve_road() 记录结果，清零 maybe 和 integral。
 * B. 当前为非 Straight（已判定在路口内）：
 *    - 检测 digital_8bit 是否为 0x18 / 0x10 / 0x08（中间传感器看到线，外侧横线消失）。
 *    - 满足条件则 serve_road(Straight)，回到正常巡线状态。
 *    - 注意：当前回 Straight 条件偏窄，实测可能需要扩展更多位图（如 0x1C, 0x38 等）。
 */
void get_road_type(Cross *cross, uint8_t road_data) {
  cross->data_buf = road_data;
  if (cross->cross == Straight) {
    if ((cross->data_buf & 0x81)) {
      if (cross->maybe == 0) {
        cross->maybe = cross->integral_times;
        cross->integral = cross->integral | cross->data_buf;
      }
    }

    if (cross->maybe > 1) {
      cross->integral = cross->integral | cross->data_buf;
      cross->maybe--;
    } else if (cross->maybe == 1) {
      switch (road_decision(cross)) {
        case UnknowRoad:
          serve_road(cross, UnknowRoad);
          break;
        case CrossRoad:
          serve_road(cross, CrossRoad);
          break;
        case TBRoad:
          serve_road(cross, TBRoad);
          break;
        case TLRoad:
          serve_road(cross, TLRoad);
          break;
        case TRRoad:
          serve_road(cross, TRRoad);
          break;
        case LeftRoad:
          serve_road(cross, LeftRoad);
          break;
        case RightRoad:
          serve_road(cross, RightRoad);
          break;
        case Straight:
          serve_road(cross, Straight);
          break;
      }
      cross->maybe = 0;
      cross->integral = 0;
    }
  } else if (((road_data & 0x81) == 0) && ((road_data & 0x3C) != 0)) {
    serve_road(cross, Straight);
  }
}

/*
 * 模拟灰度传感器总驱动入口，每个控制周期调用一次。
 * 调用方：update_status() → status.c 主循环。
 * 调用链（按顺序）：
 *   1. get_gw_raw_data()          → channel[0..7] 原始 ADC
 *   2. get_gw_analoge_digital_data() → digital_8bit 二值化
 *   3. get_gw_analogue_analogue_diff() → diff 循迹偏差
 *   4. get_road_type()            → cross.cross 路口观测
 * 调用完成后，外部可读取 status.sensor.gw_analogue 的全部观测结果：
 *   channel[] / digital_8bit / diff / cross.cross
 */
void driver_gw_analogue(GW_ANALOGUE *gw_analogue) {
  get_gw_raw_data(gw_analogue);
  get_gw_analoge_digital_data(gw_analogue);
  get_gw_analogue_analogue_diff(gw_analogue);
  get_road_type(&gw_analogue->cross, gw_analogue->digital_8bit);
}
