// @551
// 驱动介绍:   此为感为8路模拟量输出传感器的驱动文件
// 传感器链接: https://item.taobao.com/item.htm?id=902128042528
// 功能实现:   将八路传感器的模拟值进行插值输出线性的黑线位置
//            使用迟滞比较器将八路模拟量转换为一个uint8_t数字量
//            通过数字量进行路口判断
// 注意事项:   传感器使用前需要校准，调用correct_gw_analogue()函数进行校准,详细此函数

#ifndef __GW_ANALOGUE_H
#define __GW_ANALOGUE_H

#include "main.h"

typedef enum Road {    // L F R
  CrossRoad = 0b111,   // 1 1 1
  TBRoad = 0b101,      // 1 0 1
  TLRoad = 0b011,      // 1 1 0
  TRRoad = 0b110,      // 0 1 1
  LeftRoad = 0b001,    // 1 0 0
  RightRoad = 0b100,   // 0 0 1
  Straight = 0b010,    // 0 1 0
  UnknowRoad = 0b000,  // 0 0 0
} Road;

typedef struct Cross {
  uint8_t data_buf;
  uint8_t integral;
  uint8_t maybe;
  uint8_t cross_cnt;
  Road cross;
  uint8_t integral_times;
  uint8_t cross_delay;

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
  uint8_t channel[8];                 // 0-7
  uint8_t sta;                        // 0工作模式 1校准模式
  uint8_t correction_data_w[8];       // 白色校准数据
  uint8_t correction_data_b[8];       // 黑色校准数据
  uint8_t digital_8bit;               // 8bit数字量
  uint8_t digital_high_threshold[8];  // 8bit高阈值
  uint8_t digital_low_threshold[8];   // 8bit低阈值
  float diff;
  Cross cross;                        // 路口判断结构体
} GW_ANALOGUE;

// 初始化传感器
void init_gw_analogue(GW_ANALOGUE *aw_analogue);

// 获取传感器的原始数据
void get_gw_raw_data(GW_ANALOGUE *aw_analogue);

// 校准传感器
void correct_gw_analogue(GW_ANALOGUE *gw_analogue);

// 选择传感器的通道（内部调用）
void select_channel(uint8_t channel);

// 将raw数据解析为数字量数据
void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue);

// 将数字量数据打印至串口
void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue);

// 将raw数据解析为模拟量数据
void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue);

// 驱动灰度传感器（采集raw→数字量→模拟差→路口判断）
void driver_gw_analogue(GW_ANALOGUE *gw_analogue);

#endif
