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

typedef enum Road {
  CrossRoad = 0b111,   // 三方向都有路
  TBRoad = 0b101,      // 左右有路 前方无路
  TLRoad = 0b011,      // 左前有路 右侧无路
  TRRoad = 0b110,      // 前右有路 左侧无路
  LeftRoad = 0b001,    // 只有左侧有路
  RightRoad = 0b100,   // 只有右侧有路
  Straight = 0b010,    // 只有前方有路
  UnknowRoad = 0b000,  // 没识别到路
} Road;

typedef struct Cross {
  uint8_t data_buf;        // 当前帧巡线数字量
  uint8_t integral;        // 多帧按位或累积
  uint8_t maybe;           // 计数器
  uint8_t cross_cnt;       // 已通过/识别的路口计数
  Road cross;              // 当前判定出的道路类型
  uint8_t integral_times;  // 进行一次判定需要累计的帧数

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
  uint8_t channel[8];                 // 0-7
  uint8_t sta;                        // 0工作模式 1校准模式
  uint8_t correction_data_w[8];       // 白色校准数据
  uint8_t correction_data_b[8];       // 黑色校准数据
  uint8_t digital_8bit;               // 8bit数字量
  uint8_t digital_high_threshold[8];  // 8bit高阈值
  uint8_t digital_low_threshold[8];   // 8bit低阈值
  float diff;

  Cross cross;  // 路口观测状态
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
