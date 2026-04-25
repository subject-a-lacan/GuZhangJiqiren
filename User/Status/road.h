#ifndef __ROAD_H__
#define __ROAD_H__

#include "stdint.h"

#define ROAD_CROSS -30000
#define ROAD_TB 30000
#define ROAD_TL -25000
#define ROAD_TR 25000
#define ROAD_LEFT -20000
#define ROAD_RIGHT 20000

typedef enum Road {    // L F R
  CrossRoad = 0b111,   // 三方向都有路
  TBRoad = 0b101,      // 左右有路 前方无路
  TLRoad = 0b011,      // 左前有路 右侧无路
  TRRoad = 0b110,      // 前右有路 左侧无路
  LeftRoad = 0b001,    // 只有左侧有路
  RightRoad = 0b100,   // 只有右侧有路
  Straight = 0b010,    // 只有前方有路
  UnknowRoad = 0b000,  // 没识别到路
} Road;

typedef struct RoadDetermine {
  uint8_t data_buf;        // 当前帧巡线数字量
  uint8_t integral;        // 按位或：只要有一位是1就是1 用来取多帧判定的并集举例（假设窗口 4 帧）：
                          // 第1帧 data_buf = 00111100
                          // 第2帧 data_buf = 00111110
                          // 第3帧 data_buf = 01111100 
                          // 第4帧 data_buf = 00111100  第4次后 = 01111110
  uint8_t maybe;           // 计数器
  uint8_t cross_cnt;       // 已通过/识别的路口计数
  Road cross;              // 当前判定出的道路类型
  uint8_t integral_times;  // 进行一次判定需要累计的帧数  计数器的初始值

} RoadDetermine;

// 对uint8_t数据进行路口判断
// 传入参数: 巡线传感器的0-7通道数字量
// 判断到有路口时进入
void get_road_type(RoadDetermine *roaddetermine, uint8_t road_data);

#endif
