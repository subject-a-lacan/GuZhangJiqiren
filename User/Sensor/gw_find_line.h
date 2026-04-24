// @551

#ifndef __GW_FIND_LINE_H
#define __GW_FIND_LINE_H

#include "main.h"
#include "pid.h"
#include "road.h"

typedef struct GW_8BIT {
  uint8_t data_buf;          // 8路巡线数字量原始位图
  int16_t gw_bit_weight[8];  // 8路传感器权重表
  uint8_t integral;          // 路口判定积分缓存
  uint8_t maybe;             // 路口候选计数器
  uint8_t cross_cnt;         // 已识别路口计数
  Road cross;                // 当前道路类型
  PID gw_find_line_pid;      // 巡线PID参数与状态
  int32_t gw_diff;           // 计算出的线偏差
} GW_8BIT;

void get_gw_8bit_data(I2C_HandleTypeDef *hi2c, GW_8BIT *gw_8bit);
void get_gw_8bit_update_analog_data(I2C_HandleTypeDef *hi2c, GW_8BIT *gw_8bit);
void init_gw_8bit(GW_8BIT *gw_8bit);
int32_t gw_get_line_diff(GW_8BIT *gw_8bit);

#endif
