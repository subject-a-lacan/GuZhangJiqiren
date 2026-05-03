#include "road.h"

#include "log.h"
#include "status.h"
#include "usart.h"

uint8_t cross_cnt = 0;
uint8_t left_cnt = 0;
extern int32_t rw_time_cur;
extern int32_t rw_time_tar;
extern uint8_t cross_delay;
extern uint8_t speed_show_flag;

void init_road_determine(RoadDetermine *roaddetermine) {
  roaddetermine->integral = 0;
  roaddetermine->data_buf = 0;
  roaddetermine->cross = Straight;
  roaddetermine->cross_cnt = 0;
}

// 路口判定已迁移至 gw_analogue.c 的 driver_gw_analogue() 中
// 本文件保留全局变量 cross_cnt/left_cnt 供旧代码兼容
// 新任务层代码应使用 status.sensor.gw_analogue.cross
