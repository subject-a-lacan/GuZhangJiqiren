#ifndef __ROAD_H__
#define __ROAD_H__

#include "gw_anagloge.h"  // Road enum now lives in gw_analogue
#include "stdint.h"

#define ROAD_CROSS -30000
#define ROAD_TB 30000
#define ROAD_TL -25000
#define ROAD_TR 25000
#define ROAD_LEFT -20000
#define ROAD_RIGHT 20000

// RoadDetermine kept for backward compat; new code should use gw_analogue.cross
typedef struct RoadDetermine {
  uint8_t data_buf;
  uint8_t integral;
  uint8_t maybe;
  uint8_t cross_cnt;
  Road cross;
  uint8_t integral_times;
} RoadDetermine;

#endif
