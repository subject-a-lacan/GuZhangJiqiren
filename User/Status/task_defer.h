#ifndef __TASK_DEFER_H_
#define __TASK_DEFER_H_

#include <stdbool.h>
#include <stdint.h>

#include "para.h"

#define MAX_DEFER_NUM 40

typedef struct defer_cell {
  void (*task)(void* para);
  bool (*judge)(float cur, float tar);
  void* para;
  float cur;
  float tar;
  TYPE type;
  void* cur_addr;
  int32_t timeout;
} defer_cell;

typedef struct TASK_DEFER {
  uint32_t defer_num;
  bool waiting[MAX_DEFER_NUM];
  defer_cell cell[MAX_DEFER_NUM];
  uint64_t defer_clock;
} TASK_DEFER;

bool add_defer(TASK_DEFER* ctrl, void (*task)(void* para), void* para, bool(*judge), void* cur_addr, float offset, TYPE type, uint32_t timeout);
void driver_defer(TASK_DEFER* ctrl);
void init_TASK_DEFER(TASK_DEFER* ctrl);
void update_defer_clock(TASK_DEFER* ctrl, uint64_t system_time);

bool Greater_or_Equal(float cur, float tar);
bool Less_or_Equal(float cur, float tar);
bool Error_less_1(float cur, float tar);
bool Error_less_10(float cur, float tar);
bool Error_less_100(float cur, float tar);
bool Error_less_1000(float cur, float tar);
uint32_t add_defer_with_time_line(TASK_DEFER* ctrl, void (*task)(void* para), void* para, uint32_t later);

#endif
