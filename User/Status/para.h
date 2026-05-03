#ifndef __PARA_H_
#define __PARA_H_

#include "stdbool.h"
#include "stdint.h"

typedef enum TYPE {
  FLOAT = 1,
  DOUBLE,
  INT,
  UINT8,
  INT8,
  UINT16,
  INT16,
  UINT32,
  INT32,
  UINT64,
  INT64,
  UNKNOWN,
} TYPE;

float get_cur_val(void* cur_addr, TYPE type);
uint32_t get_4byte_val(void* addr, TYPE type);
bool set_4byte_val(void* addr, TYPE type, uint32_t value);

#endif
