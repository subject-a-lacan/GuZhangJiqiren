#include "para.h"

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

uint32_t get_4byte_val(void* addr, TYPE type) {
  if (addr == NULL) {
    return 0;
  }
  switch (type) {
    case INT:
    case INT32:
    case UINT32:
      return *(uint32_t*)addr;
    case FLOAT:
      return *(uint32_t*)addr;
    case DOUBLE:
      return (uint32_t)(*(double*)addr);
    case UINT8:
      return *(uint8_t*)addr;
    case INT8:
      return (uint32_t)(*(int8_t*)addr);
    case UINT16:
      return *(uint16_t*)addr;
    case INT16:
      return (uint32_t)(*(int16_t*)addr);
    case UINT64:
      return (uint32_t)(*(uint64_t*)addr);
    case INT64:
      return (uint32_t)(*(int64_t*)addr);
    default:
      return 0;
  }
}

bool set_4byte_val(void* addr, TYPE type, uint32_t value) {
  if (addr == NULL) {
    return false;
  }

  switch (type) {
    case INT:
    case INT32:
      *(int32_t*)addr = (int32_t)value;
      break;
    case UINT32:
      *(uint32_t*)addr = value;
      break;
    case FLOAT:
      memcpy(addr, &value, sizeof(float));
      break;
    case UINT8:
      *(uint8_t*)addr = (uint8_t)value;
      break;
    case INT8:
      *(int8_t*)addr = (int8_t)value;
      break;
    case UINT16:
      *(uint16_t*)addr = (uint16_t)value;
      break;
    case INT16:
      *(int16_t*)addr = (int16_t)value;
      break;
    case DOUBLE:
    case UINT64:
    case INT64:
      memcpy(addr, &value, sizeof(uint32_t));
      break;
    default:
      return false;
  }
  return true;
}

float get_cur_val(void* cur_addr, TYPE type) {
  if (cur_addr == NULL) {
    return 0.0f;
  }
  switch (type) {
    case INT:
      return *(int*)cur_addr;
    case FLOAT:
      return *(float*)cur_addr;
    case DOUBLE:
      return *(double*)cur_addr;
    case UINT8:
      return *(uint8_t*)cur_addr;
    case UINT16:
      return *(uint16_t*)cur_addr;
    case UINT32:
      return *(uint32_t*)cur_addr;
    case UINT64:
      return *(uint64_t*)cur_addr;
    case INT8:
      return *(int8_t*)cur_addr;
    case INT16:
      return *(int16_t*)cur_addr;
    case INT32:
      return *(int32_t*)cur_addr;
    case INT64:
      return *(int64_t*)cur_addr;
    default:
      return 0.0f;
  }
}
