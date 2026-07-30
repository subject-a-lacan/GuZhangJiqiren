#ifndef __DATOU_H
#define __DATOU_H
#include "Emm_v5.h"
#define ONE_CIRCLE_CLK 3200
typedef struct { uint8_t addr,mode; uint16_t speed; uint8_t acc,dir; volatile float angle; int8_t sta; uint32_t clk; } DATOU;
void init_datou(DATOU*,uint8_t,uint8_t,uint16_t,uint8_t,uint8_t); void driver_datou(DATOU*); void datou_set_zero(DATOU*); void datou_return_zero(DATOU*,uint8_t); void driver_datou_Multiple_laps(DATOU*,uint8_t); void datou_enable(void*); void datou_disable(void*);
#endif
