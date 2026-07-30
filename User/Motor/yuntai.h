#ifndef __YUNTAI_H
#define __YUNTAI_H
#include "datou.h"
typedef struct { DATOU yaw_datou,pitch_datou; uint8_t yuntai_sta; float temp_x,temp_y,yuntai_speed; } YUNTAI;
#define BASE_LINE 1.2f
void init_yuntai(YUNTAI*,uint8_t,uint8_t,uint16_t,uint16_t,uint16_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t);
void point_calculation(YUNTAI*,float,float); void driver_yuntai(YUNTAI*,float,float); void yuntai_test(YUNTAI*); void yuntai_set_zero(YUNTAI*); void driver_yuntai_Multiple_laps(YUNTAI*,float,float);
#endif
