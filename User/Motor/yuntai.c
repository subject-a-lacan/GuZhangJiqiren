#include "yuntai.h"
#include "math_tool.h"
#include "math.h"
void init_yuntai(YUNTAI*y,uint8_t ya,uint8_t pa,uint16_t ys,uint16_t ps,uint16_t bs,uint8_t yaa,uint8_t paa,uint8_t yd,uint8_t pd,uint8_t ym,uint8_t pm){init_datou(&y->yaw_datou,ya,ym,ys,yaa,yd);init_datou(&y->pitch_datou,pa,pm,ps,paa,pd);y->yuntai_sta=0;y->temp_x=y->temp_y=0;y->yuntai_speed=bs;}
void point_calculation(YUNTAI*y,float x,float p){y->yaw_datou.angle=atanf(x/sqrtf(BASE_LINE*BASE_LINE+p*p));y->pitch_datou.angle=atanf(p/sqrtf(BASE_LINE*BASE_LINE+x*x));}
void driver_yuntai(YUNTAI*y,float x,float p){point_calculation(y,x,p);float dx=x-y->temp_x,dy=p-y->temp_y,d=sqrtf(dx*dx+dy*dy);if(d>0.001f){y->yaw_datou.speed=(uint16_t)(y->yuntai_speed*ABS(dx)/d);y->pitch_datou.speed=(uint16_t)(y->yuntai_speed*ABS(dy)/d);}driver_datou(&y->yaw_datou);driver_datou(&y->pitch_datou);y->temp_x=x;y->temp_y=p;}
void driver_yuntai_Multiple_laps(YUNTAI*y,float x,float p){point_calculation(y,x,p);driver_datou_Multiple_laps(&y->yaw_datou,8);driver_datou_Multiple_laps(&y->pitch_datou,4);}
void yuntai_test(YUNTAI*y){if(y->yuntai_sta==0){Emm_V5_En_Control(y->yaw_datou.addr,false,false);Emm_V5_En_Control(y->pitch_datou.addr,false,false);y->yuntai_sta=1;}else if(y->yuntai_sta==1){Emm_V5_En_Control(y->yaw_datou.addr,true,false);Emm_V5_En_Control(y->pitch_datou.addr,true,false);y->yuntai_sta=2;}else if(y->yuntai_sta==2){Emm_V5_Origin_Set_O(y->yaw_datou.addr,true);Emm_V5_Origin_Set_O(y->pitch_datou.addr,true);Emm_V5_Reset_CurPos_To_Zero(y->yaw_datou.addr);Emm_V5_Reset_CurPos_To_Zero(y->pitch_datou.addr);y->yuntai_sta=3;}else{datou_return_zero(&y->yaw_datou,0);datou_return_zero(&y->pitch_datou,0);y->yuntai_sta=0;}}
void yuntai_set_zero(YUNTAI*y){datou_set_zero(&y->yaw_datou);datou_set_zero(&y->pitch_datou);}
