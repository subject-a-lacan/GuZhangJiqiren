#include "datou.h"
#include "math_tool.h"
void init_datou(DATOU*d,uint8_t a,uint8_t m,uint16_t s,uint8_t ac,uint8_t dir){d->addr=a;d->mode=m;d->speed=s;d->acc=ac;d->dir=dir;d->sta=0;d->clk=0;d->angle=0;}
void driver_datou(DATOU*d){if(!d->mode)Emm_V5_Vel_Control(d->addr,d->dir,d->speed,d->acc,false);else{d->clk=(uint32_t)(ABS(d->angle)*25600.0f/6.28f);d->dir=d->angle<0;Emm_V5_Pos_Control(d->addr,d->dir,d->speed,d->acc,d->clk,true,false);}}
void driver_datou_Multiple_laps(DATOU*d,uint8_t l){if(!d->mode)Emm_V5_Vel_Control(d->addr,d->dir,d->speed,d->acc,false);else{d->clk=(uint32_t)(ABS(d->angle)*3200.0f/6.28f)*l;d->dir=d->angle<0;Emm_V5_Pos_Control(d->addr,d->dir,d->speed,d->acc,d->clk,true,false);}}
void datou_set_zero(DATOU*d){if(d->sta==0){Emm_V5_En_Control(d->addr,false,false);d->sta=1;}else if(d->sta==1){Emm_V5_En_Control(d->addr,true,false);d->sta=2;}else{Emm_V5_Origin_Set_O(d->addr,true);d->sta=0;}}
void datou_return_zero(DATOU*d,uint8_t m){Emm_V5_Origin_Trigger_Return(d->addr,m,false);} void datou_enable(void*p){Emm_V5_En_Control((uint8_t)(uintptr_t)p,true,false);} void datou_disable(void*p){Emm_V5_En_Control((uint8_t)(uintptr_t)p,false,false);}
