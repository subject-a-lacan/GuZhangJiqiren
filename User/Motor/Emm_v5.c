#include "Emm_v5.h"
static void gimbal_send(const uint8_t *cmd, uint16_t len) {
  uint32_t timeout = HAL_GetTick() + 100;
  while (huart3.gState != HAL_UART_STATE_READY) {
    if (HAL_GetTick() >= timeout) {
      HAL_UART_AbortTransmit(GIMBAL_UART);
      break;
    }
  }
  if (HAL_UART_Transmit_DMA(GIMBAL_UART, (uint8_t *)cmd, len) != HAL_OK) {
    HAL_UART_AbortTransmit(GIMBAL_UART);
    HAL_UART_Transmit_DMA(GIMBAL_UART, (uint8_t *)cmd, len);
  }
}
#define SEND(a) gimbal_send((a), sizeof(a))
void Emm_V5_Reset_CurPos_To_Zero(uint8_t a){uint8_t c[]={a,0x0A,0x6D,0x6B};SEND(c);}
void Emm_V5_Reset_Clog_Pro(uint8_t a){uint8_t c[]={a,0x0E,0x52,0x6B};SEND(c);}
void Emm_V5_Read_Sys_Params(uint8_t a,SysParams_t s){uint8_t c[4]={a,0,0x6B,0};switch(s){case S_VER:c[1]=0x1F;break;case S_RL:c[1]=0x20;break;case S_PID:c[1]=0x21;break;case S_VBUS:c[1]=0x24;break;case S_CPHA:c[1]=0x27;break;case S_ENCL:c[1]=0x31;break;case S_TPOS:c[1]=0x33;break;case S_VEL:c[1]=0x35;break;case S_CPOS:c[1]=0x36;break;case S_PERR:c[1]=0x37;break;case S_FLAG:c[1]=0x3A;break;case S_Conf:c[1]=0x42;c[2]=0x6C;break;case S_State:c[1]=0x43;c[2]=0x7A;break;case S_ORG:c[1]=0x3B;break;default:return;}gimbal_send(c,c[2]==0x6B?3:4);}
void Emm_V5_Modify_Ctrl_Mode(uint8_t a,bool sv,uint8_t m){uint8_t c[]={a,0x46,0x69,sv,m,0x6B};SEND(c);}
void Emm_V5_En_Control(uint8_t a,bool st,bool sn){uint8_t c[]={a,0xF3,0xAB,st,sn,0x6B};SEND(c);}
void Emm_V5_Vel_Control(uint8_t a,uint8_t d,uint16_t v,uint8_t ac,bool sn){uint8_t c[]={a,0xF6,d,v>>8,v,ac,sn,0x6B};SEND(c);}
void Emm_V5_Pos_Control(uint8_t a,uint8_t d,uint16_t v,uint8_t ac,uint32_t k,bool ra,bool sn){uint8_t c[]={a,0xFD,d,v>>8,v,ac,k>>24,k>>16,k>>8,k,ra,sn,0x6B};SEND(c);}
void Emm_V5_Stop_Now(uint8_t a,bool sn){uint8_t c[]={a,0xFE,0x98,sn,0x6B};SEND(c);}
void Emm_V5_Synchronous_motion(uint8_t a){uint8_t c[]={a,0xFF,0x66,0x6B};SEND(c);}
void Emm_V5_Origin_Set_O(uint8_t a,bool sv){uint8_t c[]={a,0x93,0x88,sv,0x6B};SEND(c);}
void Emm_V5_Origin_Modify_Params(uint8_t a,bool sv,uint8_t om,uint8_t od,uint16_t ov,uint32_t ot,uint16_t slv,uint16_t sma,uint16_t sms,bool pot){uint8_t c[20]={a,0x4C,0xAE,sv,om,od,ov>>8,ov,ot>>24,ot>>16,ot>>8,ot,slv>>8,slv,sma>>8,sma,sms>>8,sms,pot,0x6B};gimbal_send(c,20);}
void Emm_V5_Origin_Trigger_Return(uint8_t a,uint8_t om,bool sn){uint8_t c[]={a,0x9A,om,sn,0x6B};SEND(c);}
void Emm_V5_Origin_Interrupt(uint8_t a){uint8_t c[]={a,0x9C,0x48,0x6B};SEND(c);}
