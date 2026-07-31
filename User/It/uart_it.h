#ifndef __UART_IT_H
#define __UART_IT_H

#include <stdint.h>

void init_uart_pid_tune(void);
void init_uart_gyr(void);
void init_stepper_uart(void);
void stepper_request_move(uint32_t clk, uint16_t vel, uint8_t acc);
void stepper_publish_absolute(int32_t absolute_pulse, uint16_t vel, uint8_t acc);
void stepper_target_disable(void);
void stepper_service(void);
void init_maixcam_uart(void);

#endif
