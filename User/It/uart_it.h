#ifndef __UART_IT_H
#define __UART_IT_H

#include <stdint.h>

extern volatile uint32_t uart_gyr_dma_error_count;

void init_uart_pid_tune(void);
void init_uart_gyr(void);
void consume_uart_gyr(void);
void init_maixcam_uart(void);

#endif
