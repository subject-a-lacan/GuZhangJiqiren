#ifndef __DEVICE_TEST_H
#define __DEVICE_TEST_H

#include <stdint.h>

void LED_ALL_TEST(void);
void BUZZER_1_TEST(void);
void KEY_BUTTON_TEST(void);
void GRAY_ADC_TEST(void);
void PCB_UART1_TEST(void);
void PCB_UART2_TEST(void);
void PCB_UART3_TEST(void);
void PCB_UART4_TEST(void);
void LED_STATUS_TOGGLE_TEST(void);
void BUZZER_STATUS_TOGGLE_TEST(void);
void MOTOR_test(int16_t speed0, int16_t speed1, uint8_t uart_pcb);

#endif
