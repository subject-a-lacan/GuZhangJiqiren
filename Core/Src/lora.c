#include "lora.h"
#include "main.h"
#include <stdio.h>
#include "usart.h"

/**
 * @brief  Initialize ESP8266
 * @param  ssid:WiFi ssid
 * @param  pwd:WiFi password
 * @param  ip:local IP address
 * @param  port:TCP port
 * @retval None
 */
static void uart1_tx(uint8_t *data, uint16_t size) {
    if (HAL_UART_Transmit(&huart1, data, size, 100) != HAL_OK) {
        huart1.gState = HAL_UART_STATE_READY;
    }
}

void ESP8266_Init(char *ssid, char *pwd, char *ip, char *port) {
    char buf[128];

    // 1. Set mode to Station and restart
    uart1_tx((uint8_t*)"AT+CWMODE=1\r\n", 13);
    uart1_tx((uint8_t*)"AT+RST\r\n", 8);
    HAL_Delay(5000);

    // 2. Connect to WiFi
    sprintf(buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    uart1_tx((uint8_t*)buf, strlen(buf));
    HAL_Delay(5000);

    // 3. Set single connection mode
    uart1_tx((uint8_t*)"AT+CIPMUX=0\r\n", 13);
    HAL_Delay(100);

    // 4. Establish TCP connection
    sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip, port);
    uart1_tx((uint8_t*)buf, strlen(buf));
    HAL_Delay(500);

    // 5. Enable transparent mode and start sending
    uart1_tx((uint8_t*)"AT+CIPMODE=1\r\n", 14);
    HAL_Delay(100);
    uart1_tx((uint8_t*)"AT+CIPSEND\r\n", 12);
    HAL_Delay(100);
}
