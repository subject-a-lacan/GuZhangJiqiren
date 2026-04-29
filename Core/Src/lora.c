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
void ESP8266_Init(char *ssid, char *pwd, char *ip, char *port) {
    char buf[128];

    // 1. Set mode to Station and restart
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CWMODE=1\r\n", 13, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+RST\r\n", 8, 100);
    HAL_Delay(5000);

    // 2. Connect to WiFi
    sprintf(buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    HAL_Delay(5000);

    // 3. Set single connection mode
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPMUX=0\r\n", 13, 100);
    HAL_Delay(100);

    // 4. Establish TCP connection
    sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip, port);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    HAL_Delay(500);

    // 5. Enable transparent mode and start sending
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPMODE=1\r\n", 14, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPSEND\r\n", 12, 100);
    HAL_Delay(100);
}
