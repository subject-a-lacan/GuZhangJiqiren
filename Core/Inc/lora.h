#ifndef LORA_H
#define LORA_H

#include "main.h"

extern uint8_t esp8266_ready;
void ESP8266_Init(char *ssid, char *pwd, char *ip, char *port);

#endif
