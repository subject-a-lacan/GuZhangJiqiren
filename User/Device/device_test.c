#include "device_test.h"
#include "gpio.h"
#include "usart.h"
#include "status.h"
#include "gw_anagloge.h"
#include "log.h"

static void led_set(uint8_t board, uint8_t led1, uint8_t led2) {
  status.device.led_on_board.on = board;
  status.device.led1.on = led1;
  status.device.led2.on = led2;
}

void LED_ALL_TEST(void) {
  led_set(1, 0, 0); HAL_Delay(500);
  led_set(0, 1, 0); HAL_Delay(500);
  led_set(0, 0, 1); HAL_Delay(500);
}

void BUZZER_1_TEST(void) {
  status.device.buzzer.on = 1;
  HAL_Delay(1000);
  status.device.buzzer.on = 0;
  HAL_Delay(1000);
}

void KEY_BUTTON_TEST(void) {
  driver_button(&status.device.button_D2);
  driver_button(&status.device.button_B11);
}

void GRAY_ADC_TEST(void) {
  get_gw_raw_data(&status.sensor.gw_analogue);
  log_uprintf(&huart1, "%u,%u,%u,%u,%u,%u,%u,%u\r\n",
      status.sensor.gw_analogue.channel[0],
      status.sensor.gw_analogue.channel[1],
      status.sensor.gw_analogue.channel[2],
      status.sensor.gw_analogue.channel[3],
      status.sensor.gw_analogue.channel[4],
      status.sensor.gw_analogue.channel[5],
      status.sensor.gw_analogue.channel[6],
      status.sensor.gw_analogue.channel[7]);
}

static void uart_test(UART_HandleTypeDef *huart) {
  static const uint8_t msg[] = "1,2,3\r\n";
  HAL_UART_Transmit(huart, (uint8_t *)msg, sizeof(msg) - 1, 20);
}

void PCB_UART1_TEST(void) { uart_test(&huart1); }
void PCB_UART2_TEST(void) { uart_test(&huart2); }
void PCB_UART3_TEST(void) { uart_test(&huart3); }
void PCB_UART4_TEST(void) { uart_test(&huart4); }

void LED_STATUS_TOGGLE_TEST(void) {
  status.device.led_on_board.on ^= 1;
  status.device.led1.on ^= 1;
  status.device.led2.on ^= 1;
}

void BUZZER_STATUS_TOGGLE_TEST(void) {
  status.device.buzzer.on ^= 1;
  if (status.device.buzzer.on) status.device.buzzer.off_time = status.state.time + 140;
}

void MOTOR_test(int16_t speed0, int16_t speed1, uint8_t uart_pcb) {
  (void)uart_pcb;
  status.motor.wheel[0].tar_speed = speed0;
  status.motor.wheel[1].tar_speed = speed1;
  status.state.motion = MOTOR_TEST;
}

