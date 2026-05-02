/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "status.h"
#include "uart_it.h"
#include "lora.h"
#include "wheel.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int16_t cmd_speed = 0;
volatile int16_t actual_speed0 = 0;
volatile int16_t actual_speed1 = 0;
volatile uint32_t rx_count = 0;
volatile uint8_t last_rx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int16_t clamp_pwm(int16_t pwm) {
  if (pwm > TRUST_CONFINE) {
    return TRUST_CONFINE;
  }
  if (pwm < -TRUST_CONFINE) {
    return -TRUST_CONFINE;
  }
  return pwm;
}

static uint16_t abs_pwm(int16_t pwm) {
  return (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;
}

static char feedforward_cmd_buf[16];
static uint8_t feedforward_cmd_len = 0;

static void set_feedforward_pwm(void) {
  int16_t pwm = clamp_pwm(cmd_speed);

  status.motor.wheel[0].trust = pwm;
  status.motor.wheel[1].trust = pwm;

  set_wheel_dir(&status.motor.wheel[0], pwm);
  set_wheel_dir(&status.motor.wheel[1], pwm);

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, abs_pwm(pwm));
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs_pwm(pwm));
}

static void handle_feedforward_cmd(const char *cmd) {
  int value = 0;

  if (cmd[0] != 'C') {
    return;
  }
  if (cmd[1] == 'z' || cmd[1] == 'Z') {
    cmd_speed = 0;
    return;
  }
  if (sscanf(&cmd[1], "%d", &value) == 1) {
    if (value > TRUST_CONFINE) {
      value = TRUST_CONFINE;
    } else if (value < -TRUST_CONFINE) {
      value = -TRUST_CONFINE;
    }
    cmd_speed = (int16_t)value;
  }
}

static void feedforward_cmd_apply(void) {
  if (feedforward_cmd_len > 1) {
    feedforward_cmd_buf[feedforward_cmd_len] = '\0';
    handle_feedforward_cmd(feedforward_cmd_buf);
    set_feedforward_pwm();
  }
  feedforward_cmd_len = 0;
}

static void feedforward_cmd_put_char(char ch) {
  if (ch == 'C' || ch == 'c') {
    feedforward_cmd_buf[0] = 'C';
    feedforward_cmd_len = 1;
    return;
  }

  if (feedforward_cmd_len == 0) {
    return;
  }

  if (ch == '\r' || ch == '\n') {
    feedforward_cmd_apply();
    return;
  }

  if ((ch == 'z' || ch == 'Z') && feedforward_cmd_len == 1) {
    feedforward_cmd_buf[feedforward_cmd_len++] = ch;
    feedforward_cmd_apply();
    return;
  }

  if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+') {
    if (feedforward_cmd_len < sizeof(feedforward_cmd_buf) - 1) {
      feedforward_cmd_buf[feedforward_cmd_len++] = ch;
    } else {
      feedforward_cmd_len = 0;
    }
  }
}

static void poll_feedforward_cmd(uint32_t duration_ms) {
  uint32_t start = HAL_GetTick();

  while (HAL_GetTick() - start < duration_ms) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
      last_rx = (uint8_t)(huart1.Instance->RDR & 0xff);
      rx_count++;
      feedforward_cmd_put_char((char)last_rx);
    }
  }

  feedforward_cmd_apply();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  status.state.T = 1;
  status.state.time = 0;
  init_wheel(&status.motor.wheel[0], 1, -1);
  init_wheel(&status.motor.wheel[1], 2, 1);

  ESP8266_Init("F521F520","f521f520","192.168.112.73","8080");
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  // after_init_state();

  HAL_TIM_Base_Start_IT(&htim5);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    poll_feedforward_cmd(100);
    set_feedforward_pwm();
    log_uprintf(&huart1, "%d,%d,%d,%lu,%u\r\n", cmd_speed, actual_speed0, actual_speed1, (unsigned long)rx_count, last_rx);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
