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
#include "pid.h"
#include "wheel.h"
#include "servo.h"
#include "led.h"
#include "button.h"
#include "road.h"
#include "math_tool.h"
#include "stdio.h"
#include <stdlib.h>

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
uint8_t rx_byte = 0;
uint8_t rx_cmd = 0;
uint8_t rx_state = 0;
char rx_buf[20];
uint8_t rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  
  init_status(&status, 1);

  after_init_state();

  HAL_TIM_Base_Start_IT(&htim5);

  status.state.motion = MOTOR_TEST;

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // 开启 USART2 的接收中断，准备接收调参命令

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // log_uprintf(&huart1, "%d,%d,%d,%d,%d,%d\r\n",
    //             status.motor.wheel[0].tar_speed,
    //             status.motor.wheel[0].cur_speed,
    //             status.motor.wheel[0].trust,
    //             status.motor.wheel[1].tar_speed,
    //             status.motor.wheel[1].cur_speed,
    //             status.motor.wheel[1].trust);
    // HAL_Delay(1000);
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

void UART_PID_Tune(uint8_t cmd, float val) {
  switch (cmd) {
    case 'a': status.state.status_pid.follow_line_pid.kp = val; printf("follow_line kp=%.3f\r\n", val); break;
    case 'c': status.state.status_pid.follow_line_pid.ki = val; printf("follow_line ki=%.3f\r\n", val); break;
    case 'e': status.state.status_pid.follow_line_pid.kd = val; printf("follow_line kd=%.3f\r\n", val); break;

    case 'g': status.state.status_pid.keep_angle_pid.kp = val; printf("keep_angle kp=%.3f\r\n", val); break;
    case 'i': status.state.status_pid.keep_angle_pid.ki = val; printf("keep_angle ki=%.3f\r\n", val); break;
    case 'k': status.state.status_pid.keep_angle_pid.kd = val; printf("keep_angle kd=%.3f\r\n", val); break;

    case 'm': status.motor.wheel[0].wheel_pid.kp = val; printf("wheel0 kp=%.3f\r\n", val); break;
    case 'o': status.motor.wheel[0].wheel_pid.ki = val; printf("wheel0 ki=%.3f\r\n", val); break;
    case 'q': status.motor.wheel[0].wheel_pid.kd = val; printf("wheel0 kd=%.3f\r\n", val); break;

    case 's': status.motor.wheel[1].wheel_pid.kp = val; printf("wheel1 kp=%.3f\r\n", val); break;
    case 'u': status.motor.wheel[1].wheel_pid.ki = val; printf("wheel1 ki=%.3f\r\n", val); break;
    case 'w': status.motor.wheel[1].wheel_pid.kd = val; printf("wheel1 kd=%.3f\r\n", val); break;

    case 'z': status.state.motion = STOP; printf("motion=STOP\r\n"); break;
    case 'y':
      status.state.status_pid.follow_line_pid.integral = 0;
      status.state.status_pid.follow_line_pid.last_error = 0;
      status.state.status_pid.follow_line_pid.error = 0;
      status.state.status_pid.keep_angle_pid.integral = 0;
      status.state.status_pid.keep_angle_pid.last_error = 0;
      status.state.status_pid.keep_angle_pid.error = 0;
      status.motor.wheel[0].wheel_pid.integral = 0;
      status.motor.wheel[0].wheel_pid.last_error = 0;
      status.motor.wheel[0].wheel_pid.error = 0;
      status.motor.wheel[1].wheel_pid.integral = 0;
      status.motor.wheel[1].wheel_pid.last_error = 0;
      status.motor.wheel[1].wheel_pid.error = 0;
      status.state.motion = MOTOR_TEST;
      printf("motion=MOTOR_TEST, all PID reset\r\n");
      break;
    default: break;
  }
}

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
