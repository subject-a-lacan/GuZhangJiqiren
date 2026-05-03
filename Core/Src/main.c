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
#include "math_tool.h"
#include "stdio.h"
#include <stdlib.h>
#include "task.h"
#include "lora.h"

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
int16_t cmd_speed = 40;
extern uint8_t cross_cnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
    while (!(USART1->ISR & USART_ISR_TXE));
    USART1->TDR = (uint8_t)ch;
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

  status.state.motion = STOP;
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // 开启 USART1 的接收中断，准备接收调参命令
  ESP8266_Init("F521F520","f521f520","192.168.112.85","8080");
  HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     uint8_t d = status.sensor.gw_analogue.digital_8bit;
    float gw_val = 0.0f;
    if (d & 0x80) gw_val += 1.0f;
    if (d & 0x40) gw_val += 0.1f;
    if (d & 0x20) gw_val += 0.01f;
    if (d & 0x10) gw_val += 0.001f;
    if (d & 0x08) gw_val += 0.0001f;
    if (d & 0x04) gw_val += 0.00001f;
    if (d & 0x02) gw_val += 0.000001f;
    if (d & 0x01) gw_val += 0.0000001f;
    PERIODIC_START(Task_Vofa_Print,200)
    printf("%.7f,"                            // gw 8-bit as float
           "%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,"  // fw: target, actual, out, kp, ki, kd
           "%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,"  // ka: actual, target, out, kp, ki, kd
           "%.7f,%.7f\r\n"                            // task_id, base_speed
           ,
           (double)gw_val,
           // follow_line
           (double)0.0,
           (double)status.sensor.gw_analogue.diff,
           (double)status.state.status_pid.follow_line_pid.out,
           (double)status.state.status_pid.follow_line_pid.kp,
           (double)status.state.status_pid.follow_line_pid.ki,
           (double)status.state.status_pid.follow_line_pid.kd,
           // keep_angle
           (double)status.state.cur_angle,
           (double)(status.state.tar_angle + status.state.initial_angle),
           (double)status.state.status_pid.keep_angle_pid.out,
           (double)status.state.status_pid.keep_angle_pid.kp,
           (double)status.state.status_pid.keep_angle_pid.ki,
           (double)status.state.status_pid.keep_angle_pid.kd,
           // task_id, base_speed
           (double)status.task.task_id,
           (double)status.state.base_speed
           // cross counts
          );

    PERIODIC_END
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
    // case 'a': status.state.status_pid.follow_line_pid.kp = val; log_uprintf(&huart1, "follow_line kp=%.3f\r\n", val); break;
    // case 'c': status.state.status_pid.follow_line_pid.ki = val; log_uprintf(&huart1,"follow_line ki=%.3f\r\n", val); break;
    // case 'e': status.state.status_pid.follow_line_pid.kd = val; printf("follow_line kd=%.3f\r\n", val); break;

    // case 'g': status.state.status_pid.keep_angle_pid.kp = val; printf("keep_angle kp=%.3f\r\n", val); break;
    // case 'i': status.state.status_pid.keep_angle_pid.ki = val; printf("keep_angle ki=%.3f\r\n", val); break;
    // case 'k': status.state.status_pid.keep_angle_pid.kd = val; printf("keep_angle kd=%.3f\r\n", val); break;

    // case 'm': status.motor.wheel[0].wheel_pid.kp = val; printf("wheel0 kp=%.3f\r\n", val); break;
    // case 'o': status.motor.wheel[0].wheel_pid.ki = val; printf("wheel0 ki=%.3f\r\n", val); break;
    // case 'q': status.motor.wheel[0].wheel_pid.kd = val; printf("wheel0 kd=%.3f\r\n", val); break;

    // case 's': status.motor.wheel[1].wheel_pid.kp = val; printf("wheel1 kp=%.3f\r\n", val); break;
    // case 'u': status.motor.wheel[1].wheel_pid.ki = val; printf("wheel1 ki=%.3f\r\n", val); break;
    // case 'w': status.motor.wheel[1].wheel_pid.kd = val; printf("wheel1 kd=%.3f\r\n", val); break;

    // case 'z': status.state.motion = STOP; printf("motion=STOP\r\n"); break;
    // case 'y':
    //   status.state.status_pid.follow_line_pid.integral = 0;
    //   status.state.status_pid.follow_line_pid.last_error = 0;
    //   status.state.status_pid.follow_line_pid.error = 0;
    //   status.state.status_pid.follow_line_pid.out = 0;
    //   status.state.status_pid.keep_angle_pid.integral = 0;
    //   status.state.status_pid.keep_angle_pid.last_error = 0;
    //   status.state.status_pid.keep_angle_pid.error = 0;
    //   status.state.status_pid.keep_angle_pid.out = 0;
    //   status.motor.wheel[0].wheel_pid.integral = 0;
    //   status.motor.wheel[0].wheel_pid.last_error = 0;
    //   status.motor.wheel[0].wheel_pid.error = 0;
    //   status.motor.wheel[0].wheel_pid.out = 0;
    //   status.motor.wheel[1].wheel_pid.integral = 0;
    //   status.motor.wheel[1].wheel_pid.last_error = 0;
    //   status.motor.wheel[1].wheel_pid.error = 0;
    //   status.motor.wheel[1].wheel_pid.out = 0;
    //   status.state.motion = MOTOR_TEST;
    //   break;
       case 'a': status.state.status_pid.follow_line_pid.kp = val;  break;
    case 'c': status.state.status_pid.follow_line_pid.ki = val; break;
    case 'e': status.state.status_pid.follow_line_pid.kd = val; break;

    case 'g': status.state.status_pid.keep_angle_pid.kp = val;  break;
    case 'i': status.state.status_pid.keep_angle_pid.ki = val; break;
    case 'k': status.state.status_pid.keep_angle_pid.kd = val;  break;

    case 'm': status.motor.wheel[0].wheel_pid.kp = val;  break;
    case 'o': status.motor.wheel[0].wheel_pid.ki = val;  break;
    case 'q': status.motor.wheel[0].wheel_pid.kd = val;  break;

    case 's': status.motor.wheel[1].wheel_pid.kp = val;  break;
    case 'u': status.motor.wheel[1].wheel_pid.ki = val;  break;
    case 'w': status.motor.wheel[1].wheel_pid.kd = val;  break;

    case 'b': status.motor.wheel[0].wheel_pid.integral_max = val;  break;
    case 'n': status.motor.wheel[1].wheel_pid.integral_max = val;  break;

    case 'z': status.task.task_running = 0;
              status.task.armed = 0;
              status.task.start_request = 0;
              status.task.stop_request = 0;
              status.task.stop_cmd = 1;

              status.state.motion = STOP;
              status.state.base_speed = 0;

              status.motor.wheel[0].tar_speed = 0;
              status.motor.wheel[1].tar_speed = 0;
    
              break;
    case 'y':
      status.state.status_pid.follow_line_pid.integral = 0;
      status.state.status_pid.follow_line_pid.last_error = 0;
      status.state.status_pid.follow_line_pid.error = 0;
      status.state.status_pid.follow_line_pid.out = 0;
      status.state.status_pid.keep_angle_pid.integral = 0;
      status.state.status_pid.keep_angle_pid.last_error = 0;
      status.state.status_pid.keep_angle_pid.error = 0;
      status.state.status_pid.keep_angle_pid.out = 0;
      status.motor.wheel[0].wheel_pid.integral = 0;
      status.motor.wheel[0].wheel_pid.last_error = 0;
      status.motor.wheel[0].wheel_pid.error = 0;
      status.motor.wheel[0].wheel_pid.out = 0;
      status.motor.wheel[1].wheel_pid.integral = 0;
      status.motor.wheel[1].wheel_pid.last_error = 0;
      status.motor.wheel[1].wheel_pid.error = 0;
      status.motor.wheel[1].wheel_pid.out = 0;
      status.motor.wheel[0].trust = 0;
      status.motor.wheel[1].trust = 0;
      status.task.start_request=1;
      break;
    case 'h': cmd_speed = (int16_t)val;  break;
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
