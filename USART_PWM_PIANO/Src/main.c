/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint16_t CCR1_Val = 0;
uint8_t rxData=0;
int piano_con=0;
uint32_t pwmfreq;
uint16_t piano = 0;
uint16_t DoReMi[]={523,587,659,698,783,880,987,1046};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PIANO_PLAY()
{
  TIM2->EGR = TIM2->EGR & 0xfe;
  
  pwmfreq= 84000000/DoReMi[piano];
  TIM2->ARR = pwmfreq-1;
  TIM2->CCR1 = pwmfreq/2;
  HAL_Delay(1000);
  piano_con=0;
  TIM2->ARR = 0;
  TIM2->CCR1 = 0;
  TIM2->EGR = TIM2->EGR & 0xff;
  
  
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,&rxData,1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);  //timer start!!!!
  
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    TIM2->EGR = TIM2->EGR & 0xff;
    pwmfreq= 84000000/DoReMi[piano];
    TIM2->ARR = pwmfreq-1;
    TIM2->CCR1 = pwmfreq/2;
    
    piano++;
    if(piano>8) piano = 0;
    
    HAL_Delay(1000);
    TIM2->EGR = TIM2->EGR & 0xfe;
    */
   if(piano_con==1) PIANO_PLAY(); 
   
    
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 인터럽트 
{
  if(huart->Instance == USART1)
  {
    
    HAL_UART_Receive_IT(&huart1,&rxData,1);
    HAL_UART_Transmit(&huart1,&rxData,1,10);
    
    switch(rxData)
    {
    case '0':
      piano=0;
      break;
    case '1':
      piano=1;
      break;
    case '2':
      piano=2;
      break;
    case '3':
      piano=3;
      break;
    case '4':
      piano=4;
      break;
    case '5':
      piano=5;
      break;
    case '6':
      piano=6;
      break;
    case '7':
      piano=7;
      break;
    
    }
    piano_con=1;
    
    
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
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
