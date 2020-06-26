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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//SHT2 명령어 설정
#define SHT2x_ADDR              0x40<<1
#define SHT2x_HOLD_MASTER_T     0xE3
#define SHT2x_HOLD_MASTER_RH    0xE5
#define SHT2x_NOHOLD_MASTER_T   0xF3
#define SHT2x_NOHOLD_MASTER_RH  0xF5
#define SHT2x_WRITE_USER_REG    0xE6
#define SHT2x_READ_USER_REG     0xE7
#define SHT2x_SOFT_RESET        0xFE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


//LED PWM  펄스 값
uint8_t led_pwm=0;


//Cds ADC 설정 값
uint16_t adcData=0;
char str[16];


//PIANO 설정 값
uint16_t DoReMi[]={523,587,659,698,783,880,987,1046};
uint16_t piano = 0;
uint16_t piano_pwm;



//온 습도 설정 값
uint8_t i2cData[2],mode;
uint16_t val=0;
float TEMP,HUMI;
char str[16];

//UART 통신 수신 값
uint8_t rxData='?';

uint8_t piano_con=0;


//인터럽트 컨트롤 값
uint8_t stop=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void LED();
void Cds();
void PIANO();
void Temp_Humi();
void Menu();

int fputc(int ch, FILE *stream)
{
  HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  lcdInit();
  
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adcData,sizeof(adcData));         // Cds_ADC_DMA 
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);                                        //LED PWM
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);                                       //PIANO PWM
  HAL_UART_Receive_IT(&huart1,&rxData,1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Menu();
  
  while (1)
  {
    
    
    switch(rxData)
    {
    case '0':
      piano=0;
      PIANO();
      break;
    case '1':
      piano=1;
      PIANO();
      break;
    case '2':
      piano=2;
      PIANO();
      break;
    case '3':
      piano=3;
      PIANO();
      break;
    case '4':
      piano=4;
      PIANO();
      break;
    case '5':
      piano=5;
      PIANO();
      break;
    case '6':
      piano=6;
      PIANO();
      break;
    case '7':
      piano=7;
      PIANO();
      break;
    case 'l':
      LED();
      break;
    case 'c':
      Cds();
      break;
    case 't':
      Temp_Humi();
      break;
    }
    
    rxData='?';
    HAL_Delay(5);
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  if(huart->Instance == USART1)
  {
    
    HAL_UART_Receive_IT(&huart1,&rxData,1);
      
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) //인터럽트
{
  if(GPIO_PIN==GPIO_PIN_12) stop=1;       
}

void Menu()
{
  
  printf("+++ Memu +++\n");
  printf("L : LED\n");
  printf("C : CDS(DMA)\n");
  printf("0~7 : PIANO\n");
  printf("T : Temp/Humi\n");
  printf("Push Button : stop\n\n");
  
  
}

void LED()
{
  printf("LED PWM PLAY\n");
  while(1)
  {
    if(stop==0)
    {
      
      TIM2->CCR1=led_pwm;
      
      led_pwm++;
      
      if(led_pwm==99) led_pwm=0;
      
      HAL_Delay(10);
      
    }
    else 
    {
      stop=0;
      TIM2->CCR1=0;
      printf("LED PWM OFF\n\n");
      Menu();
      break;
    }
  }
  
}

void Cds()
{
  lcdClear();
  lcdGotoXY(0,0);
  lcdPrintData("  ADC_DMA_Mode",14);
  printf("Cds LCD PLAY\n");
  while(1)
  {
    if(stop==0)
    {
      sprintf(str,"Cds : %d",adcData);
      lcdGotoXY(0,1);
      lcdPrint(str);
      HAL_Delay(500);
      
    }
    else
    {
      stop=0;
      lcdClear();
      printf("Cds LCD OFF\n\n");
      Menu();
      break;
    }
  }
  
}

void PIANO()
{
  TIM5->EGR = TIM5->EGR | 0xfe;
  
  piano_pwm= 8400000/DoReMi[piano];
  TIM5->ARR = piano_pwm-1;
  TIM5->CCR3= piano_pwm/2;
  HAL_Delay(500);
  TIM5->ARR = 0;
  TIM5->CCR3 = 0;
  TIM5->EGR = TIM5->EGR | 0xff;
  
  
}

void Temp_Humi()
{
  printf("Temp_Humi PLAY\n");
  while(1)
  {
    if(stop==0)
    {
      mode= SHT2x_NOHOLD_MASTER_T; //mode 설정.
      
      HAL_I2C_Master_Transmit(&hi2c1,SHT2x_ADDR,&mode,1,10); //mode 설정 명령.
      HAL_Delay(100);
      HAL_I2C_Master_Receive(&hi2c1,SHT2x_ADDR,i2cData,2,10); // 온도 데이터 받기.
      
      val = i2cData[0] <<8 | i2cData[1];
      
      TEMP=-46.85 + 175.72*((float)val/65536); //온도 계산
      
      lcdGotoXY(0,0);
      sprintf(str,"TEMP : %.1lf`C",TEMP);
      lcdPrint(str);
      
      mode= SHT2x_NOHOLD_MASTER_RH;
      HAL_I2C_Master_Transmit(&hi2c1,SHT2x_ADDR,&mode,1,10); //mode 설정 명령.
      HAL_Delay(100);
      HAL_I2C_Master_Receive(&hi2c1,SHT2x_ADDR,i2cData,2,10); // 온도 데이터 받기.
      
      val = i2cData[0] <<8 | i2cData[1];
      
      HUMI= -6+125*((float)val/65536); //습도 계산
      
      lcdGotoXY(0,1);
      sprintf(str,"HUMI : %.1lf%",HUMI);
      lcdPrint(str);
    }
    else
    {
        stop=0;
        lcdClear();
        printf("Temp_Humi OFF\n\n");
        Menu();
        break;
    }
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
