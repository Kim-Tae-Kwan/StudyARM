#include <GPIO.h>

void SystemClock_Config(void);
void Error_Handler(void);
void UsedPin_Config();

int main()
{
  uint16_t ledPIN[]={GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3};
  HAL_Init();
  SystemClock_Config();
  
  
  __HAL_RCC_GPIOH_CLK_ENABLE();// osilrator ����� ��Ʈ Ȱ��ȭ
  __HAL_RCC_GPIOA_CLK_ENABLE();// ST_Link ���� ��Ʈ Ȱ��ȭ
  __HAL_RCC_GPIOC_CLK_ENABLE();// LED ���� ��Ʈ Ȱ��ȭ
  UsedPin_Config();
  
  
  while(1)
  {
    for(int i=0;i<4;i++)
    {
      HAL_GPIO_WritePin(GPIOC,ledPIN[i],GPIO_PIN_SET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(GPIOC,ledPIN[i],GPIO_PIN_RESET);
      HAL_Delay(500);
    }
    
  }
  
  
  
}

void UsedPin_Config()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.Pin = GPIO_PIN_0 |GPIO_PIN_1 |GPIO_PIN_2 |GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // ���Ƿ����� �ʱ�ȭ
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Ŭ�� �ʱ�ȭ
  
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

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  
  /* USER CODE END Error_Handler_Debug */
}