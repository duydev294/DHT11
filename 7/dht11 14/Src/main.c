/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER(&htim2) < time);
}
uint8_t data[5];
void set_output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void set_input(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
//void DHT11_Start(void){
//	set_output(dht_pin_GPIO_Port,dht_pin_Pin);
//	HAL_GPIO_WritePin(dht_pin_GPIO_Port,dht_pin_Pin,0); // pull pin low
//	HAL_Delay(18);
//	HAL_GPIO_WritePin(dht_pin_GPIO_Port,dht_pin_Pin,1); // pull pin high
//	delay(40);
//	set_input(dht_pin_GPIO_Port,dht_pin_Pin);
//}
//uint8_t check_res(void)
//{
//	uint8_t res;
//	// check 80us low
//		__HAL_TIM_SET_COUNTER(&htim2,0);
//	while(__HAL_TIM_GET_COUNTER(&htim2)< 100)
//	{
//			if(HAL_GPIO_ReadPin(dht_pin_GPIO_Port,dht_pin_Pin)) {break ;}
//	}
//	if(__HAL_TIM_GET_COUNTER(&htim2) >= 100)return -1; 
//	
//	// check 80us high 
//		__HAL_TIM_SET_COUNTER(&htim2,0);
//	while(__HAL_TIM_GET_COUNTER(&htim2)< 100)
//	{
//			if(!HAL_GPIO_ReadPin(dht_pin_GPIO_Port,dht_pin_Pin)) {res = 1;break;}
//	}
//	if(__HAL_TIM_GET_COUNTER(&htim2) >= 100)return -1; 
//	
//	return res;
//}


uint8_t read_dht11(uint8_t *humi,uint8_t* temp)
{
	uint8_t data[5];
	uint8_t sum;
	uint8_t res = 1;
	//start
	set_output(dht_pin_GPIO_Port,dht_pin_Pin);
	HAL_GPIO_WritePin(dht_pin_GPIO_Port,dht_pin_Pin,0); // pull pin low
	HAL_Delay(18);
	HAL_GPIO_WritePin(dht_pin_GPIO_Port,dht_pin_Pin,1); // pull pin high
	delay(40);
	set_input(dht_pin_GPIO_Port,dht_pin_Pin);
	//check res 
		__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER(&htim2)< 100)
	{
			if(HAL_GPIO_ReadPin(dht_pin_GPIO_Port,dht_pin_Pin)) {res = 1;break;}
	}
	if(__HAL_TIM_GET_COUNTER(&htim2) >= 100)return -1; 
	
	// check 80us high 
		__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER(&htim2)< 100)
	{
			if(!HAL_GPIO_ReadPin(dht_pin_GPIO_Port,dht_pin_Pin)) {res = 1;break;}
	}
	if(__HAL_TIM_GET_COUNTER(&htim2) >= 100)return 2; 
	// read dht
	
	for(int i = 0; i < 5; i++)
	{
		data[i] = 0;
		for(int j = 0; j< 8; j ++)
		{
			// check start trans
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			while(__HAL_TIM_GET_COUNTER(&htim2) < 80)
			{
				if(HAL_GPIO_ReadPin(dht_pin_GPIO_Port,dht_pin_Pin)){res = 1; break;}
			}
			if(__HAL_TIM_GET_COUNTER(&htim2) >= 80) return 3;
			
			// check data 
			 __HAL_TIM_SET_COUNTER(&htim2,0);
			while(__HAL_TIM_GET_COUNTER(&htim2)< 120 )
			{
				if(!(HAL_GPIO_ReadPin(dht_pin_GPIO_Port,dht_pin_Pin))){res = 1; break;}
			}
			uint16_t tim; tim= __HAL_TIM_GET_COUNTER(&htim2);
			if(tim > 100 || tim <= 5) return tim;
			else{
				data[i] <<= 1;
				if(tim > 45){
					
					data[i] |= 1;
				}
			}
		}
	}
		sum = data[0]+data[1]+data[2]+data[3];
		if(sum != data[4]) return 4;
		else{
			*humi = data[0];
			*temp = data[2];
		}
		return res;
}

double Get_Temp(){
		double ntc_volt;
		double Res[5];
		double Temp;
		for(uint8_t i = 0; i < 5; i++){
		
			//HAL_ADC_PollForConversion(&hadc1,1000);
			HAL_ADC_Start(&hadc1);
			ntc_volt = (3.3/4096)*HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			Res[i] = (ntc_volt*10000)/(3.3-ntc_volt);
		}
		double res_ave = (Res[0]+Res[1]+Res[2]+Res[3]+Res[4])/5;
		double A =0.001125308852122,B = 0.001125308852122,C = 0.000000085663516,ln =log(res_ave) ;
		Temp = 1/(A + B*ln + C*ln*ln*ln) - 273.15;
		return Temp;
	}
void send_uart(char *buff)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)buff,strlen(buff),1000);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
		send_uart("uart2 _ok!\n");
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		uint8_t res;
		uint8_t temp, humi,temp_ntc10k = -1;
		char buff[30];
		res = read_dht11(&humi,&temp);
		temp_ntc10k = (uint8_t)Get_Temp();
		if(res == 1){
			// data send uart humi_dht11||temp_dht11||temp_ntc10k
			sprintf(buff,"%d||%d||%d",humi,temp,temp_ntc10k);
			send_uart(buff);
		}
//		if(res == 255) send_uart("Error\n");
		HAL_Delay(500);
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led_Pin|dht_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_Pin dht_pin_Pin */
  GPIO_InitStruct.Pin = led_Pin|dht_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
