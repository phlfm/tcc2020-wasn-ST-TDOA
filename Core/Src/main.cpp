
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
/* CH_PLOT allows user to inspect sample buffer for a given channel using SWV
 * to enable sample buffer inspection CH_PLOT must be defined
 * CH_PLOT also works as channel selection (0, 1 or 2)
 * CH_PLOT_STEP represents how many samples to skip between iterations
 * CH_WAIT_TIMER is how many cycles to wait before re-enabling sampling timer
 * CH_WAIT_UPDATE is how many cycles to wait before updating ch_plot. If the update is too fast, SWV can't keep up.
 *
 * user can monitor ch_plot for the sample buffer value and iCH for the index of the current value
 *
 **/
#define CH_PLOT_STEP 20
#define CH_WAIT_TIMER 1000
#define CH_WAIT_UPDATE 10000

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef CH_PLOT
	static uint16_t ch_plot = 0;
	static unsigned int iCH = 0;
#endif
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint16_t ADC_buffer[ADC_BUFFER_SIZE];
uint8_t UART_ReceivedChar = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Overwrite _write function from syscalls so we can use it to printf into Serial Wire Viewer (SWV) console */
int _write(int file, char *ptr, int len) {
	// Implement our write function. It is used for printf and puts
	int i = 0;
	for (i = 0; i < len; ++i) {
		ITM_SendChar((*ptr++));
	}
	return i;
} // write
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
  //Microphone<int, float> meuMic();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  // Limpa buffer
  	  for (unsigned int i = 0; i < ADC_BUFFER_SIZE; ++i) { ADC_buffer[i] = 0; }
  // Liga prefetch quando disponivel
  	  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  	  if (HAL_GetREVID() == 0x1001)
  	  {
  	    /* Enable the Flash prefetch */
  	    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  	  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Turn LED Green ON
  	  HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(GPIOD, LED_R_Pin, GPIO_PIN_RESET); // Turns ON on HAL_ERROR
  // Turn LED Green OFF if any initialization fails
  	  // Start timer
  	   	  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
  	  // Initialize ADC with DMA transfer
  	  	  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, ADC_BUFFER_SIZE) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
  	  // Tell UART to interrupt after receiving one char (8 bits)
  	  // The received char is used for command processing
  	  	HAL_UART_Receive_IT(&huart1, &UART_ReceivedChar, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_G_Pin|LED_O_Pin|LED_R_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_G_Pin LED_O_Pin LED_R_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_O_Pin|LED_R_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Every time ADC_ConvCpltCallback is called it will increment this variable
// Every time HAL_UART_TxCpltCallback is called it will decrement this variable
// So if ADC+DMA is filling the buffer faster than the UART is transmitting it will grow.
uint16_t ADC_UART_DIFF = 0;
/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_TogglePin(GPIOD, LED_O_Pin);
} // HAL_ADC_ConvHalfCpltCallback

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	++ADC_UART_DIFF;
	HAL_GPIO_TogglePin(GPIOD, LED_B_Pin);
	// Tell UART to transmit one channel
	// TODO: fix DMA memory access... Currently UART is accessing linearly (mixing ch1,2,3...)
	/** Buffer holds ADC_BUFFER_SIZE/CHANNEL_COUNT samples per channel. Each sample is 16 bits
	* UART transmits 8 bits. Thus Buffer holds (BUFFER_SIZE/CHANNEL*2) "items"
	* To transmit 1/10 total items the "size" we tell UART_DMA is: BSIZE/CHANNEL*2/10 --> SIZE/COUNT/5 **/
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&ADC_buffer), ADC_BUFFER_SIZE/CHANNEL_COUNT/5);
#ifdef CH_PLOT
	// Pause the sampling timer
		if (HAL_TIM_Base_Stop(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, 0); }
	// Loop through sample buffer loading values into ch_plot
    // User can inspect sample buffer by looking at ch_plot in SWV
	  for (iCH = CH_PLOT; iCH < ADC_BUFFER_SIZE;) {
		  ch_plot = ADC_buffer[iCH];
		  iCH += 3*CH_PLOT_STEP;
		  for (int i = 0; i < CH_WAIT_UPDATE; ++i){}
	  }
	  for (int i = 0; i < CH_WAIT_TIMER; ++i){}
  	  // Restart sampling timer
  	   	  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, 0); }
#endif // CH_PLOT
} // HAL_ADC_ConvCpltCallback

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	--ADC_UART_DIFF;
} // HAL_UART_TxCpltCallback

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		switch(UART_ReceivedChar) {
		case 'e': // toggle error (red LED)
			HAL_GPIO_TogglePin(GPIOD, LED_R_Pin); break;

		} // switch UART_ReceivedChar
	} // if huart1
} //HAL_UART_RxCpltCallback
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" {
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(GPIOD, LED_R_Pin, GPIO_PIN_SET);
	while(1) {}
  /* USER CODE END Error_Handler_Debug */
}
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
