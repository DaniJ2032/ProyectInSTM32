/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define rx_buffer_size 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint32_t adc_val[8]; //value of adc inputs
uint16_t dac_val[2];//value of dac outputs
uint8_t digital_In; //value of digital inputs
uint8_t digital_Out;//value of digital output
uint8_t uart_rx_buffer[rx_buffer_size];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC2_Init(void);
/* USER CODE BEGIN PFP */
void stream_send(uint8_t );
void intArrayToString(uint32_t[], char *, int );
uint8_t  calcular_crc8(uint8_t *, uint8_t);
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
  GPIOB->ODR=4096;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_DAC2_Init();
  /* USER CODE BEGIN 2 */

  /******Enable timers**********/
 HAL_TIM_Base_Start_IT(&htim6);
 HAL_UART_Receive_IT(&huart2, uart_rx_buffer,rx_buffer_size);
  SystemCoreClockUpdate();
  /***************************/
/*
  SysTick_Config(SystemCoreClock/1);// set systick interrupt at 1khz
  SysTick->CTRL=0;
  SysTick->VAL=0;
  SysTick->CTRL=(SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_CLKSOURCE_Msk);

 */

 // digital_write(2);

  //write in digital outputs  PA10, PB5, PB4, PB10, PA8, PA9, PC7 , PB6




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 64000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DO_5_Pin|DO_6_Pin|DO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_4_Pin|sel_0_Pin|sel_1_Pin|sel_2_Pin
                          |DO_3_Pin|DO_2_Pin|DO_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_7_GPIO_Port, DO_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DO_5_Pin DO_6_Pin DO_1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DO_5_Pin|DO_6_Pin|DO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_6_Pin DI_7_Pin DI_8_Pin */
  GPIO_InitStruct.Pin = DI_6_Pin|DI_7_Pin|DI_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_1_Pin DI_2_Pin DI_3_Pin */
  GPIO_InitStruct.Pin = DI_1_Pin|DI_2_Pin|DI_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_4_Pin sel_0_Pin sel_1_Pin sel_2_Pin
                           DO_3_Pin DO_2_Pin DO_8_Pin */
  GPIO_InitStruct.Pin = DO_4_Pin|sel_0_Pin|sel_1_Pin|sel_2_Pin
                          |DO_3_Pin|DO_2_Pin|DO_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_7_Pin */
  GPIO_InitStruct.Pin = DO_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_4_Pin DI_5_Pin */
  GPIO_InitStruct.Pin = DI_4_Pin|DI_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
}*/

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {


 // count++;
}



/* this function implements a vector with 8 digital inputs and return it like 8 bit int
 * PB2, PB11, PB12, PA11, PA12, PC5, PC6, PC8 are the inputs where PB2 is the lsb
 *
 * */
uint8_t digital_read() {
    uint8_t inputVector = 0;
    inputVector |= (GPIOB->IDR & GPIO_IDR_2) ? (1 << 0) : 0; //this sentence read directly from the reg of each gpio
    inputVector |= (GPIOB->IDR & GPIO_IDR_11) ? (1 << 1) : 0;
    inputVector |= (GPIOB->IDR & GPIO_IDR_12) ? (1 << 2) : 0;
    inputVector |= (GPIOA->IDR & GPIO_IDR_11) ? (1 << 3) : 0;
    inputVector |= (GPIOA->IDR & GPIO_IDR_12) ? (1 << 4) : 0;
    inputVector |= (GPIOC->IDR & GPIO_IDR_5) ? (1 << 5) : 0;
    inputVector |= (GPIOC->IDR & GPIO_IDR_6) ? (1 << 6) : 0;
    inputVector |= (GPIOC->IDR & GPIO_IDR_8) ? (1 << 7) : 0;
    return inputVector;
}


void digital_write(uint8_t digital_output) {

    GPIOA->ODR = (GPIOA->ODR & ~GPIO_PIN_10) | ((digital_output & 0x1) ? GPIO_PIN_10 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_5) | ((digital_output & 0x2) ? GPIO_PIN_5 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_4) | ((digital_output & 0x4) ? GPIO_PIN_4 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_10) | ((digital_output & 0x8) ? GPIO_PIN_10 : 0);
    GPIOA->ODR = (GPIOA->ODR & ~GPIO_PIN_8) | ((digital_output & 0x10) ? GPIO_PIN_8 : 0);
    GPIOA->ODR = (GPIOA->ODR & ~GPIO_PIN_9) | ((digital_output & 0x20) ? GPIO_PIN_9 : 0);
    GPIOC->ODR = (GPIOC->ODR & ~GPIO_PIN_7) | ((digital_output & 0x40) ? GPIO_PIN_7 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_6) | ((digital_output & 0x80) ? GPIO_PIN_6 : 0);
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{



	  static uint8_t stream_counter = 0;
	  static uint8_t counter = 0;
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (counter >> 0) & 0x01);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (counter >> 1) & 0x01);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (counter >> 2) & 0x01);



    HAL_ADC_Start(&hadc1); // start the adc

    HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion

    adc_val[counter] = HAL_ADC_GetValue(&hadc1)+70; // get the adc value

    HAL_ADC_Stop(&hadc1); // stop adc


    if(counter==7){

    	digital_In=digital_read();//acquire digital inputs values
    	stream_counter = (stream_counter + 1) & 0xFF;
    	stream_send(stream_counter);
    	 //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, 41);
    }

    counter = (counter + 1) % 8;  // 3-bit counter (0 to 7)

}


void intArrayToString(uint32_t arr[], char *output, int size) {
    int i;
    for (i = 0; i < size - 1; i++) {
        sprintf(output, "%ld\t", arr[i]);
        while (*output) {
            output++;
        }
    }
    sprintf(output, "%ld\r\n", arr[size - 1]);
}


void stream_send(uint8_t stream_counter){

	/*in this function we take the data from adc and converse it to array of 32bit to concatenate 96 bits
	 * after that we save the data sorted
	 *
	 * */
	uint32_t temp_chain[3];
	static uint8_t stream_data[20]={0};

	temp_chain[0]=adc_val[0]<<20 | adc_val[1]<<8 |adc_val[2]>>4;
	temp_chain[1]=adc_val[2]<<28 | adc_val[3]<<16 |adc_val[4]<<4 |adc_val[5]>>8;
	temp_chain[2]=adc_val[5]<<24 | adc_val[6]<<12 |adc_val[7];


	stream_data[0]= 0x1B;
	stream_data[1]= stream_counter;
	  // Iniciar desde la posición 2 de la cadena
	    int j = 2;

	    // Copiar los datos en la cadena en orden big-endian
	    for (int i = 0; i < 3; i++) {
	        for (int k = 3; k >= 0; k--) {
	            stream_data[j++] = (temp_chain[i] >> (k * 8)) & 0xFF;
	        }
	    }

	stream_data[14]=digital_In;
	stream_data[15]=digital_Out;

	stream_data[16] = (dac_val[0] >> 4) & 0xFF; // Obtén los bits 11-4
	stream_data[17]= ((dac_val[0] & 0xF) << 4) | ((dac_val[1] >> 8) & 0xF); // Combina bits 3-0 de variable1 con bits 11-8 de variable2
	stream_data[18]= dac_val[1] & 0xFF; // Obtén los bits 7-0 de variable2
    stream_data[19]=calcular_crc8(stream_data,19);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)stream_data, sizeof (stream_data));
}





uint8_t  calcular_crc8(uint8_t *cadena, uint8_t longitud) {
    // Polinomio generador CRC-8 (0x07)
    uint8_t polinomio_generador = 0x07;

    // Inicializa el registro CRC en cero
    uint8_t registro_crc = 0;

    for (uint8_t i = 0; i < longitud; i++) {

        registro_crc ^= cadena[i];

        for (uint8_t j = 0; j < 8; j++) {

            if (registro_crc & 0x80) {
                registro_crc = (registro_crc << 1) ^ polinomio_generador;

            } else {
                registro_crc <<= 1;

            }
        }
    }

    return registro_crc;
}







/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {

	  digital_Out=uart_rx_buffer[2];
	  dac_val[0] = (uart_rx_buffer[3] << 4) | ((uart_rx_buffer[4]>> 4) & 0x0F); // Combina Byte1 y Byte5

	    // Extrae los bits de Char2 y Char3 para formar otro valor de 16 bits
	   dac_val[1] = ((uart_rx_buffer[4] & 0x0F) << 8) | uart_rx_buffer[5];
    /* Receive one byte in interrupt mode */
	  HAL_UART_Receive_IT(&huart2,uart_rx_buffer, rx_buffer_size);
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
  while (1)
  {
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
