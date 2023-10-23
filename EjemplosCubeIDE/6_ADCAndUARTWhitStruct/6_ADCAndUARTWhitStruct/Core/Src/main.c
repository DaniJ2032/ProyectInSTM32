/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "stdint.h"

#include "My_adc.h"
#include "printf_out.h"


#define PIN1 GPIO_PIN_0 // Pines de seleccion
#define PIN2 GPIO_PIN_1 // Pines de seleccion
#define PIN3 GPIO_PIN_2 // Pines de seleccion



// Declaracion de estructuras y uniones
typedef struct {
  // Byte de caebcera.
  uint8_t start;
  // Contador.
  uint8_t count;
  // 8 entradas analógicas (12 bits).
  uint16_t inA1;
  uint16_t inA2;
  uint16_t inA3;
  uint16_t inA4;
  uint16_t inA5;
  uint16_t inA6;
  uint16_t inA7;
  uint16_t inA8;
  // 2 salidas analógicas.
  uint16_t outA1;
  uint16_t outA2;
  // 8 entradas digitales.
  uint8_t insDig;
  // 8 salidas digitales.
  uint8_t outsDig;

} frame_t; //Fin de struct

typedef union
{
  frame_t frameEntrada;
  char    frameEntradaChar[24];
} charFrame_t; //Fin de union


void SystemClock_Config(void);

int main(void){

  //-- Inicializacion de bloques --
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();

  // -- Variables del sistema --
	frame_t	dataFrame;
	static uint16_t	lecturaAdc;
	unsigned	char count = 0;
	unsigned	char contador =1;

	while (1){

		lecturaAdc = ADC_Read(); //Lectura del ADC

		switch (contador){

		case 1: dataFrame.inA1 = lecturaAdc; break;
		case 2: dataFrame.inA2 = lecturaAdc; break;
		case 3: dataFrame.inA3 = lecturaAdc; break;
		case 4: dataFrame.inA4 = lecturaAdc; break;
		case 5: dataFrame.inA5 = lecturaAdc; break;
		case 6: dataFrame.inA6 = lecturaAdc; break;
		case 7: dataFrame.inA7 = lecturaAdc; break;
		case 8:

			dataFrame.inA8 = lecturaAdc;
			dataFrame.start = 0x1B;
			dataFrame.count = count;
			dataFrame.outA1 = 0x05;
			dataFrame.outA2 = 0x05;
			dataFrame.insDig =  0xDE;
			dataFrame.outsDig = 0xDE;
			count++;

			// Este es el que se usa para mandar a QT
//			HAL_UART_Transmit(&huart1, (uint8_t *)&dataFrame, sizeof(dataFrame), 0xFFFF);

			//Para debugear en Proteus
			printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n", dataFrame.start, dataFrame.count,
					dataFrame.inA1, dataFrame.inA2, dataFrame.inA3, dataFrame.inA4, dataFrame.inA5,
					dataFrame.inA6, dataFrame.inA7, dataFrame.inA8, dataFrame.outA1, dataFrame.outA2,
					dataFrame.insDig,dataFrame.outsDig);
			contador =0;
			break;
		}

		HAL_GPIO_WritePin(GPIOB, PIN1, (contador & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, PIN2, (contador & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, PIN3, (contador & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		contador++;

		HAL_Delay(1);
	}
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1){
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
