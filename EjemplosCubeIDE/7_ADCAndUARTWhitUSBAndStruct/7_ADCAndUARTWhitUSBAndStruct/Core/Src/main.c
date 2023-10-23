/* Prueba de trama de datos por puerto USB
 *
 * */

#include "main.h"
#include "adc.h"
#include "usb_device.h"
#include "gpio.h"
#include "tim.h"
#include "string.h"
#include "usbd_cdc_if.h" // Para comunicar por puerto serie
#include "My_adc.h"
#include "stdint.h"

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


#define PIN1 GPIO_PIN_0 // Pines de seleccion
#define PIN2 GPIO_PIN_1 // Pines de seleccion
#define PIN3 GPIO_PIN_2 // Pines de seleccion

void SystemClock_Config(void);


int main(void) {

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();

  HAL_TIM_Base_Start_IT(&htim2);	//Enable timer interrup


	while (1){

	}

}

void SystemClock_Config(void)
{
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	static unsigned int contador = 1;
	static unsigned char count = 0;
	static frame_t dataFrame;
	static uint16_t lecturaAdc;

	if (htim->Instance == TIM2){

		lecturaAdc = ADC_Read(); //Lectura del ADC

        HAL_GPIO_WritePin(GPIOB, PIN1, (contador & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, PIN2, (contador & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, PIN3, (contador & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        switch (contador){

        case 1:	dataFrame.inA1 = lecturaAdc; break;
        case 2: dataFrame.inA2 = lecturaAdc; break;
        case 3: dataFrame.inA3 = lecturaAdc; break;
        case 4: dataFrame.inA4 = lecturaAdc; break;
        case 5: dataFrame.inA5 = lecturaAdc; break;
        case 6: dataFrame.inA6 = lecturaAdc; break;
        case 7: dataFrame.inA7 = lecturaAdc; break;
        case 8:
        		dataFrame.inA8 = lecturaAdc;
        		count++;
        		dataFrame.start = 0x1B;
        		dataFrame.count = count;
        	    dataFrame.outA1 = 0x05;
        	    dataFrame.outA2 = 0x05;
        	    dataFrame.insDig =  0xDE;
        	    dataFrame.outsDig = 0xDE;
        	    contador=0;
        	    CDC_Transmit_FS((uint8_t *) &dataFrame, sizeof(dataFrame));
//				printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u \r\n", dataFrame.start, dataFrame.count,
//						dataFrame.inA1, dataFrame.inA2, dataFrame.inA3, dataFrame.inA4, dataFrame.inA5,
//						dataFrame.inA6, dataFrame.inA7, dataFrame.inA8, dataFrame.outA1, dataFrame.outA2,
//						dataFrame.insDig,dataFrame.outsDig);
        break;
        }

    	contador++;
	}


}



void Error_Handler(void){

  __disable_irq();
  while (1){

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
